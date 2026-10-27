# 📡 Communication Protocol

OpenBMS firmware talks to a host over three ports:

- **UART** — 230400 baud, 8N1. The OpenBMS binary protocol: register read/write and commands, with an 8-bit additive checksum. Also used for bootloader entry and, in the simulation build, for `CC_SIM_SAMPLE`.
- **I²C** — I2C2 slave at address `0x0B`. Carries exactly the same frames as the UART.
- **CAN 2.0** — 1 Mbit/s, 11-bit IDs. A periodic status broadcast plus a small set of short commands. Register read/write is not available on CAN.

All three ports are served by `openbms_comm.c`. Host tools: [`python_scripts/bms_cmd.py`](python_scripts/bms_cmd.py) (typed commands), [`python_scripts/read_all_registers.py`](python_scripts/read_all_registers.py) (register dump) and [`python_scripts/simulation/config_tool.py`](python_scripts/simulation/config_tool.py) (write and save the config from `config.json`).

## OpenBMS Binary Protocol

The protocol is a simple request/response scheme. There is no device address in the frame — one host talks to one OpenBMS board. Every transaction is started by the host; on UART and I²C the device never speaks unsolicited.

| Layer | Name | Responsibility |
|-------|------|----------------|
| Application | OpenBMS register map | Register addresses, data types, access rights, commands |
| Transport | OpenBMS frame | Command byte, address, length byte, additive checksum |
| Physical | UART or I²C | UART 230400 baud 8N1, or I²C slave `0x0B` |

### Frame structure

All frames — in both directions — share the same layout and are at least 4 bytes long:

| Command | Address | Length | Data | Checksum |
|---------|---------|--------|------|----------|
| 1 byte | 1 byte | 1 byte | N bytes | 1 byte |

- **Command** — frame type or command, see the table below.
- **Address** — register address `0x00`–`0xFF` for read and write. `0x00` for commands, ACK and error frames.
- **Length** — payload size in **bytes**. `0x00` for read requests, ACK frames and commands with no argument.
- **Data** — `Length` bytes of payload, **little-endian**, laid out exactly as the C struct field.
- **Checksum** — sum of all preceding bytes of the frame, modulo 256.

Total frame size is always `Length + 4` bytes, at most 259 bytes (255 bytes of payload).

Register data is a raw memory image of the firmware's struct field. The host must know each register's type and element count in advance — the protocol carries no type information. See the register map below.

### Command codes

| Code | Name | Argument | Allowed in | Ports | Description |
|------|------|----------|------------|-------|-------------|
| 0x01 | `CC_WRITE` | register value | any state | UART, I²C | Write a register |
| 0x02 | `CC_READ` | — | any state | UART, I²C | Read a register |
| 0x03 | `CC_ACK` | — | — | reply | Positive reply |
| 0x04 | `CC_ERROR` | 1 byte error code | — | reply | Negative reply |
| 0x05 | `CC_EEPROM_CONFIG` | — | any state | UART, I²C | Save the config bank (`0x30`–`0x6A`) to EEPROM. `CE_EEPROM` if the write fails |
| 0x06 | `CC_HPPC_START` | — | NORMAL_RUN | UART, I²C | Start the HPPC test |
| 0x07 | `CC_NORMAL_RUN` | — | HPPC_RUN | UART, I²C, CAN | Stop the HPPC test (stop reason HOST). The device stays in HPPC_RUN with the FET off until a reboot |
| 0x08 | `CC_SET_FET` | 1 byte, `0` or `1` | NORMAL_RUN | UART, I²C, CAN | Main FET off / on |
| 0x09 | `CC_REBOOT` | — | any state | UART, I²C, CAN | ACK, then reset after 100 ms |
| 0x0A | `CC_PING` | — | any state | UART, I²C | Just ACK |
| 0x0B | `CC_RESET_PROTECTION` | — | any state | UART, I²C, CAN | Clear `protection_trigger`. A fault that is still there trips again after its normal detection time |
| 0x0C | `CC_CLEAR_ERRORS` | — | any state | UART, I²C, CAN | Clear `error_flags` |
| 0x0D | `CC_SET_PRE_FET` | 1 byte, `0` or `1` | NORMAL_RUN | UART, I²C | Pre-charge FET off / on |
| 0x0E | `CC_SIM_SAMPLE` | 1–6 samples | NORMAL_RUN, HPPC_RUN | UART, I²C | Simulation build only, see below |
| 0x0F | `CC_CLEAR_LEARNING` | — | any state | UART, I²C | Clear `learning_status` (`0x96`) and `capacity_update_count` (`0x97`) |
| 0x10 | `CC_CAN_BROADCAST` | 1 byte, `0` or `1` | any state | CAN | Status broadcast off / on (RAM only) |
| 0x11 | `CC_CAN_SEND_ONCE` | — | any state | CAN | Send the status messages once |
| 0x12 | `CC_CLEAR_FAULT` | — | any state | UART, I²C, CAN | Clear the fault snapshot (`0x70`–`0x75`) |
| 0x42 | `CC_BOOTLOADER` | — | any state | UART | Reset into the bootloader, see below |

A command with the wrong argument length or value returns `CE_WRONG_CMD`. A command sent in a state where it is not allowed returns `CE_STATE`. An unknown command returns `CE_WRONG_CMD`; this includes `CC_SIM_SAMPLE` in a normal (non-simulation) build.

### Error codes

When a request fails, the device replies with an error frame `04 00 01 <code> <checksum>`.

| Code | Name | Meaning |
|------|------|---------|
| 0x00 | `CE_OK` | No error (never sent) |
| 0x01 | `CE_WRONG_CMD` | Unknown command, wrong length, or value out of range (NaN/inf float, bad bool, bad cell count, ...) |
| 0x02 | `CE_BAD_CRC` | Checksum mismatch |
| 0x03 | `CE_NO_REG` | Address not in the register map |
| 0x04 | `CE_RO` | Write to a read-only register |
| 0x05 | `CE_EEPROM` | EEPROM write failed (`CC_EEPROM_CONFIG`) |
| 0x06 | `CE_STATE` | Command not allowed in the current `ctrl_state` |

### Read transaction

The host sends a read request with `Length` set to `0x00`. The device replies with the register's **full native size** — there is no partial or block-offset read.

**Request — read `cell_voltage` at `0x00`:**
```
02  00  00  02
│   │   │   └───── checksum — (0x02 + 0x00 + 0x00) & 0xFF
│   │   └───────── length — always 0x00 for a read request
│   └───────────── register address — 0x00
└───────────────── command — CC_READ
```

**Response:**
```
02 00 1C 00 60 67 45 00 20 67 45 ... 00 70 67 45 12
│  │  │  └─ 7 × float32 LE — cell voltages in mV (3702.0, 3698.0, ...)
│  │  └─ length — 28 bytes (7 × 4)
│  └─ register address echo — 0x00
└─ command echo — CC_READ
```

**Request and response — `fet_status` at `0x06`:**
```
02  06  00  08                  ← request
02  06  02  03 00  0D           ← response: uint16 LE 0x0003 (main FET on, pre-FET on)
```

**Request and response — `ctrl_state` at `0x13`:**
```
02  13  00  15                  ← request
02  13  01  02  18              ← response: 2 = NORMAL_RUN
```

### Write transaction

The host sends the register's **full native size** as payload; any other length returns `CE_WRONG_CMD`. The device answers with a bare ACK — it does not echo the written value.

**Request — write `ovp_slow_threshold_mv` at `0x3F` = 4220 mV:**
```
01  3F  02  7C 10  CE
│   │   │   └───┘  └───── checksum
│   │   │   └──────────── data — uint16 LE 4220
│   │   └──────────────── length — 2 bytes
│   └──────────────────── register address — 0x3F
└──────────────────────── command — CC_WRITE
```

**Response — ACK (the same for every successful write and command):**
```
03  00  00  03
│   │   │   └───── checksum
│   │   └───────── length — always 0x00
│   └───────────── address — always 0x00
└───────────────── command — CC_ACK
```

Writes are checked before they are applied:

- `cell_count` (`0x30`) must be 2–7.
- The on/off settings (`0x31`–`0x36`, `0x66`) must be `0` or `1`.
- `model_cell` (`0x9F`) must be 0–6.
- A NaN or inf float is refused, the old value stays.
- The filter noise values `kf_q_soc`, `kf_q_rc1`, `kf_q_rc2`, `kf_r_v` (`0xBB`–`0xBE`) must be above 0.

A refused write returns `CE_WRONG_CMD`.

### Commands

Commands use the command byte itself; the address is `0x00` and the length is the argument size.

**Request — ping:**
```
0A  00  00  0A
```

**Request — save the config to EEPROM:**
```
05  00  00  05
```

**Request — start the HPPC test (NORMAL_RUN only):**
```
06  00  00  06
```

**Request — turn the main FET on (NORMAL_RUN only):**
```
08  00  01  01  0A
│   │   │   │   └───── checksum
│   │   │   └───────── argument — 1 = on
│   │   └───────────── length — 1 byte
│   └───────────────── address — always 0x00
└───────────────────── command — CC_SET_FET
```

All are answered with the standard ACK frame, or with `CE_STATE` if the current state does not allow the command:
```
04  00  01  06  0B
```

### Device states

`ctrl_state` (`0x13`) decides which commands are accepted:

| Value | State | Meaning |
|-------|-------|---------|
| 1 | WAIT_CONFIG | No valid config in EEPROM. Write the config, save it with `CC_EEPROM_CONFIG` and reboot |
| 2 | NORMAL_RUN | Config loaded at boot. Protections, fuel gauge and balancing run; the FET is controlled by the host |
| 3 | HPPC_RUN | HPPC test running or finished. The FET is controlled by the test. Only a reboot goes back to NORMAL_RUN |

Saving the config does not change the state; the saved config is loaded and applied only at the next reboot.

### Bootloader entry

`CC_BOOTLOADER` (`0x42`, ASCII `B`) is handled **before** checksum validation, so on the UART a single unframed `0x42` byte (or `B\n`) is enough. The firmware erases the application flag page, replies with the ASCII string `"1\n"`, waits 100 ms and resets into the bootloader.

The bootloader itself speaks a separate line-based ASCII protocol on the same UART at the same baud rate. `python_scripts/flash.py` drives the full sequence:

| Command | Meaning | Response |
|---------|---------|----------|
| `B\n` | Ping | `0` = bootloader active · `1` = application running, it will reset into the bootloader · `2` = bootloader start failed |
| `E\n` | Erase application flash | `0` = OK |
| `P,<intel hex line>\n` | Program one Intel HEX record | `0` = OK · `1` = EOF record |
| `C,<size>,<crc32>\n` | Verify image — CRC-32/MPEG-2, matching the STM32 HAL CRC peripheral | `0` = match |
| `A\n` | Set application flag | `0` = OK |
| `R\n` | Reset into the application | — |

The application starts at `0x0800A000`; `main()` sets `SCB->VTOR` accordingly.

### UART settings

| Parameter | Value |
|-----------|-------|
| Peripheral | USART1 |
| Baud rate | 230400 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |
| Frame delimiting | Idle line (`HAL_UARTEx_ReceiveToIdle_DMA()`) plus the length byte |
| RX / TX buffers | 259 bytes each, max payload 255 bytes |
| Frame timeout | 50 ms |
| TX timeout | 50 ms |

A frame that arrives in pieces is joined until the length byte says it is complete. A partial frame that is not complete within 50 ms is dropped. A reply waits for the previous reply to finish sending; if that takes longer than 50 ms the send is aborted and `UART_TX_TIMEOUT` is set in `error_flags`. UART errors (overrun, framing, noise) are recovered and set `UART_ERROR`.

### I²C settings

| Parameter | Value |
|-----------|-------|
| Peripheral | I2C2, slave |
| Address | `0x0B` (7-bit) |
| Frames | The same as the UART |

1. The host **writes** a whole request frame in one transfer (START, address + W, frame bytes, STOP). The frame is accepted only if the byte count equals `Length + 4` at STOP.
2. The host then **reads** the reply (START, address + R, bytes, NACK, STOP). Every read starts at byte 0 of the reply.
   - Bytes read as `0x00` mean the reply is not ready yet; read again.
   - Bytes past the end of the reply read as `0xFF`.
   - The host knows the reply size from its length byte (`Length + 4`). NACK is the normal end of a read.

Bytes written while a frame is still being processed are dropped. A transfer with no STOP for 100 ms restarts the I²C and sets `I2C_ERROR`.

## CAN

CAN runs at 1 Mbit/s with 11-bit standard IDs. All values are little-endian. CAN is restarted at most every 1000 ms when it stops (bus-off, errors); this sets `CAN_ERROR`.

### Status broadcast

While the broadcast is on (default on after every boot), the device sends these messages every 100 ms. `CC_CAN_BROADCAST` turns it off or on until the next reboot; `CC_CAN_SEND_ONCE` sends them once.

| ID | Bytes | Content |
|----|-------|---------|
| `0x6B0` | 8 | `uint32` pack voltage mV, `int32` pack current mA (+ = charge) |
| `0x6B1` | 8 | `uint16` × 4 — cell 1–4 voltage mV |
| `0x6B2` | 6 | `uint16` × 3 — cell 5–7 voltage mV |
| `0x6B3` | 4 | `uint16` SOC in 0.1 % (lowest shown cell SOC), `int16` temperature in 0.1 °C |
| `0x6B4` | 8 | `uint8` ctrl_state, `uint16` fet_status, `uint16` protection_trigger, `error_flags` low 24 bits (3 bytes) |

Status messages still waiting in the transmit queue at the next broadcast are dropped, so the bus always carries fresh values.

### Commands

| ID | Direction | Content |
|----|-----------|---------|
| `0x6BF` | host → device | byte 0 = command code (`CC_*`), then its argument |
| `0x6BE` | device → host | byte 0 = command code, byte 1 = result (`CE_*`, `0` = OK) |

Allowed on CAN: `CC_CAN_BROADCAST`, `CC_CAN_SEND_ONCE`, `CC_REBOOT`, `CC_SET_FET`, `CC_RESET_PROTECTION`, `CC_CLEAR_ERRORS`, `CC_CLEAR_FAULT`, `CC_NORMAL_RUN`. Every other command returns `CE_WRONG_CMD`. Send one command and wait for its reply before the next.

**Example — turn the main FET on:**
```
ID 0x6BF  data 08 01           ← CC_SET_FET, argument 1
ID 0x6BE  data 08 00           ← reply: CC_SET_FET, CE_OK
```

## Simulation sample (`CC_SIM_SAMPLE`)

Only in a build with `OPENBMS_SIMULATION` defined in `config.h`. The host sends the measurements and the time step instead of the ADC; each sample runs the control loop once on the simulated time. The real FET and pre-FET pins stay off, but `fet_status` shows what the firmware asked for.

**Request:** `0E 00 <len> <samples> <checksum>`, with 1–6 samples of 38 bytes each (`len` = 38 × n):

| Offset | Type | Field |
|--------|------|-------|
| 0 | `float[7]` | cell voltage mV |
| 28 | `float` | pack current mA (+ = charge) |
| 32 | `float` | temperature °C |
| 36 | `uint16` | time step ms |

The board stops at the first sample that changes the reply state (FET, state, HPPC status, protection or errors); the host resends the rest.

**Reply:** `0E 00 12 <18 bytes> <checksum>`:

| Offset | Type | Field |
|--------|------|-------|
| 0 | `uint16` | fet_status |
| 2 | `uint8` | ctrl_state |
| 3 | `uint16` | hppc_status |
| 5 | `uint16` | hppc_extract_status |
| 7 | `uint16` | protection_trigger |
| 9 | `uint64` | error_flags |
| 17 | `uint8` | samples run |

Allowed in NORMAL_RUN and HPPC_RUN, otherwise `CE_STATE`. A NaN value returns `CE_WRONG_CMD`.

## Register map

Every address not listed below returns `CE_NO_REG`. Access: **r** = read-only (a write returns `CE_RO`), **r/w** = read and write.

| Bank | Range | Structure |
|------|-------|-----------|
| Peripheral data | `0x00` – `0x06` | `Peripheral_Data_t` |
| System data | `0x10` – `0x1A` | `System_Data_t` |
| Config | `0x30` – `0x6A` | `Config_Data_t` |
| Fault snapshot | `0x70` – `0x75` | `Stats_Data_t` |
| Fuel gauge | `0x90` – `0xD7` | `FuelGauge_Data_t` |

### Peripheral data — `0x00` – `0x06`

| Addr | Name | Type | Access | Unit / notes |
|------|------|------|--------|--------------|
| 0x00 | `cell_voltage` | `float[7]` | r | mV, smoothed |
| 0x01 | `pack_voltage` | `float` | r | mV, sum of `cell_count` cells |
| 0x02 | `pack_current` | `float` | r | mA, + = charge |
| 0x03 | `temperature_package` | `float` | r | °C, NTC |
| 0x04 | `temperature_stm32` | `float` | r | °C |
| 0x05 | `main_vdd_voltage_mv` | `float` | r | mV |
| 0x06 | `fet_status` | `uint16` | r | bit 0 main FET, bit 1 pre-FET, bits 2–8 balancing cell 1–7 |

### System data — `0x10` – `0x1A`

| Addr | Name | Type | Access | Unit / notes |
|------|------|------|--------|--------------|
| 0x10 | `main_control` | `uint16` | r | bit 0: 0 = normal, 1 = HPPC |
| 0x11 | `last_communication_timestamp` | `uint32` | r | ms, time of the last good frame |
| 0x12 | `protection_trigger` | `uint16` | r | protection bits, see below. `0` = none |
| 0x13 | `ctrl_state` | `uint8` | r | 1 WAIT_CONFIG, 2 NORMAL_RUN, 3 HPPC_RUN |
| 0x15 | `error_flags` | `uint64` | r | error bits, see below |
| 0x17 | `session_uptime_ms` | `uint32` | r | ms since boot |
| 0x18 | `hppc_direction` | `uint8` | r | 0 unknown, 1 charge, 2 discharge |
| 0x19 | `hppc_status` | `uint16` | r | low byte state, high byte stop reason, see below |
| 0x1A | `hppc_extract_status` | `uint16` | r | extraction bits, see below |

### Config — `0x30` – `0x6A`

All config registers are r/w. A write changes the value in RAM; `CC_EEPROM_CONFIG` saves the whole bank to EEPROM (CRC-32 protected), and the saved config is loaded at the next reboot. There are no firmware defaults — the defaults live in `config.json`.

| Addr | Name | Type | Unit / notes |
|------|------|------|--------------|
| 0x30 | `cell_count` | `uint8` | 2–7 |
| 0x31 | `use_i2c` | `bool` | |
| 0x32 | `use_can` | `bool` | |
| 0x33 | `use_uart` | `bool` | |
| 0x34 | `volt_protection` | `bool` | OVP / UVP on |
| 0x35 | `current_protection` | `bool` | OCP on |
| 0x36 | `temp_protection` | `bool` | OTP / UTP on |
| 0x37 | `cell_capacity` | `uint32` | mAh |
| 0x38 | `voltage_cell_max` | `uint16` | mV, HPPC charge stops here |
| 0x39 | `voltage_cell_min` | `uint16` | mV, HPPC discharge stops here |
| 0x3A | `charging_term_current` | `uint16` | mA |
| 0x3B | `uvp_slow_threshold_mv` | `uint16` | mV |
| 0x3C | `uvp_slow_time_ms` | `uint16` | ms |
| 0x3D | `uvp_fast_threshold_mv` | `uint16` | mV |
| 0x3E | `uvp_fast_time_ms` | `uint16` | ms |
| 0x3F | `ovp_slow_threshold_mv` | `uint16` | mV |
| 0x40 | `ovp_slow_time_ms` | `uint16` | ms |
| 0x41 | `ovp_fast_threshold_mv` | `uint16` | mV |
| 0x42 | `ovp_fast_time_ms` | `uint16` | ms |
| 0x43 | `ocp_charge_threshold_ma` | `uint16` | mA |
| 0x44 | `ocp_charge_time_ms` | `uint16` | ms |
| 0x45 | `ocp_discharge_slow_threshold_ma` | `uint16` | mA |
| 0x46 | `ocp_discharge_slow_time_ms` | `uint16` | ms |
| 0x47 | `ocp_discharge_fast_threshold_ma` | `uint16` | mA |
| 0x48 | `ocp_discharge_fast_time_ms` | `uint16` | ms |
| 0x49 | `otp_threshold_c` | `uint16` | °C |
| 0x4A | `otp_time_ms` | `uint16` | ms, also used by OTP_CHARGE |
| 0x4C | `charge_monitor_cell_mv` | `uint16` | mV, full-charge detection |
| 0x4D | `utp_threshold_c` | `int16` | °C |
| 0x4E | `utp_time_ms` | `uint16` | ms, also used by UTP_CHARGE |
| 0x4F | `otp_charge_threshold_c` | `int16` | °C |
| 0x50 | `ocv_v_lo_mv` | `uint16` | mV, HPPC OCV range |
| 0x51 | `ocv_v_hi_mv` | `uint16` | mV |
| 0x52 | `current_sensor_gain` | `float` | |
| 0x53 | `voltage_gain` | `float[7]` | |
| 0x54 | `ntc_beta` | `float` | |
| 0x55 | `ntc_r_nominal` | `float` | Ω |
| 0x56 | `ntc_r_fixed` | `float` | Ω |
| 0x57 | `ntc_t_nominal` | `float` | °C |
| 0x58 | `temperature_offset` | `float` | °C |
| 0x59 | `shunt_resistance_mohm` | `float` | mΩ |
| 0x5A | `current_offset_ma` | `float` | mA |
| 0x5B | `voltage_offset_mv` | `float[7]` | mV |
| 0x5C | `utp_charge_threshold_c` | `int16` | °C |
| 0x60 | `firmware_version` | `char[32]` | null-padded |
| 0x61 | `hardware_version` | `char[32]` | |
| 0x62 | `manufacturer_name` | `char[32]` | |
| 0x63 | `device_name` | `char[32]` | |
| 0x64 | `device_chemistry` | `char[32]` | |
| 0x65 | `manufacturer_data` | `char[32]` | |
| 0x66 | `balance_enable` | `bool` | |
| 0x67 | `balance_start_mv` | `uint16` | mV, highest cell must be at least this |
| 0x68 | `balance_delta_mv` | `uint16` | mV, cell above the lowest by more than this bleeds |
| 0x69 | `balance_max_temp_c` | `int16` | °C, no balancing above this |
| 0x6A | `balance_resistance_ohm` | `float` | Ω, bleed resistor (fuel gauge subtracts the bleed current) |

`0x4B` and `0x5D`–`0x5F` are unused.

### Fault snapshot — `0x70` – `0x75`

Taken once at the first protection trip and kept until `CC_CLEAR_FAULT`. RAM only, lost at reboot.

| Addr | Name | Type | Access | Unit / notes |
|------|------|------|--------|--------------|
| 0x70 | `voltage` | `uint16[7]` | r | mV per cell |
| 0x71 | `current` | `int16` | r | mA |
| 0x72 | `temperature` | `int8` | r | °C |
| 0x73 | `soc` | `uint8` | r | % |
| 0x74 | `fault_code` | `uint16` | r | `protection_trigger` at the trip |
| 0x75 | `fault_timestamp` | `uint32` | r | `session_uptime_ms` at the trip |

### Fuel gauge — `0x90` – `0xD7`

The per-cell model registers (`0xA1`–`0xAC` and `0xD4`) act on the cell picked with `model_cell` (`0x9F`). To read or write the tables of cell 3, write `0x9F` = 2 first.

**Outputs:**

| Addr | Name | Type | Access | Unit / notes |
|------|------|------|--------|--------------|
| 0x90 | `relative_soc` | `uint8` | r | % |
| 0x92 | `cell_soc` | `uint8[7]` | r | %, shown SOC per cell |
| 0x93 | `cell_soh` | `uint8[7]` | r | %, set by capacity learning |
| 0x94 | `cell_remaining_capacity` | `uint16[7]` | r | mAh |
| 0x96 | `learning_status` | `uint16` | r/w | bit 0 CHARGE, bit 1 DISCHARGE, bit 2 COMPLETE (HPPC tables written) |
| 0x97 | `capacity_update_count` | `uint16` | r | capacity learning updates |
| 0x9F | `model_cell` | `uint8` | r/w | 0–6, cell for `0xA1`–`0xAC` and `0xD4` |

**Model (written by the HPPC extraction, or by the host):**

| Addr | Name | Type | Access | Unit / notes |
|------|------|------|--------|--------------|
| 0xA0 | `soc_grid` | `float[20]` | r/w | %, shared by all tables |
| 0xA1 | `ocv_dis` | `float[20]` | r/w | V, selected cell |
| 0xA2 | `ocv_chg` | `float[20]` | r/w | V, selected cell |
| 0xA3 | `r0_dis` | `float[20]` | r/w | Ω, selected cell |
| 0xA4 | `r1_dis` | `float[20]` | r/w | Ω, selected cell |
| 0xA5 | `tau1_dis` | `float[20]` | r/w | s, selected cell |
| 0xA6 | `r2_dis` | `float[20]` | r/w | Ω, selected cell |
| 0xA7 | `tau2_dis` | `float[20]` | r/w | s, selected cell |
| 0xA8 | `r0_chg` | `float[20]` | r/w | Ω, selected cell |
| 0xA9 | `r1_chg` | `float[20]` | r/w | Ω, selected cell |
| 0xAA | `tau1_chg` | `float[20]` | r/w | s, selected cell |
| 0xAB | `r2_chg` | `float[20]` | r/w | Ω, selected cell |
| 0xAC | `tau2_chg` | `float[20]` | r/w | s, selected cell |
| 0xAD | `q_temp_c` | `float[5]` | r/w | °C, capacity vs temperature points |
| 0xAE | `q_temp_factor` | `float[5]` | r/w | capacity factor at those points |
| 0xAF | `q_nom_ah` | `float` | r/w | Ah |
| 0xB0 | `coulombic_efficiency` | `float` | r/w | |
| 0xB1 | `model_temp_dis_c` | `float` | r/w | °C, temperature of the discharge test |
| 0xB2 | `model_temp_chg_c` | `float` | r/w | °C, temperature of the charge test |
| 0xB6 | `ea_r0` | `float` | r/w | J/mol, Arrhenius activation energy |
| 0xB7 | `ea_r1` | `float` | r/w | J/mol |
| 0xB8 | `ea_tau1` | `float` | r/w | J/mol |
| 0xB9 | `ea_r2` | `float` | r/w | J/mol |
| 0xBA | `ea_tau2` | `float` | r/w | J/mol |
| 0xBB | `kf_q_soc` | `float` | r/w | Kalman process noise, > 0 |
| 0xBC | `kf_q_rc1` | `float` | r/w | > 0 |
| 0xBD | `kf_q_rc2` | `float` | r/w | > 0 |
| 0xBE | `kf_r_v` | `float` | r/w | Kalman measurement noise, > 0 |

**Filter state:**

| Addr | Name | Type | Access | Unit / notes |
|------|------|------|--------|--------------|
| 0xD0 | `cell_soc_f` | `float[7]` | r | 0–1, filter SOC |
| 0xD1 | `cell_vrc1` | `float[7]` | r | V, RC1 voltage |
| 0xD2 | `cell_vrc2` | `float[7]` | r | V, RC2 voltage |
| 0xD3 | `cell_p` | `float[7][6]` | r | covariance, 168 bytes |
| 0xD4 | `capacity_ah` | `float` | r/w | Ah, selected cell, learned over time |
| 0xD7 | `cycle_count` | `uint16` | r | |

### Protection bits — `protection_trigger` (`0x12`)

| Bit | Name | Bit | Name |
|-----|------|-----|------|
| 0 | OVP_SLOW | 6 | OCP_DISCHARGE_FAST |
| 1 | OVP_FAST | 7 | OTP |
| 2 | UVP_SLOW | 8 | UTP |
| 3 | UVP_FAST | 9 | OTP_CHARGE (only while charging) |
| 4 | OCP_CHARGE | 10 | UTP_CHARGE (only while charging) |
| 5 | OCP_DISCHARGE_SLOW | | |

Any trip turns the FET off and stops a running HPPC test. `CC_RESET_PROTECTION` clears the bits.

### Error bits — `error_flags` (`0x15`)

| Bit | Name | Bit | Name |
|-----|------|-----|------|
| 0 | EEPROM_READ_FAIL | 9 | HW_DRV_FAULT |
| 1 | EEPROM_WRITE_FAIL | 10 | HW_DRV_GATE_FAULT |
| 2 | EEPROM_CRC_FAIL | 11 | HW_PWR_FAIL |
| 3 | ADS_READ_FAIL | 12 | UART_ERROR |
| 4 | ADS_INIT_FAIL | 13 | UART_TX_TIMEOUT |
| 5 | ADS_ID_FAIL | 14 | NTC_FAULT |
| 6 | ADS_CRC_FAIL | 15 | I2C_ERROR |
| 7 | ADS_RESET | 16 | CAN_ERROR |
| 8 | ADS_DATA_LOST | 17 | WATCHDOG_RESET |

`CC_CLEAR_ERRORS` clears all bits.

### HPPC status — `hppc_status` (`0x19`) and `hppc_extract_status` (`0x1A`)

| State (low byte) | | Stop reason (high byte) | |
|-------|------|------|------|
| 0 | IDLE | 0 | NONE |
| 1 | DETECT | 1 | CELL_MAX |
| 2 | REST | 2 | CELL_MIN |
| 3 | RAMP | 3 | TERM_CURRENT |
| 4 | DONE | 4 | NO_CURRENT |
| 5 | ABORTED | 5 | NO_CONFIG |
| | | 6 | HOST |
| | | 7 | PROTECTION |
| | | 8 | CURRENT_RANGE |
| | | 9 | TEMPERATURE |

| Bit | `hppc_extract_status` |
|-----|------|
| 8 | BUSY — test running or fits still being worked on |
| 9 | DONE — tables written |
| 10 | OVERRUN |
| 11 | FIT_WARN |
| 12 | GAP_LO — OCV range not reached at the low end |
| 13 | GAP_HI — OCV range not reached at the high end |
| 14 | FEW_CYCLES |

## Notes for implementers

- **Reads return the whole register.** There is no partial read and no block offset.
- **Writes must carry the whole register.** Any other length returns `CE_WRONG_CMD`.
- **Config writes are applied at reboot.** Write the registers, send `CC_EEPROM_CONFIG`, then `CC_REBOOT`. `python_scripts/simulation/config_tool.py w` does this from `config.json`.
- **The fuel gauge model is per cell.** Select the cell with `0x9F` before reading or writing `0xA1`–`0xAC` or `0xD4`.
- **One transaction at a time.** There is no sequence or transaction ID, so the host must wait for a reply before sending the next request.
- **UART and I²C never send unsolicited data.** Only the CAN status broadcast is sent without a request. A host on UART or I²C polls `protection_trigger`, `error_flags` and the measurement registers.
