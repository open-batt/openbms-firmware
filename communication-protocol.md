# 📡 Communication Protocol

OpenBMS firmware talks to a host over three ports:

- **UART** — 230400 baud, 8N1. The OpenBMS binary protocol: register read/write and commands, with an 8-bit additive checksum. Also used for bootloader entry.
- **I²C** — I2C2 slave at address `0x0B`. Carries exactly the same frames as the UART.
- **CAN 2.0** — 1 Mbit/s, 11-bit IDs. A periodic status broadcast plus a small set of short commands. Register read/write is not available on CAN.

All three ports are served by `openbms_comm.c`.

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
| 0x05 | `CC_EEPROM_CONFIG` | — | STARTUP, NORMAL_RUN, FAULT | UART, I²C | Save the config registers (`0x30`–`0x6C`) to the EEPROM. Progress in `eeprom_status` (`0x1D`). `CE_STATE` while another save runs |
| 0x06 | `CC_HPPC_START` | 1 B: 1 charge, 2 discharge | NORMAL_RUN, HPPC_RUN after a test ended | UART, I²C | Start the HPPC test in the given direction. Needs a loaded config. In HPPC_RUN only once the previous test has ended and its model has been handed to the EEPROM. A missing or other argument returns `CE_WRONG_CMD` |
| 0x07 | `CC_NORMAL_RUN` | — | HPPC_RUN | UART, I²C, CAN | Stop a running HPPC test (stop reason HOST). The device stays in HPPC_RUN with the FET off until a reboot |
| 0x08 | `CC_SET_FET` | 1 byte, `0` or `1` | NORMAL_RUN with a loaded config | UART, I²C, CAN | Main FET off / on |
| 0x09 | `CC_REBOOT` | — | any state but SHUTDOWN | UART, I²C, CAN | ACK, then RESET: outputs off, fuel gauge state saved, reset after at least 100 ms |
| 0x0A | `CC_PING` | — | any state | UART, I²C | Just ACK |
| 0x0B | `CC_RESET_PROTECTION` | — | any state | UART, I²C, CAN | Clear `protection_trigger`. A fault that is still there trips again after its normal detection time. The FET and pre-FET stay off until the host switches them on again |
| 0x0C | `CC_CLEAR_ERRORS` | — | any state | UART, I²C, CAN | Clear `error_flags`. Resets the FET driver first if `HW_DRV_FAULT` is set. In FAULT the board goes back to STARTUP and runs the boot check again |
| 0x0D | `CC_SET_PRE_FET` | 1 byte, `0` or `1` | NORMAL_RUN with a loaded config | UART, I²C | Pre-charge FET off / on |
| 0x0F | `CC_CLEAR_LEARNING` | — | NORMAL_RUN, no save running | UART, I²C | Clear `learning_status` (`0x96`) and store the model. The fuel gauge stops and stays off after the next boot until new HPPC tests (both directions) are done |
| 0x10 | `CC_CAN_BROADCAST` | 1 byte, `0` or `1` | any state | CAN | Status broadcast off / on (RAM only) |
| 0x11 | `CC_CAN_SEND_ONCE` | — | any state | CAN | Send the status messages once |
| 0x12 | `CC_CLEAR_FAULT` | — | any state | UART, I²C, CAN | Clear the fault snapshot (`0x70`–`0x75`) |
| 0x13 | `CC_SOC_RESET` | — | NORMAL_RUN, model loaded, at rest | UART, I²C | Test build only (`OPENBMS_TEST`). Seeds the SOC again from the present cell voltages and saves the fuel gauge state |
| 0x14 | `CC_SHUTDOWN` | — | NORMAL_RUN, HPPC_RUN, FAULT | UART, I²C, CAN | Everything off, fuel gauge state saved, then the board switches its own supply off. See [Shutdown](#shutdown) |
| 0x15 | `CC_EEPROM_FUELGAUGE` | — | NORMAL_RUN | UART, I²C | Store the fuel gauge model (as the registers hold it) in the EEPROM. The gauge stops until the next boot, which loads the stored model. Progress in `0x1D` |
| 0x16 | `CC_EEPROM_ERASE` | — | STARTUP, NORMAL_RUN, FAULT | UART, I²C | Set the whole EEPROM to `0xFF`: config, model and state. Progress in `0x1D` |
| 0x17 | `CC_ADS_STANDBY` | 1 byte, `0` or `1` | NORMAL_RUN, FETs off | UART, I²C | Test build only. 1 puts the ADS131M08 in standby (dividers stay on), 0 wakes it |
| 0x18 | `CC_MEAS_OFF` | 1 byte, `0` or `1` | NORMAL_RUN, FETs off | UART, I²C | Test build only. 1 switches the cell dividers off (readings show the zero point), 0 on |
| 0x19 | `CC_EEPROM_FG_STATE` | 1 byte: 1 store, 2 erase | 1: NORMAL_RUN with a model loaded; 2: STARTUP, NORMAL_RUN, FAULT | UART, I²C | 1 stores the fuel gauge state written to `0x9C`, 2 erases the stored state. See [Fuel gauge state](#fuel-gauge-state--0x9b-and-0x9c) |
| 0x42 | `CC_BOOTLOADER` | — | any state but SHUTDOWN | UART | Reset into the bootloader, see below |

A command with the wrong argument length or value returns `CE_WRONG_CMD`. A command sent in a state where it is not allowed returns `CE_STATE`. An unknown command returns `CE_WRONG_CMD`; this includes the test-build commands (`0x13`, `0x17`, `0x18`) in a normal build. `0x0E` is unused.

The test build is enabled with `#define OPENBMS_TEST` in `firmware/Core/Inc/config.h`. Besides the commands above it adds two read-only registers, `0x08` and `0x1E`.

### Error codes

When a request fails, the device replies with an error frame `04 00 01 <code> <checksum>`.

| Code | Name | Meaning |
|------|------|---------|
| 0x00 | `CE_OK` | No error (never sent) |
| 0x01 | `CE_WRONG_CMD` | Unknown command, wrong length, or value out of range (NaN/inf float, bad bool, bad cell count, ...) |
| 0x02 | `CE_BAD_CRC` | Checksum mismatch |
| 0x03 | `CE_NO_REG` | Address not in the register map |
| 0x04 | `CE_RO` | Write to a read-only register |
| 0x05 | `CE_EEPROM` | Not used any more. EEPROM saves run in the background, their result is in `0x1D` |
| 0x06 | `CE_STATE` | Not allowed in the current `ctrl_state`, or an EEPROM save is still running |

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
02  13  01  01  17              ← response: 1 = NORMAL_RUN
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
- The on/off settings (`0x31`–`0x36`, `0x66`, `0x6B`) must be `0` or `1`.
- `model_cell` (`0x9F`) must be 0–6, `fg_state_chunk` (`0x9B`) 0–1.
- A NaN or inf float in the config calibration block or in the fuel gauge model is refused, the old value stays.
- `kf_r_v` (`0xBE`) must be above 0.
- Fuel gauge registers cannot be written while an EEPROM save runs (`CE_STATE`).

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

**Request — start the HPPC test, discharge:**
```
06  00  01  02  09              ← command, address 0, length 1, direction 2 = discharge
03  00  00  03                  ← ACK
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
| 0 | STARTUP | Boot check: the ADS131M08 must answer and give a first reading within 1 s, and with a config every cell below `cell_count` must read at least 1000 mV. Passes to NORMAL_RUN, otherwise FAULT |
| 1 | NORMAL_RUN | Normal operation. With a loaded config the protections, battery status, balancing and host FET control work; with a loaded model the fuel gauge runs |
| 2 | HPPC_RUN | HPPC test running or ended. The FET is controlled by the test. Kept after the test ends; only a reboot goes back to NORMAL_RUN |
| 3 | FAULT | Boot check failed, or a fault error set while running (ADS read, init, ID, reset, data lost, FET driver fault, NTC fault, cell missing). Outputs off. `CC_CLEAR_ERRORS` runs the boot check again |
| 4 | SHUTDOWN | `CC_SHUTDOWN` or the low-voltage shutdown: everything off, state saved, then the supply switches off |
| 5 | RESET | `CC_REBOOT` or `CC_BOOTLOADER`: everything off, state saved, then reset |

A board without a valid config in the EEPROM still boots to NORMAL_RUN, but the protections, FET control, balancing and HPPC stay off until a config is saved and the board is rebooted. `eeprom_status` (`0x1D`) bit 0 shows whether a config was loaded.

Config registers can be written in any state. The protection thresholds and balancing settings are used from RAM at once; the calibration and the "config loaded" state come from the EEPROM at boot. So the normal sequence is: write the registers, `CC_EEPROM_CONFIG`, wait for `0x1D` to show the save done, then `CC_REBOOT`.

### Shutdown

`CC_SHUTDOWN` switches the FETs, balancers and cell measurement off, finishes any EEPROM save, saves the fuel gauge state and then pulls PWR_ON low. Power stays on while the WAKE_UP input is high; in that case the board waits in SHUTDOWN. Only WAKE_UP (or the button) starts the board again — UART, I²C, CAN and a charger do not. `CC_REBOOT` and `CC_BOOTLOADER` are refused in SHUTDOWN.

The board also shuts itself down when a UVP has tripped, no charger is present and the lowest cell has been at or below `voltage_cell_min` for 5 minutes, so an empty pack is not drained further by the board.

### Bootloader entry

`CC_BOOTLOADER` (`0x42`, ASCII `B`) is handled **before** checksum validation, so on the UART a single unframed `0x42` byte (or `B\n`) is enough. The firmware replies with the ASCII string `"1\n"`, saves the state, erases the application flag page and resets into the bootloader.

The bootloader itself speaks a separate line-based ASCII protocol on the same UART at the same baud rate:

| Command | Meaning | Response |
|---------|---------|----------|
| `B\n` | Ping | `0` = bootloader active · `1` = application running, it will reset into the bootloader · `2` = bootloader start failed |
| `E\n` | Erase application flash | `0` = OK |
| `P,<intel hex line>\n` | Program one Intel HEX record | `0` = OK · `1` = EOF record |
| `C,<size>,<crc32>\n` | Verify image — CRC-32/MPEG-2, matching the STM32 HAL CRC peripheral | `0` = match |
| `A\n` | Set application flag | `0` = OK |
| `R\n` | Reset into the application | — |

The application starts at `0x0800A000`; `main()` sets `SCB->VTOR` accordingly. Flashing keeps the config, model and state saved in the EEPROM.

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

CAN runs at 1 Mbit/s with 11-bit standard IDs. All values are little-endian. CAN is restarted at most every 1000 ms when it stops (bus-off, errors); this sets `CAN_ERROR`. The IDs and the period are set in `config.h` (`COMM_CAN_*`).

### Status broadcast

While the broadcast is on (default on after every boot), the device sends these messages every 100 ms in every state. `CC_CAN_BROADCAST` turns it off or on until the next reboot; `CC_CAN_SEND_ONCE` sends them once.

| ID | Bytes | Content |
|----|-------|---------|
| `0x6B0` | 8 | `uint32` pack voltage mV, `int32` pack current mA (+ = charge) |
| `0x6B1` | 8 | `uint16` × 4 — cell 1–4 voltage mV |
| `0x6B2` | 6 | `uint16` × 3 — cell 5–7 voltage mV |
| `0x6B3` | 6 | `uint16` SOC in 0.1 % (lowest shown cell SOC), `int16` temperature in 0.1 °C, `uint8` `battery_status`, `uint8` `charger_present` |
| `0x6B4` | 8 | `uint8` ctrl_state, `uint16` fet_status, `uint16` protection_trigger, `error_flags` low 24 bits (3 bytes) |

Status messages still waiting in the transmit queue at the next broadcast are dropped, so the bus always carries fresh values.

### Commands

| ID | Direction | Content |
|----|-----------|---------|
| `0x6BF` | host → device | byte 0 = command code (`CC_*`), then its argument |
| `0x6BE` | device → host | byte 0 = command code, byte 1 = result (`CE_*`, `0` = OK) |

Allowed on CAN: `CC_CAN_BROADCAST`, `CC_CAN_SEND_ONCE`, `CC_REBOOT`, `CC_SET_FET`, `CC_RESET_PROTECTION`, `CC_CLEAR_ERRORS`, `CC_CLEAR_FAULT`, `CC_NORMAL_RUN`, `CC_SHUTDOWN`. Every other command returns `CE_WRONG_CMD`. Send one command and wait for its reply before the next.

**Example — turn the main FET on:**
```
ID 0x6BF  data 08 01           ← CC_SET_FET, argument 1
ID 0x6BE  data 08 00           ← reply: CC_SET_FET, CE_OK
```

## Register map

Every address not listed below returns `CE_NO_REG`. Access: **r** = read-only (a write returns `CE_RO`), **r/w** = read and write.

| Bank | Range | Structure |
|------|-------|-----------|
| Peripheral data | `0x00` – `0x08` | `Peripheral_Data_t` |
| System data | `0x10` – `0x1E` | `System_Data_t` |
| Config | `0x30` – `0x6C` | `Config_Data_t` |
| Fault snapshot | `0x70` – `0x75` | `Stats_Data_t` |
| Fuel gauge | `0x90` – `0xD7` | `FuelGauge_Data_t` |

### Peripheral data — `0x00` – `0x08`

| Addr | Name | Type | Access | Unit / notes |
|------|------|------|--------|--------------|
| 0x00 | `cell_voltage` | `float[7]` | r | mV, smoothed. A bleeding cell shows the last reading taken with its balancer off |
| 0x01 | `pack_voltage` | `float` | r | mV, sum of `cell_count` cells |
| 0x02 | `pack_current` | `float` | r | mA, + = charge |
| 0x03 | `temperature_package` | `float` | r | °C, NTC |
| 0x04 | `temperature_stm32` | `float` | r | °C |
| 0x05 | `main_vdd_voltage_mv` | `float` | r | mV |
| 0x06 | `fet_status` | `uint16` | r | bit 0 main FET, bit 1 pre-FET, bits 2–8 balancing cell 1–7 |
| 0x07 | `sys_vcc_mv` | `float` | r | mV, output terminal voltage SYS_VCC, from CHG_PRES (PA3, divider ÷11) |
| 0x08 | boot voltage offset | `float[7]` | r | Test build only. mV per cell measured at boot |

### System data — `0x10` – `0x1E`

| Addr | Name | Type | Access | Unit / notes |
|------|------|------|--------|--------------|
| 0x10 | `main_control` | `uint16` | r | bit 0: 0 = normal, 1 = HPPC |
| 0x11 | `last_communication_timestamp` | `uint32` | r | ms, time of the last good frame |
| 0x12 | `protection_trigger` | `uint16` | r | protection bits, see below. `0` = none |
| 0x13 | `ctrl_state` | `uint8` | r | see [Device states](#device-states) |
| 0x15 | `error_flags` | `uint64` | r | error bits, see below |
| 0x17 | `session_uptime_ms` | `uint32` | r | ms since boot |
| 0x18 | `hppc_direction` | `uint8` | r | 0 unknown, 1 charge, 2 discharge |
| 0x19 | `hppc_status` | `uint16` | r | low byte state, high byte stop reason, see below |
| 0x1A | `hppc_extract_status` | `uint16` | r | extraction bits, see below |
| 0x1B | `battery_status` | `uint8` | r | bit 0 FULL, bit 1 EMPTY, see below |
| 0x1C | `charger_present` | `uint8` | r | 1 when `sys_vcc_mv` (`0x07`) is above 7 V (`CTRL_CHARGER_PRESENT_MV`), else 0. Only means a charger while the FETs are off: with the FETs on the output is at pack voltage |
| 0x1D | `eeprom_status` | `uint8` | r | what was loaded at boot and the last save, see below |
| 0x1E | loop time | `uint32[2]` | r | Test build only. µs of the last and of the longest main loop pass |

### Config — `0x30` – `0x6C`

All config registers are r/w unless marked read-only. `CC_EEPROM_CONFIG` saves the whole bank to the EEPROM (CRC-32 protected), and the saved config is loaded at the next boot. There are no firmware defaults.

| Addr | Name | Type | Unit / notes |
|------|------|------|--------------|
| 0x30 | `cell_count` | `uint8` | 2–7 |
| 0x31 | `use_i2c` | `bool` | stored only, not used by the firmware yet |
| 0x32 | `use_can` | `bool` | stored only, not used by the firmware yet |
| 0x33 | `use_uart` | `bool` | stored only, not used by the firmware yet |
| 0x34 | `volt_protection` | `bool` | OVP / UVP on |
| 0x35 | `current_protection` | `bool` | OCP on |
| 0x36 | `temp_protection` | `bool` | OTP / UTP on |
| 0x37 | `cell_capacity` | `uint32` | mAh, rated capacity (HPPC current range, SOH reference) |
| 0x38 | `voltage_cell_max` | `uint16` | mV, HPPC charge stops here |
| 0x39 | `voltage_cell_min` | `uint16` | mV, HPPC discharge stops here, EMPTY flag |
| 0x3A | `charging_term_current` | `uint16` | mA, FULL flag and full-charge anchor |
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
| 0x4F | `otp_charge_threshold_c` | `int16` | °C, only while charging |
| 0x50 | `ocv_v_lo_mv` | `uint16` | mV, cell voltage taken as 0 % by the HPPC extraction |
| 0x51 | `ocv_v_hi_mv` | `uint16` | mV, cell voltage taken as 100 % |
| 0x52 | `current_sensor_gain` | `float` | |
| 0x53 | `voltage_gain` | `float[7]` | |
| 0x54 | `ntc_beta` | `float` | |
| 0x55 | `ntc_r_nominal` | `float` | Ω |
| 0x56 | `ntc_r_fixed` | `float` | Ω |
| 0x57 | `ntc_t_nominal` | `float` | K |
| 0x58 | `temperature_offset` | `float` | °C |
| 0x59 | `shunt_resistance_mohm` | `float` | mΩ |
| 0x5A | `current_offset_ma` | `float` | mA, added to every reading |
| 0x5B | `voltage_offset_mv` | `float[7]` | mV, added to every reading |
| 0x5C | `utp_charge_threshold_c` | `int16` | °C, only while charging |
| 0x60 | `firmware_version` | `char[32]` | read-only, `SYS_FIRMWARE_VERSION` in `config.h` ("1.0.0"), null-padded |
| 0x61 | `hardware_version` | `char[32]` | read-only, `SYS_HARDWARE_VERSION` ("rev B") |
| 0x62 | `manufacturer_name` | `char[32]` | read-only, `SYS_MANUFACTURER_NAME` ("OpenBatt Team") |
| 0x63 | `device_name` | `char[32]` | read-only, `SYS_DEVICE_NAME` ("OpenBMS") |
| 0x64 | `device_chemistry` | `char[32]` | written by the host, saved with the config, null-padded |
| 0x65 | `manufacturer_data` | `char[32]` | read-only, `SYS_MANUFACTURER_DATA` ("Year 2026") |
| 0x66 | `balance_enable` | `bool` | |
| 0x67 | `balance_start_mv` | `uint16` | mV, voltage balancing: the highest cell must be at least this |
| 0x68 | `balance_delta_mv` | `uint16` | mV, voltage balancing: a cell more than this above the lowest starts to bleed |
| 0x69 | `balance_max_temp_c` | `int16` | °C, no balancing at or above this |
| 0x6A | `balance_resistance_ohm` | `float` | Ω, bleed resistor (the fuel gauge subtracts the bleed current) |
| 0x6B | `balance_rest_enable` | `bool` | also balance at rest (\|I\| ≤ 50 mA), not only while charging |
| 0x6C | `balance_stop_mv` | `uint16` | mV, voltage balancing: a bleeding cell stops this close to the lowest (below `balance_delta_mv`) |

`0x4B` and `0x5D`–`0x5F` are unused.

### Fault snapshot — `0x70` – `0x75`

Taken once at the first protection trip and kept until `CC_CLEAR_FAULT`. RAM only, lost at reboot.

| Addr | Name | Type | Access | Unit / notes |
|------|------|------|--------|--------------|
| 0x70 | `voltage` | `uint16[7]` | r | mV per cell |
| 0x71 | `current` | `int16` | r | mA |
| 0x72 | `temperature` | `int8` | r | °C |
| 0x73 | `soc` | `uint8` | r | %, `relative_soc` |
| 0x74 | `fault_code` | `uint16` | r | `protection_trigger` at the trip |
| 0x75 | `fault_timestamp` | `uint32` | r | `session_uptime_ms` at the trip |

### Fuel gauge — `0x90` – `0xD7`

The per-cell model registers (`0xA1`–`0xAC` and `0xD4`) act on the cell picked with `model_cell` (`0x9F`). To read or write the tables of cell 3, write `0x9F` = 2 first. The model is loaded from the EEPROM at boot; writes change it in RAM only, `CC_EEPROM_FUELGAUGE` stores it.

**Outputs:**

| Addr | Name | Type | Access | Unit / notes |
|------|------|------|--------|--------------|
| 0x90 | `relative_soc` | `uint8` | r | %, lowest shown cell SOC |
| 0x92 | `cell_soc` | `uint8[7]` | r | %, shown SOC per cell |
| 0x93 | `cell_soh` | `uint8[7]` | r | %, learned capacity / `cell_capacity` |
| 0x94 | `cell_remaining_capacity` | `uint16[7]` | r | mAh, scaled by the capacity-temperature factor |
| 0x96 | `learning_status` | `uint16` | r/w | bit 0 CHARGE, bit 1 DISCHARGE (tables of that direction written), bit 2 COMPLETE (both) |
| 0x97 | `capacity_update_count` | `uint16` | r | capacity learning updates |
| 0x98 | `fg_status` | `uint8` | r | fuel gauge status bits, see below |
| 0x9A | `balance_target_mah` | `uint16[7]` | r | mAh each cell still has to bleed (SOC balancing) |
| 0x9B | `fg_state_chunk` | `uint8` | r/w | 0–1, which part of the state `0x9C` shows |
| 0x9C | fuel gauge state | `uint8[128]` / `uint8[4]` | r/w | `FuelGauge_State_t`, see below |
| 0x9F | `model_cell` | `uint8` | r/w | 0–6, cell for `0xA1`–`0xAC` and `0xD4` |

**Model (written by the HPPC extraction, or by the host):**

| Addr | Name | Type | Access | Unit / notes |
|------|------|------|--------|--------------|
| 0xA0 | `soc_grid` | `float[20]` | r/w | %, rising, shared by all tables |
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
| 0xAD | `q_temp_c` | `float[5]` | r/w | °C, capacity vs temperature points, rising |
| 0xAE | `q_temp_factor` | `float[5]` | r/w | capacity factor at those points |
| 0xAF | `q_nom_ah` | `float` | r/w | Ah, pack capacity (weakest cell) |
| 0xB0 | `coulombic_efficiency` | `float` | r/w | |
| 0xB1 | `model_temp_dis_c` | `float` | r/w | °C, temperature of the discharge test |
| 0xB2 | `model_temp_chg_c` | `float` | r/w | °C, temperature of the charge test |
| 0xB6 | `ea_r0` | `float` | r/w | J/mol, Arrhenius activation energy, `0` = no scaling |
| 0xB7 | `ea_r1` | `float` | r/w | J/mol |
| 0xB8 | `ea_tau1` | `float` | r/w | J/mol |
| 0xB9 | `ea_r2` | `float` | r/w | J/mol |
| 0xBA | `ea_tau2` | `float` | r/w | J/mol |
| 0xBE | `kf_r_v` | `float` | r/w | V², Kalman voltage measurement noise, > 0 |

**Filter state:**

| Addr | Name | Type | Access | Unit / notes |
|------|------|------|--------|--------------|
| 0xD0 | `cell_soc_f` | `float[7]` | r | 0–1, filter SOC |
| 0xD1 | `cell_vrc1` | `float[7]` | r | V, RC1 voltage |
| 0xD2 | `cell_vrc2` | `float[7]` | r | V, RC2 voltage |
| 0xD3 | `cell_p` | `float[7][6]` | r | covariance upper triangle, 168 bytes |
| 0xD4 | `capacity_ah` | `float` | r/w | Ah, selected cell, learned over time |
| 0xD7 | `cycle_count` | `uint16` | r | full cycles: discharged charge / `q_nom_ah` |

`0xBB`–`0xBD` (the old `kf_q_*` process noise) are gone: the filter now works out its process noise from the current sensor accuracy and the model error, see [battery-model.md](battery-model.md#filter-tuning).

### Fuel gauge state — `0x9B` and `0x9C`

The learned values (capacity per cell, SOC, shown SOC, SOH, cycle count, remaining balancing targets, last current direction) are kept in the EEPROM separately from the model and saved whenever they change by a step (for example 1 % SOC), and at shutdown or reboot. They are loaded at boot only if they belong to the stored model (same model CRC).

The host can read, back up and restore them: `FuelGauge_State_t` (132 bytes) is longer than one frame, so it is split into chunks of 128 bytes. Write `0x9B` = 0 and read `0x9C` (this takes a fresh copy), then `0x9B` = 1 and read `0x9C` again for the last 4 bytes. To restore, write both chunks the same way and send `CC_EEPROM_FG_STATE` with argument 1; `CC_EEPROM_FG_STATE` 2 erases the stored state so the next boot seeds the SOC from the cell voltages.

| Offset | Type | Field |
|--------|------|-------|
| 0 | `float[7]` | `capacity_ah`, Ah per cell |
| 28 | `float` | `q_nom_ah`, Ah |
| 32 | `float[7]` | `cell_soc_f`, 0–1 |
| 60 | `float[7]` | `cell_soc_shown_f`, 0–1 |
| 88 | `uint16` | `cycle_count` |
| 90 | `uint16` | `capacity_update_count` |
| 92 | `uint8[7]` | `cell_soh`, % |
| 99 | `float[7]` | `bal_target_ah`, Ah still to bleed |
| 127 | `uint8` | `last_charge`: 0 discharge, 1 charge, 2 not known |
| 128 | `float` | `cycle_ah`, discharged Ah toward the next full cycle |

### Protection bits — `protection_trigger` (`0x12`)

| Bit | Name | Bit | Name |
|-----|------|-----|------|
| 0 | OVP_SLOW | 6 | OCP_DISCHARGE_FAST |
| 1 | OVP_FAST | 7 | OTP |
| 2 | UVP_SLOW | 8 | UTP |
| 3 | UVP_FAST | 9 | OTP_CHARGE (only while charging, > 50 mA) |
| 4 | OCP_CHARGE | 10 | UTP_CHARGE (only while charging, > 50 mA) |
| 5 | OCP_DISCHARGE_SLOW | | |

The protections run only with a loaded config. Voltage protections use the live cell readings (not the held ones of a bleeding cell). OTP and UTP are skipped while `NTC_FAULT` is set.

Any trip turns the FET off, drops the host's FET and pre-FET requests (`CC_SET_FET` / `CC_SET_PRE_FET`; a request sent while tripped is dropped too), stops balancing and stops a running HPPC test. `CC_RESET_PROTECTION` clears the bits; the FETs stay off until the host switches them on again.

### Error bits — `error_flags` (`0x15`)

| Bit | Name | Fault | Bit | Name | Fault |
|-----|------|-------|-----|------|-------|
| 0 | EEPROM_READ_FAIL | | 9 | HW_DRV_FAULT | ✔ |
| 1 | EEPROM_WRITE_FAIL | | 10 | UART_ERROR | |
| 2 | EEPROM_CRC_FAIL | | 11 | UART_TX_TIMEOUT | |
| 3 | ADS_READ_FAIL | ✔ | 12 | NTC_FAULT | ✔ |
| 4 | ADS_INIT_FAIL | ✔ | 13 | I2C_ERROR | |
| 5 | ADS_ID_FAIL | ✔ | 14 | CAN_ERROR | |
| 6 | ADS_CRC_FAIL | | 15 | WATCHDOG_RESET | |
| 7 | ADS_RESET | ✔ | 16 | CELL_MISSING | ✔ |
| 8 | ADS_DATA_LOST | ✔ | | | |

Errors marked **Fault** put the board in FAULT (outputs off). The others are only reported. `CC_CLEAR_ERRORS` clears all bits; in FAULT it also starts the boot check again. `EEPROM_CRC_FAIL` at boot usually means nothing was stored yet, or the layout changed after a firmware update.

### Battery status — `battery_status` (`0x1B`)

| Bit | Flag | Set when | Cleared when |
|-----|------|----------|--------------|
| 0 | FULL | highest cell ≥ `charge_monitor_cell_mv` and charge current between 50 mA and `charging_term_current`, for 10 s | discharge current above 50 mA for 2 s |
| 1 | EMPTY | lowest cell ≤ `voltage_cell_min` for 2 s | charge current above 50 mA for 2 s |

The flags only report the pack's normal end of charge and discharge; they never switch the FET (that is the protections' job). At rest the cells relax (an empty cell climbs back, a full one drops a little), so the voltage never clears a flag. Both are clear after a boot and work with a loaded config. On CAN they are the fifth byte of `0x6B3`.

### EEPROM status — `eeprom_status` (`0x1D`)

| Bits | Meaning |
|------|---------|
| 0 | CONFIG_LOADED — a valid config was loaded at boot |
| 1 | MODEL_LOADED — a fuel gauge model was loaded at boot and is in use |
| 2 | STATE_LOADED — the saved fuel gauge state belonged to that model and was applied |
| 4–5 | last save: 0 none yet, 1 running, 2 done, 3 failed (EEPROM error flag set) |
| 6–7 | what that save writes: 1 config, 2 model (or state from the host), 3 erase |

Saves run in the background, one 64-byte page per main loop pass, so the protections keep running. Wait for bits 4–5 to show done before rebooting.

### Fuel gauge status — `fg_status` (`0x98`)

| Bit | Name | Meaning |
|-----|------|---------|
| 0 | AT_REST | \|I\| below 50 mA and no cell balancing |
| 1 | RELAXED | this rest: every cell settled (dV/dt below 4 µV/s between two 1 min averages) |
| 2 | REST_CORRECTED | this rest: the OCV correction was done |
| 3 | CAP_POINT | this rest: a capacity learning point was stored |
| 4 | CAP_UPDATED | this rest: a cell's capacity was updated |
| 5 | FULL_PENDING | charging and the full-charge condition holds, the 10 s hold time runs |
| 6 | FULL_SET | the gauge set 100 % in this charge, cleared by a discharge |

### HPPC status — `hppc_status` (`0x19`) and `hppc_extract_status` (`0x1A`)

| State (low byte) | | Stop reason (high byte) | |
|-------|------|------|------|
| 0 | IDLE | 0 | NONE |
| 1 | DETECT — FET on for 2 s to check the current direction | 1 | CELL_MAX — charge test ended at `voltage_cell_max` |
| 2 | REST | 2 | CELL_MIN — discharge test ended at `voltage_cell_min` |
| 3 | RAMP | 3 | TERM_CURRENT — charge current fell to `charging_term_current` |
| 4 | DONE | 4 | NO_CURRENT — no charger or load seen |
| 5 | ABORTED | 5 | NO_CONFIG — `cell_capacity` or `cell_count` not set, or `ocv_v_hi_mv` not above `ocv_v_lo_mv` |
| 6 | SETTLE — FET off, waiting for the cells to relax before the test (2–30 min) | 6 | HOST — `CC_NORMAL_RUN` |
| | | 7 | PROTECTION |
| | | 8 | CURRENT_RANGE — current outside 0.5–2 C for 2 s |
| | | 9 | TEMPERATURE — NTC fault, or outside 15–35 °C (at the start, or for 5 min while running) |
| | | 10 | WRONG_DIRECTION — measured direction is not the one in `CC_HPPC_START` |
| | | 11 | NOT_FULL — discharge test, a cell below 4100 mV at the start (`HPPC_DIS_START_MIN_MV`) |
| | | 12 | NOT_EMPTY — charge test, a cell above 3450 mV at the start (`HPPC_CHG_START_MAX_MV`) |
| | | 13 | ERRORS — a fault error set at the start or while running |
| | | 14 | SHUTDOWN — `CC_SHUTDOWN` while the test was running |

| Bit | `hppc_extract_status` |
|-----|------|
| 8 | BUSY — test running, fits still being worked on, or the model being written to the EEPROM |
| 9 | DONE — tables written |
| 10 | OVERRUN — a fit was not ready when the next rest started, that cycle was left out |
| 11 | FIT_WARN — a rest fit failed or is noisy, its cycle was interpolated from the neighbours |
| 12 | GAP_LO — lowest measured OCV more than 0.15 V above `ocv_v_lo_mv`, low end extrapolated |
| 13 | GAP_HI — highest measured OCV more than 0.15 V below `ocv_v_hi_mv`, high end extrapolated |
| 14 | FEW_CYCLES — fewer than 4 cycles finished, nothing written |
| 15 | SAVED — the model with the new tables is stored in the EEPROM; it is loaded and used from the next boot |

## Notes for implementers

- **Reads return the whole register.** There is no partial read and no block offset.
- **Writes must carry the whole register.** Any other length returns `CE_WRONG_CMD`.
- **Config and model are loaded at boot.** Write the registers, save (`CC_EEPROM_CONFIG` or `CC_EEPROM_FUELGAUGE`), wait for `0x1D` to show the save done, then `CC_REBOOT`.
- **The fuel gauge model is per cell.** Select the cell with `0x9F` before reading or writing `0xA1`–`0xAC` or `0xD4`.
- **One transaction at a time.** There is no sequence or transaction ID, so the host must wait for a reply before sending the next request.
- **UART and I²C never send unsolicited data.** Only the CAN status broadcast is sent without a request. A host on UART or I²C polls `protection_trigger`, `error_flags` and the measurement registers.
