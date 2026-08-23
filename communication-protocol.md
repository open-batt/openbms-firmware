# 📡 Communication Protocol

OpenBMS firmware implements the following communication protocols:

- **OpenBMS binary protocol over UART** — 230400 baud, 8N1, register-based read/write with an 8-bit additive checksum. This is the only protocol currently implemented and is used for host communication, configuration, calibration, data logging and bootloader entry.
- **SBS v1.1 over I²C/SMBus** — *planned.* The SMBus peripheral is initialized (slave address `0x0B`, PEC enabled) but no protocol handler exists yet.
- **CAN 2.0** — *planned.* The bxCAN peripheral is initialized (1 Mbit/s, 11-bit identifier) but no protocol handler exists yet.

A reference host implementation of the UART protocol lives in [`flashing_script/read_data.py`](flashing_script/read_data.py) (full register dump) and [`flashing_script/monitor.py`](flashing_script/monitor.py) (live logging).

## OpenBMS Binary Protocol

The protocol is a simple request/response scheme on a point-to-point UART link. There is no device address — one host talks to one OpenBMS board. Every transaction is initiated by the host; the device never speaks unsolicited.

| Layer | Name | Responsibility |
|-------|------|----------------|
| Application | OpenBMS register map | Register addresses, data types, access rights, control commands |
| Transport | OpenBMS frame | Command byte, length byte, additive checksum |
| Physical | UART | TX/RX lines, 230400 baud 8N1, idle-line frame delimiting |

### Frame structure

All frames — in both directions — share the same layout and are at least 4 bytes long:

| Command | Address | Length | Data | Checksum |
|---------|---------|--------|------|----------|
| 1 byte | 1 byte | 1 byte | N bytes | 1 byte |

- **Command** — frame type, see the table below.
- **Address** — register address, `0x00`–`0xFF`. `0x00` in ACK and error frames.
- **Length** — payload size in **bytes** (not elements, not 16-bit words). `0x00` for read requests, ACK frames and command frames with no payload.
- **Data** — `Length` bytes of payload, **little-endian**, laid out exactly as the corresponding C struct field.
- **Checksum** — sum of all preceding bytes of the frame, modulo 256.

Total frame size is always `Length + 4` bytes.

Register data is a raw memory image of the firmware's struct field. The host must know each register's type and element count in advance — the protocol carries no type information. See the register map below.

### Command codes

| Code | Name | Direction | Description |
|------|------|-----------|-------------|
| 0x01 | `CC_WRITE` | host → device | Write a register. Device answers with ACK or error. |
| 0x02 | `CC_READ` | both | Host request (`Length` = 0) and device response (`Length` = register size). |
| 0x03 | `CC_ACK` | device → host | Write or control command accepted. `Address` and `Length` are both `0x00`. |
| 0x04 | `CC_ERROR` | device → host | Request rejected. Payload is a single error code byte. |
| 0x05 | `CC_CMD` | host → device | Execute a control action rather than write a register. See [Control commands](#control-commands). |
| 0x42 | `CC_BOOTLOADER` | host → device | Erase the application flag and reboot into the bootloader. `0x42` is ASCII `B`. |

### Error codes

Errors are returned as `[0x04][0x00][0x01][error code][checksum]`.

| Code | Name | Meaning |
|------|------|---------|
| 0x00 | `CE_OK` | No error — never transmitted, internal success value only |
| 0x01 | `CE_WRONG_CMD` | Unknown command byte, or a `CC_CMD` frame with a payload length other than 1, or a control command value out of range |
| 0x02 | `CE_BAD_CRC` | Checksum mismatch |
| 0x03 | `CE_NO_REG` | Register address is not mapped |
| 0x04 | `CE_RO` | Write attempted on a read-only register, or FET control requested outside learning mode |

### Read transaction

The host sends a read request with `Length` set to `0x00`. The device replies with the register's **full native size** — a requested length is ignored, and there is no partial or block-offset read.

**Request — read `CellVoltage()` at `0x00`:**
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

**Request and response — `FETStatus()` at `0x09`:**
```
02  09  00  0B                  ← request
02  09  02  03 00  10           ← response: uint16 LE 0x0003 (main FETs on, pre-FET on)
```

**Response — `FirmwareVersion()` at `0x52`:**
```
02 52 20 31 2E 30 2E 30 00 00 ... 00 61
│  │  │  └─ char[32] — "1.0.0", null-padded
│  │  └─ length — 32 bytes
│  └─ register address echo — 0x52
└─ command echo — CC_READ
```

While a read is being served, the ADS131M08 `DRDY` interrupt (`EXTI15_10`) and the STM32 internal ADC interrupt (`ADC1`) are masked so the measurement buffers cannot be updated mid-copy. They are re-enabled immediately after.

### Write transaction

The host sends the register's **full native size** as payload. The device answers with a bare ACK — it does not echo the written value.

**Request — write `TemperatureOffset()` at `0x12` = 1.5 °C:**
```
01  12  04  00 00 C0 3F  16
│   │   │   └────────┘   └───── checksum
│   │   │   └────────────────── data — float32 LE 1.5
│   │   └────────────────────── length — 4 bytes
│   └────────────────────────── register address — 0x12
└────────────────────────────── command — CC_WRITE
```

**Request — write `OVP_SlowThreshold()` at `0x3A` = 4220 mV:**
```
01  3A  02  7C 10  C9
            └───┘   └───── checksum
            └───────────── data — uint16 LE 4220
```

**Response — ACK (identical for every successful write and control command):**
```
03  00  00  03
│   │   │   └───── checksum
│   │   └───────── length — always 0x00
│   └───────────── address — always 0x00
└───────────────── command — CC_ACK
```

### Control commands

`CC_CMD` frames trigger an action instead of writing a register. The payload length must be exactly **1 byte**; any other length returns `CE_WRONG_CMD`. Control command addresses are a separate address space from the register map.

| Address | Command | Payload | Notes |
|---------|---------|---------|-------|
| 0x00 | SetMode | 1 byte — `0`, `1` or `2` | `0` = normal, `1` = config, `2` = learning. Values ≥ `0x03` return `CE_WRONG_CMD`. Writes the mode field of `MainControl()`. |
| 0x01 | SetMainFET | 1 byte — `0` or `1` | `0` = FETs off, `1` = FETs on. Accepted **only in learning mode**; in normal or config mode the device returns `CE_RO`. Any value other than `0`/`1` returns `CE_WRONG_CMD`. |

**Request — enter learning mode:**
```
05  00  01  02  08
│   │   │   │   └───── checksum
│   │   │   └───────── payload — mode 2 (learning)
│   │   └───────────── length — 1 byte
│   └───────────────── control command address — 0x00 (SetMode)
└───────────────────── command — CC_CMD
```

**Request — turn main FETs on (learning mode only):**
```
05  01  01  01  08
```

Both are answered with the standard ACK frame, or `CE_RO` if the mode gate rejects it:
```
04  00  01  04  09
```

### Bootloader entry

`CC_BOOTLOADER` (`0x42`, ASCII `B`) is handled **before** checksum validation, so a single unframed `0x42` byte is enough. The firmware erases the application flag page, replies with the ASCII string `"1\n"`, waits 100 ms and issues a system reset into the bootloader.

The bootloader itself speaks a separate line-based ASCII protocol on the same UART at the same baud rate. `flashing_script/flash.py` drives the full sequence:

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
| Frame delimiting | Idle line — `HAL_UARTEx_ReceiveToIdle_DMA()`, one idle character time (~43 µs at 230400) |
| RX / TX | DMA1 Channel 5 (RX) and Channel 4 (TX), 2048-byte buffers each |
| Max payload | 168 bytes — the largest register is `Cell_Covariance()` at `0xA8` |

A frame must be transmitted as one continuous burst. Any idle gap inside a frame closes it early and the partial frame will be rejected on the checksum.

## Register map

Registers are grouped into three banks, each backed by one firmware data structure. The banks are not contiguous — every address outside the ranges listed below returns `CE_NO_REG`.

| Bank | Range | Structure | Source module |
|------|-------|-----------|---------------|
| Peripheral data | `0x00` – `0x12` | `Peripheral_Data_t` | `openbms_periph.c` |
| Control data | `0x30` – `0x57` | `Control_Data_t` | `openbms_ctrl.c` |
| Fuel gauge data | `0x80` – `0xAC` | `FuelGauge_Data_t` | fuel gauge (not yet wired in) |
| — | `0x13` – `0x2F`, `0x58` – `0x7F`, `0xAD` – `0xFF` | unmapped | returns `CE_NO_REG` |

`Type` is the C type of the underlying struct field; `Size` is the exact payload length in bytes for both reads and writes.

### Peripheral data — `0x00` – `0x12`

Live measurements and sensor calibration, refreshed from the peripheral module on every access.

| Address | Register | Access | Type | Size | Notes |
|---------|----------|--------|------|------|-------|
| 0x00 | CellVoltage() | r | float[7] | 28 | Per-cell voltage in mV, cell 1 first |
| 0x01 | CellVoltageFiltered() | r | float[7] | 28 | Filtered per-cell voltage in mV — this is what the voltage protections evaluate |
| 0x02 | PackVoltage() | r | float | 4 | Pack terminal voltage in mV |
| 0x03 | PackVoltageFiltered() | r | float | 4 | Filtered pack voltage in mV |
| 0x04 | PackCurrent() | r | float | 4 | Instantaneous current in mA; signed |
| 0x05 | PackCurrentFiltered() | r | float | 4 | Filtered current in mA — this is what the current protections evaluate |
| 0x06 | TemperaturePackage() | r | float | 4 | NTC pack temperature in °C — this is what the temperature protection evaluates |
| 0x07 | TemperatureSTM32() | r | float | 4 | Internal MCU die temperature in °C |
| 0x08 | MainVddVoltage() | r | float | 4 | System VDD in mV; nominally 3300 |
| 0x09 | FETStatus() | r | uint16 | 2 | Bits 15-9: reserved<br/>Bits 8-2: balancer FETs — bit 2 = cell 1 … bit 8 = cell 7; 0=off, 1=on<br/>Bit 1: pre-FET state — 0=off, 1=on<br/>Bit 0: main FETs state — 0=off, 1=on |
| 0x0A | CurrentSensorOffset() | r/w | float | 4 | Current sensor offset calibration |
| 0x0B | CurrentSensorGain() | r/w | float | 4 | Current sensor gain calibration |
| 0x0C | VoltageOffset() | r/w | float[7] | 28 | Per-cell ADC offset calibration |
| 0x0D | VoltageGain() | r/w | float[7] | 28 | Per-cell ADC gain calibration |
| 0x0E | NTC_Beta() | r/w | float | 4 | NTC beta value from datasheet, e.g. 3950.0 |
| 0x0F | NTC_R_Nominal() | r/w | float | 4 | NTC nominal resistance at 25 °C in Ω |
| 0x10 | NTC_R_Fixed() | r/w | float | 4 | Divider resistor in Ω, e.g. 10000.0 |
| 0x11 | NTC_T_Nominal() | r/w | float | 4 | NTC nominal temperature in K, e.g. 298.15 |
| 0x12 | TemperatureOffset() | r/w | float | 4 | Temperature sensor offset in °C |

### Control data — `0x30` – `0x57`

Configuration, protection thresholds, fault records, lifetime counters and device identity.

| Address | Register | Access | Type | Size | Notes |
|---------|----------|--------|------|------|-------|
| 0x30 | Configuration() | r/w | uint16 | 2 | Bits 15-10: reserved<br/>Bit 9: temperature protection — 0=disable, 1=enable<br/>Bit 8: current protection — 0=disable, 1=enable<br/>Bit 7: voltage protection — 0=disable, 1=enable<br/>Bit 6: UART state — 0=disable, 1=enable<br/>Bit 5: CAN 2.0 state — 0=disable, 1=enable<br/>Bit 4: I²C state — 0=disable, 1=enable<br/>Bits 3-0: cell count — valid range 2-7<br/>Default: `0x0387` (7 cells, all three protections enabled) |
| 0x31 | MainControl() | r/w | uint16 | 2 | Bits 15-2: reserved<br/>Bits 1-0: operating mode — `0`=normal, `1`=config, `2`=learning, `3`=reserved<br/>Default: `0x0000` (normal). Prefer the `CC_CMD` SetMode command over writing this register directly. |
| 0x32 | CellCapacity() | r/w | uint32 | 4 | Factory cell capacity in mAh; default 2000 |
| 0x33 | MaxCellVoltage() | r/w | uint16 | 2 | Max designed cell voltage in mV; default 4200 |
| 0x34 | MinCellVoltage() | r/w | uint16 | 2 | Min designed cell voltage in mV; default 2500 |
| 0x35 | ChargingTerminationCurrent() | r/w | uint16 | 2 | Termination current in mA; default 100 |
| 0x36 | UVP_SlowThreshold() | r/w | uint16 | 2 | Slow undervoltage threshold in mV; default 2700 |
| 0x37 | UVP_SlowTime() | r/w | uint16 | 2 | Slow UVP detection time in ms; default 20000; `0` disables this protection |
| 0x38 | UVP_FastThreshold() | r/w | uint16 | 2 | Fast undervoltage threshold in mV; default 2500 |
| 0x39 | UVP_FastTime() | r/w | uint16 | 2 | Fast UVP detection time in ms; default 500; `0` disables |
| 0x3A | OVP_SlowThreshold() | r/w | uint16 | 2 | Slow overvoltage threshold in mV; default 4220 |
| 0x3B | OVP_SlowTime() | r/w | uint16 | 2 | Slow OVP detection time in ms; default 20000; `0` disables |
| 0x3C | OVP_FastThreshold() | r/w | uint16 | 2 | Fast overvoltage threshold in mV; default 4300 |
| 0x3D | OVP_FastTime() | r/w | uint16 | 2 | Fast OVP detection time in ms; default 500; `0` disables |
| 0x3E | OCP_ChargeThreshold() | r/w | uint16 | 2 | Charge overcurrent threshold in mA; default 4000 |
| 0x3F | OCP_ChargeTime() | r/w | uint16 | 2 | Charge OCP detection time in ms; default 10000; `0` disables |
| 0x40 | OCP_DischargeSlowThreshold() | r/w | uint16 | 2 | Slow discharge overcurrent threshold in mA; default 15000 |
| 0x41 | OCP_DischargeSlowTime() | r/w | uint16 | 2 | Slow discharge OCP detection time in ms; default 10000; `0` disables |
| 0x42 | OCP_DischargeFastThreshold() | r/w | uint16 | 2 | Fast discharge overcurrent threshold in mA; default 18000 |
| 0x43 | OCP_DischargeFastTime() | r/w | uint16 | 2 | Fast discharge OCP detection time in ms; default 1000; `0` disables |
| 0x44 | OTP_Threshold() | r/w | uint16 | 2 | Overtemperature threshold in °C; default 60 |
| 0x45 | OTP_Time() | r/w | uint16 | 2 | OTP detection time in ms; default 60000; `0` disables |
| 0x46 | FaultSnapshotVoltage() | r | uint16[7] | 14 | Per-cell voltage at last fault in mV |
| 0x47 | FaultSnapshotCurrent() | r | int16 | 2 | Current at last fault in mA; signed |
| 0x48 | FaultSnapshotTemperature() | r | uint8 | 1 | Temperature at last fault in °C |
| 0x49 | FaultSnapshotSoC() | r | uint8 | 1 | SoC at last fault in % |
| 0x4A | FaultCode() | r | uint8[8] | 8 | Last 8 fault codes; entry 0 = oldest, entry 7 = latest |
| 0x4B | FaultTimestamp() | r | uint32[8] | 32 | Unix timestamps matching `FaultCode()`, same ordering |
| 0x4C | CellBalancingEnergy() | r | uint16[7] | 14 | Per-cell accumulated balancing energy in mWh |
| 0x4D | CellBalancingTime() | r | uint16[7] | 14 | Per-cell accumulated balancing time in minutes |
| 0x4E | CellDeepestDischarge() | r | uint8[7] | 7 | Per-cell lowest SoC ever recorded in % |
| 0x4F | CellMaxTemperature() | r | uint8[7] | 7 | Per-cell highest temperature ever recorded in °C |
| 0x50 | LastCommunicationTimestamp() | r | uint32 | 4 | Time of last host communication in ms since boot; not yet updated by the firmware |
| 0x51 | UptimeCounter() | r | uint64 | 8 | Total uptime in ms; rollover-safe accumulation of the HAL tick |
| 0x52 | FirmwareVersion() | r | char[32] | 32 | ASCII, null-padded — "1.0.0" |
| 0x53 | HardwareVersion() | r | char[32] | 32 | ASCII, null-padded — "RevA" |
| 0x54 | ManufacturerName() | r | char[32] | 32 | ASCII, null-padded — "OpenBatt Team" |
| 0x55 | DeviceName() | r | char[32] | 32 | ASCII, null-padded — "OpenBMS" |
| 0x56 | DeviceChemistry() | r | char[32] | 32 | ASCII, null-padded — "Li-Ion" |
| 0x57 | ManufacturerData() | r | char[32] | 32 | ASCII, null-padded — "Year 2026" |

Protection thresholds are evaluated every 50 ms against the *filtered* measurements. A protection fires only after its condition has held continuously for the configured detection time; a detection time of `0` disables that individual protection regardless of the `Configuration()` enable bit.

> **Note:** `0x32`–`0x34` are per-cell quantities in the firmware (`cell_capacity`, `voltage_cell_max`, `voltage_cell_min`) but are labelled `PackCapacity` / `MaxPackVoltage` / `MinPackVoltage` in `flashing_script/read_data.py`. The names above follow the firmware.

### Fuel gauge data — `0x80` – `0xAC`

Gauge outputs, the OCV/ECM cell model, temperature-dependent capacity, Kalman filter tuning and per-cell filter and aging state.

All 20-element tables share the SOC breakpoints defined by `SOC_Grid()` at `0x86` — 0 % to 95 % in 5 % steps by default. All 5-element tables share the temperature breakpoints defined by `Q_Nom_TempSetpoints()` at `0x93`. See [battery-model.md](battery-model.md) for the model these parameters feed.

| Address | Register | Access | Type | Size | Notes |
|---------|----------|--------|------|------|-------|
| 0x80 | RelativeSoC() | r | uint8 | 1 | Pack SoC as % of full charge capacity; 0-100 |
| 0x81 | CellSoC() | r | uint8[7] | 7 | Per-cell SoC in %; 0-100 |
| 0x82 | CellSoH() | r | uint8[7] | 7 | Per-cell SoH in %; 0-100 |
| 0x83 | CellRemainingCapacity() | r | uint16[7] | 14 | Per-cell remaining capacity in mAh |
| 0x84 | CellSelfDischarge() | r | uint16[7] | 14 | Per-cell self-discharge rate in mAh/month |
| 0x85 | CellQmax() | r | uint16[7] | 14 | Per-cell learned maximum capacity in mAh |
| 0x86 | SOC_Grid() | r/w | float[20] | 80 | SOC breakpoints in %; default 0 to 95 in 5 % steps |
| 0x87 | OCV_Discharge() | r/w | float[20] | 80 | Open-circuit voltage discharge curve in V per cell |
| 0x88 | OCV_Charge() | r/w | float[20] | 80 | Open-circuit voltage charge curve in V per cell |
| 0x89 | R0_Discharge() | r/w | float[20] | 80 | Series resistance, discharge, in Ω |
| 0x8A | R1_Discharge() | r/w | float[20] | 80 | First RC resistance, discharge, in Ω |
| 0x8B | Tau1_Discharge() | r/w | float[20] | 80 | First RC time constant, discharge, in s |
| 0x8C | R2_Discharge() | r/w | float[20] | 80 | Second RC resistance, discharge, in Ω |
| 0x8D | Tau2_Discharge() | r/w | float[20] | 80 | Second RC time constant, discharge, in s |
| 0x8E | R0_Charge() | r/w | float[20] | 80 | Series resistance, charge, in Ω |
| 0x8F | R1_Charge() | r/w | float[20] | 80 | First RC resistance, charge, in Ω |
| 0x90 | Tau1_Charge() | r/w | float[20] | 80 | First RC time constant, charge, in s |
| 0x91 | R2_Charge() | r/w | float[20] | 80 | Second RC resistance, charge, in Ω |
| 0x92 | Tau2_Charge() | r/w | float[20] | 80 | Second RC time constant, charge, in s |
| 0x93 | Q_Nom_TempSetpoints() | r/w | float[5] | 20 | Temperature breakpoints in °C; default −20, −10, 0, 25, 45 |
| 0x94 | Q_Nom_TempCapacity() | r/w | float[5] | 20 | Capacity in Ah at each temperature breakpoint |
| 0x95 | Q_Nom() | r/w | float | 4 | Nominal capacity at 25 °C in Ah |
| 0x96 | CoulombicEfficiency() | r/w | float | 4 | Typically 0.995 to 0.999 |
| 0x97 | R0_Ref() | r/w | float | 4 | R0 reference at 25 °C in Ω |
| 0x98 | R1_Ref() | r/w | float | 4 | R1 reference at 25 °C in Ω |
| 0x99 | Tau1_Ref() | r/w | float | 4 | tau1 reference at 25 °C in s |
| 0x9A | R2_Ref() | r/w | float | 4 | R2 reference at 25 °C in Ω |
| 0x9B | Tau2_Ref() | r/w | float | 4 | tau2 reference at 25 °C in s |
| 0x9C | Ea_R0() | r/w | float | 4 | Arrhenius activation energy for R0 in J/mol |
| 0x9D | Ea_R1() | r/w | float | 4 | Arrhenius activation energy for R1 in J/mol |
| 0x9E | Ea_Tau1() | r/w | float | 4 | Arrhenius activation energy for tau1 in J/mol |
| 0x9F | Ea_R2() | r/w | float | 4 | Arrhenius activation energy for R2 in J/mol |
| 0xA0 | Ea_Tau2() | r/w | float | 4 | Arrhenius activation energy for tau2 in J/mol |
| 0xA1 | KF_Q_SOC() | r/w | float | 4 | Kalman process noise — SOC state |
| 0xA2 | KF_Q_RC1() | r/w | float | 4 | Kalman process noise — V_RC1 state |
| 0xA3 | KF_Q_RC2() | r/w | float | 4 | Kalman process noise — V_RC2 state |
| 0xA4 | KF_R_V() | r/w | float | 4 | Kalman measurement noise — voltage sensor, V² |
| 0xA5 | Cell_SOC_f() | r | float[7] | 28 | Last estimated SOC per cell, 0.0 to 1.0 |
| 0xA6 | Cell_VRC1() | r | float[7] | 28 | Last estimated V_RC1 per cell in V |
| 0xA7 | Cell_VRC2() | r | float[7] | 28 | Last estimated V_RC2 per cell in V |
| 0xA8 | Cell_Covariance() | r/w | float[7][6] | 168 | Covariance upper triangle per cell, row-major by cell<br/>Per-cell order: `P00, P01, P02, P11, P12, P22`<br/>Cell 1: bytes 0-23 · Cell 2: bytes 24-47 · … · Cell 7: bytes 144-167 |
| 0xA9 | Cell_Q_Nom() | r | float[7] | 28 | Per-cell capacity after aging in Ah |
| 0xAA | Cell_R0_Scale() | r | float[7] | 28 | Per-cell R0 growth factor; 1.0 = nominal |
| 0xAB | CycleCount() | r | uint16 | 2 | Charge/discharge cycle counter |
| 0xAC | LearningStatus() | r | uint16 | 2 | Kalman filter convergence and learning state flags |

## Planned interfaces

Both peripherals below are initialized by `main.c` but have no protocol handler in `openbms_comm.c` yet. The intent is for both to expose the same register map and the same command and error codes as the UART protocol, so that any register reachable over UART is reachable over SMBus and CAN with identical addressing.

### SBS v1.1 over I²C/SMBus — planned

| Parameter | Value |
|-----------|-------|
| Peripheral | I2C2 in SMBus slave mode |
| Slave address | `0x0B` (7-bit) |
| PEC | Enabled |
| Timeouts | `TIDLE` and `TEXTEN` enabled |
| Bus speed | ~100 kHz |
| Status | Hardware initialized, no protocol handler |

The plan is a Smart Battery Specification v1.1 compliant register map at `0x00`–`0x3F` with the OpenBMS registers exposed as manufacturer-defined extensions above `0x40`. Refer to the [SBS Specification](https://sbs-forum.org/specs/sbdat110.pdf) for the standard portion.

> **Note:** `I2C1` is a separate master-mode bus used internally for the on-board 4 kB EEPROM at address `0xA0`. It is not a host interface.

### CAN 2.0 — planned

| Parameter | Value |
|-----------|-------|
| Peripheral | CAN1 (bxCAN) on PA11 / PA12 |
| Bit rate | 1 Mbit/s — prescaler 8, BS1 7 TQ, BS2 2 TQ, SJW 1 TQ at 80 MHz PCLK1 |
| Frame format | CAN 2.0A — 11-bit identifier |
| Max payload | 8 bytes per frame |
| Auto bus-off recovery | Enabled |
| Auto retransmission | Enabled |
| Interrupts | `CAN1_RX0`, `CAN1_TX`, `CAN1_SCE` enabled |
| Termination | 120 Ω at each end of the bus |
| Status | Hardware initialized, no protocol handler |

Because bxCAN carries at most 8 bytes per frame while several registers are far larger — `Cell_Covariance()` alone is 168 bytes — a multi-frame transport with a sequence byte will be required. The frame layout, CAN IDs and node addressing are not yet fixed and will be specified once the handler is implemented.

## Notes for implementers

Behaviour worth knowing when writing a host tool against the current firmware:

- **Reads return the whole register.** The length byte in a read request is ignored; the device always sends the register's full native size. There is no partial read and no block offset.
- **Writes must carry the whole register.** The firmware copies the register's native size out of the receive buffer regardless of the length the host declared. A short write will pull in whatever follows in the buffer, so always send exactly `Size` bytes.
- **The device never initiates traffic.** There are no alarm broadcasts, no unsolicited status frames and no charger negotiation. A host that needs alarm awareness must poll `FETStatus()`, the fault registers and the measurement registers.
- **One transaction at a time.** There is no sequence or transaction ID, so the host must wait for a response before issuing the next request. The reference scripts flush the input buffer before each request.
- **Register writes are not yet persisted or propagated.** Writable registers in the peripheral and control banks are copied into a module-local snapshot inside `openbms_comm.c`; there is no setter path back into `openbms_periph.c` or `openbms_ctrl.c`, and nothing is written to EEPROM. Writes therefore do not currently take effect and do not survive a reset.
- **The fuel gauge bank is not yet wired up.** `0x80`–`0xAC` are served from a zero-initialized static structure — the fuel gauge module does not populate it yet, so these registers read as zeros.
- **`0xAD`–`0xD9` decode into the fuel gauge bank but map to nothing.** They return `CE_NO_REG` like any other unmapped address.
