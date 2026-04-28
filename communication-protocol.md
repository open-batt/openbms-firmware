# 📡 Communication Protocol

OpenBMS firmware implements the following communication protocols:

- **SBS v1.1 over I²C/SMBus** — Smart Battery Specification v1.1, compatible with any SBS-compliant host. SMBus address `0x0B`, speed up to 100 kHz, PEC error checking.
- **Modbus RTU over UART** — 115200 baud, 8N1, same register addresses as SBS. Used for debugging and external communication.
- **CAN 2.0** — 500 kbit/s, 11-bit identifier, same register addresses as SBS. Used for communication with host, charger and other system components.

### SBS v1.1 Protocol

Registers in range `0x00` - `0x3F` are standard SBS protocol registers and their description can be found in the table below. More information can be found at [SBS Specification](https://sbs-forum.org/specs/sbdat110.pdf).

| Layer | Name | Responsibility |
|-------|------|----------------|
| Application | SBS | Register map, data meaning, device addresses, alarm broadcasts, charger negotiation |
| Transport | SMBus | Transaction types (Read/Write Word, Read Block), PEC, timing rules |
| Physical | I²C | SDA/SCL lines, electrical signaling, ACK/NACK |

**Register description:**

| Address | Function | Access | Type | Notes |
|---------|----------|--------|------|-------|
| 0x00 | ManufacturerAccess() | r/w | word | Vendor side-channel |
| 0x01 | RemainingCapacityAlarm() | r/w | word | mAh or 10mWh; 0 disables alarm |
| 0x02 | RemainingTimeAlarm() | r/w | word | Minutes; 0 disables alarm |
| 0x03 | BatteryMode() | r/w | word | Bit 15: CAPACITY_MODE — 0=mA/mAh, 1=10mW/10mWh<br/>Bit 14: CHARGER_MODE — 0=broadcast ChargingCurrent/Voltage, 1=disable<br/>Bit 13: ALARM_MODE — 0=broadcast AlarmWarning, 1=disable<br/>Bit 9: PRIMARY_BATTERY — 0=secondary, 1=primary<br/>Bit 8: CHARGE_CONTROLLER_ENABLED — 0=off, 1=on<br/>Bit 7: CONDITION_FLAG — 1=conditioning cycle requested<br/>Bit 1: PRIMARY_BATTERY_SUPPORT — 1=supported<br/>Bit 0: INTERNAL_CHARGE_CONTROLLER — 1=supported<br/>Bits 12-10, 6-2: reserved |
| 0x04 | AtRate() | r/w | word | Signed mA or 10mW; hypothetical rate; positive=charge, negative=discharge, zero=default |
| 0x05 | AtRateTimeToFull() | r | word | Minutes to full at AtRate charge rate — purely hypothetical; 65535 = not charging |
| 0x06 | AtRateTimeToEmpty() | r | word | Minutes to empty at AtRate discharge rate — purely hypothetical; 65535 = not discharging |
| 0x07 | AtRateOK() | r | word | Boolean — can battery sustain AtRate on top of present current for 10s?; always TRUE if AtRate() >= 0 |
| 0x08 | Temperature() | r | word | 0.1K units |
| 0x09 | Voltage() | r | word | mV |
| 0x0A | Current() | r | word | Signed mA; positive = charge, negative = discharge |
| 0x0B | AverageCurrent() | r | word | Signed mA; 1-minute rolling average |
| 0x0C | MaxError() | r | word | % gauge uncertainty |
| 0x0D | RelativeStateOfCharge() | r | word | % of FullChargeCapacity() |
| 0x0E | AbsoluteStateOfCharge() | r | word | % of DesignCapacity(); can exceed 100% |
| 0x0F | RemainingCapacity() | r | word | mAh or 10mWh |
| 0x10 | FullChargeCapacity() | r | word | mAh or 10mWh; learned full capacity |
| 0x11 | RunTimeToEmpty() | r | word | Minutes at present rate; 65535 = not discharging |
| 0x12 | AverageTimeToEmpty() | r | word | Minutes; 1-minute rolling average; 65535 = not discharging |
| 0x13 | AverageTimeToFull() | r | word | Minutes; 1-minute rolling average; 65535 = not charging |
| 0x14 | ChargingCurrent() | r/w | word | mA; broadcast to charger; 65535 = charger acts as voltage source |
| 0x15 | ChargingVoltage() | r/w | word | mV; broadcast to charger; 65535 = charger acts as current source |
| 0x16 | BatteryStatus() / AlarmWarning() | r | word | Bit 15: OVER_CHARGED_ALARM<br/>Bit 14: TERMINATE_CHARGE_ALARM<br/>Bit 13: reserved<br/>Bit 12: OVER_TEMP_ALARM<br/>Bit 11: TERMINATE_DISCHARGE_ALARM<br/>Bit 10: reserved<br/>Bit 9: REMAINING_CAPACITY_ALARM<br/>Bit 8: REMAINING_TIME_ALARM<br/>Bit 7: INITIALIZED<br/>Bit 6: DISCHARGING<br/>Bit 5: FULLY_CHARGED<br/>Bit 4: FULLY_DISCHARGED<br/>Bits 3-0: ERROR_CODE — 0x0=OK, 0x1=Busy, 0x2=Reserved, 0x3=Unsupported, 0x4=Access Denied, 0x5=Overflow, 0x6=Bad Size, 0x7=Unknown |
| 0x17 | CycleCount() | r | word | Charge/discharge cycle counter; 65535 = ≥65535 cycles |
| 0x18 | DesignCapacity() | r | word | mAh or 10mWh; nominal factory capacity |
| 0x19 | DesignVoltage() | r | word | mV; nominal pack voltage |
| 0x1A | SpecificationInfo() | r | word | Bits 15-12: IPScale — current/capacity multiplier (10^n)<br/>Bits 11-8: VScale — voltage multiplier (10^n)<br/>Bits 7-4: Version — 0x1=SBS1.0, 0x2=SBS1.1, 0x3=SBS1.1+PEC<br/>Bits 3-0: Revision — always 0x1<br/>Note: scaling does not apply to ChargingCurrent() and ChargingVoltage() |
| 0x1B | ManufactureDate() | r | word | Packed: (year−1980)×512 + month×32 + day |
| 0x1C | SerialNumber() | r | word | Combined with name + date = unique battery ID |
| 0x1D | — | — | — | Undefined |
| 0x1E | — | — | — | Undefined |
| 0x1F | — | — | — | Undefined |
| 0x20 | ManufacturerName() | r | block | String |
| 0x21 | DeviceName() | r | block | String |
| 0x22 | DeviceChemistry() | r | block | String; e.g. LION, NiMH, LiP |
| 0x23 | ManufacturerData() | r | block | Vendor-defined payload |
| 0x24–0x2F | — | — | — | Optional manufacturer-defined block registers |
| 0x30–0x3F | — | — | — | Reserved |

`BatteryStatus()` and `AlarmWarning()` share address `0x16` but work in opposite directions.
`BatteryStatus()` is host-initiated — the host polls the battery at any time and receives the current status word with the real error code in bits 3–0.
`AlarmWarning()` is battery-initiated — when any alarm bit is set, the battery becomes bus master and broadcasts the same word unsolicited every 10 seconds until the condition clears, but with the error nibble forced to `0xF` to signal that this is an alarm broadcast and not a response to a command.

| | BatteryStatus() | AlarmWarning() |
|--|----------------|----------------|
| Direction | Host reads from battery | Battery broadcasts to host and/or charger |
| Trigger | Host polls at any time | Battery sends when any alarm bit is set |
| Error nibble | Actual error code | Forced to 0xF (all ones) before sending |
| Interval | On demand | Every 10 seconds until condition clears |
| Bits 9–8 alarms | Sent to host only when read | Sent to host only |
| Bits 15–11 alarms | Sent to host only when read | Sent to both host and charger |

## SBS v1.1 extended non-standard registers

Registers in range `0x40` - `0xFF` are OpenBMS-specific extensions and are not part of the SBS v1.1 specification. They follow the same SMBus transaction conventions as standard SBS registers (Read Word, Write Word, Read Block) to maintain compatibility with any SBS-compliant host, but their content and meaning are specific to OpenBMS firmware.

**Register description:**

| Address | Function | Access | Type | Notes |
|---------|----------|--------|------|-------|
| 0x40 | Configuration() | r/w | word | Bits 15-7: reserved<br/>Bit 6: UART state - 0=disable, 1=enable<br/>Bit 5: CAN 2.0 state - 0=disable, 1=enable<br/>Bit 4: I2C state - 0=disable, 1=enable<br/>Bits 3-0: cell count - valid range 2-7 |
| 0x41 | MainControl() | r/w | word | Bits 15-6: reserved<br/>Bit 5: temperature protections - 0=disable, 1=enable<br/>Bit 4: current protections - 0=disable, 1=enable<br/>Bit 3: voltage protections - 0=disable, 1=enable<br/>Bit 2: reserved<br/>Bit 1: test mode - 0=normal, 1=test<br/>Bit 0: OpenBMS state - 0=disable, 1=enable |
| 0x42 | FETState() | r/w | word | Bits 15-2: reserved<br/>Bit 1: aux FET state - 0=disable, 1=enable<br/>Bit 0: main FETs state - 0=disable, 1=enable |
| 0x43 | VoltageProtectionControl() | r/w | block | Bytes 0-1: slow UVP threshold - UINT16 mV, range 0-65535<br/>Bytes 2-3: slow UVP detection time - UINT16 ms, range 0-65535<br/>Bytes 4-5: fast UVP threshold - UINT16 mV, range 0-65535<br/>Bytes 6-7: fast UVP detection time - UINT16 ms, range 0-65535<br/>Bytes 8-9: slow OVP threshold - UINT16 mV, range 0-65535<br/>Bytes 10-11: slow OVP detection time - UINT16 ms, range 0-65535<br/>Bytes 12-13: fast OVP threshold - UINT16 mV, range 0-65535<br/>Bytes 14-15: fast OVP detection time - UINT16 ms, range 0-65535 |
| 0x44 | CurrentProtectionControl() | r/w | block | Bytes 0-1: charge OCP threshold - UINT16 mA, range 0-65535<br/>Bytes 2-3: charge OCP detection time - UINT16 ms, range 0-65535<br/>Bytes 4-5: slow discharge OCP threshold - UINT16 mA, range 0-65535<br/>Bytes 6-7: slow discharge OCP detection time - UINT16 ms, range 0-65535<br/>Bytes 8-9: fast discharge OCP threshold - UINT16 mA, range 0-65535<br/>Bytes 10-11: fast discharge OCP detection time - UINT16 ms, range 0-65535 |
| 0x45 | TemperatureProtectionControl() | r/w | block | Bytes 0-1: OTP threshold - UINT16 °C, range 0-65535<br/>Bytes 2-3: OTP detection time - UINT16 ms, range 0-65535 |
| 0x46 | OCV() | r/w | block | Pack OCV table — 51 SoC points (0% to 100% in 2% steps) × 4 temperatures × UINT16 mV = 408 bytes<br/>Bytes 0-101: OCV at -10°C (51 × UINT16)<br/>Bytes 102-203: OCV at 0°C (51 × UINT16)<br/>Bytes 204-305: OCV at 25°C (51 × UINT16)<br/>Bytes 306-407: OCV at 45°C (51 × UINT16) |
| 0x47 | Impedance() | r/w | block | Per-cell impedance table — 7 cells × 51 SoC points (0% to 100% in 2% steps) × 4 temperatures × UINT16 mΩ = 2856 bytes<br/>Cell 1: Bytes 0-101 at -10°C, Bytes 102-203 at 0°C, Bytes 204-305 at 25°C, Bytes 306-407 at 45°C<br/>Cell 2: Bytes 408-509 at -10°C, Bytes 510-611 at 0°C, Bytes 612-713 at 25°C, Bytes 714-815 at 45°C<br/>Cell 3: Bytes 816-917 at -10°C, Bytes 918-1019 at 0°C, Bytes 1020-1121 at 25°C, Bytes 1122-1223 at 45°C<br/>Cell 4: Bytes 1224-1325 at -10°C, Bytes 1326-1427 at 0°C, Bytes 1428-1529 at 25°C, Bytes 1530-1631 at 45°C<br/>Cell 5: Bytes 1632-1733 at -10°C, Bytes 1734-1835 at 0°C, Bytes 1836-1937 at 25°C, Bytes 1938-2039 at 45°C<br/>Cell 6: Bytes 2040-2141 at -10°C, Bytes 2142-2243 at 0°C, Bytes 2244-2345 at 25°C, Bytes 2346-2447 at 45°C<br/>Cell 7: Bytes 2448-2549 at -10°C, Bytes 2550-2651 at 0°C, Bytes 2652-2753 at 25°C, Bytes 2754-2855 at 45°C |
| 0x48 | CellVoltage() | r | block | Per-cell voltages — 7 × UINT16 mV = 14 bytes<br/>Bytes 0-1: Cell 1 voltage<br/>Bytes 2-3: Cell 2 voltage<br/>Bytes 4-5: Cell 3 voltage<br/>Bytes 6-7: Cell 4 voltage<br/>Bytes 8-9: Cell 5 voltage<br/>Bytes 10-11: Cell 6 voltage<br/>Bytes 12-13: Cell 7 voltage |
| 0x49 | CellTemperature() | r | block | Per-cell temperatures — 7 × INT16 0.1°C = 14 bytes<br/>Bytes 0-1: Cell 1 temperature<br/>Bytes 2-3: Cell 2 temperature<br/>Bytes 4-5: Cell 3 temperature<br/>Bytes 6-7: Cell 4 temperature<br/>Bytes 8-9: Cell 5 temperature<br/>Bytes 10-11: Cell 6 temperature<br/>Bytes 12-13: Cell 7 temperature |
| 0x4A | FETStatus() | r | word | Actual FET state readback<br/>Bits 15-2: reserved<br/>Bit 1: aux FET state - 0=off, 1=on<br/>Bit 0: main FETs state - 0=off, 1=on |
| 0x4B | CellSoC() | r | block | Per-cell SoC — 7 × UINT8 % = 7 bytes<br/>Byte 0: Cell 1 SoC<br/>Byte 1: Cell 2 SoC<br/>Byte 2: Cell 3 SoC<br/>Byte 3: Cell 4 SoC<br/>Byte 4: Cell 5 SoC<br/>Byte 5: Cell 6 SoC<br/>Byte 6: Cell 7 SoC |
| 0x4C | CellSoH() | r | block | Per-cell SoH — 7 × UINT8 % = 7 bytes<br/>Byte 0: Cell 1 SoH<br/>Byte 1: Cell 2 SoH<br/>Byte 2: Cell 3 SoH<br/>Byte 3: Cell 4 SoH<br/>Byte 4: Cell 5 SoH<br/>Byte 5: Cell 6 SoH<br/>Byte 6: Cell 7 SoH |
| 0x4D | CellRemainingCapacity() | r | block | Per-cell remaining capacity — 7 × UINT16 mAh = 14 bytes<br/>Bytes 0-1: Cell 1<br/>Bytes 2-3: Cell 2<br/>Bytes 4-5: Cell 3<br/>Bytes 6-7: Cell 4<br/>Bytes 8-9: Cell 5<br/>Bytes 10-11: Cell 6<br/>Bytes 12-13: Cell 7 |
| 0x4E | CellSelfDischarge() | r | block | Per-cell self-discharge rate — 7 × UINT16 mAh/month = 14 bytes<br/>Bytes 0-1: Cell 1<br/>Bytes 2-3: Cell 2<br/>Bytes 4-5: Cell 3<br/>Bytes 6-7: Cell 4<br/>Bytes 8-9: Cell 5<br/>Bytes 10-11: Cell 6<br/>Bytes 12-13: Cell 7 |
| 0x4F | FaultSnapshot() | r | block | Snapshot at last fault event — 18 bytes<br/>Bytes 0-1: Cell 1 voltage - UINT16 mV<br/>Bytes 2-3: Cell 2 voltage - UINT16 mV<br/>Bytes 4-5: Cell 3 voltage - UINT16 mV<br/>Bytes 6-7: Cell 4 voltage - UINT16 mV<br/>Bytes 8-9: Cell 5 voltage - UINT16 mV<br/>Bytes 10-11: Cell 6 voltage - UINT16 mV<br/>Bytes 12-13: Cell 7 voltage - UINT16 mV<br/>Bytes 14-15: current at fault - INT16 mA<br/>Byte 16: temperature at fault - UINT8 °C<br/>Byte 17: SoC at fault - UINT8 % |
| 0x50 | FaultHistory() | r | block | Last 8 fault events — 8 × (UINT8 fault code + UINT32 timestamp) = 40 bytes<br/>Each entry: Byte 0 = fault code, Bytes 1-4 = Unix timestamp<br/>Entry 1 (oldest): Bytes 0-4<br/>Entry 2: Bytes 5-9<br/>Entry 3: Bytes 10-14<br/>Entry 4: Bytes 15-19<br/>Entry 5: Bytes 20-24<br/>Entry 6: Bytes 25-29<br/>Entry 7: Bytes 30-34<br/>Entry 8 (latest): Bytes 35-39 |
| 0x51 | ProtectionEventCounters() | r | block | Per-protection trigger counters — 7 cells × 5 protections × UINT16 = 70 bytes<br/>Per cell order: OVP counter, UVP counter, OCP counter, OTP counter, UTP counter<br/>Cell 1: Bytes 0-9<br/>Cell 2: Bytes 10-19<br/>Cell 3: Bytes 20-29<br/>Cell 4: Bytes 30-39<br/>Cell 5: Bytes 40-49<br/>Cell 6: Bytes 50-59<br/>Cell 7: Bytes 60-69 |
| 0x52 | CurrentSensorCalibration() | r/w | block | Current sensor offset + gain — 2 × INT16 = 4 bytes<br/>Bytes 0-1: offset calibration value<br/>Bytes 2-3: gain calibration value |
| 0x53 | VoltageCalibration() | r/w | block | Per-cell ADC offset + gain — 7 × 2 × INT16 = 28 bytes<br/>Cell 1: Bytes 0-1 offset, Bytes 2-3 gain<br/>Cell 2: Bytes 4-5 offset, Bytes 6-7 gain<br/>Cell 3: Bytes 8-9 offset, Bytes 10-11 gain<br/>Cell 4: Bytes 12-13 offset, Bytes 14-15 gain<br/>Cell 5: Bytes 16-17 offset, Bytes 18-19 gain<br/>Cell 6: Bytes 20-21 offset, Bytes 22-23 gain<br/>Cell 7: Bytes 24-25 offset, Bytes 26-27 gain |
| 0x54 | TemperatureCalibration() | r/w | block | Temperature sensor offset — UINT8 °C = 1 byte<br/>Byte 0: temperature sensor offset calibration value |
| 0x55 | CellBalancingEnergy() | r | block | Per-cell accumulated balancing energy — 7 × UINT16 mWh = 14 bytes<br/>Bytes 0-1: Cell 1<br/>Bytes 2-3: Cell 2<br/>Bytes 4-5: Cell 3<br/>Bytes 6-7: Cell 4<br/>Bytes 8-9: Cell 5<br/>Bytes 10-11: Cell 6<br/>Bytes 12-13: Cell 7 |
| 0x56 | CellBalancingTime() | r | block | Per-cell accumulated balancing time — 7 × UINT16 minutes = 14 bytes<br/>Bytes 0-1: Cell 1<br/>Bytes 2-3: Cell 2<br/>Bytes 4-5: Cell 3<br/>Bytes 6-7: Cell 4<br/>Bytes 8-9: Cell 5<br/>Bytes 10-11: Cell 6<br/>Bytes 12-13: Cell 7 |
| 0x57 | CellDeepestDischarge() | r | block | Per-cell lowest SoC ever recorded — 7 × UINT8 % = 7 bytes<br/>Byte 0: Cell 1<br/>Byte 1: Cell 2<br/>Byte 2: Cell 3<br/>Byte 3: Cell 4<br/>Byte 4: Cell 5<br/>Byte 5: Cell 6<br/>Byte 6: Cell 7 |
| 0x58 | CellMaxTemperature() | r | block | Per-cell highest temperature ever recorded — 7 × UINT8 °C = 7 bytes<br/>Byte 0: Cell 1<br/>Byte 1: Cell 2<br/>Byte 2: Cell 3<br/>Byte 3: Cell 4<br/>Byte 4: Cell 5<br/>Byte 5: Cell 6<br/>Byte 6: Cell 7 |
| 0x59 | CellQmax() | r | block | Per-cell learned maximum capacity — 7 × UINT16 mAh = 14 bytes<br/>Bytes 0-1: Cell 1<br/>Bytes 2-3: Cell 2<br/>Bytes 4-5: Cell 3<br/>Bytes 6-7: Cell 4<br/>Bytes 8-9: Cell 5<br/>Bytes 10-11: Cell 6<br/>Bytes 12-13: Cell 7 |
| 0x5A | FirmwareVersion() | r | block | Firmware version string — ASCII, null-terminated |
| 0x5B | HardwareVersion() | r | block | Hardware version string — ASCII, null-terminated |
| 0x5C | BoardSerialNumber() | r | block | Board serial number string — ASCII, null-terminated |
| 0x5D | LastCommunicationTimestamp() | r | block | Timestamp of last host communication — UINT32 Unix timestamp = 4 bytes<br/>Bytes 0-3: Unix timestamp |
| 0x5E | UptimeCounter() | r | word | Total uptime since first boot — UINT32 seconds |
| 0x5F | BalancingStatus() | r | word | Bitmask — which cells are currently balancing<br/>Bits 15-7: reserved<br/>Bit 6: Cell 7<br/>Bit 5: Cell 6<br/>Bit 4: Cell 5<br/>Bit 3: Cell 4<br/>Bit 2: Cell 3<br/>Bit 1: Cell 2<br/>Bit 0: Cell 1 |
| 0x60 | BalancingControl() | r/w | word | Force balancing on/off per cell — bitmask<br/>Bits 15-7: reserved<br/>Bit 6: Cell 7<br/>Bit 5: Cell 6<br/>Bit 4: Cell 5<br/>Bit 3: Cell 4<br/>Bit 2: Cell 3<br/>Bit 1: Cell 2<br/>Bit 0: Cell 1 |

## UART Protocol

OpenBMS uses Modbus RTU over UART for communication with external devices. Modbus register addresses are identical to SBS/I²C register addresses, so the same register map applies to both communication interfaces.

### Modbus RTU frame structure

| Device Address | Function Code | Data | CRC-16/IBM |
|----------------|---------------|------|------------|
| 1 byte | 1 byte | N bytes | 2 bytes |

### Supported function codes

| Code | Name | Description |
|------|------|-------------|
| 0x03 | Read Holding Registers | Read one or more 16-bit registers |
| 0x06 | Write Single Register | Write one 16-bit register |
| 0x10 | Write Multiple Registers | Write multiple 16-bit registers |

### Example — Read cell voltages

Host reads 7 cell voltages from register `0x48` (`CellVoltage()`):

**Request:**
```
01  03  00 48  00 07  B6 54
│   │   └──┘   └──┘   └─────  CRC (2 bytes)
│   │   │      └────────────  quantity — read 7 registers
│   │   └───────────────────  start address — 0x0048
│   └───────────────────────  function code — Read Holding Registers
└───────────────────────────  device address — 0x01
```

**Response:**
```
01 03 0E 0F A0 0F A2 0F 9E 0F A1 0F 9F 0F A0 0F A3 4E 4A
│  │  │  └─ 7 × UINT16 cell voltages in mV
│  │  └─ byte count — 14 bytes (7 registers × 2 bytes)
│  └─ function code — echo
└─ device address — echo
```

### Example — Write FET state

Host writes to `FETState()` at `0x42`:

**Request:**
```
01  06  00 42  00 01  49 28
│   │   └──┘   └──┘   └─────  CRC (2 bytes)
│   │   │      └────────────  value — 0x0001 (main FETs on)
│   │   └───────────────────  register address — 0x0042
│   └───────────────────────  function code — Write Single Register
└───────────────────────────  device address — 0x01
```

**Response — echo of request if successful:**
```
01 06 00 42 00 01 49 28
```

### Error response

| Exception code | Meaning |
|----------------|---------|
| 0x01 | Illegal function code |
| 0x02 | Illegal data address |
| 0x03 | Illegal data value |
| 0x04 | Device failure |

**Error response frame:**
```
01  83  02  50 41
│   │   │   └───── CRC (2 bytes)
│   │   └───────── exception code
│   └───────────── function code | 0x80 — error flag
└───────────────── device address
```

### UART settings

| Parameter | Value |
|-----------|-------|
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Frame end | 3.5 character times silence (~0.3ms at 115200) |

### CAN 2.0 Protocol

OpenBMS uses CAN 2.0 (bxCAN) for communication with external devices. The STM32L431CCU6 has one CAN 2.0 peripheral (bxCAN) with a maximum payload of 8 bytes per frame.

CAN protocol follows the same register map and function codes as the Modbus RTU over UART interface — same register addresses, same function codes, same data format. This means any register readable over UART is also readable over CAN using identical addressing.

#### CAN frame structure

| CAN ID | Byte 0 | Byte 1-2 | Byte 3 | Byte 4-7 |
|--------|--------|----------|--------|----------|
| 11-bit | function code | register address | sequence byte | data |

No CRC needed — CAN 2.0 hardware handles error detection automatically.

### CAN IDs

| Direction | CAN ID |
|-----------|--------|
| Host → OpenBMS (request) | 0x600 + node ID |
| OpenBMS → Host (response) | 0x580 + node ID |
| OpenBMS broadcast | 0x180 + node ID |

Default node ID is `0x01`, configurable via `Configuration()` register at `0x40`.

### Function codes — same as Modbus RTU

| Code | Name |
|------|------|
| 0x03 | Read register |
| 0x06 | Write single register |
| 0x10 | Write multiple registers |
| 0x83 | Error response |

### Example — Read FET status

Host reads `FETStatus()` from register `0x4A`:

**Request:**
```
CAN ID: 0x601
03  00 4A  01  00 00 00 00
│   └──┘   │   └────────────  padding
│   │      └────────────────  sequence byte — 0x01
│   └───────────────────────  register address — 0x004A
└───────────────────────────  function code — read
```

**Response:**
```
CAN ID: 0x581
03  00 4A  01  00 03  00 00
│   └──┘   │   └──┘   └─────  padding
│   │      │   └────────────  register value — 0x0003 (main FETs on, aux FET on)
│   │      └────────────────  sequence byte echo — 0x01
│   └───────────────────────  register address echo — 0x004A
└───────────────────────────  function code echo
```

### Multi-frame transfers

Since CAN 2.0 is limited to 8 bytes per frame and Byte 3 is reserved for the sequence byte, each frame carries 4 bytes of data (Bytes 4-7). Block registers larger than 4 bytes are split across multiple frames:

```
CAN ID: 0x581
│ 03 00 48 01 XX XX XX XX  ← sequence 01 — Cell 1 and Cell 2 voltages
│ 03 00 48 02 XX XX XX XX  ← sequence 02 — Cell 3 and Cell 4 voltages
│ 03 00 48 03 XX XX XX XX  ← sequence 03 — Cell 5 and Cell 6 voltages
│ 03 00 48 04 XX XX 00 00  ← sequence 04 — Cell 7 voltage + padding
```

### Error response

```
CAN ID: 0x581
83  00 4A  01  02  00 00 00
│   └──┘   │   │   └────────  padding
│   │      │   └────────────  exception code — 0x02 = illegal data address
│   │      └────────────────  sequence byte — 0x01
│   └───────────────────────  register address echo — 0x004A
└───────────────────────────  function code | 0x80 — error flag (0x03 | 0x80 = 0x83)
```

### CAN bus settings

| Parameter | Value |
|-----------|-------|
| Bit rate | 500 kbit/s |
| Termination | 120Ω at each end of the bus |
| Max nodes | 127 |
| Frame format | CAN 2.0A — 11-bit identifier |
