| Address | Function | Access | Type | Notes |
|---------|----------|--------|------|-------|
| 0x00 | ManufacturerAccess() | r/w | word | Vendor side-channel |
| 0x01 | RemainingCapacityAlarm() | r/w | word | mAh or 10mWh; 0 disables alarm |
| 0x02 | RemainingTimeAlarm() | r/w | word | Minutes; 0 disables alarm |
| 0x03 | BatteryMode() | r/w | word | Config flags; LSB read-only, MSB read/write |
| 0x04 | AtRate() | r/w | word | Signed mA or 10mW; hypothetical rate |
| 0x05 | AtRateTimeToFull() | r | word | Minutes; 65535 = not charging |
| 0x06 | AtRateTimeToEmpty() | r | word | Minutes; 65535 = not discharging |
| 0x07 | AtRateOK() | r | word | Boolean; can battery deliver AtRate for 10s? |
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
| 0x16 | BatteryStatus() / AlarmWarning() | r | word | Alarm + status flags + 4-bit error code; shared with AlarmWarning() broadcast |
| 0x17 | CycleCount() | r | word | Charge/discharge cycle counter; 65535 = ≥65535 cycles |
| 0x18 | DesignCapacity() | r | word | mAh or 10mWh; nominal factory capacity |
| 0x19 | DesignVoltage() | r | word | mV; nominal pack voltage |
| 0x1A | SpecificationInfo() | r | word | Packed: SBS version + VScale + IPScale |
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
