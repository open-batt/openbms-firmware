#!/usr/bin/env python3

import sys
import serial
import time
import struct
from collections import namedtuple

# ---------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------
BAUD_RATE        = 230400
TIMEOUT_RESPONSE = 1.0

CMD_WRITE = 0x01
CMD_READ  = 0x02
CMD_ACK   = 0x03

# ---------------------------------------------------------------
# Packet layout (all directions, always >= 4 bytes):
#
#   Read command  (host → device):
#       [0x02][address][0x00][crc]
#
#   Read response (device → host):
#       [0x02][address][length][data_0 ... data_n][crc]
#       total = length + 4 bytes
#
#   Write command (host → device):
#       [0x01][address][length][data_0 ... data_n][crc]
#       total = length + 4 bytes
#
#   Write ACK    (device → host):
#       [0x03][0x00][0x00][crc]
#       total = 4 bytes
#
# CRC = sum of all bytes before CRC, modulo 256.
# ---------------------------------------------------------------

# ---------------------------------------------------------------
# Register definition
# ---------------------------------------------------------------
# fmt:   struct format character for one element
#        'H'  uint16    'h'  int16
#        'I'  uint32    'i'  int32
#        'B'  uint8     'b'  int8
#        'f'  float32
#        's'  char string  (count = total byte length of string)
# count: number of elements  (for 's': byte length of the char array)
# ---------------------------------------------------------------
Reg = namedtuple('Reg', ['address', 'name', 'unit', 'fmt', 'count'])

# ---------------------------------------------------------------
# SBS registers  0x00 — 0x23
# ---------------------------------------------------------------
SBS_REGISTERS = [
    Reg(0x00, "ManufacturerAccess",         "",          'H', 1 ),
    Reg(0x01, "RemainingCapacityAlarm",     "mAh",       'H', 1 ),
    Reg(0x02, "RemainingTimeAlarm",         "min",       'H', 1 ),
    Reg(0x03, "BatteryMode",                "flags",     'H', 1 ),
    Reg(0x04, "AtRate",                     "mA",        'h', 1 ),
    Reg(0x05, "AtRateTimeToFull",           "min",       'H', 1 ),
    Reg(0x06, "AtRateTimeToEmpty",          "min",       'H', 1 ),
    Reg(0x07, "AtRateOK",                   "bool",      'H', 1 ),
    Reg(0x08, "Temperature",                "0.1K",      'H', 1 ),
    Reg(0x09, "Voltage",                    "mV",        'H', 1 ),
    Reg(0x0A, "Current",                    "mA",        'h', 1 ),
    Reg(0x0B, "AverageCurrent",             "mA",        'h', 1 ),
    Reg(0x0C, "MaxError",                   "%",         'H', 1 ),
    Reg(0x0D, "RelativeStateOfCharge",      "%",         'H', 1 ),
    Reg(0x0E, "AbsoluteStateOfCharge",      "%",         'H', 1 ),
    Reg(0x0F, "RemainingCapacity",          "mAh",       'H', 1 ),
    Reg(0x10, "FullChargeCapacity",         "mAh",       'H', 1 ),
    Reg(0x11, "RunTimeToEmpty",             "min",       'H', 1 ),
    Reg(0x12, "AverageTimeToEmpty",         "min",       'H', 1 ),
    Reg(0x13, "AverageTimeToFull",          "min",       'H', 1 ),
    Reg(0x14, "ChargingCurrent",            "mA",        'H', 1 ),
    Reg(0x15, "ChargingVoltage",            "mV",        'H', 1 ),
    Reg(0x16, "BatteryStatus",              "flags",     'H', 1 ),
    Reg(0x17, "CycleCount",                 "cycles",    'H', 1 ),
    Reg(0x18, "DesignCapacity",             "mAh",       'H', 1 ),
    Reg(0x19, "DesignVoltage",              "mV",        'H', 1 ),
    Reg(0x1A, "SpecificationInfo",          "",          'H', 1 ),
    Reg(0x1B, "ManufactureDate",            "packed",    'H', 1 ),
    Reg(0x1C, "SerialNumber",               "",          'H', 1 ),
    Reg(0x20, "ManufacturerName",           "",          's', 32),
    Reg(0x21, "DeviceName",                 "",          's', 32),
    Reg(0x22, "DeviceChemistry",            "",          's', 32),
    Reg(0x23, "ManufacturerData",           "",          's', 32),
]

# ---------------------------------------------------------------
# Control and configuration registers  0x40 — 0x6F
# ---------------------------------------------------------------
CTRL_REGISTERS = [
    # System control
    Reg(0x40, "Configuration",                  "flags",    'H', 1),
    Reg(0x41, "MainControl",                    "flags",    'H', 1),
    Reg(0x42, "PackCapacity",                   "mAh",      'I', 1),
    Reg(0x43, "MaxPackVoltage",                 "mV",       'H', 1),
    Reg(0x44, "MinPackVoltage",                 "mV",       'H', 1),

    # Calibration
    Reg(0x45, "CurrentSensorOffset",            "",         'f', 1),
    Reg(0x46, "CurrentSensorGain",              "",         'f', 1),
    Reg(0x47, "VoltageOffset",                  "",         'f', 7),
    Reg(0x48, "VoltageGain",                    "",         'f', 7),

    # NTC constants
    Reg(0x49, "NTC_Beta",                       "",         'f', 1),
    Reg(0x4A, "NTC_R_Nominal",                  "Ohm",      'f', 1),
    Reg(0x4B, "NTC_R_Fixed",                    "Ohm",      'f', 1),
    Reg(0x4C, "NTC_T_Nominal",                  "K",        'f', 1),
    Reg(0x4D, "TemperatureOffset",              "C",        'f', 1),

    # Voltage protection
    Reg(0x60, "UVP_SlowThreshold",              "mV",       'H', 1),
    Reg(0x61, "UVP_SlowTime",                   "ms",       'H', 1),
    Reg(0x62, "UVP_FastThreshold",              "mV",       'H', 1),
    Reg(0x63, "UVP_FastTime",                   "ms",       'H', 1),
    Reg(0x64, "OVP_SlowThreshold",              "mV",       'H', 1),
    Reg(0x65, "OVP_SlowTime",                   "ms",       'H', 1),
    Reg(0x66, "OVP_FastThreshold",              "mV",       'H', 1),
    Reg(0x67, "OVP_FastTime",                   "ms",       'H', 1),

    # Current protection
    Reg(0x68, "OCP_ChargeThreshold",            "mA",       'H', 1),
    Reg(0x69, "OCP_ChargeTime",                 "ms",       'H', 1),
    Reg(0x6A, "OCP_DischargeSlowThreshold",     "mA",       'H', 1),
    Reg(0x6B, "OCP_DischargeSlowTime",          "ms",       'H', 1),
    Reg(0x6C, "OCP_DischargeFastThreshold",     "mA",       'H', 1),
    Reg(0x6D, "OCP_DischargeFastTime",          "ms",       'H', 1),

    # Temperature protection
    Reg(0x6E, "OTP_Threshold",                  "C",        'H', 1),
    Reg(0x6F, "OTP_Time",                       "ms",       'H', 1),
]

# ---------------------------------------------------------------
# Analog measurement registers  0x80 — 0x82
# ---------------------------------------------------------------
ANALOG_REGISTERS = [
    Reg(0x80, "FETStatus",                      "flags",    'H', 1),
    Reg(0x81, "MainVddVoltage",                 "mV",       'f', 1),
    Reg(0x82, "TemperatureSTM32",               "C",        'f', 1),
]

# ---------------------------------------------------------------
# Fault registers  0x90 — 0x99
# ---------------------------------------------------------------
FAULT_REGISTERS = [
    Reg(0x90, "FaultSnapshotVoltage",           "mV",       'H', 7),
    Reg(0x91, "FaultSnapshotCurrent",           "mA",       'h', 1),
    Reg(0x92, "FaultSnapshotTemperature",       "C",        'B', 1),
    Reg(0x93, "FaultSnapshotSoC",               "%",        'B', 1),
    Reg(0x94, "FaultCode",                      "",         'B', 8),
    Reg(0x95, "FaultTimestamp",                 "Unix",     'I', 8),
    Reg(0x96, "CellBalancingEnergy",            "mWh",      'H', 7),
    Reg(0x97, "CellBalancingTime",              "min",      'H', 7),
    Reg(0x98, "CellDeepestDischarge",           "%",        'B', 7),
    Reg(0x99, "CellMaxTemperature",             "C",        'B', 7),
]

# ---------------------------------------------------------------
# Hardware configuration registers  0xA0 — 0xA9
# ---------------------------------------------------------------
HW_REGISTERS = [
    Reg(0xA0, "CellVoltageResistanceFactor",    "",         'f', 1),
    Reg(0xA1, "BattVoltageResistanceFactor",    "",         'f', 1),
    Reg(0xA2, "ShuntResistance",                "mOhm",     'f', 1),
    Reg(0xA3, "CurrentSenseOffsetMv",           "mV",       'f', 1),
    Reg(0xA4, "CurrentSenseGain",               "",         'f', 1),
    Reg(0xA5, "BalancerResistor",               "mOhm",     'I', 1),
    Reg(0xA6, "FirmwareVersion",                "",         's', 32),
    Reg(0xA7, "HardwareVersion",                "",         's', 32),
    Reg(0xA8, "LastCommunicationTimestamp",     "Unix",     'I', 1),
    Reg(0xA9, "UptimeCounter",                  "s",        'I', 1),
]

# ---------------------------------------------------------------
# Fuel gauge registers  0xB0 — 0xB5
# ---------------------------------------------------------------
FUEL_REGISTERS = [
    Reg(0xB0, "CellVoltage",                    "mV",       'f', 7),
    Reg(0xB1, "CellSoC",                        "%",        'B', 7),
    Reg(0xB2, "CellSoH",                        "%",        'B', 7),
    Reg(0xB3, "CellRemainingCapacity",          "mAh",      'H', 7),
    Reg(0xB4, "CellSelfDischarge",              "mAh/month",'H', 7),
    Reg(0xB5, "CellQmax",                       "mAh",      'H', 7),
]

# ---------------------------------------------------------------
# ECM / Kalman registers  0xB6 — 0xDC
# ---------------------------------------------------------------
ECM_REGISTERS = [
    # OCV / ECM tables
    Reg(0xB6, "SOC_Grid",               "%",        'f', 20),
    Reg(0xB7, "OCV_Discharge",          "V",        'f', 20),
    Reg(0xB8, "OCV_Charge",             "V",        'f', 20),
    Reg(0xB9, "R0_Discharge",           "Ohm",      'f', 20),
    Reg(0xBA, "R1_Discharge",           "Ohm",      'f', 20),
    Reg(0xBB, "Tau1_Discharge",         "s",        'f', 20),
    Reg(0xBC, "R2_Discharge",           "Ohm",      'f', 20),
    Reg(0xBD, "Tau2_Discharge",         "s",        'f', 20),
    Reg(0xBE, "R0_Charge",              "Ohm",      'f', 20),
    Reg(0xBF, "R1_Charge",              "Ohm",      'f', 20),
    Reg(0xC0, "Tau1_Charge",            "s",        'f', 20),
    Reg(0xC1, "R2_Charge",              "Ohm",      'f', 20),
    Reg(0xC2, "Tau2_Charge",            "s",        'f', 20),

    # Capacity model
    Reg(0xC3, "Q_Nom_TempSetpoints",    "C",        'f', 5 ),
    Reg(0xC4, "Q_Nom_TempCapacity",     "Ah",       'f', 5 ),
    Reg(0xC5, "Q_Nom",                  "Ah",       'f', 1 ),
    Reg(0xC6, "CoulombicEfficiency",    "",         'f', 1 ),

    # Reference parameters
    Reg(0xC7, "R0_Ref",                 "Ohm",      'f', 1 ),
    Reg(0xC8, "R1_Ref",                 "Ohm",      'f', 1 ),
    Reg(0xC9, "Tau1_Ref",               "s",        'f', 1 ),
    Reg(0xCA, "R2_Ref",                 "Ohm",      'f', 1 ),
    Reg(0xCB, "Tau2_Ref",               "s",        'f', 1 ),

    # Activation energies
    Reg(0xCC, "Ea_R0",                  "J/mol",    'f', 1 ),
    Reg(0xCD, "Ea_R1",                  "J/mol",    'f', 1 ),
    Reg(0xCE, "Ea_Tau1",                "J/mol",    'f', 1 ),
    Reg(0xCF, "Ea_R2",                  "J/mol",    'f', 1 ),
    Reg(0xD0, "Ea_Tau2",                "J/mol",    'f', 1 ),

    # Kalman noise parameters
    Reg(0xD1, "KF_Q_SOC",               "",         'f', 1 ),
    Reg(0xD2, "KF_Q_RC1",               "",         'f', 1 ),
    Reg(0xD3, "KF_Q_RC2",               "",         'f', 1 ),
    Reg(0xD4, "KF_R_V",                 "V2",       'f', 1 ),

    # Per-cell Kalman state (read-only)
    Reg(0xD5, "Cell_SOC_f",             "",         'f', 7 ),
    Reg(0xD6, "Cell_VRC1",              "V",        'f', 7 ),
    Reg(0xD7, "Cell_VRC2",              "V",        'f', 7 ),
    Reg(0xD8, "Cell_Covariance",        "",         'f', 42),  # cell_p[7][6]

    # Per-cell aging state (read-only)
    Reg(0xD9, "Cell_Q_Nom",             "Ah",       'f', 7 ),
    Reg(0xDA, "Cell_R0_Scale",          "",         'f', 7 ),
    Reg(0xDC, "LearningStatus",         "flags",    'H', 1 ),
]


# ---------------------------------------------------------------
# Checksum — sum of all bytes modulo 256
# ---------------------------------------------------------------
def checksum(data):
    return sum(data) % 256


# ---------------------------------------------------------------
# Build binary read command — always 4 bytes
# [CMD_READ][address][0x00][crc]
# ---------------------------------------------------------------
def build_read_command(address):
    packet = bytes([CMD_READ, address, 0x00])
    return packet + bytes([checksum(packet)])


# ---------------------------------------------------------------
# Receive one binary packet from the device
#
# All packets are at least 4 bytes:
#   [cmd][byte1][length][data_0 ... data_n][crc]
#   total = length + 4
#
# Returns raw bytes, or None on timeout / short read.
# ---------------------------------------------------------------
def receive_packet(ser):
    header = ser.read(3)
    if len(header) < 3:
        return None

    payload_length = header[2]

    rest = ser.read(payload_length + 1)
    if len(rest) < payload_length + 1:
        return None

    return header + rest


# ---------------------------------------------------------------
# Validate CRC of a raw packet
# CRC covers all bytes except the last one (the CRC itself)
# ---------------------------------------------------------------
def validate_crc(raw):
    return checksum(raw[:-1]) == raw[-1]


# ---------------------------------------------------------------
# Parse and validate a read response against a register definition
# Returns (value_str, error_str) — one of the two is always None
# ---------------------------------------------------------------
def parse_read_response(raw, reg):
    if raw is None:
        return None, "TIMEOUT"

    if len(raw) < 4:
        return None, "RESPONSE TOO SHORT"

    if not validate_crc(raw):
        crc_calc = checksum(raw[:-1])
        return None, f"CRC MISMATCH (calc={crc_calc} recv={raw[-1]})"

    cmd          = raw[0]
    resp_address = raw[1]
    payload_len  = raw[2]
    data         = raw[3:3 + payload_len]

    if cmd != CMD_READ:
        return None, f"UNEXPECTED CMD 0x{cmd:02X}"

    if resp_address != reg.address:
        return None, f"ADDRESS MISMATCH (exp=0x{reg.address:02X} got=0x{resp_address:02X})"

    # ---- string / char array ----
    if reg.fmt == 's':
        text = data[:reg.count].rstrip(b'\x00').decode('ascii', errors='replace').strip()
        return text, None

    # ---- scalar / array ----
    elem_size     = struct.calcsize('<' + reg.fmt)
    expected_size = reg.count * elem_size

    if len(data) < expected_size:
        return None, f"DATA TOO SHORT (exp={expected_size} got={len(data)})"

    values  = [struct.unpack_from('<' + reg.fmt, data, i * elem_size)[0]
               for i in range(reg.count)]
    fmt_val = (lambda v: f"{v:.5f}") if reg.fmt == 'f' else str

    if reg.count == 1:
        return fmt_val(values[0]), None

    return ', '.join(fmt_val(v) for v in values), None


# ---------------------------------------------------------------
# Print a section header
# ---------------------------------------------------------------
def print_header(title):
    print(f"\n{'=' * 84}")
    print(f"  {title}")
    print(f"{'=' * 84}")
    print(f"{'Address':<10} {'Register':<38} {'Value':<26} Unit")
    print(f"{'-'*8:<10} {'-'*36:<38} {'-'*24:<26} {'-'*10}")


# ---------------------------------------------------------------
# Read and print a list of registers
# Long arrays (> 7 elements) are folded onto continuation lines
# ---------------------------------------------------------------
def read_registers(ser, registers, errors):
    for reg in registers:
        ser.reset_input_buffer()
        ser.write(build_read_command(reg.address))

        raw          = receive_packet(ser)
        value, error = parse_read_response(raw, reg)

        if error:
            print(f"0x{reg.address:02X}       {reg.name:<38} ERR: {error}")
            errors += 1
        else:
            if reg.count > 7 and ',' in value:
                parts  = [v.strip() for v in value.split(',')]
                chunks = [parts[i:i + 7] for i in range(0, len(parts), 7)]
                print(f"0x{reg.address:02X}       {reg.name:<38} {', '.join(chunks[0]):<26} {reg.unit}")
                for chunk in chunks[1:]:
                    print(f"{'':10} {'':38} {', '.join(chunk)}")
            else:
                print(f"0x{reg.address:02X}       {reg.name:<38} {value:<26} {reg.unit}")

    return errors


# ---------------------------------------------------------------
# Main
# ---------------------------------------------------------------
def main():
    if len(sys.argv) != 2:
        print("Usage: python read_data.py <COM_PORT>")
        print("Example: python read_data.py COM3")
        sys.exit(1)

    port = sys.argv[1]

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT_RESPONSE)
        time.sleep(0.3)
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(f"ERR: failed to open {port}: {e}")
        sys.exit(1)

    print(f"Connected to {port} at {BAUD_RATE} baud")

    all_groups = [
        ("SBS v1.1 Registers  (0x00 - 0x23)",                SBS_REGISTERS  ),
        ("Control & Configuration  (0x40 - 0x6F)",           CTRL_REGISTERS ),
        ("Analog Measurements  (0x80 - 0x82)",               ANALOG_REGISTERS),
        ("Fault & Lifetime History  (0x90 - 0x99)",          FAULT_REGISTERS ),
        ("Hardware Configuration  (0xA0 - 0xA9)",            HW_REGISTERS   ),
        ("Fuel Gauge  (0xB0 - 0xB5)",                        FUEL_REGISTERS ),
        ("ECM / Kalman  (0xB6 - 0xDC)",                      ECM_REGISTERS  ),
    ]

    errors = 0
    total  = sum(len(regs) for _, regs in all_groups)

    try:
        for title, regs in all_groups:
            print_header(title)
            errors = read_registers(ser, regs, errors)

    except KeyboardInterrupt:
        print("\nAborted by user.")

    finally:
        ser.close()

    print(f"\n{'=' * 84}")
    print(f"Done.  {total} registers queried, {errors} errors.")


if __name__ == "__main__":
    main()
