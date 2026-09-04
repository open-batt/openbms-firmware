#!/usr/bin/env python3
"""
read_all_registers.py
======================
Reads and prints every register in the OpenBMS UART register map -- a full
dump for a quick "is the board alive and sane" check. Register addresses,
names, types and element counts below are kept in sync with the register
map in communication-protocol.md; if that file changes, update the three
lists below (PERIPH_REGISTERS / CTRL_REGISTERS / FUEL_REGISTERS) to match.

Run:
    python read_all_registers.py <COM_PORT>
    python read_all_registers.py COM3

(Renamed from read_data.py -- same tool, name now describes what it does.)
"""

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
#        'f'  float32   'Q'  uint64
#        's'  char string  (count = total byte length of string)
# count: number of elements  (for 's': byte length of the char array)
# ---------------------------------------------------------------
Reg = namedtuple('Reg', ['address', 'name', 'unit', 'fmt', 'count'])

# ---------------------------------------------------------------
# Peripheral_Data_t  — 0x00 — 0x12
# ---------------------------------------------------------------
PERIPH_REGISTERS = [
    # Measurements (read-only)
    Reg(0x00, "CellVoltage",                    "mV",       'f', 7 ),
    Reg(0x01, "CellVoltageFiltered",            "mV",       'f', 7 ),
    Reg(0x02, "PackVoltage",                    "mV",       'f', 1 ),
    Reg(0x03, "PackVoltageFiltered",            "mV",       'f', 1 ),
    Reg(0x04, "PackCurrent",                    "mA",       'f', 1 ),
    Reg(0x05, "PackCurrentFiltered",            "mA",       'f', 1 ),
    Reg(0x06, "TemperaturePackage",             "C",        'f', 1 ),
    Reg(0x07, "TemperatureSTM32",               "C",        'f', 1 ),
    Reg(0x08, "MainVddVoltage",                 "mV",       'f', 1 ),
    Reg(0x09, "FETStatus",                      "flags",    'H', 1 ),
    # Calibration (read-write)
    Reg(0x0A, "CurrentSensorOffset",            "",         'f', 1 ),
    Reg(0x0B, "CurrentSensorGain",              "",         'f', 1 ),
    Reg(0x0C, "VoltageOffset",                  "",         'f', 7 ),
    Reg(0x0D, "VoltageGain",                    "",         'f', 7 ),
    Reg(0x0E, "NTC_Beta",                       "",         'f', 1 ),
    Reg(0x0F, "NTC_R_Nominal",                  "Ohm",      'f', 1 ),
    Reg(0x10, "NTC_R_Fixed",                    "Ohm",      'f', 1 ),
    Reg(0x11, "NTC_T_Nominal",                  "K",        'f', 1 ),
    Reg(0x12, "TemperatureOffset",              "C",        'f', 1 ),
]

# ---------------------------------------------------------------
# Control_Data_t  — 0x30 — 0x57
# ---------------------------------------------------------------
CTRL_REGISTERS = [
    # System control
    Reg(0x30, "Configuration",                  "flags",    'H', 1 ),
    Reg(0x31, "MainControl",                    "flags",    'H', 1 ),
    Reg(0x32, "CellCapacity",                   "mAh",      'I', 1 ),
    Reg(0x33, "MaxCellVoltage",                 "mV",       'H', 1 ),
    Reg(0x34, "MinCellVoltage",                 "mV",       'H', 1 ),
    Reg(0x35, "ChargingTerminationCurrent",     "mA",       'H', 1 ),
    # Voltage protection
    Reg(0x36, "UVP_SlowThreshold",              "mV",       'H', 1 ),
    Reg(0x37, "UVP_SlowTime",                   "ms",       'H', 1 ),
    Reg(0x38, "UVP_FastThreshold",              "mV",       'H', 1 ),
    Reg(0x39, "UVP_FastTime",                   "ms",       'H', 1 ),
    Reg(0x3A, "OVP_SlowThreshold",              "mV",       'H', 1 ),
    Reg(0x3B, "OVP_SlowTime",                   "ms",       'H', 1 ),
    Reg(0x3C, "OVP_FastThreshold",              "mV",       'H', 1 ),
    Reg(0x3D, "OVP_FastTime",                   "ms",       'H', 1 ),
    # Current protection
    Reg(0x3E, "OCP_ChargeThreshold",            "mA",       'H', 1 ),
    Reg(0x3F, "OCP_ChargeTime",                 "ms",       'H', 1 ),
    Reg(0x40, "OCP_DischargeSlowThreshold",     "mA",       'H', 1 ),
    Reg(0x41, "OCP_DischargeSlowTime",          "ms",       'H', 1 ),
    Reg(0x42, "OCP_DischargeFastThreshold",     "mA",       'H', 1 ),
    Reg(0x43, "OCP_DischargeFastTime",          "ms",       'H', 1 ),
    # Temperature protection
    Reg(0x44, "OTP_Threshold",                  "C",        'H', 1 ),
    Reg(0x45, "OTP_Time",                       "ms",       'H', 1 ),
    # Fault registers (read-only)
    Reg(0x46, "FaultSnapshotVoltage",           "mV",       'H', 7 ),
    Reg(0x47, "FaultSnapshotCurrent",           "mA",       'h', 1 ),
    Reg(0x48, "FaultSnapshotTemperature",       "C",        'B', 1 ),
    Reg(0x49, "FaultSnapshotSoC",               "%",        'B', 1 ),
    Reg(0x4A, "FaultCode",                      "",         'B', 8 ),
    Reg(0x4B, "FaultTimestamp",                 "Unix",     'I', 8 ),
    # Lifetime history (read-only)
    Reg(0x4C, "CellBalancingEnergy",            "mWh",      'H', 7 ),
    Reg(0x4D, "CellBalancingTime",              "min",      'H', 7 ),
    Reg(0x4E, "CellDeepestDischarge",           "%",        'B', 7 ),
    Reg(0x4F, "CellMaxTemperature",             "C",        'B', 7 ),
    # System info (read-only)
    Reg(0x50, "LastCommunicationTimestamp",     "ms",       'I', 1 ),
    Reg(0x51, "UptimeCounter",                  "ms",       'Q', 1 ),
    Reg(0x52, "FirmwareVersion",                "",         's', 32),
    Reg(0x53, "HardwareVersion",                "",         's', 32),
    Reg(0x54, "ManufacturerName",               "",         's', 32),
    Reg(0x55, "DeviceName",                     "",         's', 32),
    Reg(0x56, "DeviceChemistry",                "",         's', 32),
    Reg(0x57, "ManufacturerData",               "",         's', 32),
]

# ---------------------------------------------------------------
# FuelGauge_Data_t  — 0x80 — 0xAC
# ---------------------------------------------------------------
FUEL_REGISTERS = [
    # SoC / SoH (read-only)
    Reg(0x80, "RelativeSoC",                    "%",        'B', 1 ),
    Reg(0x81, "CellSoC",                        "%",        'B', 7 ),
    Reg(0x82, "CellSoH",                        "%",        'B', 7 ),
    Reg(0x83, "CellRemainingCapacity",          "mAh",      'H', 7 ),
    Reg(0x84, "CellSelfDischarge",              "mAh/month",'H', 7 ),
    Reg(0x85, "CellQmax",                       "mAh",      'H', 7 ),
    # OCV / ECM tables
    Reg(0x86, "SOC_Grid",                       "%",        'f', 20),
    Reg(0x87, "OCV_Discharge",                  "V",        'f', 20),
    Reg(0x88, "OCV_Charge",                     "V",        'f', 20),
    Reg(0x89, "R0_Discharge",                   "Ohm",      'f', 20),
    Reg(0x8A, "R1_Discharge",                   "Ohm",      'f', 20),
    Reg(0x8B, "Tau1_Discharge",                 "s",        'f', 20),
    Reg(0x8C, "R2_Discharge",                   "Ohm",      'f', 20),
    Reg(0x8D, "Tau2_Discharge",                 "s",        'f', 20),
    Reg(0x8E, "R0_Charge",                      "Ohm",      'f', 20),
    Reg(0x8F, "R1_Charge",                      "Ohm",      'f', 20),
    Reg(0x90, "Tau1_Charge",                    "s",        'f', 20),
    Reg(0x91, "R2_Charge",                      "Ohm",      'f', 20),
    Reg(0x92, "Tau2_Charge",                    "s",        'f', 20),
    # Capacity model
    Reg(0x93, "Q_Nom_TempSetpoints",            "C",        'f', 5 ),
    Reg(0x94, "Q_Nom_TempCapacity",             "Ah",       'f', 5 ),
    Reg(0x95, "Q_Nom",                          "Ah",       'f', 1 ),
    Reg(0x96, "CoulombicEfficiency",            "",         'f', 1 ),
    # Reference parameters
    Reg(0x97, "R0_Ref",                         "Ohm",      'f', 1 ),
    Reg(0x98, "R1_Ref",                         "Ohm",      'f', 1 ),
    Reg(0x99, "Tau1_Ref",                       "s",        'f', 1 ),
    Reg(0x9A, "R2_Ref",                         "Ohm",      'f', 1 ),
    Reg(0x9B, "Tau2_Ref",                       "s",        'f', 1 ),
    # Activation energies
    Reg(0x9C, "Ea_R0",                          "J/mol",    'f', 1 ),
    Reg(0x9D, "Ea_R1",                          "J/mol",    'f', 1 ),
    Reg(0x9E, "Ea_Tau1",                        "J/mol",    'f', 1 ),
    Reg(0x9F, "Ea_R2",                          "J/mol",    'f', 1 ),
    Reg(0xA0, "Ea_Tau2",                        "J/mol",    'f', 1 ),
    # Kalman noise parameters
    Reg(0xA1, "KF_Q_SOC",                       "",         'f', 1 ),
    Reg(0xA2, "KF_Q_RC1",                       "",         'f', 1 ),
    Reg(0xA3, "KF_Q_RC2",                       "",         'f', 1 ),
    Reg(0xA4, "KF_R_V",                         "V2",       'f', 1 ),
    # Per-cell Kalman state (read-only)
    Reg(0xA5, "Cell_SOC_f",                     "",         'f', 7 ),
    Reg(0xA6, "Cell_VRC1",                      "V",        'f', 7 ),
    Reg(0xA7, "Cell_VRC2",                      "V",        'f', 7 ),
    Reg(0xA8, "Cell_Covariance",                "",         'f', 42),  # cell_p[7][6]
    # Per-cell aging state (read-only)
    Reg(0xA9, "Cell_Q_Nom",                     "Ah",       'f', 7 ),
    Reg(0xAA, "Cell_R0_Scale",                  "",         'f', 7 ),
    Reg(0xAB, "CycleCount",                     "cycles",   'H', 1 ),
    Reg(0xAC, "LearningStatus",                 "flags",    'H', 1 ),
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
        print("Usage: python read_all_registers.py <COM_PORT>")
        print("Example: python read_all_registers.py COM3")
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
        ("Peripheral Data  (0x00 - 0x12)",       PERIPH_REGISTERS),
        ("Control Data     (0x30 - 0x57)",        CTRL_REGISTERS  ),
        ("Fuel Gauge Data  (0x80 - 0xAC)",        FUEL_REGISTERS  ),
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
