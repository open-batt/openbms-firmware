#!/usr/bin/env python3
"""
read_all_registers.py
======================
Reads and prints every register in the OpenBMS UART register map -- a full
dump for a quick "is the board alive and sane" check. Register addresses,
names, types and element counts below are kept in sync with the register
map in communication-protocol.md; if that file changes, update the lists
below (one <BANK>_REGISTERS list per register bank) to match.

Run:
    python read_all_registers.py            (finds the board by pinging every serial port)
    python read_all_registers.py <COM_PORT> (uses exactly that port)
    python read_all_registers.py COM3

(Renamed from read_data.py -- same tool, name now describes what it does.)
"""

import sys
import serial
import time
import struct
from collections import namedtuple
from serial.tools import list_ports

# ---------------------------------------------------------------
# Protocol constants (see openbms_comm.h)
# ---------------------------------------------------------------
BAUD_RATE        = 230400
TIMEOUT_RESPONSE = 1.0
TIMEOUT_PING     = 1.0     # s -- one ping
TIMEOUT_READY    = 5.0     # s -- how long to wait for the board after opening the port
PING_TRIES       = 2       # pings per port while looking for the board
OPEN_SETTLE      = 0.5     # s -- pause after opening a port (adapters / boards need a moment)

CMD_WRITE = 0x01
CMD_READ  = 0x02
CMD_ACK   = 0x03
CMD_ERROR = 0x04
CMD_PING  = 0x0A

CELL_COUNT       = 7       # cells in the pack (BD_CELL_COUNT_MAX)
REG_MODEL_CELL   = 0x9F    # picks the cell the per-cell model registers (0xA1 - 0xAC, 0xD4) act on

ERROR_NAMES = {
    0: "CE_OK",
    1: "CE_WRONG_CMD",
    2: "CE_BAD_CRC",
    3: "CE_NO_REG",
    4: "CE_RO",
    5: "CE_EEPROM",
    6: "CE_STATE",
}

# ---------------------------------------------------------------
# Packet layout (all directions, always >= 4 bytes):
#
#   Read command  (host -> device):
#       [0x02][address][0x00][crc]
#
#   Read response (device -> host):
#       [0x02][address][length][data_0 ... data_n][crc]
#       total = length + 4 bytes
#
#   Ping command  (host -> device):
#       [0x0A][0x00][0x00][crc]
#
#   ACK          (device -> host):
#       [0x03][0x00][0x00][crc]
#       total = 4 bytes
#
#   Error        (device -> host):
#       [0x04][0x00][0x01][error code][crc]
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
# Peripheral live readings (Peripheral_Data_t)  — 0x00 - 0x0F
# ---------------------------------------------------------------
PERIPH_REGISTERS = [
    Reg(0x00, "CellVoltage",                           "mV",        'f', 7  ),
    Reg(0x01, "PackVoltage",                           "mV",        'f', 1  ),
    Reg(0x02, "PackCurrent",                           "mA",        'f', 1  ),
    Reg(0x03, "TemperaturePackage",                    "C",         'f', 1  ),
    Reg(0x04, "TemperatureSTM32",                      "C",         'f', 1  ),
    Reg(0x05, "MainVddVoltage",                        "mV",        'f', 1  ),
    Reg(0x06, "FETStatus",                             "flags",     'H', 1  ),
]

# ---------------------------------------------------------------
# System state (System_Data_t)  — 0x10 - 0x1F
# ---------------------------------------------------------------
SYSTEM_REGISTERS = [
    Reg(0x10, "MainControl",                           "flags",     'H', 1  ),
    Reg(0x11, "LastCommunicationTimestamp",            "ms",        'I', 1  ),
    Reg(0x12, "ProtectionTrigger",                     "flags",     'B', 1  ),
    Reg(0x13, "ControlState",                          "state",     'B', 1  ),
    Reg(0x15, "ErrorFlags",                            "flags",     'Q', 1  ),
    Reg(0x17, "SessionUptime",                         "ms",        'I', 1  ),
    Reg(0x18, "HppcDirection",                         "",          'B', 1  ),
    Reg(0x19, "HppcStatus",                            "flags",     'H', 1  ),
    Reg(0x1A, "HppcExtractStatus",                     "flags",     'H', 1  ),
]

# ---------------------------------------------------------------
# Calibration offsets, measured at every boot (Calibration_Data_t)  — 0x20 - 0x2F
# The gains, NTC, shunt and bias values are config registers 0x52 - 0x5A.
# ---------------------------------------------------------------
CALIBRATION_REGISTERS = [
    Reg(0x20, "CurrentSensorOffset",                   "mV",        'f', 1  ),
    Reg(0x22, "VoltageOffset",                         "mV",        'f', 7  ),
]

# ---------------------------------------------------------------
# Config (Config_Data_t)  — 0x30 - 0x5F
# ---------------------------------------------------------------
CONFIG_REGISTERS = [
    Reg(0x30, "CellCount",                             "cells",     'B', 1  ),
    Reg(0x31, "UseI2C",                                "0/1",       'B', 1  ),
    Reg(0x32, "UseCAN",                                "0/1",       'B', 1  ),
    Reg(0x33, "UseUART",                               "0/1",       'B', 1  ),
    Reg(0x34, "VoltageProtection",                     "0/1",       'B', 1  ),
    Reg(0x35, "CurrentProtection",                     "0/1",       'B', 1  ),
    Reg(0x36, "TemperatureProtection",                 "0/1",       'B', 1  ),
    Reg(0x37, "CellCapacity",                          "mAh",       'I', 1  ),
    Reg(0x38, "MaxCellVoltage",                        "mV",        'H', 1  ),
    Reg(0x39, "MinCellVoltage",                        "mV",        'H', 1  ),
    Reg(0x3A, "ChargingTerminationCurrent",            "mA",        'H', 1  ),
    Reg(0x3B, "UVP_SlowThreshold",                     "mV",        'H', 1  ),
    Reg(0x3C, "UVP_SlowTime",                          "ms",        'H', 1  ),
    Reg(0x3D, "UVP_FastThreshold",                     "mV",        'H', 1  ),
    Reg(0x3E, "UVP_FastTime",                          "ms",        'H', 1  ),
    Reg(0x3F, "OVP_SlowThreshold",                     "mV",        'H', 1  ),
    Reg(0x40, "OVP_SlowTime",                          "ms",        'H', 1  ),
    Reg(0x41, "OVP_FastThreshold",                     "mV",        'H', 1  ),
    Reg(0x42, "OVP_FastTime",                          "ms",        'H', 1  ),
    Reg(0x43, "OCP_ChargeThreshold",                   "mA",        'H', 1  ),
    Reg(0x44, "OCP_ChargeTime",                        "ms",        'H', 1  ),
    Reg(0x45, "OCP_DischargeSlowThreshold",            "mA",        'H', 1  ),
    Reg(0x46, "OCP_DischargeSlowTime",                 "ms",        'H', 1  ),
    Reg(0x47, "OCP_DischargeFastThreshold",            "mA",        'H', 1  ),
    Reg(0x48, "OCP_DischargeFastTime",                 "ms",        'H', 1  ),
    Reg(0x49, "OTP_Threshold",                         "C",         'H', 1  ),
    Reg(0x4A, "OTP_Time",                              "ms",        'H', 1  ),
    Reg(0x4B, "ChargeStopCellVoltage",                 "mV",        'H', 1  ),
    Reg(0x4C, "ChargeMonitorCellVoltage",              "mV",        'H', 1  ),
    Reg(0x4D, "UTP_Threshold",                         "C",         'h', 1  ),
    Reg(0x4E, "UTP_Time",                              "ms",        'H', 1  ),
    Reg(0x4F, "OTP_ChargeThreshold",                   "C",         'h', 1  ),
    Reg(0x50, "OCV_VoltageLow",                        "mV",        'H', 1  ),
    Reg(0x51, "OCV_VoltageHigh",                       "mV",        'H', 1  ),
    Reg(0x52, "CurrentSensorGain",                     "",          'f', 1  ),
    Reg(0x53, "VoltageGain",                           "",          'f', 7  ),
    Reg(0x54, "NTC_Beta",                              "",          'f', 1  ),
    Reg(0x55, "NTC_R_Nominal",                         "Ohm",       'f', 1  ),
    Reg(0x56, "NTC_R_Fixed",                           "Ohm",       'f', 1  ),
    Reg(0x57, "NTC_T_Nominal",                         "K",         'f', 1  ),
    Reg(0x58, "TemperatureOffset",                     "C",         'f', 1  ),
    Reg(0x59, "ShuntResistance",                       "mOhm",      'f', 1  ),
    Reg(0x5A, "CurrentBias",                           "mA",        'f', 1  ),
]

# ---------------------------------------------------------------
# Device info (Config_Data_t, written by the host)  — 0x60 - 0x6F
# ---------------------------------------------------------------
DEVICE_INFO_REGISTERS = [
    Reg(0x60, "FirmwareVersion",                       "",          's', 32 ),
    Reg(0x61, "HardwareVersion",                       "",          's', 32 ),
    Reg(0x62, "ManufacturerName",                      "",          's', 32 ),
    Reg(0x63, "DeviceName",                            "",          's', 32 ),
    Reg(0x64, "DeviceChemistry",                       "",          's', 32 ),
    Reg(0x65, "ManufacturerData",                      "",          's', 32 ),
]

# ---------------------------------------------------------------
# Stats (Stats_Data_t)  — 0x70 - 0x7F
# ---------------------------------------------------------------
STATS_REGISTERS = [
    Reg(0x70, "FaultSnapshotVoltage",                  "mV",        'H', 7  ),
    Reg(0x71, "FaultSnapshotCurrent",                  "mA",        'h', 1  ),
    Reg(0x72, "FaultSnapshotTemperature",              "C",         'B', 1  ),
    Reg(0x73, "FaultSnapshotSoC",                      "%",         'B', 1  ),
    Reg(0x74, "FaultCode",                             "",          'B', 8  ),
    Reg(0x75, "FaultTimestamp",                        "Unix",      'I', 8  ),
    Reg(0x76, "CellBalancingEnergy",                   "mWh",       'H', 7  ),
    Reg(0x77, "CellBalancingTime",                     "min",       'H', 7  ),
    Reg(0x78, "CellDeepestDischarge",                  "%",         'B', 7  ),
    Reg(0x79, "CellMaxTemperature",                    "C",         'B', 7  ),
]

# ---------------------------------------------------------------
# Fuel gauge outputs  — 0x90 - 0x9F
# ---------------------------------------------------------------
FUEL_OUT_REGISTERS = [
    Reg(0x90, "RelativeSoC",                           "%",         'B', 1  ),
    Reg(0x91, "AbsoluteSoC",                           "%",         'B', 1  ),
    Reg(0x92, "CellSoC",                               "%",         'B', 7  ),
    Reg(0x93, "CellSoH",                               "%",         'B', 7  ),
    Reg(0x94, "CellRemainingCapacity",                 "mAh",       'H', 7  ),
    Reg(0x95, "CellSelfDischarge",                     "mAh/month", 'H', 7  ),
    Reg(0x96, "LearningStatus",                        "flags",     'H', 1  ),
]

# ---------------------------------------------------------------
# Fuel gauge model  — 0xA0 - 0xCF
# ---------------------------------------------------------------
# Shared by all cells (0xA0, 0xAD - 0xBE)
FUEL_MODEL_REGISTERS = [
    Reg(0xA0, "SOC_Grid",                              "%",         'f', 20 ),
    Reg(0xAD, "Q_Nom_TempSetpoints",                   "C",         'f', 5  ),
    Reg(0xAE, "Q_Nom_TempCapacity",                    "Ah",        'f', 5  ),
    Reg(0xAF, "Q_Nom",                                 "Ah",        'f', 1  ),
    Reg(0xB0, "CoulombicEfficiency",                   "",          'f', 1  ),
    Reg(0xB6, "Ea_R0",                                 "J/mol",     'f', 1  ),
    Reg(0xB7, "Ea_R1",                                 "J/mol",     'f', 1  ),
    Reg(0xB8, "Ea_Tau1",                               "J/mol",     'f', 1  ),
    Reg(0xB9, "Ea_R2",                                 "J/mol",     'f', 1  ),
    Reg(0xBA, "Ea_Tau2",                               "J/mol",     'f', 1  ),
    Reg(0xBB, "KF_Q_SOC",                              "",          'f', 1  ),
    Reg(0xBC, "KF_Q_RC1",                              "",          'f', 1  ),
    Reg(0xBD, "KF_Q_RC2",                              "",          'f', 1  ),
    Reg(0xBE, "KF_R_V",                                "V2",        'f', 1  ),
]

# Per cell: the same addresses show the cell picked with register 0x9F
FUEL_CELL_MODEL_REGISTERS = [
    Reg(0x9F, "ModelCell",                             "cell",      'B', 1  ),
    Reg(0xA1, "OCV_Discharge",                         "V",         'f', 20 ),
    Reg(0xA2, "OCV_Charge",                            "V",         'f', 20 ),
    Reg(0xA3, "R0_Discharge",                          "Ohm",       'f', 20 ),
    Reg(0xA4, "R1_Discharge",                          "Ohm",       'f', 20 ),
    Reg(0xA5, "Tau1_Discharge",                        "s",         'f', 20 ),
    Reg(0xA6, "R2_Discharge",                          "Ohm",       'f', 20 ),
    Reg(0xA7, "Tau2_Discharge",                        "s",         'f', 20 ),
    Reg(0xA8, "R0_Charge",                             "Ohm",       'f', 20 ),
    Reg(0xA9, "R1_Charge",                             "Ohm",       'f', 20 ),
    Reg(0xAA, "Tau1_Charge",                           "s",         'f', 20 ),
    Reg(0xAB, "R2_Charge",                             "Ohm",       'f', 20 ),
    Reg(0xAC, "Tau2_Charge",                           "s",         'f', 20 ),
    Reg(0xD4, "Cell_Capacity",                         "Ah",        'f', 1  ),
]

# ---------------------------------------------------------------
# Fuel gauge state  — 0xD0 - 0xDF
# ---------------------------------------------------------------
FUEL_STATE_REGISTERS = [
    Reg(0xD0, "Cell_SOC_f",                            "",          'f', 7  ),
    Reg(0xD1, "Cell_VRC1",                             "V",         'f', 7  ),
    Reg(0xD2, "Cell_VRC2",                             "V",         'f', 7  ),
    Reg(0xD3, "Cell_Covariance",                       "",          'f', 42 ),
    Reg(0xD6, "CellQmax",                              "mAh",       'H', 7  ),
    Reg(0xD7, "CycleCount",                            "cycles",    'H', 1  ),
]


# ---------------------------------------------------------------
# Checksum — sum of all bytes modulo 256
# ---------------------------------------------------------------
def checksum(data):
    return sum(data) % 256


# ---------------------------------------------------------------
# Build a binary command — always [cmd][address][length][data...][crc]
# ---------------------------------------------------------------
def build_command(cmd, address=0):
    packet = bytes([cmd, address, 0x00])
    return packet + bytes([checksum(packet)])


def build_read_command(address):
    return build_command(CMD_READ, address)


def build_write_command(address, payload):
    packet = bytes([CMD_WRITE, address, len(payload)]) + bytes(payload)
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

    if cmd == CMD_ERROR:
        code = raw[3]
        return None, f"BOARD ERROR {ERROR_NAMES.get(code, code)}"

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
# Pick the cell the per-cell model registers act on (register 0x9F)
# Returns None when the board accepted it, else an error text
# ---------------------------------------------------------------
def select_model_cell(ser, cell):
    ser.reset_input_buffer()
    ser.write(build_write_command(REG_MODEL_CELL, [cell]))

    raw = receive_packet(ser)
    if raw is None:
        return "TIMEOUT"
    if len(raw) < 4 or not validate_crc(raw):
        return "BAD ANSWER"
    if raw[0] == CMD_ERROR:
        return f"BOARD ERROR {ERROR_NAMES.get(raw[3], raw[3])}"
    if raw[0] != CMD_ACK:
        return f"UNEXPECTED CMD 0x{raw[0]:02X}"
    return None


# ---------------------------------------------------------------
# Port handling
# ---------------------------------------------------------------
def open_port(port):
    """Opens the port and gives the adapter / board a moment to settle."""
    ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT_RESPONSE)
    time.sleep(OPEN_SETTLE)
    ser.reset_input_buffer()
    return ser


def ping(ser):
    """Ping once. True if a valid frame comes back (an ACK, or an error frame from older firmware)."""
    ser.reset_input_buffer()
    ser.write(build_command(CMD_PING))
    ser.timeout = TIMEOUT_PING
    raw = receive_packet(ser)
    ser.timeout = TIMEOUT_RESPONSE
    if raw is None:
        return False
    return validate_crc(raw) and raw[0] in (CMD_ACK, CMD_ERROR)


def ping_port(port):
    """True if an OpenBMS answers a ping on this port."""
    try:
        with open_port(port) as ser:
            for _ in range(PING_TRIES):
                if ping(ser):
                    return True
    except (serial.SerialException, OSError):
        pass    # port busy, gone, or not a real serial port -- not our board
    return False


def find_board():
    """Pings every serial port and returns the one that answers, or None."""
    ports = sorted(p.device for p in list_ports.comports())
    if not ports:
        print("ERR: no serial ports found on this computer")
        return None

    print(f"Looking for the board on {', '.join(ports)} ...")
    found = [port for port in ports if ping_port(port)]

    if not found:
        print("ERR: no OpenBMS answered a ping on any port (is it powered and connected? "
              "Is another program using the port?). Pass the port by hand to force one.")
        return None
    if len(found) > 1:
        print(f"ERR: more than one OpenBMS answered ({', '.join(found)}). Pass the port you want.")
        return None

    print(f"Found the board on {found[0]}")
    return found[0]


def wait_until_ready(ser):
    """Ping until the board answers. Opening a port can reset some boards."""
    deadline = time.time() + TIMEOUT_READY
    while time.time() < deadline:
        if ping(ser):
            return True
    return False


# ---------------------------------------------------------------
# Main
# ---------------------------------------------------------------
def main():
    if len(sys.argv) > 2:
        print("Usage: python read_all_registers.py [COM_PORT]")
        print("Example: python read_all_registers.py COM3")
        print("Without a port the script pings every serial port and uses the one that answers.")
        sys.exit(1)

    if len(sys.argv) == 2:
        port = sys.argv[1]
    else:
        port = find_board()
        if port is None:
            sys.exit(1)

    try:
        ser = open_port(port)
    except (serial.SerialException, OSError) as e:
        print(f"ERR: failed to open {port}: {e}")
        sys.exit(1)

    print(f"Connected to {port} at {BAUD_RATE} baud")

    if not wait_until_ready(ser):
        print(f"ERR: the board did not answer a ping within {TIMEOUT_READY:.0f} s (wrong port? not powered?)")
        ser.close()
        sys.exit(1)

    # (title, registers, cell): cell is None for a plain group, or the cell to
    # pick with register 0x9F before reading the per-cell model registers
    all_groups = [
        ("Peripheral live readings (Peripheral_Data_t)  (0x00 - 0x0F)", PERIPH_REGISTERS, None),
        ("System state (System_Data_t)  (0x10 - 0x1F)", SYSTEM_REGISTERS, None),
        ("Calibration offsets, measured at boot  (0x20 - 0x2F)", CALIBRATION_REGISTERS, None),
        ("Config (Config_Data_t)  (0x30 - 0x5F)", CONFIG_REGISTERS, None),
        ("Device info (Config_Data_t)  (0x60 - 0x6F)", DEVICE_INFO_REGISTERS, None),
        ("Stats (Stats_Data_t)  (0x70 - 0x7F)", STATS_REGISTERS, None),
        ("Fuel gauge outputs  (0x90 - 0x9F)", FUEL_OUT_REGISTERS, None),
        ("Fuel gauge model, shared by all cells  (0xA0, 0xAD - 0xBE)", FUEL_MODEL_REGISTERS, None),
    ]
    for cell in range(CELL_COUNT):
        all_groups.append((f"Fuel gauge model, cell {cell + 1}  (0x9F = {cell}: 0xA1 - 0xAC, 0xD4)",
                           FUEL_CELL_MODEL_REGISTERS, cell))
    all_groups.append(("Fuel gauge state  (0xD0 - 0xDF)", FUEL_STATE_REGISTERS, None))

    errors = 0
    total  = sum(len(regs) for _, regs, _ in all_groups)

    try:
        for title, regs, cell in all_groups:
            print_header(title)

            if cell is not None:
                error = select_model_cell(ser, cell)
                if error:
                    print(f"0x{REG_MODEL_CELL:02X}       {'ModelCell (select)':<38} ERR: {error}")
                    errors += len(regs)
                    continue

            errors = read_registers(ser, regs, errors)

    except KeyboardInterrupt:
        print("\nAborted by user.")

    finally:
        # Leave the model registers on cell 1 again, where the board starts
        try:
            select_model_cell(ser, 0)
        except (serial.SerialException, OSError):
            pass
        ser.close()

    print(f"\n{'=' * 84}")
    print(f"Done.  {total} registers queried, {errors} errors.")


if __name__ == "__main__":
    main()
