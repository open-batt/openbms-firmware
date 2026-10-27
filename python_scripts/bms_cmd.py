#!/usr/bin/env python3
"""
bms_cmd.py -- send commands to the OpenBMS by typing them.

Usage:
    python bms_cmd.py [COM_PORT]                 interactive: type commands at the "bms>" prompt
    python bms_cmd.py [COM_PORT] <command>       runs one command and exits, e.g. python bms_cmd.py fet on

Without COM_PORT the tool pings every serial port and uses the one where an OpenBMS
answers. With COM_PORT it uses exactly that port.

Commands:
    ping                   check the board answers
    reset                  reboot the STM32
    fet on | fet off       main FET (normal run only)
    prefet on | prefet off pre-FET (normal run only)
    hppc start             start an HPPC test (normal run only)
    hppc stop              stop a running HPPC test (FET off, normal run again only after reset)
    save                   save the config to the EEPROM
    reset protection       clear a protection trip
    clear errors           clear the latched error flags
    clear learning         clear the learning status bits (HPPC charge / discharge / both done)
    clear fault            clear the fault snapshot (0x70 - 0x75), the next protection trip is stored again
    status                 read voltages, current, temperatures, FET, states and errors
    read <addr>            read one register, e.g. read 0x13 (raw bytes)
    help                   this list
    exit | quit            leave

Needs pyserial (pip install pyserial).
"""

import struct
import sys
import time

import importlib.util
import subprocess

# Third-party modules this script needs (import name: pip package). Missing ones are installed on the first run.
REQUIRED_MODULES = {"serial": "pyserial"}
for _module, _package in REQUIRED_MODULES.items():
    if importlib.util.find_spec(_module) is None:
        print(f"Installing {_package} ...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", _package])

import serial
from serial.tools import list_ports

BAUD_RATE = 230400

CC_READ             = 0x02
CC_ACK              = 0x03
CC_ERROR            = 0x04
CC_EEPROM_CONFIG    = 0x05
CC_HPPC_START       = 0x06
CC_NORMAL_RUN       = 0x07
CC_SET_FET          = 0x08
CC_REBOOT           = 0x09
CC_PING             = 0x0A
CC_RESET_PROTECTION = 0x0B
CC_CLEAR_ERRORS     = 0x0C
CC_SET_PRE_FET      = 0x0D
CC_CLEAR_LEARNING   = 0x0F
CC_CLEAR_FAULT      = 0x12

CE_NAMES = {
    0: "CE_OK",
    1: "CE_WRONG_CMD",
    2: "CE_BAD_CRC",
    3: "CE_NO_REG",
    4: "CE_RO",
    5: "CE_EEPROM",
    6: "CE_STATE (not allowed in this state)",
}

CTRL_STATES = {1: "WAIT_CONFIG", 2: "NORMAL_RUN", 3: "HPPC_RUN"}
# protection_trigger bits (0x12): CTRL_PROT_* in openbms_data.h, bit 0 first
PROTECTION_NAMES = [
    "OVP_SLOW", "OVP_FAST", "UVP_SLOW", "UVP_FAST",
    "OCP_CHARGE", "OCP_DISCHARGE_SLOW", "OCP_DISCHARGE_FAST", "OTP",
    "UTP", "OTP_CHARGE", "UTP_CHARGE",
]
# error_flags bits (0x15): OpenBMS_Error_t in openbms_error.h, bit 0 first
ERROR_NAMES = [
    "EEPROM_READ_FAIL", "EEPROM_WRITE_FAIL", "EEPROM_CRC_FAIL",
    "ADS_READ_FAIL", "ADS_INIT_FAIL", "ADS_ID_FAIL", "ADS_CRC_FAIL", "ADS_RESET", "ADS_DATA_LOST",
    "HW_DRV_FAULT", "HW_DRV_GATE_FAULT", "HW_PWR_FAIL",
    "UART_ERROR", "UART_TX_TIMEOUT", "NTC_FAULT", "I2C_ERROR", "CAN_ERROR", "WATCHDOG_RESET",
]
FET_BALANCE_SHIFT = 2   # fet_status bits 2..8: balancing cell 1..7
HPPC_STATES = {0: "IDLE", 1: "DETECT", 2: "REST", 3: "RAMP", 4: "DONE", 5: "ABORTED"}
HPPC_STOPS = {
    0: "none", 1: "cell max", 2: "cell min", 3: "term current", 4: "no current",
    5: "no config", 6: "host", 7: "protection", 8: "current range", 9: "temperature",
}

TIMEOUT_REPLY = 1.0     # s -- normal command / register read
TIMEOUT_SAVE  = 3.0     # s -- the EEPROM save takes longer
PORT_POLL     = 0.02    # s -- serial read timeout
OPEN_SETTLE   = 0.5     # s -- pause after opening a port

# Commands without an argument: typed words -> (command code, reply timeout, text when done)
SIMPLE_COMMANDS = {
    "ping":             (CC_PING,             TIMEOUT_REPLY, "board answered"),
    "reset":            (CC_REBOOT,           TIMEOUT_REPLY, "STM32 is rebooting"),
    "hppc start":       (CC_HPPC_START,       TIMEOUT_REPLY, "HPPC test started"),
    "hppc stop":        (CC_NORMAL_RUN,       TIMEOUT_REPLY, "HPPC test stopped, FET off. Type reset to go back to normal run"),
    "save":             (CC_EEPROM_CONFIG,    TIMEOUT_SAVE,  "config saved to the EEPROM"),
    "reset protection": (CC_RESET_PROTECTION, TIMEOUT_REPLY, "protection trip cleared"),
    "clear errors":     (CC_CLEAR_ERRORS,     TIMEOUT_REPLY, "error flags cleared"),
    "clear learning":   (CC_CLEAR_LEARNING,   TIMEOUT_REPLY, "learning status cleared"),
    "clear fault":      (CC_CLEAR_FAULT,      TIMEOUT_REPLY, "fault snapshot cleared"),
}

# Commands with on / off: typed word -> command code
ON_OFF_COMMANDS = {
    "fet":    CC_SET_FET,
    "prefet": CC_SET_PRE_FET,
}


class CmdError(Exception):
    pass


def checksum(data):
    return sum(data) % 256


def build_frame(cmd, address=0, payload=b""):
    packet = bytes([cmd, address, len(payload)]) + payload
    return packet + bytes([checksum(packet)])


def transact(ser, frame, timeout):
    ser.reset_input_buffer()
    ser.write(frame)

    buf = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        first = ser.read(1)
        if first:
            buf += first + ser.read(ser.in_waiting)
        if len(buf) >= 3 and len(buf) >= buf[2] + 4:
            break

    if len(buf) < 4 or len(buf) < buf[2] + 4:
        raise CmdError("no answer from the board")

    raw = buf[:buf[2] + 4]
    if checksum(raw[:-1]) != raw[-1]:
        raise CmdError("answer with a bad checksum")
    if raw[0] == CC_ERROR:
        code = raw[3]
        raise CmdError(f"board answered {CE_NAMES.get(code, f'error {code}')}")
    return raw


def send_command(ser, cmd, timeout, payload=b""):
    raw = transact(ser, build_frame(cmd, 0, payload), timeout)
    if raw[0] != CC_ACK:
        raise CmdError(f"unexpected answer 0x{raw[0]:02X}")


def read_register(ser, address):
    raw = transact(ser, build_frame(CC_READ, address), TIMEOUT_REPLY)
    return raw[3:3 + raw[2]]


def read_value(ser, address, fmt, count=1):
    data = read_register(ser, address)
    values = struct.unpack(f"<{count}{fmt}", data[:count * struct.calcsize(fmt)])
    if count == 1:
        return values[0]
    return list(values)


def learning_text(value):
    """LearningStatus (0x96) as names, FG_LEARN_* in openbms_fuelgauge.h."""
    names = []
    if value & 0x01:
        names.append("CHARGE")
    if value & 0x02:
        names.append("DISCHARGE")
    if value & 0x04:
        names.append("COMPLETE")
    if not names:
        return "none"
    return " ".join(names)


def protection_text(value):
    if not value:
        return "none"
    names = []
    for bit, name in enumerate(PROTECTION_NAMES):
        if value & (1 << bit):
            names.append(name)
    return ",".join(names)


def error_text(value):
    if not value:
        return "none"
    names = []
    for bit in range(64):
        if value & (1 << bit):
            if bit < len(ERROR_NAMES):
                names.append(ERROR_NAMES[bit])
            else:
                names.append(f"bit{bit}")
    return ",".join(names)


def balance_text(fet, cell_count):
    cells = []
    for i in range(cell_count):
        if fet & (1 << (FET_BALANCE_SHIFT + i)):
            cells.append(str(i + 1))
    if not cells:
        return "none"
    return "cells " + " ".join(cells)


def print_status(ser):
    cell_count = read_value(ser, 0x30, "B")
    cells = read_value(ser, 0x00, "f", 7)[:cell_count]
    pack = read_value(ser, 0x01, "f")
    current = read_value(ser, 0x02, "f")
    temp_pack = read_value(ser, 0x03, "f")
    temp_stm32 = read_value(ser, 0x04, "f")
    fet = read_value(ser, 0x06, "H")
    trip = read_value(ser, 0x12, "H")
    state = read_value(ser, 0x13, "B")
    errors = read_value(ser, 0x15, "Q")
    uptime = read_value(ser, 0x17, "I")
    hppc = read_value(ser, 0x19, "H")
    extract = read_value(ser, 0x1A, "H")
    soc = read_value(ser, 0x90, "B")
    learning = read_value(ser, 0x96, "H")

    if fet & 0x01:
        main_fet = "on"
    else:
        main_fet = "off"
    if fet & 0x02:
        pre_fet = "on"
    else:
        pre_fet = "off"

    print(f"  cells        {' '.join(f'{v:7.1f}' for v in cells)} mV")
    print(f"  pack         {pack / 1000.0:.3f} V")
    print(f"  current      {current:.1f} mA")
    print(f"  temperature  pack {temp_pack:.1f} C, STM32 {temp_stm32:.1f} C")
    print(f"  FET          main {main_fet}, pre {pre_fet}")
    print(f"  balancing    {balance_text(fet, cell_count)}")
    print(f"  state        {CTRL_STATES.get(state, state)}, uptime {uptime / 1000.0:.1f} s")
    print(f"  HPPC         {HPPC_STATES.get(hppc & 0xFF, '?')} (stop: {HPPC_STOPS.get(hppc >> 8, '?')}), "
          f"extract 0x{extract:04X}")
    print(f"  protection   trip 0x{trip:04X} {protection_text(trip)}")
    print(f"  errors       0x{errors:X} {error_text(errors)}")
    print(f"  SOC          {soc} %")
    print(f"  learning     {learning_text(learning)}")


def run_command(ser, line):
    """Runs one typed command. Returns False when the user wants to leave."""
    words = line.strip().lower().split()
    if not words:
        return True
    text = " ".join(words)

    if text in ("exit", "quit"):
        return False

    if text == "help":
        print(__doc__.split("Commands:")[1].split("Needs pyserial")[0].rstrip())
    elif text in SIMPLE_COMMANDS:
        cmd, timeout, done = SIMPLE_COMMANDS[text]
        send_command(ser, cmd, timeout)
        print(done)
    elif len(words) == 2 and words[0] in ON_OFF_COMMANDS and words[1] in ("on", "off"):
        if words[1] == "on":
            value = 1
        else:
            value = 0
        send_command(ser, ON_OFF_COMMANDS[words[0]], TIMEOUT_REPLY, bytes([value]))
        print(f"{words[0]} {words[1]}")
    elif text == "status":
        print_status(ser)
    elif len(words) == 2 and words[0] == "read":
        address = int(words[1], 0)
        data = read_register(ser, address)
        print(f"0x{address:02X}: {' '.join(f'{b:02X}' for b in data)}")
    else:
        print(f"unknown command '{text}', type help")
    return True


def ping_port(port):
    try:
        with serial.Serial(port, BAUD_RATE, timeout=PORT_POLL) as ser:
            time.sleep(OPEN_SETTLE)
            transact(ser, build_frame(CC_PING), TIMEOUT_REPLY)
            return True
    except (CmdError, serial.SerialException, OSError):
        return False


def find_board():
    ports = sorted(p.device for p in list_ports.comports())
    found = [port for port in ports if ping_port(port)]
    if not found:
        raise CmdError("no OpenBMS answered a ping on any port. Pass the port by hand.")
    if len(found) > 1:
        raise CmdError(f"more than one OpenBMS answered ({', '.join(found)}). Pass the port you want.")
    return found[0]


def is_port(arg):
    upper = arg.upper()
    return upper.startswith("COM") or arg.startswith("/dev/")


def main(argv):
    args = argv[1:]
    port = None
    if args and is_port(args[0]):
        port = args[0]
        args = args[1:]

    try:
        if port is None:
            port = find_board()
        print(f"OpenBMS on {port}")

        with serial.Serial(port, BAUD_RATE, timeout=PORT_POLL) as ser:
            time.sleep(OPEN_SETTLE)

            # One command from the command line
            if args:
                run_command(ser, " ".join(args))
                return 0

            # Interactive
            print("Type a command, help for the list, exit to leave.")
            while True:
                try:
                    line = input("bms> ")
                except (EOFError, KeyboardInterrupt):
                    print()
                    break
                try:
                    if not run_command(ser, line):
                        break
                except (CmdError, ValueError) as e:
                    print(f"Error: {e}")
    except (CmdError, serial.SerialException, OSError, ValueError) as e:
        print(f"Error: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
