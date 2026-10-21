#!/usr/bin/env python3
"""
config_tool.py -- read or write the OpenBMS config over UART.

Usage:
    python config_tool.py r [COM_PORT]
    python config_tool.py w [COM_PORT] [config.json]
    python config_tool.py p [COM_PORT]

    r   Reads the whole config out of the board and prints it.
    w   Writes every value from config.json (the settings and the device info
        strings) to the board, saves the config to the board's EEPROM, reboots
        the board, then reads everything back and checks it against the file.

    p   Pings the board five times and prints the raw bytes sent and received,
        with timing. For finding connection problems. Without COM_PORT it
        does this on every serial port.

Without COM_PORT the tool pings every serial port on the computer and uses the
one where an OpenBMS answers. With COM_PORT it uses exactly that port.
An argument ending in .json is the config file; config.json next to this
script is used when none is given. Keys are the field names of Config_Data_t
in openbms_data.h.

Needs pyserial (pip install pyserial).
"""

import json
import os
import struct
import sys
import time

import serial
from serial.tools import list_ports

# ---------------------------------------------------------------
# Protocol constants (see openbms_comm.h)
# ---------------------------------------------------------------
BAUD_RATE = 230400

CC_WRITE         = 0x01
CC_READ          = 0x02
CC_ACK           = 0x03
CC_ERROR         = 0x04
CC_EEPROM_CONFIG = 0x05
CC_REBOOT        = 0x09
CC_PING          = 0x0A

CE_NAMES = {
    0: "CE_OK",
    1: "CE_WRONG_CMD",
    2: "CE_BAD_CRC",
    3: "CE_NO_REG",
    4: "CE_RO",
    5: "CE_EEPROM",
    6: "CE_STATE",
}

TIMEOUT_READ  = 1.0    # s -- register read / write
TIMEOUT_PING  = 1.0    # s -- one ping
PING_TRIES    = 2      # pings per port while looking for the board
PORT_POLL     = 0.02   # s -- serial read timeout; replies are collected until their own deadline
OPEN_SETTLE   = 0.5    # s -- pause after opening a port (adapters / boards need a moment, like flash.py)
TIMEOUT_SAVE  = 3.0    # s -- the EEPROM save can take a few hundred ms
TIMEOUT_BOOT  = 10.0   # s -- how long to wait for the board after opening the port or a reboot
REBOOT_PAUSE  = 1.0    # s -- give the board time to actually reset

REG_CTRL_STATE = 0x13

CTRL_STATES = {
    1: "WAIT_CONFIG (no valid saved config)",
    2: "NORMAL_RUN",
    3: "HPPC_RUN",
}

# ---------------------------------------------------------------
# Config registers (Config_Data_t)
# ---------------------------------------------------------------
CELL_COUNT_MIN = 2     # BD_CELL_COUNT_MIN in openbms_data.h
CELL_COUNT_MAX = 7     # BD_CELL_COUNT_MAX in openbms_data.h

# (name, address, struct format, unit) -- in register order
FIELDS = [
    ("cell_count",                      0x30, "B", ""),
    ("use_i2c",                         0x31, "?", ""),
    ("use_can",                         0x32, "?", ""),
    ("use_uart",                        0x33, "?", ""),
    ("volt_protection",                 0x34, "?", ""),
    ("current_protection",              0x35, "?", ""),
    ("temp_protection",                 0x36, "?", ""),
    ("cell_capacity",                   0x37, "I", "mAh"),
    ("voltage_cell_max",                0x38, "H", "mV"),
    ("voltage_cell_min",                0x39, "H", "mV"),
    ("charging_term_current",           0x3A, "H", "mA"),
    ("uvp_slow_threshold_mv",           0x3B, "H", "mV"),
    ("uvp_slow_time_ms",                0x3C, "H", "ms"),
    ("uvp_fast_threshold_mv",           0x3D, "H", "mV"),
    ("uvp_fast_time_ms",                0x3E, "H", "ms"),
    ("ovp_slow_threshold_mv",           0x3F, "H", "mV"),
    ("ovp_slow_time_ms",                0x40, "H", "ms"),
    ("ovp_fast_threshold_mv",           0x41, "H", "mV"),
    ("ovp_fast_time_ms",                0x42, "H", "ms"),
    ("ocp_charge_threshold_ma",         0x43, "H", "mA"),
    ("ocp_charge_time_ms",              0x44, "H", "ms"),
    ("ocp_discharge_slow_threshold_ma", 0x45, "H", "mA"),
    ("ocp_discharge_slow_time_ms",      0x46, "H", "ms"),
    ("ocp_discharge_fast_threshold_ma", 0x47, "H", "mA"),
    ("ocp_discharge_fast_time_ms",      0x48, "H", "ms"),
    ("otp_threshold_c",                 0x49, "H", "C"),
    ("otp_time_ms",                     0x4A, "H", "ms"),
    ("charge_stop_cell_mv",             0x4B, "H", "mV"),
    ("charge_monitor_cell_mv",          0x4C, "H", "mV"),
    ("utp_threshold_c",                 0x4D, "h", "C"),
    ("utp_time_ms",                     0x4E, "H", "ms"),
    ("otp_charge_threshold_c",          0x4F, "h", "C"),
    ("ocv_v_lo_mv",                     0x50, "H", "mV"),
    ("ocv_v_hi_mv",                     0x51, "H", "mV"),
    ("current_sensor_gain",             0x52, "f", ""),
    ("voltage_gain",                    0x53, "7f", ""),     # one per cell, always 7 values
    ("ntc_beta",                        0x54, "f", ""),
    ("ntc_r_nominal",                   0x55, "f", "ohm"),
    ("ntc_r_fixed",                     0x56, "f", "ohm"),
    ("ntc_t_nominal",                   0x57, "f", "K"),
    ("temperature_offset",              0x58, "f", "C"),
    ("shunt_resistance_mohm",           0x59, "f", "mohm"),
    ("current_bias_ma",                 0x5A, "f", "mA"),
]

# Device info strings -- written from the file like the settings, saved with the config.
# Each register is a 32 byte char array: up to 31 ASCII characters, zero-terminated.
STRING_SIZE = 32

DEVICE_INFO = [
    ("firmware_version",  0x60),
    ("hardware_version",  0x61),
    ("manufacturer_name", 0x62),
    ("device_name",       0x63),
    ("device_chemistry",  0x64),
    ("manufacturer_data", 0x65),
]

FORMAT_RANGES = {
    "B": (0, 0xFF),
    "H": (0, 0xFFFF),
    "h": (-0x8000, 0x7FFF),
    "I": (0, 0xFFFFFFFF),
}


class ToolError(Exception):
    pass


def count_of(fmt):
    """Number of values in a field: 1 for "H", 7 for "7f"."""
    return int(fmt[:-1]) if len(fmt) > 1 else 1


def pack_value(fmt, value):
    if count_of(fmt) > 1:
        return struct.pack("<" + fmt, *value)
    return struct.pack("<" + fmt, value)


def unpack_value(fmt, data):
    values = struct.unpack("<" + fmt, data)
    if count_of(fmt) > 1:
        return list(values)
    return values[0]


def wire_value(fmt, value):
    """The value as the board holds it (floats are 32 bit there), for comparing against what was read back."""
    return unpack_value(fmt, pack_value(fmt, value))


# ---------------------------------------------------------------
# Protocol helpers
# ---------------------------------------------------------------
def checksum(data):
    return sum(data) % 256


def build_frame(cmd, address=0, payload=b""):
    packet = bytes([cmd, address, len(payload)]) + payload
    return packet + bytes([checksum(packet)])


def hex_bytes(data):
    return " ".join(f"{b:02X}" for b in data) if data else "nothing"


def transact(ser, frame, timeout, what):
    """Send a frame and collect the reply frame [cmd][arg][len][payload...][crc].
    Raises ToolError, saying what (if anything) arrived, when no good frame comes back in time."""
    ser.reset_input_buffer()
    ser.write(frame)

    buf = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        first = ser.read(1)                 # waits up to PORT_POLL
        if first:
            buf += first + ser.read(ser.in_waiting)
        if len(buf) >= 3 and len(buf) >= buf[2] + 4:
            break

    if len(buf) < 4 or len(buf) < buf[2] + 4:
        raise ToolError(f"{what}: no complete answer from the board (received: {hex_bytes(buf)})")

    raw = buf[:buf[2] + 4]
    if checksum(raw[:-1]) != raw[-1]:
        raise ToolError(f"{what}: answer with a bad checksum (received: {hex_bytes(buf)})")
    return raw


def expect_ack(raw, what):
    if raw[0] == CC_ACK:
        return
    if raw[0] == CC_ERROR:
        code = raw[3] if len(raw) > 3 else raw[1]
        raise ToolError(f"{what}: board answered {CE_NAMES.get(code, f'error {code}')}")
    raise ToolError(f"{what}: unexpected answer 0x{raw[0]:02X}")


def read_register(ser, address):
    raw = transact(ser, build_frame(CC_READ, address), TIMEOUT_READ, f"read 0x{address:02X}")
    if raw[0] == CC_ERROR:
        code = raw[3] if len(raw) > 3 else raw[1]
        raise ToolError(f"read 0x{address:02X}: board answered {CE_NAMES.get(code, f'error {code}')}")
    if raw[0] != CC_READ or raw[1] != address:
        raise ToolError(f"read 0x{address:02X}: unexpected answer")
    return raw[3:3 + raw[2]]


def write_register(ser, address, payload):
    raw = transact(ser, build_frame(CC_WRITE, address, payload), TIMEOUT_READ, f"write 0x{address:02X}")
    expect_ack(raw, f"write 0x{address:02X}")


def send_command(ser, cmd, timeout, what):
    raw = transact(ser, build_frame(cmd), timeout, what)
    expect_ack(raw, what)


# ---------------------------------------------------------------
# Config <-> registers
# ---------------------------------------------------------------
def read_config(ser):
    """Returns (config dict as in config.json, device info dict, ctrl_state)."""
    state = struct.unpack("<B", read_register(ser, REG_CTRL_STATE))[0]

    config = {}
    for name, address, fmt, _ in FIELDS:
        data = read_register(ser, address)
        config[name] = unpack_value(fmt, data)

    info = {}
    for name, address in DEVICE_INFO:
        data = read_register(ser, address)
        info[name] = data.split(b"\0", 1)[0].decode("ascii", errors="replace")

    return config, info, state


# ---------------------------------------------------------------
# Config file
# ---------------------------------------------------------------
def load_config_file(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
    except (OSError, ValueError) as e:
        raise ToolError(f"cannot read {path}: {e}")

    if not isinstance(cfg, dict):
        raise ToolError(f"{path}: expected a JSON object")

    expected = {name for name, _, _, _ in FIELDS} | {name for name, _ in DEVICE_INFO}
    missing = sorted(expected - set(cfg))
    unknown = sorted(set(cfg) - expected)
    if missing:
        raise ToolError(f"{path}: missing keys: {', '.join(missing)}")
    if unknown:
        raise ToolError(f"{path}: unknown keys: {', '.join(unknown)}")

    count = cfg["cell_count"]
    if isinstance(count, bool) or not isinstance(count, int) or not CELL_COUNT_MIN <= count <= CELL_COUNT_MAX:
        raise ToolError(f"{path}: cell_count must be a whole number from {CELL_COUNT_MIN} to {CELL_COUNT_MAX}")

    # device info strings
    for name, _ in DEVICE_INFO:
        text = cfg[name]
        if not isinstance(text, str) or not text.isascii() or "\0" in text or len(text) > STRING_SIZE - 1:
            raise ToolError(f"{path}: {name} must be text of up to {STRING_SIZE - 1} ASCII characters")

    # plain registers
    for name, _, fmt, unit in FIELDS:
        value = cfg[name]
        if fmt == "?":
            if not isinstance(value, bool):
                raise ToolError(f"{path}: {name} must be true or false")
            continue
        if fmt.endswith("f"):
            if count_of(fmt) > 1:
                if not isinstance(value, list) or len(value) != count_of(fmt):
                    raise ToolError(f"{path}: {name} must be a list of {count_of(fmt)} numbers")
                numbers = value
            else:
                numbers = [value]
            for number in numbers:
                if isinstance(number, bool) or not isinstance(number, (int, float)) or abs(number) > 3.0e38:
                    raise ToolError(f"{path}: {name} must be a number")
            continue
        low, high = FORMAT_RANGES[fmt]
        if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
            raise ToolError(f"{path}: {name} must be a whole number from {low} to {high} ({unit})")

    return cfg


def warn_about_values(cfg):
    """Things that are legal but probably a mistake. Printed, not fatal."""
    warnings = []
    if cfg["voltage_cell_min"] >= cfg["voltage_cell_max"]:
        warnings.append("voltage_cell_min is not below voltage_cell_max")
    if cfg["uvp_fast_threshold_mv"] > cfg["uvp_slow_threshold_mv"]:
        warnings.append("uvp_fast_threshold_mv is above uvp_slow_threshold_mv")
    if cfg["ovp_fast_threshold_mv"] < cfg["ovp_slow_threshold_mv"]:
        warnings.append("ovp_fast_threshold_mv is below ovp_slow_threshold_mv")
    if cfg["ocp_discharge_fast_threshold_ma"] < cfg["ocp_discharge_slow_threshold_ma"]:
        warnings.append("ocp_discharge_fast_threshold_ma is below ocp_discharge_slow_threshold_ma")
    if cfg["ocv_v_lo_mv"] >= cfg["ocv_v_hi_mv"]:
        warnings.append("ocv_v_lo_mv is not below ocv_v_hi_mv")
    return warnings


# ---------------------------------------------------------------
# Output
# ---------------------------------------------------------------
def print_config(config, info, state):
    print(f"  ctrl_state      : {CTRL_STATES.get(state, state)}")
    print()
    print("  Device info")
    for name, _ in DEVICE_INFO:
        print(f"    {name:<34}{info[name]}")
    print()
    print("  Config")
    for name, address, fmt, unit in FIELDS:
        if fmt == "?":
            shown = "yes" if config[name] else "no"
        elif count_of(fmt) > 1:
            shown = "[" + ", ".join(f"{v:.7g}" for v in config[name]) + "]"
        elif fmt == "f":
            shown = f"{config[name]:.7g}"
        else:
            shown = config[name]
        print(f"    {name:<34}{shown:<8}{unit:<5}(0x{address:02X})")
    print()


def compare(wanted, got, info):
    """Returns a list of 'name: wanted X, board has Y' lines."""
    diffs = []
    for name, _, fmt, _ in FIELDS:
        if got[name] != wire_value(fmt, wanted[name]):
            diffs.append(f"{name}: file {wanted[name]}, board {got[name]}")
    for name, _ in DEVICE_INFO:
        if info[name] != wanted[name]:
            diffs.append(f"{name}: file {wanted[name]!r}, board {info[name]!r}")
    return diffs


# ---------------------------------------------------------------
# Actions
# ---------------------------------------------------------------
def open_port(port):
    """Opens the port and gives the adapter / board a moment, like the other scripts do."""
    ser = serial.Serial(port, BAUD_RATE, timeout=PORT_POLL)
    time.sleep(OPEN_SETTLE)
    ser.reset_input_buffer()
    return ser


def ping(ser):
    """Ping once. Any valid reply frame counts (older firmware answers an error frame).
    Raises ToolError, saying what arrived, if there is no good reply."""
    transact(ser, build_frame(CC_PING), TIMEOUT_PING, "ping")


def is_alive(ser):
    try:
        ping(ser)
        return True
    except ToolError:
        return False


def wait_until_ready(ser):
    """Ping until the board answers. Opening a port can reset some boards, and a reboot takes a moment."""
    deadline = time.time() + TIMEOUT_BOOT
    last = ""
    while time.time() < deadline:
        try:
            ping(ser)
            return
        except ToolError as e:
            last = str(e)
    raise ToolError(f"the board did not answer a ping within {TIMEOUT_BOOT:.0f} s. Last try: {last}")


def ping_port(port):
    """True if an OpenBMS answers a ping on this port."""
    try:
        with open_port(port) as ser:
            for _ in range(PING_TRIES):
                if is_alive(ser):
                    return True
    except (serial.SerialException, OSError):
        pass    # port busy, gone, or not a real serial port -- not our board
    return False


def find_board():
    """Pings every serial port and returns the one that answers."""
    ports = sorted(p.device for p in list_ports.comports())
    if not ports:
        raise ToolError("no serial ports found on this computer")

    print(f"Looking for the board on {', '.join(ports)} ...")
    found = [port for port in ports if ping_port(port)]

    if not found:
        raise ToolError("no OpenBMS answered a ping on any port (is it powered and connected? "
                        "Is another program using the port?). Pass the port by hand to force one.")
    if len(found) > 1:
        raise ToolError(f"more than one OpenBMS answered ({', '.join(found)}). Pass the port you want.")

    print(f"Found the board on {found[0]}")
    return found[0]


def do_read(port):
    with open_port(port) as ser:
        wait_until_ready(ser)
        config, info, state = read_config(ser)
    print(f"Config read from the board on {port}:")
    print()
    print_config(config, info, state)
    return 0


def do_write(port, path):
    wanted = load_config_file(path)

    for line in warn_about_values(wanted):
        print(f"WARNING: {line}")

    with open_port(port) as ser:
        # 1. Make sure the board is there before touching anything
        wait_until_ready(ser)

        # 2. Write every config register and the device info strings
        print(f"Writing {len(FIELDS) + len(DEVICE_INFO)} config registers from {path} ...")
        for name, address, fmt, _ in FIELDS:
            write_register(ser, address, pack_value(fmt, wanted[name]))
        for name, address in DEVICE_INFO:
            write_register(ser, address, wanted[name].encode("ascii").ljust(STRING_SIZE, b"\0"))

        # 3. Save to the EEPROM -- without this the reboot below would drop the values
        print("Saving to the EEPROM ...")
        send_command(ser, CC_EEPROM_CONFIG, TIMEOUT_SAVE, "save to EEPROM")

        # 4. Reboot and wait for the board to come back
        print("Rebooting the board ...")
        send_command(ser, CC_REBOOT, TIMEOUT_READ, "reboot")
        time.sleep(REBOOT_PAUSE)
        wait_until_ready(ser)

        # 5. Read everything back
        config, info, state = read_config(ser)

    print()
    print("Config read back after the reboot:")
    print()
    print_config(config, info, state)

    diffs = compare(wanted, config, info)
    ok = True
    if diffs:
        ok = False
        print("MISMATCH between the file and the board:")
        for line in diffs:
            print(f"  {line}")
    else:
        print("OK: every value on the board matches the file.")

    if state != 2:
        ok = False
        print(f"WARNING: the board is not in RUN after the reboot (ctrl_state = {CTRL_STATES.get(state, state)}), "
              "so it did not accept the saved config.")

    return 0 if ok else 1


def ping_trace(port, tries):
    """Raw ping dump: what was sent, and every byte that came back, with timing."""
    print(f"{port}:")
    got_ack = False
    with open_port(port) as ser:
        for i in range(tries):
            frame = build_frame(CC_PING)
            ser.reset_input_buffer()
            start = time.time()
            ser.write(frame)

            buf = b""
            first = None
            while time.time() - start < 1.0:
                chunk = ser.read(1)
                if chunk:
                    if first is None:
                        first = time.time() - start
                    buf += chunk + ser.read(ser.in_waiting)

            answer = f"{hex_bytes(buf)} (first byte after {first * 1000:.0f} ms)" if buf else "nothing within 1 s"
            print(f"  ping {i + 1}: sent {hex_bytes(frame)}, received {answer}")
            if buf[:4] == bytes([CC_ACK, 0, 0, CC_ACK]):
                got_ack = True
            time.sleep(0.2)
    return got_ack


def do_ping(port):
    if port is not None:
        return 0 if ping_trace(port, 5) else 1

    ports = sorted(p.device for p in list_ports.comports())
    if not ports:
        raise ToolError("no serial ports found on this computer")
    ok = False
    for p in ports:
        try:
            ok = ping_trace(p, 3) or ok
        except (serial.SerialException, OSError) as e:
            print(f"  cannot open {p}: {e}")
    return 0 if ok else 1


def main(argv):
    if len(argv) < 2 or argv[1].lower() not in ("r", "w", "p", "read", "write", "ping"):
        print(__doc__)
        return 2

    mode = argv[1].lower()[0]
    port = None
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")

    for arg in argv[2:]:
        if arg.lower().endswith(".json"):
            path = arg
        elif port is None:
            port = arg
        else:
            print(__doc__)
            return 2

    try:
        if mode == "p":
            return do_ping(port)
        if port is None:
            port = find_board()
        if mode == "r":
            return do_read(port)
        return do_write(port, path)
    except ToolError as e:
        print(f"ERROR: {e}")
        return 1
    except serial.SerialException as e:
        print(f"ERROR: serial port {port}: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
