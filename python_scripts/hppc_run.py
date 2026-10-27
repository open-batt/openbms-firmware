#!/usr/bin/env python3
"""
hppc_run.py -- start, stop and watch the HPPC test on a real OpenBMS board, with a CSV log.

Usage:
    python hppc_run.py [COM_PORT] monitor     watch the board, change nothing
    python hppc_run.py [COM_PORT] start       check the board, count down 5 s, start the HPPC test, then watch
    python hppc_run.py [COM_PORT] stop        stop a running HPPC test, then watch
    python hppc_run.py plot <csv_log>         plot a log written by this script (no board needed)

Without COM_PORT the tool pings every serial port and uses the one where an OpenBMS answers.

For the normal firmware build only: with OPENBMS_SIMULATION defined the board does not measure,
so the script stops (the board's uptime does not move without simulation samples).

start checks first that the board is in NORMAL_RUN, has no protection trip and no error flags,
and that the pack temperature is inside the HPPC window (15 - 35 C). If one check fails, it says
why and does not start. During the 5 s countdown q cancels the start.

While watching, one live line shows the HPPC state, direction, pack voltage, current,
temperature, every cell voltage and OK or FAIL, updated in place. New lines are printed for
events: HPPC state changes, the end of the test with its stop reason, protection trips, new error
flags, extraction warnings, and the board not answering or answering again.

Press q to leave (on Windows without Enter, elsewhere q then Enter). Leaving never changes the
HPPC test: a running test keeps running on the board.

Log: python_scripts/logs/hppc_<date>_<time>.csv, a new file for every run of the script, one row
per second.

The extracted model is not read out: it lives in the board's RAM until EEPROM storage for it
exists.

Needs pyserial and matplotlib (installed on the first run if missing).
"""

import csv
import datetime
import os
import select
import struct
import sys
import time

import importlib.util
import subprocess

# Third-party modules this script needs (import name: pip package). Missing ones are installed on the first run.
REQUIRED_MODULES = {"serial": "pyserial", "matplotlib": "matplotlib"}
for _module, _package in REQUIRED_MODULES.items():
    if importlib.util.find_spec(_module) is None:
        print(f"Installing {_package} ...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", _package])

import serial
from serial.tools import list_ports

# Key presses without Enter: msvcrt on Windows, select on stdin elsewhere
try:
    import msvcrt
except ImportError:
    msvcrt = None

BAUD_RATE = 230400

CC_READ             = 0x02
CC_ACK              = 0x03
CC_ERROR            = 0x04
CC_HPPC_START       = 0x06
CC_NORMAL_RUN       = 0x07
CC_PING             = 0x0A

CE_NAMES = {
    0: "CE_OK",
    1: "CE_WRONG_CMD",
    2: "CE_BAD_CRC",
    3: "CE_NO_REG",
    4: "CE_RO",
    5: "CE_EEPROM",
    6: "CE_STATE (not allowed in this state)",
}
CE_STATE = 6

CTRL_STATE_NORMAL_RUN = 2
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
HPPC_STATES = {0: "IDLE", 1: "DETECT", 2: "REST", 3: "RAMP", 4: "DONE", 5: "ABORTED"}
HPPC_STATE_DONE    = 4
HPPC_STATE_ABORTED = 5
HPPC_STOP_HOST     = 6
HPPC_STOPS = {
    0: "none", 1: "cell max", 2: "cell min", 3: "term current", 4: "no current",
    5: "no config", 6: "host", 7: "protection", 8: "current range", 9: "temperature",
}
HPPC_DIRECTIONS = {0: "-", 1: "charge", 2: "discharge"}
# hppc_extract_status (0x1A): HX_STATUS_* in openbms_hppc_extract.h, the bits that warn
HX_WARNINGS = {
    1 << 10: "OVERRUN (a fit was not ready, that cycle left out)",
    1 << 11: "FIT_WARN (a rest fit failed or is noisy, cycle interpolated)",
    1 << 12: "GAP_LO (lowest OCV far above ocv_v_lo_mv, low end extrapolated)",
    1 << 13: "GAP_HI (highest OCV far below ocv_v_hi_mv, high end extrapolated)",
    1 << 14: "FEW_CYCLES (too few cycles, nothing written)",
}

# HPPC temperature window, the same as HPPC_TEMP_MIN_C / HPPC_TEMP_MAX_C in config.h
HPPC_TEMP_MIN_C = 15.0
HPPC_TEMP_MAX_C = 35.0

START_COUNTDOWN_S = 5       # s -- countdown before the HPPC test is started
LOG_PERIOD_S      = 1.0     # s -- one read and one log row this often
BUILD_CHECK_S     = 0.5     # s -- the uptime must move within this time (not a simulation build)
TIMEOUT_REPLY     = 1.0     # s -- normal command / register read
PORT_POLL         = 0.02    # s -- serial read timeout
OPEN_SETTLE       = 0.5     # s -- pause after opening a port
QUIT_KEY          = "q"

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

MODES = ("monitor", "start", "stop")


class CmdError(Exception):
    pass


def checksum(data):
    return sum(data) % 256


def build_frame(cmd, address=0, payload=b""):
    packet = bytes([cmd, address, len(payload)]) + payload
    return packet + bytes([checksum(packet)])


def transact(ser, frame, timeout):
    """Sends a frame and returns the reply frame. A CC_ERROR reply raises CmdError with .code set."""
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
        err = CmdError(f"board answered {CE_NAMES.get(code, f'error {code}')}")
        err.code = code
        raise err
    return raw


def send_command(ser, cmd):
    raw = transact(ser, build_frame(cmd), TIMEOUT_REPLY)
    if raw[0] != CC_ACK:
        raise CmdError(f"unexpected answer 0x{raw[0]:02X}")


def read_value(ser, address, fmt, count=1):
    raw = transact(ser, build_frame(CC_READ, address), TIMEOUT_REPLY)
    data = raw[3:3 + raw[2]]
    values = struct.unpack(f"<{count}{fmt}", data[:count * struct.calcsize(fmt)])
    if count == 1:
        return values[0]
    return list(values)


def bits_text(value, names):
    if not value:
        return "none"
    out = []
    for bit in range(64):
        if value & (1 << bit):
            if bit < len(names):
                out.append(names[bit])
            else:
                out.append(f"bit{bit}")
    return ",".join(out)


def read_state(ser, cell_count):
    """Everything the monitor shows and logs, as a dict."""
    s = {}
    s["cells_mv"]      = read_value(ser, 0x00, "f", 7)[:cell_count]
    s["pack_mv"]       = read_value(ser, 0x01, "f")
    s["current_ma"]    = read_value(ser, 0x02, "f")
    s["temp_pack_c"]   = read_value(ser, 0x03, "f")
    s["fet_status"]    = read_value(ser, 0x06, "H")
    s["protection"]    = read_value(ser, 0x12, "H")
    s["ctrl_state"]    = read_value(ser, 0x13, "B")
    s["error_flags"]   = read_value(ser, 0x15, "Q")
    s["uptime_ms"]     = read_value(ser, 0x17, "I")
    s["direction"]     = read_value(ser, 0x18, "B")
    s["hppc_status"]   = read_value(ser, 0x19, "H")
    s["extract"]       = read_value(ser, 0x1A, "H")
    return s


# ---------------------------------------------------------------
# Keyboard
# ---------------------------------------------------------------
def quit_pressed():
    if msvcrt is not None:
        while msvcrt.kbhit():
            if msvcrt.getwch().lower() == QUIT_KEY:
                return True
        return False
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        return sys.stdin.readline().strip().lower() == QUIT_KEY
    return False


def wait_or_quit(seconds):
    """Waits, watching the keyboard. True when q was pressed."""
    end = time.time() + seconds
    while time.time() < end:
        if quit_pressed():
            return True
        time.sleep(0.05)
    return False


# ---------------------------------------------------------------
# Board
# ---------------------------------------------------------------
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


def check_build(ser):
    """A simulation build only moves its time with CC_SIM_SAMPLE, so the uptime stands still there."""
    first = read_value(ser, 0x17, "I")
    time.sleep(BUILD_CHECK_S)
    second = read_value(ser, 0x17, "I")
    if second == first:
        raise CmdError("the board's uptime does not move: this looks like a simulation build "
                       "(OPENBMS_SIMULATION). Flash a normal build for a real HPPC test.")


def start_checks(s):
    """Returns the reasons the HPPC test must not start, empty when all is fine."""
    reasons = []
    if s["ctrl_state"] != CTRL_STATE_NORMAL_RUN:
        reasons.append(f"board is in {CTRL_STATES.get(s['ctrl_state'], s['ctrl_state'])}, not NORMAL_RUN "
                       f"(after an HPPC test reboot it: bms_cmd.py reset)")
    if s["protection"]:
        reasons.append(f"protection tripped: {bits_text(s['protection'], PROTECTION_NAMES)} "
                       f"(bms_cmd.py reset protection)")
    if s["error_flags"]:
        reasons.append(f"error flags set: {bits_text(s['error_flags'], ERROR_NAMES)} (bms_cmd.py clear errors)")
    if s["temp_pack_c"] < HPPC_TEMP_MIN_C or s["temp_pack_c"] > HPPC_TEMP_MAX_C:
        reasons.append(f"pack temperature {s['temp_pack_c']:.1f} C is outside {HPPC_TEMP_MIN_C:.0f} - "
                       f"{HPPC_TEMP_MAX_C:.0f} C")
    return reasons


# ---------------------------------------------------------------
# Log
# ---------------------------------------------------------------
def open_log(cell_count):
    os.makedirs(LOG_DIR, exist_ok=True)
    name = datetime.datetime.now().strftime("hppc_%Y%m%d_%H%M%S.csv")
    path = os.path.join(LOG_DIR, name)
    f = open(path, "w", newline="")
    writer = csv.writer(f)
    header = ["time", "elapsed_s", "uptime_ms", "ctrl_state", "hppc_state", "hppc_stop", "hppc_status",
              "direction", "extract_status"]
    header += [f"cell_mv_{i + 1}" for i in range(cell_count)]
    header += ["pack_mv", "current_ma", "temp_pack_c", "fet_status", "protection_trigger", "error_flags"]
    writer.writerow(header)
    f.flush()
    return f, writer, path


def log_row(f, writer, elapsed_s, s):
    row = [datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), f"{elapsed_s:.1f}", s["uptime_ms"],
           CTRL_STATES.get(s["ctrl_state"], s["ctrl_state"]),
           HPPC_STATES.get(s["hppc_status"] & 0xFF, "?"), HPPC_STOPS.get(s["hppc_status"] >> 8, "?"),
           f"0x{s['hppc_status']:04X}", HPPC_DIRECTIONS.get(s["direction"], "?"), f"0x{s['extract']:04X}"]
    row += [f"{v:.1f}" for v in s["cells_mv"]]
    row += [f"{s['pack_mv']:.1f}", f"{s['current_ma']:.1f}", f"{s['temp_pack_c']:.2f}",
            f"0x{s['fet_status']:04X}", f"0x{s['protection']:04X}", f"0x{s['error_flags']:X}"]
    writer.writerow(row)
    f.flush()


# ---------------------------------------------------------------
# Monitor
# ---------------------------------------------------------------
def event(text):
    """An event line: clears the live line first."""
    print("\r\033[K" + datetime.datetime.now().strftime("%H:%M:%S ") + text)


def live_line(s, failures):
    state = s["hppc_status"] & 0xFF
    if failures:
        health = "FAIL: " + ", ".join(failures)
    else:
        health = "OK"
    cells = " ".join(f"{v:.0f}" for v in s["cells_mv"])
    text = (f"{HPPC_STATES.get(state, '?'):7} {HPPC_DIRECTIONS.get(s['direction'], '?'):9} | "
            f"{s['pack_mv'] / 1000.0:6.3f} V {s['current_ma']:7.0f} mA {s['temp_pack_c']:5.1f} C | "
            f"cells {cells} mV | {health}")
    sys.stdout.write("\r\033[K" + text)
    sys.stdout.flush()


def monitor(ser, cell_count):
    f, writer, path = open_log(cell_count)
    print(f"Log: {path}")
    print("Press q to leave (the HPPC test is not changed).")

    t0 = time.time()
    last = None
    lost = False
    failures = []
    try:
        while True:
            try:
                s = read_state(ser, cell_count)
            except CmdError as e:
                if not lost:
                    event(f"FAIL: board not answering ({e})")
                    lost = True
                if wait_or_quit(LOG_PERIOD_S):
                    break
                continue
            if lost:
                event("board answers again")
                lost = False

            failures = check_events(last, s, failures)
            log_row(f, writer, time.time() - t0, s)
            live_line(s, failures)
            last = s

            if wait_or_quit(LOG_PERIOD_S):
                break
    finally:
        f.close()
    print()
    print(f"Left the monitor. Log saved: {path}")


def check_events(last, s, failures):
    """Prints the events since the last read and returns the failure list for the live line."""
    state = s["hppc_status"] & 0xFF
    stop = s["hppc_status"] >> 8
    failures = list(failures)

    if last is None:
        event(f"state {CTRL_STATES.get(s['ctrl_state'], s['ctrl_state'])}, HPPC {HPPC_STATES.get(state, '?')}, "
              f"direction {HPPC_DIRECTIONS.get(s['direction'], '?')}")
        last_state = None
        last_protection = 0
        last_errors = 0
        last_extract = 0
        last_ctrl = s["ctrl_state"]
    else:
        last_state = last["hppc_status"] & 0xFF
        last_protection = last["protection"]
        last_errors = last["error_flags"]
        last_extract = last["extract"]
        last_ctrl = last["ctrl_state"]

    if s["ctrl_state"] != last_ctrl:
        event(f"board state {CTRL_STATES.get(last_ctrl, last_ctrl)} -> "
              f"{CTRL_STATES.get(s['ctrl_state'], s['ctrl_state'])}")

    if last is not None and state != last_state:
        event(f"HPPC {HPPC_STATES.get(last_state, '?')} -> {HPPC_STATES.get(state, '?')}, "
              f"direction {HPPC_DIRECTIONS.get(s['direction'], '?')}")

    if state != last_state and state == HPPC_STATE_DONE:
        event(f"HPPC test DONE ({HPPC_STOPS.get(stop, '?')}), extraction status 0x{s['extract']:04X}")
    if state != last_state and state == HPPC_STATE_ABORTED:
        text = f"HPPC test ABORTED ({HPPC_STOPS.get(stop, '?')})"
        if stop == HPPC_STOP_HOST:
            # Stopped by hand (hppc_run.py stop or bms_cmd.py hppc stop): not a failure
            event(text)
        else:
            event("FAIL: " + text)
            failures.append(text)

    new_prot = s["protection"] & ~last_protection
    if new_prot:
        text = f"protection {bits_text(new_prot, PROTECTION_NAMES)}"
        event("FAIL: " + text)
        failures.append(text)

    new_errors = s["error_flags"] & ~last_errors
    if new_errors:
        text = f"errors {bits_text(new_errors, ERROR_NAMES)}"
        event("FAIL: " + text)
        failures.append(text)

    for bit, name in HX_WARNINGS.items():
        if (s["extract"] & bit) and not (last_extract & bit):
            event("FAIL: extraction " + name)
            failures.append("extraction " + name.split(" ")[0])

    return failures


# ---------------------------------------------------------------
# Plot
# ---------------------------------------------------------------
def plot_log(path):
    import matplotlib.pyplot as plt

    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print(f"Error: {path} has no rows")
        return 1

    n = len([k for k in rows[0] if k.startswith("cell_mv_")])
    t = [float(r["elapsed_s"]) / 3600.0 for r in rows]

    def col(key):
        return [float(r[key]) for r in rows]

    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(12, 10))
    fig.canvas.manager.set_window_title(f"HPPC {os.path.basename(path)}")
    fig.suptitle(os.path.basename(path))

    for i in range(n):
        ax[0].plot(t, col(f"cell_mv_{i + 1}"), label=f"cell {i + 1}")
    ax[0].set_ylabel("cell [mV]")
    ax[0].legend(loc="best", ncol=n)

    ax[1].plot(t, col("current_ma"))
    ax[1].set_ylabel("pack current [mA]")

    ax[2].plot(t, col("temp_pack_c"))
    ax[2].set_ylabel("temperature [C]")

    names = list(HPPC_STATES.values())
    states = []
    for r in rows:
        if r["hppc_state"] in names:
            states.append(names.index(r["hppc_state"]))
        else:
            states.append(-1)
    ax[3].step(t, states, where="post")
    ax[3].set_yticks(list(range(len(names))))
    ax[3].set_yticklabels(names)
    ax[3].set_ylabel("HPPC state")
    ax[3].set_xlabel("time [h]")

    # Where the test stopped: first row in DONE or ABORTED after a running state
    running = False
    for r, ti in zip(rows, t):
        if r["hppc_state"] in ("DETECT", "REST", "RAMP"):
            running = True
        elif running and r["hppc_state"] in ("DONE", "ABORTED"):
            for a in ax:
                a.axvline(ti, color="red", linewidth=1.0)
            fig.suptitle(f"{os.path.basename(path)}: {r['hppc_state']} ({r['hppc_stop']})")
            running = False

    for a in ax:
        a.grid(True, alpha=0.3)
    fig.tight_layout()
    plt.show()
    return 0


# ---------------------------------------------------------------
# Main
# ---------------------------------------------------------------
def usage():
    print(__doc__.split("Without COM_PORT")[0].rstrip())


def main(argv):
    args = argv[1:]

    if args and args[0].lower() == "plot":
        if len(args) != 2:
            usage()
            return 1
        return plot_log(args[1])

    port = None
    if args and is_port(args[0]):
        port = args[0]
        args = args[1:]
    if len(args) != 1 or args[0].lower() not in MODES:
        usage()
        return 1
    mode = args[0].lower()

    # Lets the Windows console understand the escape code that clears the live line
    if os.name == "nt":
        os.system("")

    try:
        if port is None:
            port = find_board()
        print(f"OpenBMS on {port}")

        with serial.Serial(port, BAUD_RATE, timeout=PORT_POLL) as ser:
            time.sleep(OPEN_SETTLE)
            check_build(ser)
            cell_count = read_value(ser, 0x30, "B")

            if mode == "start":
                reasons = start_checks(read_state(ser, cell_count))
                if reasons:
                    print("HPPC test not started:")
                    for reason in reasons:
                        print(f"  - {reason}")
                    return 1
                for left in range(START_COUNTDOWN_S, 0, -1):
                    sys.stdout.write(f"\rStarting the HPPC test in {left} s (q cancels) ")
                    sys.stdout.flush()
                    if wait_or_quit(1.0):
                        print("\nCancelled, the HPPC test was not started.")
                        return 0
                print()
                send_command(ser, CC_HPPC_START)
                print("HPPC test started.")

            elif mode == "stop":
                try:
                    send_command(ser, CC_NORMAL_RUN)
                    print("HPPC test stopped. The FET stays off until a reboot (bms_cmd.py reset).")
                except CmdError as e:
                    if getattr(e, "code", None) == CE_STATE:
                        print("No HPPC test running (the board is not in HPPC_RUN).")
                    else:
                        raise

            monitor(ser, cell_count)
    except (CmdError, serial.SerialException, OSError) as e:
        print(f"Error: {e}")
        return 1
    except KeyboardInterrupt:
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
