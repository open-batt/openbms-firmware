#!/usr/bin/env python3
"""
battery_cycler.py — live serial test harness for OpenBMS hardware.

Drives the board through one of several test modes (read-only monitor,
continuous or pulsed charge/discharge, or a full HPPC checkpoint run),
logging every 20Hz read to a timestamped CSV and rendering a live
terminal dashboard at 5Hz. This is the script that actually produces
the CSV logs that preview_logs.py, hppc_pipeline.py, and
soc_estimator.py all consume.

Usage:
    python battery_cycler.py <MODE> <COM_PORT>

Windows-only (uses msvcrt for keypress detection).
"""

import sys
import serial
import time
import struct
import msvcrt
import csv
from datetime import datetime

# ---------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------
BAUD_RATE        = 230400
TIMEOUT_RESPONSE = 0.03    # 30ms — tight enough for 20Hz cycle

CC_WRITE      = 0x01
CC_READ       = 0x02
CC_ACK        = 0x03
CC_ERROR      = 0x04
CC_CMD        = 0x05
CC_BOOTLOADER = 0x42

CE_NAMES = {
    0: "CE_OK",
    1: "CE_WRONG_CMD",
    2: "CE_BAD_CRC",
    3: "CE_NO_REG",
    4: "CE_RO",
}

# ---------------------------------------------------------------
# Registers — Control_Data_t (charge configuration, loaded once)
# ---------------------------------------------------------------
REG_CONFIGURATION         = 0x30   # uint16
REG_MAIN_CONTROL          = 0x31   # uint16
REG_CELL_CAPACITY         = 0x32   # uint32 — mAh  (design/factory capacity)
REG_VOLTAGE_CELL_MAX      = 0x33   # uint16 — mV
REG_VOLTAGE_CELL_MIN      = 0x34   # uint16 — mV
REG_CHARGING_TERM_CURRENT = 0x35   # uint16 — mA

# ---------------------------------------------------------------
# Registers — Peripheral_Data_t / FuelGauge_Data_t (20Hz monitoring)
# ---------------------------------------------------------------
REG_CELL_VOLTAGE      = 0x00   # float[7]  — mV
REG_CELL_VOLTAGE_FILT = 0x01   # float[7]  — mV
REG_PACK_VOLTAGE      = 0x02   # float     — mV
REG_PACK_VOLTAGE_FILT = 0x03   # float     — mV
REG_PACK_CURRENT      = 0x04   # float     — mA
REG_PACK_CURRENT_FILT = 0x05   # float     — mA
REG_TEMPERATURE       = 0x06   # float     — °C
REG_FET_STATUS        = 0x09   # uint16    — flags
REG_UPTIME            = 0x51   # uint64    — ms
REG_LEARNING_STATUS   = 0xAC   # uint16    — flags

# ---------------------------------------------------------------
# Command addresses — Controls_Set (issued via CC_CMD)
# ---------------------------------------------------------------
CMD_ADDR_MODE = 0x00
CMD_ADDR_FET  = 0x01

MODE_NORMAL   = 0x00
MODE_CONFIG   = 0x01
MODE_LEARNING = 0x02

FET_DISABLE = 0x00
FET_ENABLE  = 0x01

# ---------------------------------------------------------------
# Bit definitions
# ---------------------------------------------------------------
BD_MAIN_CTR_MODE_MASK     = 0x03
BD_MAIN_CTR_MODE_LEARNING = 0x02

BD_FET_MAIN = (1 << 0)
BD_FET_PRE  = (1 << 1)

POLL_INTERVAL_S     = 0.05     # 20 Hz
DISPLAY_EVERY       = 4        # update screen every 4th read = 5 Hz
MODE_VERIFY_RETRIES = 10
MODE_VERIFY_DELAY_S = 0.05

PULSE_ON_S  = 2.0
PULSE_OFF_S = 2.0

# Charging termination rules (apply to C, CP, CHPPC):
#   - hard ceiling: stop immediately if any cell's filtered voltage
#     reaches CHARGE_STOP_CELL_MV, regardless of current.
#   - taper/termination-current stop: only evaluated once any cell's
#     filtered voltage has reached CHARGE_CURRENT_MONITOR_MV — below
#     that, current is not monitored for termination purposes at all.
CHARGE_STOP_CELL_MV        = 4300
CHARGE_CURRENT_MONITOR_MV  = 4100

# ---------------------------------------------------------------
# HPPC test parameters
# ---------------------------------------------------------------
HPPC_REST_S = 450.0

# SOC checkpoints for discharge HPPC, in % of design capacity
DISCHARGE_HPPC_POINTS = [100, 98, 95, 90, 85, 80, 70, 60, 50, 40,
                          30, 25, 20, 15, 12, 10, 8, 6, 4, 2]
# Charge HPPC runs the same checkpoints in the opposite direction
CHARGE_HPPC_POINTS = list(reversed(DISCHARGE_HPPC_POINTS))

# ---------------------------------------------------------------
# Mode argument mapping — short codes
# ---------------------------------------------------------------
MODE_MAP = {
    "M":     "monitor",
    "C":     "charging",
    "D":     "discharging",
    "CP":    "charging_pulse",
    "DP":    "discharging_pulse",
    "CHPPC": "charging_hppc",
    "DHPPC": "discharging_hppc",
}

# ---------------------------------------------------------------
# ANSI codes
# ---------------------------------------------------------------
HOME   = "\033[H"
EOL    = "\033[K"
BOLD   = "\033[1m"
RESET  = "\033[0m"
CYAN   = "\033[96m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
DIM    = "\033[2m"


# ---------------------------------------------------------------
# Protocol helpers
# ---------------------------------------------------------------
def checksum(data):
    return sum(data) % 256

def build_frame(cmd_type, address, payload=b''):
    header = bytes([cmd_type, address, len(payload)])
    packet = header + payload
    return packet + bytes([checksum(packet)])

def read_response(ser):
    """Read a full response frame: [cmd][arg1][len][...payload][crc].
    Works for CC_READ, CC_ACK and CC_ERROR frames alike, since the
    device always echoes a length byte at index 2, followed by that
    many payload bytes and a trailing CRC."""
    header = ser.read(3)
    if len(header) < 3:
        return None

    payload_length = header[2]
    rest = ser.read(payload_length + 1)
    if len(rest) < payload_length + 1:
        return None

    raw = header + rest
    if checksum(raw[:-1]) != raw[-1]:
        return None

    return raw


def read_register(ser, address):
    ser.reset_input_buffer()
    ser.write(build_frame(CC_READ, address))

    raw = read_response(ser)
    if raw is None:
        return None
    if raw[0] != CC_READ or raw[1] != address:
        return None

    length = raw[2]
    return raw[3:3 + length]


def send_command(ser, address, payload_byte):
    """Issue a CC_CMD frame (routed to Controls_Set on the device).
    Returns (success: bool, info: str)."""
    ser.reset_input_buffer()
    ser.write(build_frame(CC_CMD, address, bytes([payload_byte])))

    raw = read_response(ser)
    if raw is None:
        return False, "TIMEOUT/BAD_CRC"

    if raw[0] == CC_ACK:
        return True, "ACK"
    elif raw[0] == CC_ERROR:
        code = raw[1]
        return False, CE_NAMES.get(code, f"UNKNOWN(0x{code:02X})")
    else:
        return False, f"UNEXPECTED_RESPONSE(0x{raw[0]:02X})"


def read_float_array(ser, address, count):
    data = read_register(ser, address)
    if data is None or len(data) < count * 4:
        return None
    return [struct.unpack_from('<f', data, i * 4)[0] for i in range(count)]

def read_float(ser, address):
    result = read_float_array(ser, address, 1)
    return result[0] if result else None

def read_uint16(ser, address):
    data = read_register(ser, address)
    if data is None or len(data) < 2:
        return None
    return struct.unpack_from('<H', data)[0]

def read_uint32(ser, address):
    data = read_register(ser, address)
    if data is None or len(data) < 4:
        return None
    return struct.unpack_from('<I', data)[0]

def read_uint64(ser, address):
    data = read_register(ser, address)
    if data is None or len(data) < 8:
        return None
    return struct.unpack_from('<Q', data)[0]


def set_mode(ser, mode):
    return send_command(ser, CMD_ADDR_MODE, mode)

def set_fet(ser, enable):
    return send_command(ser, CMD_ADDR_FET, FET_ENABLE if enable else FET_DISABLE)


# ---------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------
def fmt_f(val, dec=2):
    return f"{val:.{dec}f}" if val is not None else "ERR"

def fmt_h(val):
    return f"0x{val:04X}" if val is not None else "ERR"

def fmt_csv(val):
    return "" if val is None else str(val)

def bar(val, lo, hi, width=24):
    if val is None:
        return DIM + "[" + "?" * width + "]" + RESET
    ratio  = max(0.0, min(1.0, (val - lo) / (hi - lo)))
    filled = int(ratio * width)
    return DIM + "[" + RESET + GREEN + "█" * filled + DIM + "░" * (width - filled) + "]" + RESET

def cell_color(v):
    if v is None:             return RED
    if v < 2800 or v > 4200: return RED
    if v < 3000 or v > 4150: return YELLOW
    return GREEN

def current_color(v):
    if v is None:      return RED
    if abs(v) > 18000: return RED
    if abs(v) > 12000: return YELLOW
    return GREEN

def temp_color(t):
    if t is None: return RED
    if t >= 55:   return RED
    if t >= 45:   return YELLOW
    return GREEN

def fet_text(fet_status, bit, on_label="ENABLED", off_label="DISABLED"):
    if fet_status is None:
        return RED + "ERR" + RESET
    if fet_status & bit:
        return GREEN + BOLD + on_label + RESET
    return DIM + off_label + RESET

def ln(text=''):
    print(text + EOL)


# ---------------------------------------------------------------
# CSV logging — continuous / pulse modes (M, C, D, CP, DP)
# ---------------------------------------------------------------
CSV_HEADER = [
    "elapsed_s",
    "uptime_ms",
    "cell_v1",  "cell_v2",  "cell_v3",  "cell_v4",  "cell_v5",  "cell_v6",  "cell_v7",
    "cell_vf1", "cell_vf2", "cell_vf3", "cell_vf4", "cell_vf5", "cell_vf6", "cell_vf7",
    "pack_voltage_mv",
    "pack_voltage_filtered_mv",
    "current_ma",
    "current_filtered_ma",
    "temperature_c",
    "fet_status",
    "fet_main_enabled",
    "commanded_fet_state",
    "learning_status",
    "stop_reason",
]

def make_csv_row(elapsed_s, uptime, cell_v, cell_v_filt, pack_v, pack_vf,
                 current, curr_filt, temp_c, fet_status, learning_status,
                 commanded_fet_state='', stop_reason=''):
    row = [f"{elapsed_s:.2f}", fmt_csv(uptime)]
    row += [fmt_csv(cell_v[i]      if cell_v      else None) for i in range(7)]
    row += [fmt_csv(cell_v_filt[i] if cell_v_filt else None) for i in range(7)]
    row += [fmt_csv(pack_v), fmt_csv(pack_vf)]
    row += [fmt_csv(current), fmt_csv(curr_filt), fmt_csv(temp_c)]
    row += [fmt_csv(fet_status)]
    row += [fmt_csv(bool(fet_status & BD_FET_MAIN) if fet_status is not None else None)]
    row += [commanded_fet_state]
    row += [fmt_csv(learning_status)]
    row += [stop_reason]
    return row


# ---------------------------------------------------------------
# CSV logging — HPPC modes (CHPPC, DHPPC)
# ---------------------------------------------------------------
HPPC_CSV_HEADER = [
    "elapsed_s",
    "uptime_ms",
    "cell_v1",  "cell_v2",  "cell_v3",  "cell_v4",  "cell_v5",  "cell_v6",  "cell_v7",
    "cell_vf1", "cell_vf2", "cell_vf3", "cell_vf4", "cell_vf5", "cell_vf6", "cell_vf7",
    "pack_voltage_mv",
    "pack_voltage_filtered_mv",
    "current_ma",
    "current_filtered_ma",
    "temperature_c",
    "fet_status",
    "fet_main_enabled",
    "commanded_fet_state",
    "learning_status",
    "segment_type",         # REST or RAMP
    "target_pct",           # checkpoint this segment is heading to / resting at
    "segment_coulomb_mah",  # accumulated |current| x time for the active RAMP segment
    "stop_reason",
]

def make_hppc_csv_row(elapsed_s, r, commanded_fet_state, segment_type, target_pct,
                      segment_coulomb_mah, stop_reason=''):
    row = [f"{elapsed_s:.2f}", fmt_csv(r['uptime'])]
    row += [fmt_csv(r['cell_v'][i]      if r['cell_v']      else None) for i in range(7)]
    row += [fmt_csv(r['cell_v_filt'][i] if r['cell_v_filt'] else None) for i in range(7)]
    row += [fmt_csv(r['pack_v']), fmt_csv(r['pack_vf'])]
    row += [fmt_csv(r['current']), fmt_csv(r['curr_filt']), fmt_csv(r['temp_c'])]
    row += [fmt_csv(r['fet_status'])]
    row += [fmt_csv(bool(r['fet_status'] & BD_FET_MAIN) if r['fet_status'] is not None else None)]
    row += [commanded_fet_state]
    row += [fmt_csv(r['learning_status'])]
    row += [segment_type, fmt_csv(target_pct), f"{segment_coulomb_mah:.3f}"]
    row += [stop_reason]
    return row


def read_all_monitor_regs(ser):
    """Single 20Hz read of every monitored register. Returns a dict."""
    uptime          = read_uint64     (ser, REG_UPTIME                    )
    cell_v          = read_float_array(ser, REG_CELL_VOLTAGE,      7      )
    cell_v_filt     = read_float_array(ser, REG_CELL_VOLTAGE_FILT, 7      )
    pack_v          = read_float      (ser, REG_PACK_VOLTAGE               )
    pack_vf         = read_float      (ser, REG_PACK_VOLTAGE_FILT          )
    current         = read_float      (ser, REG_PACK_CURRENT               )
    curr_filt       = read_float      (ser, REG_PACK_CURRENT_FILT          )
    temp_c          = read_float      (ser, REG_TEMPERATURE                )
    fet_status      = read_uint16     (ser, REG_FET_STATUS                 )
    learning_status = read_uint16     (ser, REG_LEARNING_STATUS            )

    return dict(uptime=uptime, cell_v=cell_v, cell_v_filt=cell_v_filt,
                pack_v=pack_v, pack_vf=pack_vf, current=current, curr_filt=curr_filt,
                temp_c=temp_c, fet_status=fet_status, learning_status=learning_status)


# ---------------------------------------------------------------
# Shared setup helpers
# ---------------------------------------------------------------
def load_charge_config(ser):
    """Read the Control_Data_t charge configuration registers once.
    Aborts the script (sys.exit) if any of them fail to read."""
    print("Loading charge configuration...")
    configuration         = read_uint16(ser, REG_CONFIGURATION)
    main_control_initial  = read_uint16(ser, REG_MAIN_CONTROL)
    cell_capacity         = read_uint32(ser, REG_CELL_CAPACITY)
    voltage_cell_max      = read_uint16(ser, REG_VOLTAGE_CELL_MAX)
    voltage_cell_min      = read_uint16(ser, REG_VOLTAGE_CELL_MIN)
    charging_term_current = read_uint16(ser, REG_CHARGING_TERM_CURRENT)

    if any(v is None for v in [configuration, main_control_initial, cell_capacity,
                                voltage_cell_max, voltage_cell_min, charging_term_current]):
        print("ERR: failed to read charge configuration registers. Aborting.")
        sys.exit(1)

    cell_count = configuration & 0x0F

    print(f"  Configuration           : 0x{configuration:04X}")
    print(f"  Cell count              : {cell_count}")
    print(f"  Main control (initial)  : 0x{main_control_initial:04X}")
    print(f"  Cell capacity (design)  : {cell_capacity} mAh")
    print(f"  Voltage cell max        : {voltage_cell_max} mV")
    print(f"  Voltage cell min        : {voltage_cell_min} mV")
    print(f"  Charging term current   : {charging_term_current} mA")

    return dict(configuration=configuration, main_control_initial=main_control_initial,
                cell_capacity=cell_capacity, voltage_cell_max=voltage_cell_max,
                voltage_cell_min=voltage_cell_min, charging_term_current=charging_term_current,
                cell_count=cell_count)


def request_and_verify_learning_mode(ser):
    """Request learning mode and poll main_control until confirmed.
    Aborts the script (sys.exit) on failure."""
    print("Requesting learning mode...")
    ok, info = set_mode(ser, MODE_LEARNING)
    if not ok:
        print(f"ERR: set_mode(LEARNING) failed: {info}. Aborting.")
        sys.exit(1)

    in_learning  = False
    main_control = None
    for attempt in range(MODE_VERIFY_RETRIES):
        main_control = read_uint16(ser, REG_MAIN_CONTROL)
        if main_control is not None and \
           (main_control & BD_MAIN_CTR_MODE_MASK) == BD_MAIN_CTR_MODE_LEARNING:
            in_learning = True
            break
        time.sleep(MODE_VERIFY_DELAY_S)

    if not in_learning:
        mc_str = f"0x{main_control:04X}" if main_control is not None else "ERR"
        print(f"ERR: device did not confirm learning mode (main_control={mc_str}). Aborting.")
        sys.exit(1)

    print("Learning mode confirmed.")


# ---------------------------------------------------------------
# Monitor mode ('M') — pure read-only, no mode/FET commands ever sent
# ---------------------------------------------------------------
def do_monitor(ser, port):
    cfg = load_charge_config(ser)
    cell_capacity    = cfg['cell_capacity']
    cell_count       = cfg['cell_count']
    voltage_cell_max = cfg['voltage_cell_max']
    voltage_cell_min = cfg['voltage_cell_min']
    print()

    log_filename = f"openbms_monitor_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    log_file     = open(log_filename, 'w', newline='')
    log_writer   = csv.writer(log_file)
    log_writer.writerow(CSV_HEADER)

    print("\033[2J", end='', flush=True)
    print(f"Logging to: {log_filename}")
    time.sleep(0.5)

    t_start_all  = time.time()
    display_iter = 0
    errors       = 0

    try:
        while True:
            if msvcrt.kbhit():
                if msvcrt.getwch().lower() == 'q':
                    print(RESET + "\nStopped.")
                    break

            t_start = time.time()
            r = read_all_monitor_regs(ser)
            elapsed_ms = (time.time() - t_start) * 1000
            elapsed_s  = time.time() - t_start_all

            if any(v is None for v in r.values()):
                errors += 1

            log_writer.writerow(make_csv_row(elapsed_s, r['uptime'], r['cell_v'], r['cell_v_filt'],
                                             r['pack_v'], r['pack_vf'], r['current'], r['curr_filt'],
                                             r['temp_c'], r['fet_status'], r['learning_status']))
            log_file.flush()

            if display_iter % DISPLAY_EVERY == 0:
                print(HOME, end='')
                ln(BOLD + CYAN + "=" * 72 + RESET)
                ln(f"  OpenBMS Monitor  —  {port}  —  20Hz read / 5Hz display  —  errors: {errors}")
                ln(BOLD + CYAN + "=" * 72 + RESET)

                up = r['uptime']
                ln()
                if up is not None:
                    h  = up // 3_600_000
                    m  = (up % 3_600_000) // 60_000
                    s  = (up % 60_000) // 1000
                    ms = up % 1000
                    ln(BOLD + f"  Uptime: {h:02d}:{m:02d}:{s:02d}.{ms:03d}  ({up} ms)" + RESET)
                else:
                    ln(BOLD + "  Uptime: ERR" + RESET)

                ln()
                ln(BOLD + "  Pack Info" + RESET)
                ln(f"  Design capacity:   {cell_capacity} mAh")
                ln(f"  Cell count:        {cell_count}")
                ln(f"  Cell voltage max:  {voltage_cell_max} mV")
                ln(f"  Cell voltage min:  {voltage_cell_min} mV")

                v, vf = r['cell_v'], r['cell_v_filt']
                ln()
                ln(BOLD + "  Cell Voltages (mV)" + RESET)
                ln(DIM + f"  {'Cell':<6} {'Raw':>10}   {'Filtered':>10}   Bar ({voltage_cell_min} — {voltage_cell_max} mV)" + RESET)
                ln(DIM + f"  {'-'*6}   {'-'*10}   {'-'*10}   {'-'*26}" + RESET)
                for i in range(7):
                    vi  = v[i]  if v  else None
                    vfi = vf[i] if vf else None
                    ln(f"  {cell_color(vfi)}Cell {i+1}  {fmt_f(vi):>10}   {fmt_f(vfi):>10}   {bar(vfi, voltage_cell_min, voltage_cell_max)}{RESET}")

                pack_v_volts  = (r['pack_v']  / 1000.0) if r['pack_v']  is not None else None
                pack_vf_volts = (r['pack_vf'] / 1000.0) if r['pack_vf'] is not None else None
                ln()
                ln(BOLD + "  Pack Voltage" + RESET)
                ln(f"  Raw:      {GREEN}{fmt_f(pack_v_volts):>9}V{RESET}   {bar(pack_v_volts,  19.6, 29.4)}")
                ln(f"  Filtered: {GREEN}{fmt_f(pack_vf_volts):>9}V{RESET}   {bar(pack_vf_volts, 19.6, 29.4)}")

                ln()
                ln(BOLD + "  Current (mA)" + RESET)
                ln(f"  Raw:      {current_color(r['current'])}{fmt_f(r['current']):>10} mA{RESET}   {bar(r['current'],  -20000, 20000)}")
                ln(f"  Filtered: {current_color(r['curr_filt'])}{fmt_f(r['curr_filt']):>10} mA{RESET}   {bar(r['curr_filt'], -20000, 20000)}")

                ln()
                ln(BOLD + "  Temperature" + RESET)
                ln(f"  Package:  {temp_color(r['temp_c'])}{fmt_f(r['temp_c']):>10} C{RESET}    {bar(r['temp_c'], -20, 80)}")

                fet_status = r['fet_status']
                ln()
                ln(BOLD + "  Status" + RESET)
                ln(f"  Main FET:         {fet_text(fet_status, BD_FET_MAIN)}")
                ln(f"  Pre-charge FET:   {fet_text(fet_status, BD_FET_PRE)}")
                ln(f"  FET Status raw:   {CYAN}{fmt_h(fet_status)}{RESET}")
                ln(f"  Learning Status:  {CYAN}{fmt_h(r['learning_status'])}{RESET}")

                ln()
                ln(DIM + f"  Read: {elapsed_ms:.1f} ms  |  Log: {log_filename}  |  Press 'q' to quit" + RESET)

            display_iter += 1
            sleep_s = POLL_INTERVAL_S - (time.time() - t_start)
            if sleep_s > 0:
                time.sleep(sleep_s)
    finally:
        log_file.close()


# ---------------------------------------------------------------
# Charging / Discharging modes ('C', 'D', 'CP', 'DP')
#
# Continuous (C/D) and Pulse (CP/DP, toggling FET ON/OFF every
# PULSE_ON_S/PULSE_OFF_S):
#   - C/CP (charging) stop controlling (and disable the FET) when
#     either:
#       a) any cell's filtered voltage reaches CHARGE_STOP_CELL_MV
#          (hard ceiling, always checked), or
#       b) filtered current drops to/below charging_term_current —
#          but only once any cell's filtered voltage has reached
#          CHARGE_CURRENT_MONITOR_MV (current isn't monitored for
#          termination before that point). In pulse mode this is
#          only checked while the FET is ON.
#   - D/DP (discharging) stop controlling (and disable the FET) when
#     any cell's filtered voltage <= voltage_cell_min (read from the
#     device).
#
# IMPORTANT: hitting a stop condition does NOT end the script. It
# just disables the FET (retrying every tick until it succeeds) and
# the script keeps reading, logging, and displaying at 20Hz — same
# CSV file — in a passive monitoring state. Only pressing 'q' ends
# the script.
#
# Current-direction (sign) checking is disabled in all modes.
#
# In every case, the FET is guaranteed to be disabled on the way
# out (stop condition, 'q', or a crash).
# ---------------------------------------------------------------
def do_charge_discharge(ser, port, mode):
    is_charging = mode.startswith("charging")
    pulse       = mode.endswith("_pulse")
    label       = ("Charging" if is_charging else "Discharging") + (" (Pulse)" if pulse else "")

    fet_is_on = False

    try:
        # -------------------------------------------------------
        # Step 1 — load charge configuration once
        # -------------------------------------------------------
        cfg = load_charge_config(ser)
        voltage_cell_max      = cfg['voltage_cell_max']
        voltage_cell_min      = cfg['voltage_cell_min']
        charging_term_current = cfg['charging_term_current']
        cell_capacity          = cfg['cell_capacity']
        cell_count             = cfg['cell_count']

        if pulse:
            print(f"  Pulse timing            : {PULSE_ON_S:.1f}s ON / {PULSE_OFF_S:.1f}s OFF")
        print()

        # -------------------------------------------------------
        # Step 2/3 — enter and verify learning mode
        # -------------------------------------------------------
        request_and_verify_learning_mode(ser)

        # -------------------------------------------------------
        # Step 4 — enable FET / start first pulse ON phase
        # -------------------------------------------------------
        print(f"Starting {label}...")
        ok, info = set_fet(ser, True)
        if not ok:
            print(f"ERR: set_fet(ENABLE) failed: {info}. Aborting.")
            sys.exit(1)
        fet_is_on = True
        print(f"{label} started.")
        time.sleep(0.5)

        # Pulse phase tracking (only meaningful when pulse == True)
        phase        = "ON"
        phase_start  = time.time()
        phase_reads  = 0

        # -------------------------------------------------------
        # Step 5 — monitor at 20Hz until stop condition
        # -------------------------------------------------------
        log_filename = f"openbms_{mode}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        log_file     = open(log_filename, 'w', newline='')
        log_writer   = csv.writer(log_file)
        log_writer.writerow(CSV_HEADER)

        print("\033[2J", end='', flush=True)
        print(f"Logging to: {log_filename}")
        time.sleep(0.5)

        t_start_all   = time.time()
        display_iter  = 0
        errors        = 0
        toggle_errors = 0
        disable_errors = 0
        user_quit       = False
        stopped         = False   # True once a voltage/current condition has fired
        final_stop_reason = None

        while not user_quit:

            if msvcrt.kbhit():
                if msvcrt.getwch().lower() == 'q':
                    user_quit = True
                    break

            t_start = time.time()

            # -------------------------------------------------------
            # Pulse toggle — only while still actively controlling.
            # Once stopped, the FET stays off and this is skipped.
            # -------------------------------------------------------
            if not stopped and pulse:
                threshold = PULSE_ON_S if phase == "ON" else PULSE_OFF_S
                if (t_start - phase_start) >= threshold:
                    want_on = (phase == "OFF")
                    ok, info = set_fet(ser, want_on)
                    if ok:
                        phase       = "ON" if want_on else "OFF"
                        fet_is_on   = want_on
                        phase_start = t_start
                        phase_reads = 0
                    else:
                        # retry on the next tick; don't advance phase_start
                        toggle_errors += 1

            r = read_all_monitor_regs(ser)
            elapsed_ms = (time.time() - t_start) * 1000
            elapsed_s  = time.time() - t_start_all

            if any(v is None for v in r.values()):
                errors += 1

            cell_v_filt = r['cell_v_filt']
            curr_filt   = r['curr_filt']

            newly_triggered_reason = ''

            if not stopped:
                # -------------------------------------------------------
                # Voltage stop condition — evaluated every read, both
                # continuous and pulse modes, regardless of FET phase.
                # Charging: hard ceiling at CHARGE_STOP_CELL_MV.
                # Discharging: device's voltage_cell_min register.
                # -------------------------------------------------------
                if cell_v_filt is not None:
                    for i, vfi in enumerate(cell_v_filt):
                        if vfi is None:
                            continue
                        if is_charging and vfi >= CHARGE_STOP_CELL_MV:
                            final_stop_reason = f"cell {i+1} filtered voltage {vfi:.1f} mV >= {CHARGE_STOP_CELL_MV} mV"
                            break
                        if (not is_charging) and vfi <= voltage_cell_min:
                            final_stop_reason = f"cell {i+1} filtered voltage {vfi:.1f} mV <= min {voltage_cell_min} mV"
                            break

                # -------------------------------------------------------
                # Termination-current stop condition — charging only, and
                # only monitored once any cell's filtered voltage has
                # reached CHARGE_CURRENT_MONITOR_MV. In pulse mode this is
                # only meaningful (and only checked) while the FET is ON.
                # -------------------------------------------------------
                if final_stop_reason is None and is_charging and cell_v_filt is not None and curr_filt is not None:
                    voltage_gate = any(vfi is not None and vfi >= CHARGE_CURRENT_MONITOR_MV
                                       for vfi in cell_v_filt)
                    current_check_ok = (not pulse) or (phase == "ON")
                    if voltage_gate and current_check_ok and abs(curr_filt) <= charging_term_current:
                        final_stop_reason = (f"filtered current {curr_filt:.1f} mA within termination current "
                                        f"{charging_term_current} mA (monitored above {CHARGE_CURRENT_MONITOR_MV} mV/cell)")

                if final_stop_reason is not None:
                    stopped = True
                    newly_triggered_reason = final_stop_reason

            # -------------------------------------------------------
            # Once stopped, keep the FET disabled — retry every tick
            # until the disable command actually succeeds
            # -------------------------------------------------------
            if stopped and fet_is_on:
                ok, info = set_fet(ser, False)
                if ok:
                    fet_is_on = False
                else:
                    disable_errors += 1

            phase_reads += 1

            commanded_fet_state = "ON" if fet_is_on else "OFF"

            # -------------------------------------------------------
            # Log to CSV — every read (20 Hz)
            # -------------------------------------------------------
            log_writer.writerow(make_csv_row(elapsed_s, r['uptime'], r['cell_v'], cell_v_filt,
                                             r['pack_v'], r['pack_vf'], r['current'], curr_filt,
                                             r['temp_c'], r['fet_status'], r['learning_status'],
                                             commanded_fet_state, newly_triggered_reason))
            log_file.flush()

            # -------------------------------------------------------
            # Refresh display — every 4th read (5 Hz)
            # -------------------------------------------------------
            if display_iter % DISPLAY_EVERY == 0:
                print(HOME, end='')

                ln(BOLD + CYAN + "=" * 72 + RESET)
                ln(f"  OpenBMS {label}  —  {port}  —  20Hz read / 5Hz display  —  errors: {errors}"
                   + (f"  |  toggle errs: {toggle_errors}" if (pulse and not stopped) else "")
                   + (f"  |  disable errs: {disable_errors}" if disable_errors else ""))
                ln(BOLD + CYAN + "=" * 72 + RESET)

                ln()
                h  = int(elapsed_s) // 3600
                m  = (int(elapsed_s) % 3600) // 60
                s  = int(elapsed_s) % 60
                ln(BOLD + f"  {label} elapsed: {h:02d}:{m:02d}:{s:02d}" + RESET)
                up = r['uptime']
                ln(f"  Device uptime:  {up} ms" if up is not None else "  Device uptime:  ERR")

                ln()
                ln(BOLD + "  Pack Info" + RESET)
                ln(f"  Design capacity:   {cell_capacity} mAh")
                ln(f"  Cell count:        {cell_count}")
                ln(f"  Cell voltage max:  {voltage_cell_max} mV")
                ln(f"  Cell voltage min:  {voltage_cell_min} mV")

                if stopped:
                    ln()
                    ln(RED + BOLD + "  STOPPED: " + RESET + RED + f"{final_stop_reason}" + RESET)
                    ln(DIM + "  FET disabled — now monitoring only. Press 'q' to quit." + RESET)
                elif pulse:
                    threshold_now = PULSE_ON_S if phase == "ON" else PULSE_OFF_S
                    remaining = max(0.0, threshold_now - (time.time() - phase_start))
                    phase_col = GREEN if phase == "ON" else DIM
                    ln(f"  Pulse phase:    {phase_col}{BOLD}{phase}{RESET}   (next toggle in {remaining:.1f}s)")

                v, vf = r['cell_v'], cell_v_filt
                bar_hi = CHARGE_STOP_CELL_MV if is_charging else voltage_cell_max
                ln()
                ln(BOLD + "  Cell Voltages (mV)" + RESET)
                ln(DIM + f"  {'Cell':<6} {'Raw':>10}   {'Filtered':>10}   Bar ({voltage_cell_min} — {bar_hi} mV)" + RESET)
                ln(DIM + f"  {'-'*6}   {'-'*10}   {'-'*10}   {'-'*26}" + RESET)
                for i in range(7):
                    vi  = v[i]  if v  else None
                    vfi = vf[i] if vf else None
                    ln(f"  {cell_color(vfi)}Cell {i+1}  {fmt_f(vi):>10}   {fmt_f(vfi):>10}   "
                       f"{bar(vfi, voltage_cell_min, bar_hi)}{RESET}")
                if is_charging and not stopped:
                    ln(DIM + f"  (charging stop: any cell >= {CHARGE_STOP_CELL_MV} mV)" + RESET)

                pack_v_volts  = (r['pack_v']  / 1000.0) if r['pack_v']  is not None else None
                pack_vf_volts = (r['pack_vf'] / 1000.0) if r['pack_vf'] is not None else None
                ln()
                ln(BOLD + "  Pack Voltage" + RESET)
                ln(f"  Raw:      {GREEN}{fmt_f(pack_v_volts):>9}V{RESET}")
                ln(f"  Filtered: {GREEN}{fmt_f(pack_vf_volts):>9}V{RESET}")

                ln()
                ln(BOLD + "  Current (mA)" + RESET)
                ln(f"  Raw:      {current_color(r['current'])}{fmt_f(r['current']):>10} mA{RESET}")
                ln(f"  Filtered: {current_color(curr_filt)}{fmt_f(curr_filt):>10} mA{RESET}")
                if is_charging and not stopped:
                    gate_on = bool(cell_v_filt) and any(
                        vfi is not None and vfi >= CHARGE_CURRENT_MONITOR_MV for vfi in cell_v_filt)
                    gate_txt = (GREEN + "ACTIVE" + RESET) if gate_on else (DIM + "waiting for >= " + f"{CHARGE_CURRENT_MONITOR_MV} mV/cell" + RESET)
                    ln(f"  Term. current stop:  <= {charging_term_current} mA   [{gate_txt}]"
                       + ("  (checked only while FET is ON)" if pulse else ""))

                ln()
                ln(BOLD + "  Temperature" + RESET)
                ln(f"  Package:  {temp_color(r['temp_c'])}{fmt_f(r['temp_c']):>10} C{RESET}")

                fet_status = r['fet_status']
                ln()
                ln(BOLD + "  Status" + RESET)
                status_txt = (DIM + "STOPPED (monitoring)" + RESET) if stopped else (GREEN + BOLD + "ACTIVE" + RESET)
                ln(f"  {label}:{' ' * max(1, 16 - len(label))}{status_txt}")
                ln(f"  Main FET:         {fet_text(fet_status, BD_FET_MAIN)}")
                ln(f"  Pre-charge FET:   {fet_text(fet_status, BD_FET_PRE)}")
                ln(f"  FET Status raw:   {CYAN}{fmt_h(fet_status)}{RESET}")
                ln(f"  Learning Status:  {CYAN}{fmt_h(r['learning_status'])}{RESET}")

                ln()
                ln(DIM + f"  Read: {elapsed_ms:.1f} ms  |  Log: {log_filename}  |  Press 'q' to stop" + RESET)

            display_iter += 1
            sleep_s = POLL_INTERVAL_S - (time.time() - t_start)
            if sleep_s > 0:
                time.sleep(sleep_s)

        print(RESET)
        if final_stop_reason:
            print(f"{mode} stop condition hit: {final_stop_reason}")
            print("FET was disabled and monitoring continued until you quit.")
        else:
            print(f"Stopped {mode}: user requested stop ('q')")

        log_file.close()

    finally:
        # -------------------------------------------------------
        # Always disable the FET on the way out, whatever happened
        # -------------------------------------------------------
        if fet_is_on:
            print("Disabling FET...")
            ok, info = set_fet(ser, False)
            if ok:
                print("FET disabled.")
                fet_is_on = False
            else:
                print(f"WARNING: failed to disable FET ({info}). Check the device manually!")


# ---------------------------------------------------------------
# HPPC dashboard renderer, shared by REST and RAMP segments
# ---------------------------------------------------------------
def render_hppc_dashboard(port, label, r, segment_type, target_pct, coulomb_mah, target_mah,
                          rest_remaining_s, voltage_cell_min, voltage_cell_max,
                          charging_term_current, is_charging, errors, log_filename, elapsed_s,
                          cell_capacity, cell_count, final_stop_reason=None):
    print(HOME, end='')

    ln(BOLD + CYAN + "=" * 72 + RESET)
    ln(f"  OpenBMS {label}  —  {port}  —  20Hz read / 5Hz display  —  errors: {errors}")
    ln(BOLD + CYAN + "=" * 72 + RESET)

    ln()
    h  = int(elapsed_s) // 3600
    m  = (int(elapsed_s) % 3600) // 60
    s  = int(elapsed_s) % 60
    ln(BOLD + f"  {label} elapsed: {h:02d}:{m:02d}:{s:02d}" + RESET)
    up = r['uptime']
    ln(f"  Device uptime:  {up} ms" if up is not None else "  Device uptime:  ERR")

    ln()
    ln(BOLD + "  Pack Info" + RESET)
    ln(f"  Design capacity:   {cell_capacity} mAh")
    ln(f"  Cell count:        {cell_count}")
    ln(f"  Cell voltage max:  {voltage_cell_max} mV")
    ln(f"  Cell voltage min:  {voltage_cell_min} mV")

    ln()
    ln(BOLD + "  HPPC Segment" + RESET)
    if segment_type == "STOPPED":
        ln(RED + BOLD + "  STOPPED: " + RESET + RED + f"{final_stop_reason}" + RESET)
        ln(DIM + "  FET disabled — now monitoring only. Press 'q' to quit." + RESET)
    elif segment_type == "REST":
        ln(f"  Phase:          {DIM}RESTING{RESET}   at checkpoint {target_pct}%"
           f"   (next in {rest_remaining_s:.0f}s)")
    else:
        if target_mah is not None:
            ln(f"  Phase:          {GREEN}{BOLD}RAMPING{RESET}   toward checkpoint {target_pct}%")
            ln(f"  Coulomb count:  {coulomb_mah:8.2f} / {target_mah:.2f} mAh   "
               f"{bar(coulomb_mah, 0, target_mah)}")
        else:
            ln(f"  Phase:          {GREEN}{BOLD}RAMPING{RESET}   (open-ended — no more checkpoints,"
               f" running until safety stop)")
            ln(f"  Coulomb count:  {coulomb_mah:8.2f} mAh")

    v, vf = r['cell_v'], r['cell_v_filt']
    bar_hi = CHARGE_STOP_CELL_MV if is_charging else voltage_cell_max
    ln()
    ln(BOLD + "  Cell Voltages (mV)" + RESET)
    ln(DIM + f"  {'Cell':<6} {'Raw':>10}   {'Filtered':>10}   Bar ({voltage_cell_min} — {bar_hi} mV)" + RESET)
    ln(DIM + f"  {'-'*6}   {'-'*10}   {'-'*10}   {'-'*26}" + RESET)
    for i in range(7):
        vi  = v[i]  if v  else None
        vfi = vf[i] if vf else None
        ln(f"  {cell_color(vfi)}Cell {i+1}  {fmt_f(vi):>10}   {fmt_f(vfi):>10}   "
           f"{bar(vfi, voltage_cell_min, bar_hi)}{RESET}")
    if is_charging and segment_type != "STOPPED":
        ln(DIM + f"  (charging stop: any cell >= {CHARGE_STOP_CELL_MV} mV)" + RESET)

    pack_v_volts  = (r['pack_v']  / 1000.0) if r['pack_v']  is not None else None
    pack_vf_volts = (r['pack_vf'] / 1000.0) if r['pack_vf'] is not None else None
    ln()
    ln(BOLD + "  Pack Voltage" + RESET)
    ln(f"  Raw:      {GREEN}{fmt_f(pack_v_volts):>9}V{RESET}")
    ln(f"  Filtered: {GREEN}{fmt_f(pack_vf_volts):>9}V{RESET}")

    ln()
    ln(BOLD + "  Current (mA)" + RESET)
    ln(f"  Raw:      {current_color(r['current'])}{fmt_f(r['current']):>10} mA{RESET}")
    ln(f"  Filtered: {current_color(r['curr_filt'])}{fmt_f(r['curr_filt']):>10} mA{RESET}")
    if is_charging and segment_type != "STOPPED":
        gate_on = bool(vf) and any(vfi is not None and vfi >= CHARGE_CURRENT_MONITOR_MV for vfi in vf)
        gate_txt = (GREEN + "ACTIVE" + RESET) if gate_on else (DIM + f"waiting for >= {CHARGE_CURRENT_MONITOR_MV} mV/cell" + RESET)
        ln(f"  Term. current stop:  <= {charging_term_current} mA   [{gate_txt}]")

    ln()
    ln(BOLD + "  Temperature" + RESET)
    ln(f"  Package:  {temp_color(r['temp_c'])}{fmt_f(r['temp_c']):>10} C{RESET}")

    fet_status = r['fet_status']
    ln()
    ln(BOLD + "  Status" + RESET)
    ln(f"  Main FET:         {fet_text(fet_status, BD_FET_MAIN)}")
    ln(f"  Pre-charge FET:   {fet_text(fet_status, BD_FET_PRE)}")
    ln(f"  FET Status raw:   {CYAN}{fmt_h(fet_status)}{RESET}")
    ln(f"  Learning Status:  {CYAN}{fmt_h(r['learning_status'])}{RESET}")

    ln()
    ln(DIM + f"  Log: {log_filename}  |  Press 'q' to stop" + RESET)


# ---------------------------------------------------------------
# HPPC modes ('CHPPC', 'DHPPC')
#
# Alternates REST (450s, FET off) and RAMP (FET continuously ON)
# segments. Each RAMP segment targets the next SOC checkpoint,
# tracked via coulomb counting (|filtered current| integrated over
# time) against a target derived from design capacity. After the
# last checkpoint's rest, ramping continues open-ended until the
# safety stop condition is hit:
#   - discharging: any cell's filtered voltage <= voltage_cell_min
#     (read from the device)
#   - charging: any cell's filtered voltage >= CHARGE_STOP_CELL_MV
#     (hard ceiling, always checked), OR filtered current drops
#     to/below charging_term_current — but only once any cell's
#     filtered voltage has reached CHARGE_CURRENT_MONITOR_MV
# The safety condition is checked on every read throughout every
# RAMP segment, so the test can end mid-ramp before reaching the
# next checkpoint.
#
# IMPORTANT: hitting the safety stop condition does NOT end the
# script. It disables the FET (retrying every tick until it
# succeeds) and the checkpoint schedule is abandoned, but the
# script keeps reading, logging, and displaying at 20Hz — same CSV
# file — in a passive monitoring state. Only pressing 'q' ends the
# script.
# ---------------------------------------------------------------
def do_hppc(ser, port, mode):
    is_charging = mode.startswith("charging")
    label       = "Charging HPPC" if is_charging else "Discharging HPPC"
    points      = CHARGE_HPPC_POINTS if is_charging else DISCHARGE_HPPC_POINTS

    fet_is_on = False

    try:
        # -------------------------------------------------------
        # Step 1 — load charge configuration once
        # -------------------------------------------------------
        cfg = load_charge_config(ser)
        voltage_cell_max      = cfg['voltage_cell_max']
        voltage_cell_min      = cfg['voltage_cell_min']
        charging_term_current = cfg['charging_term_current']
        design_capacity_mah   = float(cfg['cell_capacity'])
        cell_count            = cfg['cell_count']

        print(f"  HPPC checkpoints (%)    : {points}")
        print(f"  Rest per checkpoint     : {HPPC_REST_S:.0f}s")
        print()

        # -------------------------------------------------------
        # Step 2/3 — enter and verify learning mode
        # -------------------------------------------------------
        request_and_verify_learning_mode(ser)

        # -------------------------------------------------------
        # Ensure a known FET state (OFF) before the test begins
        # -------------------------------------------------------
        ok, info = set_fet(ser, False)
        if not ok:
            print(f"ERR: failed to ensure FET disabled before start: {info}. Aborting.")
            sys.exit(1)
        fet_is_on = False

        print(f"Starting {label}.")
        time.sleep(0.5)

        # -------------------------------------------------------
        # Build the segment list:
        #   initial REST at points[0], then for each subsequent
        #   checkpoint: RAMP to it (coulomb-counted), then REST
        #   there. Finally, an open-ended RAMP with no target.
        # -------------------------------------------------------
        segments = [("REST", HPPC_REST_S, points[0])]
        for idx in range(1, len(points)):
            delta_pct   = abs(points[idx] - points[idx - 1])
            target_mah  = design_capacity_mah * delta_pct / 100.0
            segments.append(("RAMP", target_mah, points[idx]))
            segments.append(("REST", HPPC_REST_S, points[idx]))
        segments.append(("RAMP", None, None))   # open-ended final ramp

        log_filename = f"openbms_{mode}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        log_file     = open(log_filename, 'w', newline='')
        log_writer   = csv.writer(log_file)
        log_writer.writerow(HPPC_CSV_HEADER)

        print("\033[2J", end='', flush=True)
        print(f"Logging to: {log_filename}")
        time.sleep(0.5)

        t_start_all  = time.time()
        display_iter = 0
        errors       = 0
        user_quit          = False
        safety_stop_reason = None

        seg_index = 0
        while seg_index < len(segments) and not user_quit and safety_stop_reason is None:
            seg_type, seg_param, seg_target_pct = segments[seg_index]

            # =====================================================
            # REST segment — FET off, wait out the duration
            # =====================================================
            if seg_type == "REST":
                duration = seg_param

                if fet_is_on:
                    ok, info = set_fet(ser, False)
                    if ok:
                        fet_is_on = False
                    else:
                        print(f"WARNING: failed to disable FET before rest ({info})")

                rest_start = time.time()
                while (time.time() - rest_start) < duration and not user_quit:
                    if msvcrt.kbhit():
                        if msvcrt.getwch().lower() == 'q':
                            user_quit = True
                            break

                    t_start = time.time()
                    r = read_all_monitor_regs(ser)
                    elapsed_s = time.time() - t_start_all

                    if any(v is None for v in r.values()):
                        errors += 1

                    log_writer.writerow(make_hppc_csv_row(elapsed_s, r, "OFF", "REST",
                                                          seg_target_pct, 0.0))
                    log_file.flush()

                    if display_iter % DISPLAY_EVERY == 0:
                        remaining = max(0.0, duration - (time.time() - rest_start))
                        render_hppc_dashboard(port, label, r, "REST", seg_target_pct, 0.0, None,
                                              remaining, voltage_cell_min, voltage_cell_max,
                                              charging_term_current, is_charging, errors,
                                              log_filename, elapsed_s,
                                              cfg['cell_capacity'], cell_count)

                    display_iter += 1
                    sleep_s = POLL_INTERVAL_S - (time.time() - t_start)
                    if sleep_s > 0:
                        time.sleep(sleep_s)

            # =====================================================
            # RAMP segment — FET on, coulomb-count toward target
            # (or run open-ended if target_mah is None). If a safety
            # condition fires, the FET is disabled immediately and
            # the segment loop (and outer segments loop) ends; the
            # script then falls through to passive monitoring below.
            # =====================================================
            else:
                target_mah = seg_param

                if not fet_is_on:
                    ok, info = set_fet(ser, True)
                    if not ok:
                        print(f"ERR: set_fet(ENABLE) failed: {info}. Aborting.")
                        sys.exit(1)
                    fet_is_on = True

                coulomb_mah   = 0.0
                last_t        = time.time()
                segment_done  = False

                while not user_quit and not segment_done and safety_stop_reason is None:
                    if msvcrt.kbhit():
                        if msvcrt.getwch().lower() == 'q':
                            user_quit = True
                            break

                    t_start = time.time()
                    r = read_all_monitor_regs(ser)
                    elapsed_s = time.time() - t_start_all

                    if any(v is None for v in r.values()):
                        errors += 1

                    cell_v_filt = r['cell_v_filt']
                    curr_filt   = r['curr_filt']

                    now  = time.time()
                    dt_h = (now - last_t) / 3600.0
                    last_t = now
                    if curr_filt is not None:
                        coulomb_mah += abs(curr_filt) * dt_h

                    # -------------------------------------------
                    # Safety stop condition — checked every read.
                    # Charging: hard ceiling at CHARGE_STOP_CELL_MV,
                    # plus a termination-current stop that's only
                    # monitored once any cell reaches
                    # CHARGE_CURRENT_MONITOR_MV. Discharging: the
                    # device's voltage_cell_min register.
                    # -------------------------------------------
                    if cell_v_filt is not None:
                        for i, vfi in enumerate(cell_v_filt):
                            if vfi is None:
                                continue
                            if is_charging and vfi >= CHARGE_STOP_CELL_MV:
                                safety_stop_reason = f"cell {i+1} filtered voltage {vfi:.1f} mV >= {CHARGE_STOP_CELL_MV} mV"
                                break
                            if (not is_charging) and vfi <= voltage_cell_min:
                                safety_stop_reason = f"cell {i+1} filtered voltage {vfi:.1f} mV <= min {voltage_cell_min} mV"
                                break

                    if safety_stop_reason is None and is_charging and cell_v_filt is not None and curr_filt is not None:
                        voltage_gate = any(vfi is not None and vfi >= CHARGE_CURRENT_MONITOR_MV
                                           for vfi in cell_v_filt)
                        if voltage_gate and abs(curr_filt) <= charging_term_current:
                            safety_stop_reason = (f"filtered current {curr_filt:.1f} mA within termination current "
                                            f"{charging_term_current} mA (monitored above {CHARGE_CURRENT_MONITOR_MV} mV/cell)")

                    newly_triggered_reason = ''
                    if safety_stop_reason is not None:
                        newly_triggered_reason = safety_stop_reason
                        # Disable the FET immediately — retried in the
                        # passive-monitoring phase below if this fails
                        ok, info = set_fet(ser, False)
                        if ok:
                            fet_is_on = False

                    log_writer.writerow(make_hppc_csv_row(elapsed_s, r, "ON" if fet_is_on else "OFF", "RAMP",
                                                          seg_target_pct, coulomb_mah,
                                                          newly_triggered_reason))
                    log_file.flush()

                    if display_iter % DISPLAY_EVERY == 0:
                        render_hppc_dashboard(port, label, r, "RAMP", seg_target_pct, coulomb_mah,
                                              target_mah, 0.0, voltage_cell_min, voltage_cell_max,
                                              charging_term_current, is_charging, errors,
                                              log_filename, elapsed_s,
                                              cfg['cell_capacity'], cell_count)

                    display_iter += 1

                    if safety_stop_reason is None and target_mah is not None and coulomb_mah >= target_mah:
                        segment_done = True

                    sleep_s = POLL_INTERVAL_S - (time.time() - t_start)
                    if sleep_s > 0:
                        time.sleep(sleep_s)

            seg_index += 1

        # -------------------------------------------------------
        # If a safety condition stopped the test (rather than the
        # user), the FET is already disabled (or being retried) —
        # keep reading, logging, and displaying at 20Hz in a passive
        # monitoring state until the user presses 'q'.
        # -------------------------------------------------------
        disable_errors = 0
        while safety_stop_reason is not None and not user_quit:
            if msvcrt.kbhit():
                if msvcrt.getwch().lower() == 'q':
                    user_quit = True
                    break

            t_start = time.time()
            r = read_all_monitor_regs(ser)
            elapsed_s = time.time() - t_start_all

            if any(v is None for v in r.values()):
                errors += 1

            if fet_is_on:
                ok, info = set_fet(ser, False)
                if ok:
                    fet_is_on = False
                else:
                    disable_errors += 1

            log_writer.writerow(make_hppc_csv_row(elapsed_s, r, "OFF", "STOPPED", None, 0.0, ''))
            log_file.flush()

            if display_iter % DISPLAY_EVERY == 0:
                render_hppc_dashboard(port, label, r, "STOPPED", None, 0.0, None, 0.0,
                                      voltage_cell_min, voltage_cell_max, charging_term_current,
                                      is_charging, errors, log_filename, elapsed_s,
                                      cfg['cell_capacity'], cell_count,
                                      final_stop_reason=safety_stop_reason)

            display_iter += 1
            sleep_s = POLL_INTERVAL_S - (time.time() - t_start)
            if sleep_s > 0:
                time.sleep(sleep_s)

        print(RESET)
        if safety_stop_reason:
            print(f"{mode} safety stop hit: {safety_stop_reason}")
            print("FET was disabled and monitoring continued until you quit.")
        else:
            print(f"Stopped {mode}: user requested stop ('q')")

        log_file.close()

    finally:
        if fet_is_on:
            print("Disabling FET...")
            ok, info = set_fet(ser, False)
            if ok:
                print("FET disabled.")
                fet_is_on = False
            else:
                print(f"WARNING: failed to disable FET ({info}). Check the device manually!")


# ---------------------------------------------------------------
# Main
# ---------------------------------------------------------------
def main():
    if len(sys.argv) != 3 or sys.argv[1].upper() not in MODE_MAP:
        print("Usage: python battery_cycler.py <MODE> <COM_PORT>")
        print("  MODE:")
        print("    M      monitor              — read-only, no mode/FET commands")
        print("    C      charging             — continuous charge")
        print("    D      discharging          — continuous discharge")
        print("    CP     charging_pulse       — pulsed charge (2s ON / 2s OFF)")
        print("    DP     discharging_pulse    — pulsed discharge (2s ON / 2s OFF)")
        print("    CHPPC  charging_hppc        — charge HPPC test (checkpoint ramp + 450s rests)")
        print("    DHPPC  discharging_hppc     — discharge HPPC test (checkpoint ramp + 450s rests)")
        print("Example: python battery_cycler.py DHPPC COM3")
        sys.exit(1)

    mode = MODE_MAP[sys.argv[1].upper()]
    port = sys.argv[2]

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT_RESPONSE)
        time.sleep(0.3)
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(f"ERR: failed to open {port}: {e}")
        sys.exit(1)

    try:
        if mode == "monitor":
            do_monitor(ser, port)
        elif mode in ("charging_hppc", "discharging_hppc"):
            do_hppc(ser, port, mode)
        else:
            do_charge_discharge(ser, port, mode)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
