#!/usr/bin/env python3
"""
hppc_simulate.py -- runs the HPPC test on the OpenBMS simulation build, both directions, with
the failure cases first, and saves the fuel gauge registers.

Usage:
    python hppc_simulate.py <test_folder> [c|d] [COM_PORT]
    python hppc_simulate.py <test_folder> plot [c|d] [run]

<test_folder> is a path, or just the name of a folder next to this script, for example
7_cell_2000mAh. It holds hppc.json (the simulation file); the result is written to
<test_folder>/fuel_gauge_registers.json and every run is logged to <test_folder>/log_hppc.csv.
Without COM_PORT the board is found by pinging every
serial port. Without arguments, the test folders found here are listed.

The board must run firmware built with OPENBMS_SIMULATION (config.h) and carry the config of the
test folder (config_tool.py w <test_folder>), so it comes back in normal run after a reboot.

c runs only the charge direction, d only the discharge direction, without it both run.

For charge, then for discharge, the script runs five HPPC tests, each after a reboot of the board
and each from a fresh simulated battery:

    1. high current       from faults.start_after_s on the charger or load gives high_current_c (C of
                          capacity_mah, above HPPC_CURRENT_MAX_C): the board must stop with current
                          range (config.json keeps ocp_charge_threshold_ma above it, so OCP does not trip)
    2. low current        the same with low_current_c: the board must stop (current range)
    3. high temperature   the pack temperature goes to high_temperature_c: the board must stop
                          (temperature, after HPPC_TEMP_OUT_MS outside its window)
    4. low temperature    the same with low_temperature_c
    5. good run           the test runs to its end, the board writes its tables; the script compares
                          them with the simulated cells and reads every fuel gauge register

A failure run that has not ended faults.timeout_s after the fault started is stopped and FAILs.
At the end a PASS/FAIL summary is printed. When both good runs wrote tables, the two register
sets are merged (charge tables from the charge run, everything else from the discharge run) and
saved to fuel_gauge_registers.json, the battery model the other simulation scripts load. Each good
run updates the file right after it, so a stop later on keeps it: charge replaces the charge
tables and model_temp_chg_c, discharge everything else; the other direction stays as the file
had it.

Log (log_hppc.csv): one file for all runs of one script call, written new each time. A row per
read-back (once per real second): direction, run (the fault name or good_run), simulated time,
true current, temperature and SOC of each simulated cell, then the board's registers (MONITOR_REGS).

While a test runs, one live line shows the board's read-back (voltages, current, temperature,
FET, HPPC state), updated in place. New lines are printed only for events: HPPC state changes,
the fault starting, protection trips, error flags and the end of the test.

plot: no test runs and no board is needed. The existing <test_folder>/log_hppc.csv is plotted, one
window per run in the log (c or d: only that direction; run: only that run, e.g. good_run): true and
board current, the board's cell voltages, true and board temperature, true SOC of every cell and the
HPPC state. A red line marks where the HPPC test stopped (DONE or ABORTED).

Press q to stop (on Windows without Enter, elsewhere q then Enter): the running test is stopped;
a direction whose good run had already finished is saved, the running one is not.

Simulation file (hppc.json):
    cell_count        number of simulated cells, must match the board's cell_count
    dt_ms             simulated time step per sample
    temperature_c     pack temperature sent with every sample (the board uses it instead of its NTC)
    max_sim_time_s    a good run is stopped after this much simulated time
    start_soc_pct     {"charge": .., "discharge": ..}: SOC the simulated cells start each run at
    discharge         load_ohm: load resistor across the pack
    charge            current_ma: charger current, voltage_limit_mv: charger voltage (pack)
    faults            start_after_s, timeout_s, high_current_c, low_current_c,
                      high_temperature_c, low_temperature_c
    noise             gaussian noise: voltage_mv (per cell), current_ma; seed
    cell_default      the model of one cell:
                        capacity_mah,
                        ocv: formula coefficients k0..k5 (see "formula"); below soc_limit_low the
                             voltage keeps falling like a real empty cell, above 1 - soc_limit_high it is held,
                        soc_pct grid (2 % steps) with r0_mohm, r1_mohm, tau1_s, r2_mohm, tau2_s
    cells             one entry per cell; any key given there replaces cell_default for that cell

Current sign is the firmware's: + charge, - discharge.

Needs pyserial (pip install pyserial).
"""

import csv
import json
import math
import os
import random
import shutil
import struct
import sys
import time

# Key presses without Enter: msvcrt on Windows, select on stdin elsewhere
try:
    import msvcrt
except ImportError:
    msvcrt = None
    import select

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

# ---------------------------------------------------------------
# Protocol constants (see openbms_comm.h)
# ---------------------------------------------------------------
BAUD_RATE = 230400

CC_WRITE      = 0x01
CC_READ       = 0x02
CC_ACK        = 0x03
CC_ERROR      = 0x04
CC_HPPC_START = 0x06
CC_NORMAL_RUN = 0x07
CC_REBOOT     = 0x09
CC_PING       = 0x0A
CC_SIM_SAMPLE = 0x0E

CE_NAMES = {
    0: "CE_OK",
    1: "CE_WRONG_CMD",
    2: "CE_BAD_CRC",
    3: "CE_NO_REG",
    4: "CE_RO",
    5: "CE_EEPROM",
    6: "CE_STATE",
}

CELL_COUNT_MAX = 7      # SYS_CELL_COUNT_MAX in openbms_data.h, the sample always carries 7 cells

TIMEOUT_READ  = 1.0     # s -- register read / write / command
TIMEOUT_BOOT  = 10.0    # s -- how long to wait for the board after opening the port
PORT_POLL     = 0.02    # s -- serial read timeout
OPEN_SETTLE   = 0.5     # s -- pause after opening a port
REBOOT_PAUSE  = 1.0     # s -- give the board time to actually reset before pinging it

REG_CTRL_STATE = 0x13
REG_SOC_GRID   = 0xA0
REG_MODEL_CELL = 0x9F

CTRL_STATE_NORMAL_RUN = 2
CTRL_STATE_HPPC_RUN   = 3

FET_MAIN = 0x01         # PER_FET_MAIN

CTRL_STATES = {1: "WAIT_CONFIG", 2: "NORMAL_RUN", 3: "HPPC_RUN"}
HPPC_STATES = {0: "IDLE", 1: "DETECT", 2: "REST", 3: "RAMP", 4: "DONE", 5: "ABORTED"}
HPPC_STOPS = {
    0: "none", 1: "cell max", 2: "cell min", 3: "term current", 4: "no current",
    5: "no config", 6: "host", 7: "protection", 8: "current range", 9: "temperature",
}
HPPC_X_BITS = {8: "BUSY", 9: "DONE", 10: "OVERRUN", 11: "FIT_WARN", 12: "GAP_LO", 13: "GAP_HI", 14: "FEW_CYCLES"}
HPPC_X_BUSY = 1 << 8
HPPC_STATE_DONE    = 4
HPPC_STATE_ABORTED = 5
# error_flags bits: OpenBMS_Error_t in openbms_error.h, in order
ERROR_NAMES = [
    "EEPROM_READ_FAIL", "EEPROM_WRITE_FAIL", "EEPROM_CRC_FAIL",
    "ADS_READ_FAIL", "ADS_INIT_FAIL", "ADS_ID_FAIL", "ADS_CRC_FAIL", "ADS_RESET", "ADS_DATA_LOST",
    "HW_DRV_FAULT", "HW_DRV_GATE_FAULT", "HW_PWR_FAIL",
    "UART_ERROR", "UART_TX_TIMEOUT", "NTC_FAULT",
    "I2C_ERROR", "CAN_ERROR",
]
HPPC_X_DONE = 1 << 9
# protection_trigger bits (0x12): CTRL_PROT_* in openbms_data.h, bit 0 first
PROTECTION_NAMES = [
    "OVP_SLOW", "OVP_FAST", "UVP_SLOW", "UVP_FAST",
    "OCP_CHARGE", "OCP_DISCHARGE_SLOW", "OCP_DISCHARGE_FAST", "OTP",
    "UTP", "OTP_CHARGE", "UTP_CHARGE",
]

# Table registers per direction: OCV, R0, R1, tau1, R2, tau2 (units V, Ohm, Ohm, s, Ohm, s)
TABLE_REGS = {
    "discharge": [0xA1, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7],
    "charge":    [0xA2, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC],
}
TABLE_NAMES = ["ocv_v", "r0_mohm", "r1_mohm", "tau1_s", "r2_mohm", "tau2_s"]
TABLE_SCALE = [1.0, 1000.0, 1000.0, 1.0, 1000.0, 1.0]    # board unit -> printed unit
SOC_GRID_POINTS = 20

READ_EVERY_S  = 1.0     # real (wall clock) seconds between register read-backs, shown in the live line
QUIT_KEY      = "q"     # pressing it stops the run: the HPPC test is stopped (the board stays in HPPC run until the next reboot)
STOP_SAMPLES  = 100     # most samples sent after the stop command until the test shows it ended

# Registers read back once per READ_EVERY_S: (address, name, struct format, count)
MONITOR_REGS = [
    (0x00, "cell_mv",          "f", 7),
    (0x01, "pack_mv",          "f", 1),
    (0x02, "current_ma",       "f", 1),
    (0x03, "temp_pack_c",      "f", 1),
    (0x04, "temp_stm32_c",     "f", 1),
    (0x06, "fet_status",       "H", 1),
    (0x12, "protection_trip",  "H", 1),
    (0x13, "ctrl_state",       "B", 1),
    (0x15, "error_flags",      "Q", 1),
    (0x17, "uptime_ms",        "I", 1),
    (0x18, "hppc_direction",   "B", 1),
    (0x19, "hppc_status",      "H", 1),
    (0x1A, "hppc_extract",     "H", 1),
    (0x90, "soc_rel_pct",      "B", 1),
    (0x92, "cell_soc_pct",     "B", 7),
    (0x94, "cell_remain_mah",  "H", 7),
]

# Sample: float cell_mv[7], float current_ma, float temperature_c, uint16 dt_ms. Reply: see Comm_SimSample in
# openbms_comm.c (state after the last sample run, then the number of samples run)
SAMPLE_FORMAT = "<7fffH"
REPLY_FORMAT  = "<HBHHHQB"
SIM_BATCH_MAX = 6           # samples in one CC_SIM_SAMPLE frame: 255 B payload / 38 B per sample


CONFIG_FILE = "hppc.json"
OUTPUT_FILE = "fuel_gauge_registers.json"
LOG_FILE    = "log_hppc.csv"

# Failure runs, in order: name, the stop reasons that pass
FAULTS = {
    "high_current":     ("high current",     (8,)),        # HPPC current range, not the OCP protection
    "low_current":      ("low current",      (8,)),
    "high_temperature": ("high temperature", (9,)),
    "low_temperature":  ("low temperature",  (9,)),
}
# Stop reasons of a good run per direction
GOOD_STOPS = {"charge": (1, 3), "discharge": (2,)}

# learning_status (0x96) bits
LEARN_CHARGE    = 1 << 0
LEARN_DISCHARGE = 1 << 1
LEARN_COMPLETE  = 1 << 2

# Fuel gauge registers saved to the JSON file: (address, name, struct format, count), board units
FG_SHARED_REGISTERS = [
    (0x90, "relative_soc",            "B", 1),
    (0x92, "cell_soc",                "B", 7),
    (0x93, "cell_soh",                "B", 7),
    (0x94, "cell_remaining_capacity", "H", 7),
    (0x96, "learning_status",         "H", 1),
    (0xA0, "soc_grid",                "f", 20),
    (0xAD, "q_temp_c",                "f", 5),
    (0xAE, "q_temp_factor",           "f", 5),
    (0xAF, "q_nom_ah",                "f", 1),
    (0xB0, "coulombic_efficiency",    "f", 1),
    (0xB1, "model_temp_dis_c",        "f", 1),
    (0xB2, "model_temp_chg_c",        "f", 1),
    (0xB6, "ea_r0",                   "f", 1),
    (0xB7, "ea_r1",                   "f", 1),
    (0xB8, "ea_tau1",                 "f", 1),
    (0xB9, "ea_r2",                   "f", 1),
    (0xBA, "ea_tau2",                 "f", 1),
    (0xBB, "kf_q_soc",                "f", 1),
    (0xBC, "kf_q_rc1",                "f", 1),
    (0xBD, "kf_q_rc2",                "f", 1),
    (0xBE, "kf_r_v",                  "f", 1),
    (0xD0, "cell_soc_f",              "f", 7),
    (0xD1, "cell_vrc1",               "f", 7),
    (0xD2, "cell_vrc2",               "f", 7),
    (0xD3, "cell_p",                  "f", 42),
    (0xD7, "cycle_count",             "H", 1),
]
# Per cell, after the cell is picked with 0x9F
FG_CELL_REGISTERS = [
    (0xA1, "ocv_dis",     "f", 20),
    (0xA2, "ocv_chg",     "f", 20),
    (0xA3, "r0_dis",      "f", 20),
    (0xA4, "r1_dis",      "f", 20),
    (0xA5, "tau1_dis",    "f", 20),
    (0xA6, "r2_dis",      "f", 20),
    (0xA7, "tau2_dis",    "f", 20),
    (0xA8, "r0_chg",      "f", 20),
    (0xA9, "r1_chg",      "f", 20),
    (0xAA, "tau1_chg",    "f", 20),
    (0xAB, "r2_chg",      "f", 20),
    (0xAC, "tau2_chg",    "f", 20),
    (0xD4, "capacity_ah", "f", 1),
]
FG_CHARGE_CELL_NAMES = ["ocv_chg", "r0_chg", "r1_chg", "tau1_chg", "r2_chg", "tau2_chg"]

# Optional direction argument
DIRECTIONS = {"c": "charge", "d": "discharge"}


class SimError(Exception):
    pass


# ---------------------------------------------------------------
# Serial link (frames like config_tool.py)
# ---------------------------------------------------------------
def checksum(data):
    return sum(data) % 256


def build_frame(cmd, address=0, payload=b""):
    packet = bytes([cmd, address, len(payload)]) + payload
    return packet + bytes([checksum(packet)])


def transact(ser, frame, what):
    ser.reset_input_buffer()
    ser.write(frame)

    buf = b""
    deadline = time.time() + TIMEOUT_READ
    while time.time() < deadline:
        first = ser.read(1)
        if first:
            buf += first + ser.read(ser.in_waiting)
        if len(buf) >= 3 and len(buf) >= buf[2] + 4:
            break

    if len(buf) < 4 or len(buf) < buf[2] + 4:
        raise SimError(f"{what}: no complete answer from the board")

    raw = buf[:buf[2] + 4]
    if checksum(raw[:-1]) != raw[-1]:
        raise SimError(f"{what}: answer with a bad checksum")
    if raw[0] == CC_ERROR:
        code = raw[3]
        raise SimError(f"{what}: board answered {CE_NAMES.get(code, f'error {code}')}")
    return raw


def read_register(ser, address):
    raw = transact(ser, build_frame(CC_READ, address), f"read 0x{address:02X}")
    return raw[3:3 + raw[2]]


def write_register(ser, address, payload):
    raw = transact(ser, build_frame(CC_WRITE, address, payload), f"write 0x{address:02X}")
    if raw[0] != CC_ACK:
        raise SimError(f"write 0x{address:02X}: unexpected answer")


def send_command(ser, cmd, what):
    raw = transact(ser, build_frame(cmd), what)
    if raw[0] != CC_ACK:
        raise SimError(f"{what}: unexpected answer")


def send_samples(ser, samples):
    """Sends 1..SIM_BATCH_MAX samples (cell_mv, current_ma, temperature_c, dt_ms) in one frame. The board
    runs them in order and stops after the first one that changes its reply (FET, ctrl state, HPPC,
    protection, errors). Returns the number of samples it ran and the reply after the last one."""
    payload = b"".join(struct.pack(SAMPLE_FORMAT, *cell_mv, current_ma, temperature_c, dt_ms)
                       for cell_mv, current_ma, temperature_c, dt_ms in samples)
    raw = transact(ser, build_frame(CC_SIM_SAMPLE, 0, payload), "sim sample")
    if raw[0] != CC_SIM_SAMPLE or raw[2] != struct.calcsize(REPLY_FORMAT):
        raise SimError("sim sample: unexpected answer (is the firmware built with OPENBMS_SIMULATION, "
                       "and as new as this script?)")
    fet, state, hppc, extract, trip, errors, count = struct.unpack(REPLY_FORMAT, raw[3:3 + raw[2]])
    return count, {"fet": fet, "ctrl_state": state, "hppc_status": hppc,
                   "extract": extract, "trip": trip, "errors": errors}


def send_sample(ser, cell_mv, current_ma, temperature_c, dt_ms):
    return send_samples(ser, [(cell_mv, current_ma, temperature_c, dt_ms)])[1]


def snapshot(objects, rng):
    """State of the simulation objects (their plain attributes) and the random generator, to go back to"""
    return [(o, dict(o.__dict__)) for o in objects], rng, rng.getstate()


def restore(snap):
    objects, rng, rng_state = snap
    for o, attributes in objects:
        o.__dict__.clear()
        o.__dict__.update(attributes)
    rng.setstate(rng_state)


class SampleBatch:
    """Collects samples and sends up to SIM_BATCH_MAX of them in one frame (fewer round trips over the
    USB-UART bridge). Each sample comes with a snapshot taken before it was made. When the board stops
    a batch early, the later samples were made for the old reply (e.g. the FET still on): flush() hands
    back the snapshot of the first sample not run, and the caller goes back to it and makes them again."""

    def __init__(self, ser):
        self.ser = ser
        self.items = []
        self.reply = None       # reply after the last sample run, None before the first one

    def add(self, snap, sample, info=None):
        """Queues a sample. True when the batch should be sent now (full, or the reply is not known yet)."""
        self.items.append((snap, sample, info))
        return self.reply is None or len(self.items) >= SIM_BATCH_MAX

    def flush(self):
        """Sends the queued samples. Returns [(info, reply)] for each sample the board ran, and the
        snapshot to go back to (None when it ran them all)."""
        if not self.items:
            return [], None

        count, reply = send_samples(self.ser, [sample for _, sample, _ in self.items])
        if not 1 <= count <= len(self.items):
            raise SimError(f"sim sample: the board ran {count} of {len(self.items)} samples")

        done = []
        for k in range(count):
            # The board stops at a change: every sample before the last one left the reply as it was
            if k == count - 1:
                done.append((self.items[k][2], reply))
            else:
                done.append((self.items[k][2], self.reply))

        back = None
        if count < len(self.items):
            back = self.items[count][0]
        self.reply = reply
        self.items = []
        return done, back


def find_board():
    for port in sorted(p.device for p in list_ports.comports()):
        try:
            with serial.Serial(port, BAUD_RATE, timeout=PORT_POLL) as ser:
                time.sleep(OPEN_SETTLE)
                transact(ser, build_frame(CC_PING), "ping")
                return port
        except (SimError, serial.SerialException, OSError):
            pass
    raise SimError("no OpenBMS answered a ping on any port. Pass the port by hand.")


def wait_until_ready(ser):
    deadline = time.time() + TIMEOUT_BOOT
    while time.time() < deadline:
        try:
            transact(ser, build_frame(CC_PING), "ping")
            return
        except SimError:
            pass
    raise SimError(f"the board did not answer a ping within {TIMEOUT_BOOT:.0f} s")


# ---------------------------------------------------------------
# Battery model: OCV(SOC) + R0 + two RC pairs per cell, parameters vs SOC
# ---------------------------------------------------------------
def interp(x, xs, ys):
    """Linear interpolation, held flat outside the grid."""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            f = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + f * (ys[i] - ys[i - 1])
    return ys[-1]


class Cell:
    def __init__(self, p, start_soc_pct):
        self.capacity_as = p["capacity_mah"] / 1000.0 * 3600.0
        self.k = p["ocv"]["k"]
        self.soc_limit_low = p["ocv"]["soc_limit_low"]
        self.soc_limit_high = p["ocv"]["soc_limit_high"]
        self.grid = p["soc_pct"]
        self.tables = {name: p[name] for name in ("r0_mohm", "r1_mohm", "tau1_s", "r2_mohm", "tau2_s")}
        for name, values in self.tables.items():
            if len(values) != len(self.grid):
                raise SimError(f"{name} has {len(values)} values, soc_pct has {len(self.grid)}")

        self.soc = start_soc_pct / 100.0
        self.v1 = 0.0       # RC1 voltage in V
        self.v2 = 0.0       # RC2 voltage in V

    def ocv(self, soc=None):
        if soc is None:
            soc = self.soc
        s = min(max(soc, self.soc_limit_low), 1.0 - self.soc_limit_high)
        k = self.k
        return k[0] + k[1] * s + k[2] * s * s + k[3] * s ** 3 + k[4] * math.log(s) + k[5] * math.log(1.0 - s)

    def param(self, name, soc=None):
        if soc is None:
            soc = self.soc
        return interp(soc * 100.0, self.grid, self.tables[name])

    def r0(self):
        return self.param("r0_mohm") / 1000.0

    def emf(self):
        """Voltage without the R0 drop: OCV plus the RC voltages."""
        return self.ocv() + self.v1 + self.v2

    def step(self, current_a, dt_s):
        """Advances the cell by dt_s with a constant current (+ charge). The RC pairs are solved exactly."""
        r1 = self.param("r1_mohm") / 1000.0
        r2 = self.param("r2_mohm") / 1000.0
        a1 = math.exp(-dt_s / self.param("tau1_s"))
        a2 = math.exp(-dt_s / self.param("tau2_s"))
        self.v1 = self.v1 * a1 + current_a * r1 * (1.0 - a1)
        self.v2 = self.v2 * a2 + current_a * r2 * (1.0 - a2)
        self.soc += current_a * dt_s / self.capacity_as

    def terminal(self, current_a):
        return self.emf() + current_a * self.r0()


def load_json(path):
    with open(path) as f:
        cfg = json.load(f)

    if not 1 <= cfg["cell_count"] <= CELL_COUNT_MAX:
        raise SimError(f"cell_count must be 1..{CELL_COUNT_MAX}")
    if len(cfg["cells"]) < cfg["cell_count"]:
        raise SimError(f"cells has {len(cfg['cells'])} entries, cell_count is {cfg['cell_count']}")
    return cfg


def make_cells(cfg, mode):
    start = cfg["start_soc_pct"][mode]
    cells = []
    for i in range(cfg["cell_count"]):
        p = dict(cfg["cell_default"])
        p.update(cfg["cells"][i])
        cells.append(Cell(p, start))
    return cells


def fault_active(fault, cfg, sim_time):
    return fault is not None and sim_time >= cfg["faults"]["start_after_s"]


def pack_current(cfg, mode, fault, cells, fet_on, sim_time):
    """Current in A (+ charge) for the next step, from the load or the charger. A current fault replaces
    the load or the charger by a fixed current of so many C once it is active."""
    if not fet_on:
        return 0.0

    if fault_active(fault, cfg, sim_time) and fault in ("high_current", "low_current"):
        current_a = cfg["faults"][fault + "_c"] * cfg["cell_default"]["capacity_mah"] / 1000.0
        if mode == "discharge":
            return -current_a
        return current_a

    emf = sum(c.emf() for c in cells)
    r0 = sum(c.r0() for c in cells)

    if mode == "discharge":
        return -emf / (cfg["discharge"]["load_ohm"] + r0)

    # Charger: constant current until the pack reaches the voltage limit, then constant voltage
    i_cc = cfg["charge"]["current_ma"] / 1000.0
    v_cv = cfg["charge"]["voltage_limit_mv"] / 1000.0
    if emf + i_cc * r0 <= v_cv:
        return i_cc
    return max(0.0, (v_cv - emf) / r0)


def pack_temperature(cfg, fault, sim_time):
    """Pack temperature sent to the board. A temperature fault replaces it once it is active."""
    if fault_active(fault, cfg, sim_time) and fault in ("high_temperature", "low_temperature"):
        return float(cfg["faults"][fault + "_c"])
    return float(cfg["temperature_c"])


# ---------------------------------------------------------------
# Run
# ---------------------------------------------------------------
def extract_text(value):
    """hppc_extract_status as names: the HX_STATUS_* bits, bits 0-6 = cells whose tables failed."""
    names = [name for bit, name in HPPC_X_BITS.items() if value & (1 << bit)]
    failed = [str(i + 1) for i in range(CELL_COUNT_MAX) if value & (1 << i)]
    if failed:
        names.append("FAILED_CELLS " + ",".join(failed))
    if not names:
        return "-"
    return " ".join(names)


def trip_text(value):
    """OK, or TRIPPED with the names of the protections that tripped."""
    if not value:
        return "OK"
    names = []
    for bit, name in enumerate(PROTECTION_NAMES):
        if value & (1 << bit):
            names.append(name)
    return "TRIPPED " + ",".join(names)


def errors_text(value):
    names = []
    for bit in range(64):
        if value & (1 << bit):
            if bit < len(ERROR_NAMES):
                names.append(ERROR_NAMES[bit])
            else:
                names.append(f"BIT{bit}")
    if not names:
        return "none"
    return ",".join(names)


def describe(r):
    state = HPPC_STATES.get(r["hppc_status"] & 0xFF, "?")
    stop = HPPC_STOPS.get(r["hppc_status"] >> 8, "?")
    if r["fet"] & FET_MAIN:
        fet = "on"
    else:
        fet = "off"
    return (f"{CTRL_STATES.get(r['ctrl_state'], r['ctrl_state'])}, hppc {state} (stop: {stop}), extract {extract_text(r['extract'])}, "
            f"FET {fet}, protection {trip_text(r['trip'])}, errors {errors_text(r['errors'])}")


def read_monitor(ser):
    """Reads MONITOR_REGS. Returns {name: value or list of values}."""
    values = {}
    for address, name, fmt, count in MONITOR_REGS:
        data = read_register(ser, address)
        v = struct.unpack(f"<{count}{fmt}", data[:count * struct.calcsize(fmt)])
        if count == 1:
            values[name] = v[0]
        else:
            values[name] = list(v)
    return values


def log_header(cell_count):
    header = ["direction", "run", "sim_time_s", "true_current_ma", "true_temperature_c"]
    header += [f"true_soc_pct_{i + 1}" for i in range(cell_count)]
    for _, name, _, count in MONITOR_REGS:
        if count == 1:
            header.append(name)
        else:
            header += [f"{name}_{i + 1}" for i in range(count)]
    return header


def log_row(mode, run_name, sim_time, current_a, temperature_c, cells, m):
    row = [mode, run_name, round(sim_time, 2), round(current_a * 1000.0, 1), round(temperature_c, 2)]
    row += [round(c.soc * 100.0, 3) for c in cells]
    for _, name, _, count in MONITOR_REGS:
        if count == 1:
            v = m[name]
            if isinstance(v, float):
                v = round(v, 3)
            row.append(v)
        else:
            for v in m[name]:
                if isinstance(v, float):
                    v = round(v, 3)
                row.append(v)
    return row


def describe_monitor(m, cell_count):
    """The live line: the board's read-back, short enough for one console line."""
    state = HPPC_STATES.get(m["hppc_status"] & 0xFF, "?")
    if m["fet_status"] & FET_MAIN:
        fet = "on"
    else:
        fet = "off"
    cells = m["cell_mv"][:cell_count]
    return (f"cells {min(cells):4.0f}..{max(cells):4.0f} mV  I {m['current_ma']:6.0f} mA  "
            f"T {m['temp_pack_c']:4.1f} C  FET {fet:3s}  {state}")


# The live line is rewritten in place (carriage return), events get their own line above it
_live_shown = False


def live(text):
    global _live_shown
    width = shutil.get_terminal_size((120, 20)).columns - 1
    sys.stdout.write("\r" + text[:width].ljust(width))
    sys.stdout.flush()
    _live_shown = True


def event(text):
    global _live_shown
    if _live_shown:
        width = shutil.get_terminal_size((120, 20)).columns - 1
        sys.stdout.write("\r" + " " * width + "\r")
        _live_shown = False
    print(text)


def report_changes(t, old, new):
    """Event lines for what changed in the board's reply: HPPC state, protection trips, error flags."""
    if (new["hppc_status"] & 0xFF) != (old["hppc_status"] & 0xFF):
        event(f"{t:9.1f} s  hppc {HPPC_STATES.get(new['hppc_status'] & 0xFF, '?')}")
    if new["trip"] != old["trip"]:
        event(f"{t:9.1f} s  protection {trip_text(new['trip'])}")
    if new["errors"] != old["errors"]:
        event(f"{t:9.1f} s  errors {errors_text(new['errors'])}")


def quit_pressed():
    """True when QUIT_KEY was pressed (Windows: no Enter needed; elsewhere: q then Enter)."""
    if msvcrt is not None:
        while msvcrt.kbhit():
            if msvcrt.getwch().lower() == QUIT_KEY:
                return True
        return False

    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        return sys.stdin.readline().strip().lower() == QUIT_KEY
    return False


def test_ended(r):
    """The board stays in HPPC_RUN after a test (only a reboot goes back to normal run), so the
    end is the HPPC state: ABORTED, or DONE once the extraction is no longer busy with the tables."""
    state = r["hppc_status"] & 0xFF
    if state == HPPC_STATE_ABORTED:
        return True
    if state == HPPC_STATE_DONE and not (r["extract"] & HPPC_X_BUSY):
        return True
    return False


def stop_test(ser, cell_mv, temperature_c, dt_ms):
    """Stops the HPPC test and keeps the board's loop running (last voltages, zero current)
    until the test shows it ended. Ctrl_Run only runs on a sample in the simulation build."""
    send_command(ser, CC_NORMAL_RUN, "HPPC stop")
    r = None
    for _ in range(STOP_SAMPLES):
        r = send_sample(ser, cell_mv, 0.0, temperature_c, dt_ms)
        if test_ended(r):
            break
    return r


def pass_text(ok):
    if ok:
        return "PASS"
    return "FAIL"


class UserStop(Exception):
    pass


def reboot(ser):
    send_command(ser, CC_REBOOT, "reboot")
    time.sleep(REBOOT_PAUSE)
    wait_until_ready(ser)


def run(ser, cfg, mode, fault, log):
    """One HPPC test from a fresh battery. fault is None (a good run) or a FAULTS key. Every
    read-back is written to log (a csv writer).
    Returns (stop reason, extract status, cells), stop reason None when the test did not end by itself."""
    cells = make_cells(cfg, mode)
    rng = random.Random(cfg["noise"]["seed"])
    noise_v = cfg["noise"]["voltage_mv"]
    noise_i = cfg["noise"]["current_ma"]
    dt_ms = int(cfg["dt_ms"])
    dt_s = dt_ms / 1000.0
    end_time = cfg["max_sim_time_s"]
    run_name = "good_run"
    if fault is not None:
        run_name = fault
        end_time = cfg["faults"]["start_after_s"] + cfg["faults"]["timeout_s"]

    state = struct.unpack("<B", read_register(ser, REG_CTRL_STATE)[:1])[0]
    if state != CTRL_STATE_NORMAL_RUN:
        raise SimError(f"the board is in ctrl state {state}, it must be in normal run ({CTRL_STATE_NORMAL_RUN})")

    send_command(ser, CC_HPPC_START, "HPPC start")

    fet_on = False
    sim_time = 0.0
    started = False
    fault_shown = False
    r = None
    last = {"hppc_status": 0, "trip": 0, "errors": 0}     # what the board showed before the start (IDLE)
    batch = SampleBatch(ser)
    wall_start = time.time()
    next_read = wall_start + READ_EVERY_S

    while sim_time < end_time:
        if fault_active(fault, cfg, sim_time) and not fault_shown:
            event(f"{sim_time:9.1f} s  fault starts: {FAULTS[fault][0]}")
            fault_shown = True

        snap = (snapshot(cells, rng), sim_time)
        current_a = pack_current(cfg, mode, fault, cells, fet_on, sim_time)
        temperature_c = pack_temperature(cfg, fault, sim_time)
        for c in cells:
            c.step(current_a, dt_s)
        sim_time += dt_s

        cell_mv = [0.0] * CELL_COUNT_MAX
        for i, c in enumerate(cells):
            cell_mv[i] = c.terminal(current_a) * 1000.0 + rng.gauss(0.0, noise_v)
        current_ma = current_a * 1000.0 + rng.gauss(0.0, noise_i)

        if not batch.add(snap, (cell_mv, current_ma, temperature_c, dt_ms), sim_time) and time.time() < next_read:
            continue

        done, back = batch.flush()
        for t, reply in done:
            report_changes(t, last, reply)
            last = reply
            r = reply
            if r["ctrl_state"] == CTRL_STATE_HPPC_RUN:
                started = True
            if started and test_ended(r):
                # The test ended by itself (DONE or ABORTED) and its tables are finished
                event(f"{t:9.1f} s  test finished: {describe(r)}")
                return r["hppc_status"] >> 8, r["extract"], cells
        fet_on = bool(r["fet"] & FET_MAIN)
        if back is not None:
            # The board stopped at a change (e.g. the FET): make the samples after it again
            restore(back[0])
            sim_time = back[1]
            continue

        if quit_pressed():
            r = stop_test(ser, cell_mv, temperature_c, dt_ms)
            event(f"{sim_time:9.1f} s  stopped by the user: {describe(r)}")
            raise UserStop()

        # Once per real second: read the board's registers back into the live line
        if time.time() >= next_read:
            m = read_monitor(ser)
            log.writerow(log_row(mode, run_name, sim_time, current_a, temperature_c, cells, m))
            next_read += READ_EVERY_S
            soc = sum(c.soc for c in cells) / len(cells) * 100.0
            rate = sim_time / max(time.time() - wall_start, 1e-3)
            live(f"{sim_time:9.1f} s  x{rate:4.0f}  true SOC {soc:5.1f} %  {describe_monitor(m, len(cells))}")

    r = stop_test(ser, cell_mv, temperature_c, dt_ms)
    event(f"{sim_time:9.1f} s  the test did not end by itself, stopped: {describe(r)}")
    return None, r["extract"], cells


def compare(ser, mode, cells):
    grid = struct.unpack(f"<{SOC_GRID_POINTS}f", read_register(ser, REG_SOC_GRID))
    regs = TABLE_REGS[mode]

    worst = {name: 0.0 for name in TABLE_NAMES}
    for ci, cell in enumerate(cells):
        write_register(ser, REG_MODEL_CELL, bytes([ci]))
        board = []
        for reg, scale in zip(regs, TABLE_SCALE):
            values = struct.unpack(f"<{SOC_GRID_POINTS}f", read_register(ser, reg))
            board.append([v * scale for v in values])

        for gi, soc_pct in enumerate(grid):
            soc = soc_pct / 100.0
            true = [cell.ocv(soc), cell.param("r0_mohm", soc), cell.param("r1_mohm", soc),
                    cell.param("tau1_s", soc), cell.param("r2_mohm", soc), cell.param("tau2_s", soc)]
            for ti, name in enumerate(TABLE_NAMES):
                err = (board[ti][gi] - true[ti]) / true[ti] * 100.0
                worst[name] = max(worst[name], abs(err))
    write_register(ser, REG_MODEL_CELL, bytes([0]))

    print("Largest error per parameter against the simulated cells, all cells and SOC points:")
    for name in TABLE_NAMES:
        print(f"  {name:8s} {worst[name]:7.2f} %")


# ---------------------------------------------------------------
# Fuel gauge registers -> JSON
# ---------------------------------------------------------------
def read_value(ser, address, fmt, count):
    """One register as a number (count 1) or a list. NaN becomes None, JSON has no NaN."""
    size = struct.calcsize("<" + fmt)
    data = read_register(ser, address)[:count * size]
    values = []
    for v in struct.unpack(f"<{count}{fmt}", data):
        if isinstance(v, float) and not math.isfinite(v):
            values.append(None)
        elif isinstance(v, float):
            values.append(float(f"{v:.7g}"))    # float32 precision, no 2.4999999e-05 noise
        else:
            values.append(v)
    if count == 1:
        return values[0]
    return values


def read_fuel_gauge(ser, cell_count):
    """Every fuel gauge register: the shared ones, then the per-cell ones for each cell."""
    result = {"cell_count": cell_count, "shared": {}, "cells": []}
    for address, name, fmt, count in FG_SHARED_REGISTERS:
        result["shared"][name] = read_value(ser, address, fmt, count)
    for cell in range(cell_count):
        write_register(ser, REG_MODEL_CELL, bytes([cell]))
        cell_values = {"cell": cell + 1}
        for address, name, fmt, count in FG_CELL_REGISTERS:
            cell_values[name] = read_value(ser, address, fmt, count)
        result["cells"].append(cell_values)
    write_register(ser, REG_MODEL_CELL, bytes([0]))     # back on cell 1, where the board starts
    return result


def merge_fuel_gauge(charge, discharge):
    """One register set from a charge and a discharge set (a good run, or the saved file). The board's
    tables live in RAM, so after the reboot the discharge read holds the board's default charge
    tables: the charge tables and model_temp_chg_c come from the charge set, the rest from the
    discharge set."""
    merged = json.loads(json.dumps(discharge))
    merged["shared"]["model_temp_chg_c"] = charge["shared"]["model_temp_chg_c"]
    learning = (charge["shared"]["learning_status"] & LEARN_CHARGE) | (discharge["shared"]["learning_status"] & LEARN_DISCHARGE)
    if learning == LEARN_CHARGE | LEARN_DISCHARGE:
        learning |= LEARN_COMPLETE
    merged["shared"]["learning_status"] = learning
    for cell_merged, cell_charge in zip(merged["cells"], charge["cells"]):
        for name in FG_CHARGE_CELL_NAMES:
            cell_merged[name] = cell_charge[name]
    return merged


def save_direction(out_path, port, mode, registers):
    """Puts one direction's registers into the output file right after its good run, so a stop
    later on does not lose it. The other direction stays as the file had it."""
    if os.path.isfile(out_path):
        with open(out_path) as f:
            saved = json.load(f)
        if mode == "charge":
            registers = merge_fuel_gauge(registers, saved)
        else:
            registers = merge_fuel_gauge(saved, registers)

    data = {"port": port, "time": time.strftime("%Y-%m-%d %H:%M:%S")}
    for key, value in registers.items():
        if key not in data:
            data[key] = value
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Fuel gauge registers of the {mode} run saved: {out_path}")


# ---------------------------------------------------------------
# Test sequence
# ---------------------------------------------------------------
def run_direction(ser, cfg, mode, results, log):
    """The failure runs, then the good run of one direction. Returns the fuel gauge registers read
    after the good run, or None when it wrote no tables."""
    for fault in FAULTS:
        print(f"\n=== {mode}: {FAULTS[fault][0]} ===")
        print("Rebooting the board ...")
        reboot(ser)
        reason, _, _ = run(ser, cfg, mode, fault, log)
        expected = FAULTS[fault][1]
        ok = reason in expected
        results.append((mode, FAULTS[fault][0], ok, reason))
        print(f"stop reason {HPPC_STOPS.get(reason, reason)}, expected "
              f"{' or '.join(HPPC_STOPS[e] for e in expected)}: {pass_text(ok)}")

    print(f"\n=== {mode}: good run ===")
    print("Rebooting the board ...")
    reboot(ser)
    reason, extract, cells = run(ser, cfg, mode, None, log)
    ok = reason in GOOD_STOPS[mode] and (extract & HPPC_X_DONE) != 0
    results.append((mode, "good run", ok, reason))
    print(f"stop reason {HPPC_STOPS.get(reason, reason)}, extract {extract_text(extract)}: {pass_text(ok)}")
    if not extract & HPPC_X_DONE:
        print("the board wrote no tables")
        return None

    compare(ser, mode, cells)
    print("Reading the fuel gauge registers ...")
    return read_fuel_gauge(ser, cfg["cell_count"])


def plot_log(log_path, only_mode, only_run):
    """Plots log_hppc.csv: one window per run (direction and run name), filtered by only_mode / only_run."""
    import matplotlib.pyplot as plt

    runs = {}
    with open(log_path, newline="") as f:
        for row in csv.DictReader(f):
            if only_mode is not None and row["direction"] != only_mode:
                continue
            if only_run is not None and row["run"] != only_run:
                continue
            runs.setdefault((row["direction"], row["run"]), []).append(row)
    if not runs:
        print(f"Error: no matching rows in {log_path}")
        return 1

    for (mode, run_name), rows in runs.items():
        n = len([k for k in rows[0] if k.startswith("true_soc_pct_")])
        t = [float(r["sim_time_s"]) / 3600.0 for r in rows]

        def col(key):
            return [float(r[key]) for r in rows]

        fig, ax = plt.subplots(5, 1, sharex=True, figsize=(12, 11))
        fig.canvas.manager.set_window_title(f"HPPC {mode} {run_name}")
        fig.suptitle(f"{mode}: {run_name}")

        ax[0].plot(t, col("true_current_ma"), label="true")
        ax[0].plot(t, col("current_ma"), label="board")
        ax[0].set_ylabel("current [mA]")
        ax[0].legend(loc="best")

        for i in range(n):
            ax[1].plot(t, col(f"cell_mv_{i + 1}"), label=f"cell {i + 1}")
        ax[1].set_ylabel("cell [mV]")
        ax[1].legend(loc="best", ncol=n)

        ax[2].plot(t, col("true_temperature_c"), label="true")
        ax[2].plot(t, col("temp_pack_c"), label="board")
        ax[2].set_ylabel("temperature [C]")
        ax[2].legend(loc="best")

        for i in range(n):
            ax[3].plot(t, col(f"true_soc_pct_{i + 1}"), label=f"cell {i + 1}")
        ax[3].set_ylabel("true SOC [%]")
        ax[3].legend(loc="best", ncol=n)

        states = [int(r["hppc_status"]) & 0xFF for r in rows]
        ax[4].step(t, states, where="post")
        ax[4].set_yticks(list(HPPC_STATES))
        ax[4].set_yticklabels(list(HPPC_STATES.values()))
        ax[4].set_ylabel("HPPC state")
        ax[4].set_xlabel("simulated time [h]")

        # Where the test stopped: first row in DONE or ABORTED
        for r, ti, st in zip(rows, t, states):
            if HPPC_STATES.get(st) in ("DONE", "ABORTED"):
                stop = HPPC_STOPS.get(int(r["hppc_status"]) >> 8, "?")
                for a in ax:
                    a.axvline(ti, color="red", linewidth=1.0)
                fig.suptitle(f"{mode}: {run_name}, {HPPC_STATES[st]} ({stop})")
                break

        for a in ax:
            a.grid(True)
        fig.tight_layout()

    plt.show()
    return 0


def main(argv):
    here = os.path.dirname(os.path.abspath(__file__))
    if len(argv) < 2:
        print(__doc__.strip().split("\n\n")[1])
        print("Test folders next to this script:")
        for f in sorted(os.listdir(here)):
            if os.path.isdir(os.path.join(here, f)) and not f.startswith(("_", ".")):
                print(f"  {f}")
        return 1

    # A bare folder name is looked up next to this script
    folder = argv[1]
    if not os.path.isdir(folder) and os.path.isdir(os.path.join(here, folder)):
        folder = os.path.join(here, folder)
    if not os.path.isdir(folder):
        print(f"Error: no test folder called {argv[1]}")
        return 1

    if len(argv) > 2 and argv[2] == "plot":
        log_path = os.path.join(folder, LOG_FILE)
        if not os.path.isfile(log_path):
            print(f"Error: no log {log_path}, run the tests first")
            return 1
        only_mode = None
        only_run = None
        rest = argv[3:]
        if rest and rest[0].lower() in DIRECTIONS:
            only_mode = DIRECTIONS[rest[0].lower()]
            rest = rest[1:]
        if rest:
            only_run = rest[0]
        return plot_log(log_path, only_mode, only_run)

    cfg_path = os.path.join(folder, CONFIG_FILE)
    out_path = os.path.join(folder, OUTPUT_FILE)
    log_path = os.path.join(folder, LOG_FILE)
    # Optional direction (c or d), then optional port
    modes = ["charge", "discharge"]
    port = None
    rest = argv[2:]
    if rest and rest[0].lower() in DIRECTIONS:
        modes = [DIRECTIONS[rest[0].lower()]]
        rest = rest[1:]
    if rest:
        port = rest[0]

    results = []
    sets = {}
    try:
        print(f"Simulation file: {cfg_path}")
        cfg = load_json(cfg_path)
        if port is None:
            port = find_board()
        with serial.Serial(port, BAUD_RATE, timeout=PORT_POLL) as ser, \
                open(log_path, "w", newline="") as log_file:
            time.sleep(OPEN_SETTLE)
            log = csv.writer(log_file)
            log.writerow(log_header(cfg["cell_count"]))
            print(f"Log: {log_path}")
            print(f"Press {QUIT_KEY} to stop.")
            for mode in modes:
                sets[mode] = run_direction(ser, cfg, mode, results, log)
                if sets[mode] is not None:
                    save_direction(out_path, port, mode, sets[mode])
    except UserStop:
        print("Stopped. A direction whose good run had finished is already saved, the running one is not")
        return 1
    except (SimError, serial.SerialException, OSError, KeyError, ValueError) as e:
        print(f"Error: {e}")
        return 1

    print("\nSummary:")
    for mode, name, ok, reason in results:
        print(f"  {mode:9s}  {name:24s}  stop {HPPC_STOPS.get(reason, reason):14s}  {pass_text(ok)}")

    if any(sets.get(mode) is None for mode in modes):
        print(f"\nA good run wrote no tables, its direction in {OUTPUT_FILE} was not updated")
        return 1

    if not all(ok for _, _, ok, _ in results):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
