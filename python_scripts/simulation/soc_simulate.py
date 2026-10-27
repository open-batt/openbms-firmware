#!/usr/bin/env python3
"""
soc_simulate.py -- fuel gauge (SOC) tests for the OpenBMS simulation build: estimation, the
rest and full-charge corrections, capacity learning and the temperature model.

Usage:
    python soc_simulate.py <test_folder> [test_name] [COM_PORT]
    python soc_simulate.py <test_folder> plot [test_name]

<test_folder> is a path, or just the name of a folder next to this script, for example
7_cell_2000mAh. It must hold fuel_gauge_registers.json (the battery model, written by
hppc_simulate.py) and soc.json, which defines every SOC test. Without test_name all tests run,
one after the other; with it only that one (for example rest_correction). Without COM_PORT the
board is found by pinging every serial port.

The board must run firmware built with OPENBMS_SIMULATION (config.h) and carry the config of the
test folder (config_tool.py w <test_folder>). For every test the script:

    1. reboots the board, checks normal run and cell_count, writes the battery model (SOC grid,
       the tables and capacity of every cell, q_nom, temperature model, Kalman values), so the
       board's fuel gauge uses the same model as the simulated battery,
    2. rests for the fuel gauge seed, then applies the test's soc_offset_pct,
    3. simulates every cell with that model (OCV, R0 and two RC pairs per SOC, the charge or
       discharge tables by the current direction, R and tau scaled by the segment temperature
       like the board's temperature model) and runs the profile,
    4. every read_every_s of simulated time reads per cell the board's internal SOC (0xD0), shown
       SOC (0x92), remaining capacity (0x94) and capacity (0xD4), and for the pack SOC (0x90),
       q_nom_ah (0xAF), SOH (0x93), capacity_update_count (0x97) and cycle_count (0xD7),
    5. checks the test's expect values (see below) and gives PASS or FAIL.

While a test runs, one live line shows the simulated time, current and SOC of cell 1, updated in
place. New lines are printed only for events: segments, a rest correction (the internal SOC
jumps at rest), a full-charge correction (the shown SOC jumps to 100 % while charging), the pack
SOC reaching 100 %, a capacity update and a new cycle. At the end a PASS/FAIL summary is printed.

Log: <test_folder>/log_soc.csv, one row per read of every test run, the test name in the first
column.

Press q to stop (on Windows without Enter, elsewhere q then Enter).

plot: no test runs and no board is needed. The existing <test_folder>/log_soc.csv is plotted, one
window per test in the log (only test_name when given): SOC of cell 1 (true, internal, shown) and
the pack SOC, the SOC error of every cell (internal - true), current and temperature, and the
capacity of every cell with q_nom_ah.

soc.json:
    dt_ms, temperature_c, read_every_s, cell_min_mv, cell_max_mv, repeat, noise
              defaults of every test; a test can give its own
    tests     {name: test}, run in this order. A test has:
      description
      branch            OCV branch the battery starts on, "dis" or "chg"
      start_soc_pct     true SOC of every cell at the start
      soc_offset_pct    after the seed the true SOC is moved by this much (the board is then off by
                        minus this)
      capacity_scale    real cell capacity / the board's capacity_ah, e.g. 0.95
      voltage_offset_mv added to every cell voltage sent to the board, a measurement error
      board_temperature_model   false writes ea_* = 0 to the board, the simulated cells still
                        follow the temperature
      repeat            how many times the profile is run
      profile           segments, run in order. Current sign: + charge, - discharge.
                          {"type": "rest",     "duration_s": 600}
                          {"type": "constant", "current_ma": -2000, "duration_s": 1800}
                          {"type": "resistor", "load_ohm": 14.0, "duration_s": 1800}
                          {"type": "pulse",    "high_ma": -4000, "low_ma": 0, "period_s": 20, "duty": 0.5, "duration_s": 600}
                          {"type": "sine",     "offset_ma": -2000, "amplitude_ma": 1500, "period_s": 60, "duration_s": 600}
                          {"type": "random",   "min_ma": -5000, "max_ma": 0, "hold_s": 5, "duration_s": 600}
                          {"type": "cccv",     "current_ma": 2000, "cell_voltage_mv": 4200, "end_current_ma": 100, "duration_s": 14400}
                        resistor: a load resistor across the pack. cccv: constant current until the
                        highest cell reaches cell_voltage_mv, then that voltage is held; ends when the
                        current falls to end_current_ma. Every segment may carry "temperature_c"
                        and "note". (protection_simulate.py also uses {"type": "reset_protection"}.)
      expect            the pass conditions, all optional (errors are board internal SOC - true SOC,
                        worst cell, in % SOC):
                          max_error_pct              at every read
                          max_final_error_pct        at the last read
                          max_mean_flow_error_pct    mean over the reads while current flows
                          min_mean_flow_error_pct    the same, at least (shows a model is needed)
                          min_rest_corrections, min_full_charges, min_capacity_updates
                          max_final_capacity_error_pct   learned capacity vs the real one, worst cell
                          final_shown_pct            shown SOC of every cell at the last read
                          remaining_factor           [low, high]: remaining capacity / (shown SOC x
                                                     capacity) of cell 1 at the last read
"""

import csv
import json
import math
import os
import random
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

# Serial link and helpers are shared with the HPPC simulator
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hppc_simulate as hs

REG_CELL_COUNT      = 0x30

DT_MS_MAX           = 500       # FG_MAX_DT_S: the fuel gauge skips longer steps
SEED_REST_S         = 1.0       # rest before the profile: the fuel gauge seeds after FG_SEED_DELAY_MS (500 ms)
SEGMENT_TYPES       = ("rest", "constant", "resistor", "pulse", "sine", "random", "cccv", "reset_protection")
CC_SET_FET          = 0x08
CC_RESET_PROTECTION = 0x0B
REGISTERS_FILE      = "fuel_gauge_registers.json"

# Model registers written to the board from fuel_gauge_registers.json: (address, name, struct format)
SHARED_MODEL_REGISTERS = [
    (0xA0, "soc_grid",             "f"),
    (0xAD, "q_temp_c",             "f"),
    (0xAE, "q_temp_factor",        "f"),
    (0xAF, "q_nom_ah",             "f"),
    (0xB0, "coulombic_efficiency", "f"),
    (0xB1, "model_temp_dis_c",     "f"),
    (0xB2, "model_temp_chg_c",     "f"),
    (0xB6, "ea_r0",                "f"),
    (0xB7, "ea_r1",                "f"),
    (0xB8, "ea_tau1",              "f"),
    (0xB9, "ea_r2",                "f"),
    (0xBA, "ea_tau2",              "f"),
    (0xBB, "kf_q_soc",             "f"),
    (0xBC, "kf_q_rc1",             "f"),
    (0xBD, "kf_q_rc2",             "f"),
    (0xBE, "kf_r_v",               "f"),
    (0x96, "learning_status",      "H"),
]
# Per cell, after the cell is picked with 0x9F
CELL_MODEL_REGISTERS = [
    (0xA1, "ocv_dis",     "f"),
    (0xA2, "ocv_chg",     "f"),
    (0xA3, "r0_dis",      "f"),
    (0xA4, "r1_dis",      "f"),
    (0xA5, "tau1_dis",    "f"),
    (0xA6, "r2_dis",      "f"),
    (0xA7, "tau2_dis",    "f"),
    (0xA8, "r0_chg",      "f"),
    (0xA9, "r1_chg",      "f"),
    (0xAA, "tau1_chg",    "f"),
    (0xAB, "r2_chg",      "f"),
    (0xAC, "tau2_chg",    "f"),
    (0xD4, "capacity_ah", "f"),
]


# ---------------------------------------------------------------
# Battery model from the fuel gauge registers: the same 2-RC model the board's gauge uses
# ---------------------------------------------------------------
class ModelCell:
    def __init__(self, cell, soc_grid, q_nom_ah, start_soc_pct, branch):
        self.grid = soc_grid
        self.tables = {name: cell[name] for name, _ in self.table_names()}
        capacity_ah = cell["capacity_ah"]
        if not capacity_ah:
            capacity_ah = q_nom_ah          # a charge-only model has no cell capacity
        if not capacity_ah:
            raise hs.SimError(f"cell {cell['cell']} has no capacity_ah and q_nom_ah is 0")
        self.capacity_as = capacity_ah * 3600.0
        self.soc = start_soc_pct / 100.0
        self.branch = branch                # "dis" or "chg": tables in use, kept while at rest
        self.v1 = 0.0                       # RC1 voltage in V
        self.v2 = 0.0                       # RC2 voltage in V
        for name in self.tables:
            if not any(self.tables[name]):
                raise hs.SimError(f"cell {cell['cell']}: {name} is all zero, the model needs both directions")

    @staticmethod
    def table_names():
        names = []
        for p in ("ocv", "r0", "r1", "tau1", "r2", "tau2"):
            for d in ("dis", "chg"):
                names.append((f"{p}_{d}", p))
        return names

    def table(self, p):
        return self.tables[f"{p}_{self.branch}"]

    def ocv(self):
        """OCV of the branch in use. Below the first grid point the line through the first two
        points goes on falling, like a real empty cell; above the last point it is held."""
        x = self.soc * 100.0
        ocv = self.table("ocv")
        if x < self.grid[0]:
            slope = (ocv[1] - ocv[0]) / (self.grid[1] - self.grid[0])
            return ocv[0] + slope * (x - self.grid[0])
        return hs.interp(x, self.grid, ocv)

    def param(self, p):
        return hs.interp(self.soc * 100.0, self.grid, self.table(p))

    def r0(self):
        return self.param("r0")

    def emf(self):
        """Voltage without the R0 drop: OCV plus the RC voltages."""
        return self.ocv() + self.v1 + self.v2

    def step(self, current_a, dt_s):
        """Advances the cell by dt_s with a constant current (+ charge). The RC pairs are solved exactly."""
        if current_a > 0.0:
            self.branch = "chg"
        elif current_a < 0.0:
            self.branch = "dis"
        a1 = math.exp(-dt_s / self.param("tau1"))
        a2 = math.exp(-dt_s / self.param("tau2"))
        self.v1 = self.v1 * a1 + current_a * self.param("r1") * (1.0 - a1)
        self.v2 = self.v2 * a2 + current_a * self.param("r2") * (1.0 - a2)
        self.soc += current_a * dt_s / self.capacity_as

    def terminal(self, current_a):
        return self.emf() + current_a * self.r0()


def load_registers(path):
    with open(path) as f:
        regs = json.load(f)
    if not 1 <= regs["cell_count"] <= hs.CELL_COUNT_MAX:
        raise hs.SimError(f"{REGISTERS_FILE}: cell_count must be 1..{hs.CELL_COUNT_MAX}")
    if len(regs["cells"]) < regs["cell_count"]:
        raise hs.SimError(f"{REGISTERS_FILE}: cells has {len(regs['cells'])} entries, cell_count is {regs['cell_count']}")
    return regs


def make_cells(regs, cfg, branch):
    cells = []
    for i in range(regs["cell_count"]):
        cells.append(ModelCell(regs["cells"][i], regs["shared"]["soc_grid"], regs["shared"]["q_nom_ah"],
                               cfg["start_soc_pct"], branch))
    return cells


# ---------------------------------------------------------------
# Current profile
# ---------------------------------------------------------------
class Segment:
    """One profile segment: gives the pack current for the next step and says when it is over."""

    def __init__(self, seg, rng, default_temp_c, cell_min_mv, cell_max_mv, dt_s):
        if seg["type"] not in SEGMENT_TYPES:
            raise hs.SimError(f"unknown segment type {seg['type']!r}, expected one of {', '.join(SEGMENT_TYPES)}")
        self.seg = seg
        self.rng = rng
        self.dt_s = dt_s
        self.duration_s = float(seg.get("duration_s", 0.0))
        self.cell_min_v = cell_min_mv / 1000.0
        self.cell_max_v = cell_max_mv / 1000.0
        if "temperature_c" in seg:
            self.temperature_c = float(seg["temperature_c"])
        else:
            self.temperature_c = default_temp_c
        self.t = 0.0
        self.random_ma = 0.0
        self.random_next_s = 0.0
        self.end_reason = None

    def name(self):
        return self.seg["type"]

    def current_a(self, cells):
        """Current (+ charge) for the next step at segment time self.t."""
        s = self.seg
        kind = s["type"]

        if kind == "rest" or kind == "reset_protection":
            return 0.0
        if kind == "constant":
            return s["current_ma"] / 1000.0
        if kind == "resistor":
            emf = sum(c.emf() for c in cells)
            r0 = sum(c.r0() for c in cells)
            return -emf / (s["load_ohm"] + r0)
        if kind == "pulse":
            phase = (self.t % s["period_s"]) / s["period_s"]
            if phase < s["duty"]:
                return s["high_ma"] / 1000.0
            return s["low_ma"] / 1000.0
        if kind == "sine":
            return (s["offset_ma"] + s["amplitude_ma"] * math.sin(2.0 * math.pi * self.t / s["period_s"])) / 1000.0
        if kind == "random":
            if self.t >= self.random_next_s:
                self.random_ma = self.rng.uniform(s["min_ma"], s["max_ma"])
                self.random_next_s = self.t + s["hold_s"]
            return self.random_ma / 1000.0

        # cccv: the current that keeps the highest cell at the limit, at most the CC current.
        # The model's OCV is held flat at 100 %, so it would take charge for ever: no cell
        # is charged above 100 % instead, which makes the CV current fall to the end current.
        i_cc = s["current_ma"] / 1000.0
        v_lim = s["cell_voltage_mv"] / 1000.0
        i = i_cc
        for c in cells:
            i_cell = (v_lim - c.emf()) / c.r0()
            if i_cell < i:
                i = i_cell
            i_full = (1.0 - c.soc) * c.capacity_as / self.dt_s
            if i_full < i:
                i = i_full
        if i < 0.0:
            i = 0.0
        return i

    def finished(self, cells, current_a):
        """True when the segment is over: its time is up, or a cell limit or the CV end current is reached."""
        if self.t >= self.duration_s:
            self.end_reason = "time"
            return True

        kind = self.seg["type"]
        if kind == "cccv":
            if current_a * 1000.0 <= self.seg["end_current_ma"]:
                self.end_reason = "end current"
                return True
            return False

        if current_a < 0.0 and min(c.terminal(current_a) for c in cells) < self.cell_min_v:
            self.end_reason = "cell min"
            return True
        if current_a > 0.0 and max(c.terminal(current_a) for c in cells) > self.cell_max_v:
            self.end_reason = "cell max"
            return True
        return False


def check_board(ser, regs):
    state = hs.read_register(ser, hs.REG_CTRL_STATE)[0]
    if state != hs.CTRL_STATE_NORMAL_RUN:
        raise hs.SimError(f"the board is in ctrl state {hs.CTRL_STATES.get(state, state)}, it must be in NORMAL_RUN")

    cell_count = hs.read_register(ser, REG_CELL_COUNT)[0]
    if cell_count != regs["cell_count"]:
        raise hs.SimError(f"the board has cell_count {cell_count}, {REGISTERS_FILE} {regs['cell_count']}")


def pack_values(fmt, value):
    if not isinstance(value, list):
        value = [value]
    return struct.pack(f"<{len(value)}{fmt}", *value)


def write_model(ser, regs):
    """Writes the model of fuel_gauge_registers.json to the board. Values saved as null (NaN) are skipped."""
    for address, name, fmt in SHARED_MODEL_REGISTERS:
        value = regs["shared"][name]
        if value is None or (isinstance(value, list) and None in value):
            continue
        hs.write_register(ser, address, pack_values(fmt, value))
    for i in range(regs["cell_count"]):
        hs.write_register(ser, hs.REG_MODEL_CELL, bytes([i]))
        for address, name, fmt in CELL_MODEL_REGISTERS:
            value = regs["cells"][i][name]
            if value is None or (isinstance(value, list) and None in value):
                continue
            hs.write_register(ser, address, pack_values(fmt, value))
    hs.write_register(ser, hs.REG_MODEL_CELL, bytes([0]))     # back on cell 1, where the board starts


def cell_samples(cells, current_a, rng, noise_v):
    cell_mv = [0.0] * hs.CELL_COUNT_MAX
    for i, c in enumerate(cells):
        cell_mv[i] = c.terminal(current_a) * 1000.0 + rng.gauss(0.0, noise_v)
    return cell_mv


def list_test_folders(here):
    print("Test folders next to this script:")
    for f in sorted(os.listdir(here)):
        if os.path.isdir(os.path.join(here, f)) and not f.startswith(("_", ".")):
            print(f"  {f}")


# ---------------------------------------------------------------
# SOC tests
# ---------------------------------------------------------------
CONFIG_FILE       = "soc.json"
LOG_FILE          = "log_soc.csv"
TEST_DEFAULTS     = ("dt_ms", "temperature_c", "read_every_s", "cell_min_mv", "cell_max_mv", "repeat", "noise")

JUMP_PCT          = 0.5     # internal SOC moving this much more than the true SOC between two reads at rest is a rest correction
JUMP_SHOWN_PCT    = 1       # shown SOC rising more than this between two reads while charging is the full-charge correction

REG_PACK_SOC      = 0x90
REG_CELL_SOC      = 0x92
REG_CELL_SOH      = 0x93
REG_CELL_REMAINING = 0x94
REG_CAP_UPDATES   = 0x97
REG_Q_NOM_AH      = 0xAF
REG_CELL_SOC_F    = 0xD0
REG_CAPACITY_AH   = 0xD4
REG_CYCLE_COUNT   = 0xD7
REG_EA            = (0xB6, 0xB7, 0xB8, 0xB9, 0xBA)     # ea_r0, ea_r1, ea_tau1, ea_r2, ea_tau2

GAS_CONSTANT      = 8.314   # J / (mol K)
KELVIN_OFFSET     = 273.15
TEMP_PARAMS       = ("r0", "r1", "tau1", "r2", "tau2")

EXPECT_KEYS = ("max_error_pct", "max_final_error_pct", "max_mean_flow_error_pct", "min_mean_flow_error_pct",
               "min_rest_corrections", "min_full_charges", "min_capacity_updates",
               "max_final_capacity_error_pct", "final_shown_pct", "remaining_factor")


class UserStop(Exception):
    pass


class TempCell(ModelCell):
    """Model cell whose resistances and time constants follow the temperature, like the board's model."""
    scale = {"dis": {}, "chg": {}}

    def param(self, p):
        return ModelCell.param(self, p) * self.scale[self.branch].get(p, 1.0)


def set_temperature(cells, regs, temp_c):
    """Arrhenius factor per parameter at temp_c, against the model temperature of each branch's tables."""
    shared = regs["shared"]
    t = temp_c + KELVIN_OFFSET
    scale = {}
    for branch in ("dis", "chg"):
        t_model = shared[f"model_temp_{branch}_c"] + KELVIN_OFFSET
        scale[branch] = {}
        for p in TEMP_PARAMS:
            scale[branch][p] = math.exp(shared["ea_" + p] / GAS_CONSTANT * (1.0 / t - 1.0 / t_model))
    for c in cells:
        c.scale = scale


def pass_text(ok):
    if ok:
        return "PASS"
    return "FAIL"


def load_tests(path):
    """soc.json as {name: test}, each test with the defaults filled in."""
    with open(path) as f:
        cfg = json.load(f)
    tests = {}
    for name, test in cfg["tests"].items():
        t = {key: cfg[key] for key in TEST_DEFAULTS}
        t.update(test)
        if t.get("branch", "dis") not in ("dis", "chg"):
            raise hs.SimError(f"{name}: branch must be dis or chg, not {t['branch']!r}")
        if not 1 <= t["dt_ms"] <= DT_MS_MAX:
            raise hs.SimError(f"{name}: dt_ms must be 1..{DT_MS_MAX}, the fuel gauge skips longer steps")
        if not t["profile"]:
            raise hs.SimError(f"{name}: profile is empty")
        for seg in t["profile"]:
            if seg.get("type") not in SEGMENT_TYPES:
                raise hs.SimError(f"{name}: unknown segment type {seg.get('type')!r}")
        for key in t.get("expect", {}):
            if key not in EXPECT_KEYS:
                raise hs.SimError(f"{name}: unknown expect key {key!r}")
        tests[name] = t
    return tests


def read_board(ser, n):
    """Everything the tests look at, in one dict."""
    b = {}
    soc_f = struct.unpack(f"<{hs.CELL_COUNT_MAX}f", hs.read_register(ser, REG_CELL_SOC_F)[:hs.CELL_COUNT_MAX * 4])
    b["internal"] = [v * 100.0 for v in soc_f[:n]]
    b["shown"] = list(hs.read_register(ser, REG_CELL_SOC)[:n])
    remaining = struct.unpack(f"<{hs.CELL_COUNT_MAX}H", hs.read_register(ser, REG_CELL_REMAINING)[:hs.CELL_COUNT_MAX * 2])
    b["remaining"] = list(remaining[:n])
    b["pack"] = hs.read_register(ser, REG_PACK_SOC)[0]
    b["soh"] = list(hs.read_register(ser, REG_CELL_SOH)[:n])
    b["q_nom"] = struct.unpack("<f", hs.read_register(ser, REG_Q_NOM_AH)[:4])[0]
    b["updates"] = struct.unpack("<H", hs.read_register(ser, REG_CAP_UPDATES)[:2])[0]
    b["cycles"] = struct.unpack("<H", hs.read_register(ser, REG_CYCLE_COUNT)[:2])[0]
    b["capacity"] = []
    for i in range(n):
        hs.write_register(ser, hs.REG_MODEL_CELL, bytes([i]))
        b["capacity"].append(struct.unpack("<f", hs.read_register(ser, REG_CAPACITY_AH)[:4])[0])
    hs.write_register(ser, hs.REG_MODEL_CELL, bytes([0]))
    return b


def log_header(n):
    return (["test", "sim_time_s", "segment", "segment_type", "current_ma", "temperature_c", "pack_soc_pct",
             "q_nom_ah", "capacity_update_count", "cycle_count"]
            + [f"true_soc_pct_{i + 1}" for i in range(n)]
            + [f"internal_soc_pct_{i + 1}" for i in range(n)]
            + [f"shown_soc_pct_{i + 1}" for i in range(n)]
            + [f"remaining_mah_{i + 1}" for i in range(n)]
            + [f"capacity_ah_{i + 1}" for i in range(n)]
            + [f"soh_pct_{i + 1}" for i in range(n)])


def run(ser, regs, name, cfg, log):
    """Runs one test. Returns (rows, counts, cells): rows = (sim time, current mA, true list, board dict)."""
    cells = make_cells(regs, cfg, cfg.get("branch", "dis"))
    for c in cells:
        c.__class__ = TempCell
    n = len(cells)
    capacity_scale = float(cfg.get("capacity_scale", 1.0))
    for c in cells:
        c.capacity_as *= capacity_scale
    rng = random.Random(cfg["noise"]["seed"])
    noise_v = cfg["noise"]["voltage_mv"]
    noise_i = cfg["noise"]["current_ma"]
    dt_ms = int(cfg["dt_ms"])
    dt_s = dt_ms / 1000.0
    temp_c = float(cfg["temperature_c"])

    check_board(ser, regs)
    write_model(ser, regs)
    if not cfg.get("board_temperature_model", True):
        for address in REG_EA:
            hs.write_register(ser, address, struct.pack("<f", 0.0))
        hs.event("Temperature model of the board turned off (ea_* = 0)")
    set_temperature(cells, regs, temp_c)

    # Rest first: the fuel gauge seeds its SOC from the OCV
    for _ in range(int(math.ceil(SEED_REST_S / dt_s))):
        hs.send_sample(ser, cell_samples(cells, 0.0, rng, noise_v), 0.0, temp_c, dt_ms)
    if not any(read_board(ser, n)["internal"]):
        raise hs.SimError("the fuel gauge is not running (SOC 0): the board found no complete model at boot")

    v_offset = float(cfg.get("voltage_offset_mv", 0.0))
    offset = float(cfg.get("soc_offset_pct", 0.0))
    if offset:
        for c in cells:
            c.soc += offset / 100.0

    rows = []
    counts = {"rest_corrections": 0, "full_charges": 0}
    sim_time = 0.0
    next_read = 0.0
    wall_start = time.time()
    wall_next = wall_start
    prev = None         # (true list, board dict) of the previous read

    batch = hs.SampleBatch(ser)
    for rep in range(int(cfg["repeat"])):
        for index, seg_cfg in enumerate(cfg["profile"]):
            seg_no = index + 1
            seg = Segment(seg_cfg, rng, temp_c, cfg["cell_min_mv"], cfg["cell_max_mv"], dt_s)
            set_temperature(cells, regs, seg.temperature_c)
            run_text = ""
            if int(cfg["repeat"]) > 1:
                run_text = f", run {rep + 1}/{cfg['repeat']}"
            hs.event(f"{sim_time:9.1f} s  segment {seg_no}/{len(cfg['profile'])} {seg.name()}{run_text}  {seg_cfg.get('note', '')}")

            while True:
                snap = (hs.snapshot(cells + [seg], rng), sim_time)
                current_a = seg.current_a(cells)
                end = seg.finished(cells, current_a)

                if not end:
                    for c in cells:
                        c.step(current_a, dt_s)
                    sim_time += dt_s
                    seg.t += dt_s

                    current_ma = current_a * 1000.0 + rng.gauss(0.0, noise_i)
                    cell_mv = cell_samples(cells, current_a, rng, noise_v)
                    for i in range(n):
                        cell_mv[i] += v_offset
                    full = batch.add(snap, (cell_mv, current_ma, seg.temperature_c, dt_ms))
                    if not full and sim_time < next_read:
                        continue

                # The batch goes out when it is full, before a read-back and at the end of the segment
                _, back = batch.flush()
                if back is not None:
                    # The board stopped at a change of its state: make the samples after it again
                    hs.restore(back[0])
                    sim_time = back[1]
                    continue
                if end:
                    break

                if sim_time >= next_read:
                    next_read += cfg["read_every_s"]
                    true = [c.soc * 100.0 for c in cells]
                    b = read_board(ser, n)
                    rows.append((sim_time, current_a * 1000.0, true, b))
                    log.writerow([name, round(sim_time, 2), seg_no, seg.name(), round(current_a * 1000.0, 1),
                                  seg.temperature_c, b["pack"], round(b["q_nom"], 4), b["updates"], b["cycles"]]
                                 + [round(v, 3) for v in true] + [round(v, 3) for v in b["internal"]]
                                 + b["shown"] + b["remaining"] + [round(v, 4) for v in b["capacity"]] + b["soh"])
                    if prev is not None:
                        report_changes(sim_time, cells, current_a, prev, true, b, counts)
                    prev = (true, b)

                    if time.time() >= wall_next:
                        wall_next = time.time() + hs.READ_EVERY_S
                        rate = sim_time / max(time.time() - wall_start, 1e-3)
                        hs.live(f"{sim_time:9.1f} s  x{rate:5.0f}  I {current_a * 1000.0:6.0f} mA  cell 1: true {true[0]:5.1f}  "
                                f"internal {b['internal'][0]:5.1f}  shown {b['shown'][0]:3d} %  cap {b['capacity'][0]:.4f} Ah  "
                                f"updates {b['updates']}  cycles {b['cycles']}")

                if hs.quit_pressed():
                    hs.event(f"{sim_time:9.1f} s  stopped by the user")
                    raise UserStop()

    return rows, counts, cells


def report_changes(t, cells, current_a, prev, true, b, counts):
    """Event lines for the corrections, capacity updates and cycles seen between two reads."""
    n = len(true)
    ptrue, pb = prev
    # Rest correction: at rest the internal SOC moved clearly more than the true SOC did.
    # While current flows the filter itself moves the SOC, that is not reported.
    moves = [(b["internal"][i] - pb["internal"][i]) - (true[i] - ptrue[i]) for i in range(n)]
    worst = max(range(n), key=lambda i: abs(moves[i]))
    if current_a == 0.0 and abs(moves[worst]) >= JUMP_PCT:
        counts["rest_corrections"] += 1
        hs.event(f"{t:9.1f} s  SOC CORRECTION cell {worst + 1}: internal {pb['internal'][worst]:.1f} -> "
                 f"{b['internal'][worst]:.1f} %, true {true[worst]:.1f} %, shown {b['shown'][worst]} %  "
                 f"({sum(abs(m) >= JUMP_PCT for m in moves)} cells moved)")
    # Full charge: while charging the shown SOC of a cell jumps up to 100 %
    for i in range(n):
        if current_a > 0.0 and b["shown"][i] == 100 and b["shown"][i] - pb["shown"][i] > JUMP_SHOWN_PCT:
            counts["full_charges"] += 1
            hs.event(f"{t:9.1f} s  FULL CHARGE cell {i + 1}: shown {pb['shown'][i]} -> {b['shown'][i]} %, "
                     f"internal {b['internal'][i]:.1f} %, true {true[i]:.1f} %")
    if b["pack"] == 100 and pb["pack"] != 100:
        hs.event(f"{t:9.1f} s  PACK SOC 100 %, true {min(true):.1f} - {max(true):.1f} %")
    if b["updates"] != pb["updates"]:
        hs.event(f"{t:9.1f} s  CAPACITY UPDATE #{b['updates']}: cell 1 {pb['capacity'][0]:.4f} -> {b['capacity'][0]:.4f} Ah "
                 f"(real {cells[0].capacity_as / 3600.0:.4f}), q_nom {b['q_nom']:.4f} Ah, SOH {b['soh'][0]} %")
    if b["cycles"] != pb["cycles"]:
        hs.event(f"{t:9.1f} s  CYCLE {b['cycles']}")


def evaluate(cfg, rows, counts, cells):
    """Measured values and the checks of the test's expect block. Returns (ok, measured text, failures)."""
    if not rows:
        return False, "no SOC read back", ["no SOC read back"]
    n = len(cells)
    errors = [max(abs(r[3]["internal"][i] - r[2][i]) for i in range(n)) for r in rows]
    flowing = [errors[k] for k, r in enumerate(rows) if r[1] != 0.0]
    b = rows[-1][3]
    mean_flow = 0.0
    if flowing:
        mean_flow = sum(flowing) / len(flowing)
    m = {
        "max_error_pct": max(errors),
        "max_final_error_pct": errors[-1],
        "mean_flow_error_pct": mean_flow,
        "rest_corrections": counts["rest_corrections"],
        "full_charges": counts["full_charges"],
        "capacity_updates": b["updates"],
        "final_capacity_error_pct": max(abs(b["capacity"][i] / (cells[i].capacity_as / 3600.0) - 1.0) * 100.0 for i in range(n)),
        "final_shown_pct": min(b["shown"]),
        "remaining_factor": b["remaining"][0] / max(b["shown"][0] * b["capacity"][0] * 10.0, 1e-9),
    }

    checks = {
        "max_error_pct":                lambda v: m["max_error_pct"] <= v,
        "max_final_error_pct":          lambda v: m["max_final_error_pct"] <= v,
        "max_mean_flow_error_pct":      lambda v: m["mean_flow_error_pct"] <= v,
        "min_mean_flow_error_pct":      lambda v: m["mean_flow_error_pct"] >= v,
        "min_rest_corrections":         lambda v: m["rest_corrections"] >= v,
        "min_full_charges":             lambda v: m["full_charges"] >= v,
        "min_capacity_updates":         lambda v: m["capacity_updates"] >= v,
        "max_final_capacity_error_pct": lambda v: m["final_capacity_error_pct"] <= v,
        "final_shown_pct":              lambda v: all(s == v for s in b["shown"]),
        "remaining_factor":             lambda v: v[0] <= m["remaining_factor"] <= v[1],
    }
    failures = []
    for key, value in cfg.get("expect", {}).items():
        if not checks[key](value):
            failures.append(f"{key} {value}")

    measured = (f"error max {m['max_error_pct']:.2f} / final {m['max_final_error_pct']:.2f} / mean while current "
                f"{m['mean_flow_error_pct']:.2f} %, rest corrections {m['rest_corrections']}, full charges "
                f"{m['full_charges']}, capacity updates {m['capacity_updates']} (error {m['final_capacity_error_pct']:.2f} %), "
                f"shown {m['final_shown_pct']} %, remaining factor {m['remaining_factor']:.3f}")
    return not failures, measured, failures


def plot_log(log_path, only_test):
    """Plots log_soc.csv: one window per test (or only only_test)."""
    import matplotlib.pyplot as plt

    tests = {}
    with open(log_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            tests.setdefault(row["test"], []).append(row)
    if only_test is not None:
        if only_test not in tests:
            print(f"Error: no test {only_test} in {log_path}, it holds: {', '.join(tests)}")
            return 1
        tests = {only_test: tests[only_test]}
    if not tests:
        print(f"Error: {log_path} holds no rows")
        return 1

    for name, rows in tests.items():
        n = len([k for k in rows[0] if k.startswith("true_soc_pct_")])
        t = [float(r["sim_time_s"]) / 3600.0 for r in rows]

        def col(key):
            return [float(r[key]) for r in rows]

        fig, ax = plt.subplots(4, 1, sharex=True, figsize=(12, 10))
        fig.canvas.manager.set_window_title(f"SOC test {name}")
        fig.suptitle(name)

        ax[0].plot(t, col("true_soc_pct_1"), label="true cell 1")
        ax[0].plot(t, col("internal_soc_pct_1"), label="internal cell 1")
        ax[0].plot(t, col("shown_soc_pct_1"), label="shown cell 1")
        ax[0].plot(t, col("pack_soc_pct"), "--", label="pack")
        ax[0].set_ylabel("SOC [%]")
        ax[0].legend(loc="best")

        for i in range(n):
            true = col(f"true_soc_pct_{i + 1}")
            internal = col(f"internal_soc_pct_{i + 1}")
            ax[1].plot(t, [b - a for a, b in zip(true, internal)], label=f"cell {i + 1}")
        ax[1].axhline(0.0, color="gray", linewidth=0.5)
        ax[1].set_ylabel("internal - true [%]")
        ax[1].legend(loc="best", ncol=n)

        ax[2].plot(t, col("current_ma"), label="current")
        ax[2].set_ylabel("current [mA]")
        ax_temp = ax[2].twinx()
        ax_temp.plot(t, col("temperature_c"), color="tab:red", label="temperature")
        ax_temp.set_ylabel("temperature [C]", color="tab:red")

        for i in range(n):
            ax[3].plot(t, col(f"capacity_ah_{i + 1}"), label=f"cell {i + 1}")
        ax[3].plot(t, col("q_nom_ah"), "k--", label="q_nom")
        ax[3].set_ylabel("capacity [Ah]")
        ax[3].set_xlabel("simulated time [h]")
        ax[3].legend(loc="best", ncol=n + 1)

        for a in ax:
            a.grid(True)
        fig.tight_layout()

    plt.show()
    return 0


def main(argv):
    here = os.path.dirname(os.path.abspath(__file__))
    if len(argv) < 2:
        print(__doc__.strip().split("\n\n")[1])
        list_test_folders(here)
        return 1

    # A bare folder name is looked up next to this script
    folder = argv[1]
    if not os.path.isdir(folder) and os.path.isdir(os.path.join(here, folder)):
        folder = os.path.join(here, folder)
    if not os.path.isdir(folder):
        print(f"Error: no test folder called {argv[1]}")
        list_test_folders(here)
        return 1

    if len(argv) > 2 and argv[2] == "plot":
        log_path = os.path.join(folder, LOG_FILE)
        if not os.path.isfile(log_path):
            print(f"Error: no log {log_path}, run the tests first")
            return 1
        only_test = None
        if len(argv) > 3:
            only_test = argv[3]
        return plot_log(log_path, only_test)

    results = []
    try:
        tests = load_tests(os.path.join(folder, CONFIG_FILE))

        # Optional test name, then optional port
        names = list(tests)
        port = None
        rest = argv[2:]
        if rest and rest[0] in tests:
            names = [rest[0]]
            rest = rest[1:]
        if rest:
            port = rest[0]

        regs = load_registers(os.path.join(folder, REGISTERS_FILE))
        if port is None:
            port = hs.find_board()
        print(f"OpenBMS on {port}")
        print(f"Press {hs.QUIT_KEY} to stop.")
        log_path = os.path.join(folder, LOG_FILE)
        print(f"Log: {log_path}")
        with serial.Serial(port, hs.BAUD_RATE, timeout=hs.PORT_POLL) as ser, open(log_path, "w", newline="") as log_file:
            log = csv.writer(log_file)
            log.writerow(log_header(regs["cell_count"]))
            time.sleep(hs.OPEN_SETTLE)
            for name in names:
                print(f"\n=== {name}: {tests[name]['description']} ===")
                # Start from a fresh board: learned capacity, counters and fuel gauge as after power-up
                hs.reboot(ser)
                rows, counts, cells = run(ser, regs, name, tests[name], log)
                ok, measured, failures = evaluate(tests[name], rows, counts, cells)
                results.append((name, ok, failures))
                hs.event(f"{name}: {measured}")
                if failures:
                    hs.event(f"{name}: FAIL, not met: {', '.join(failures)}")
                else:
                    hs.event(f"{name}: PASS")
    except UserStop:
        print("Stopped")
        return 1
    except (hs.SimError, serial.SerialException, OSError, KeyError, ValueError) as e:
        hs.event(f"Error: {e}")
        return 1

    print("\nSummary:")
    for name, ok, failures in results:
        print(f"  {name:20s}  {pass_text(ok)}  {', '.join(failures)}")

    if not all(ok for _, ok, _ in results):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
