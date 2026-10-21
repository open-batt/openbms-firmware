#!/usr/bin/env python3
"""
bms_sim.py -- battery simulator for the OpenBMS simulation build.

Usage:
    python bms_sim.py <simulation.json> [COM_PORT]

The simulation file is a path, or just the name of a .json file next to this script,
for example 7cell_2000mAh_discharge_2A.json. Without one, the files found here are listed.

The board must run firmware built with OPENBMS_SIMULATION (config.h), be configured
(config_tool.py w) and be in normal run. The script:

    1. starts an HPPC test on the board (CC_HPPC_START),
    2. simulates the battery and the load, and sends one sample per transaction
       (CC_SIM_SAMPLE: 7 cell voltages, pack current, dt). The board runs its control
       loop once per sample and answers with the FET state and the HPPC status,
    3. when the board turns the FET on, the load (or the charger) draws current,
       when it turns the FET off, the current is 0,
    4. stops when the board is back in normal run (test and parameter extraction done),
       then reads the extracted tables and compares them with the simulated battery.

The comparison is printed and saved in results/sim_compare_<simulation file name>.csv next to this script.

Simulation file:
    mode              "discharge" (load resistor) or "charge" (CC/CV charger)
    cell_count        number of simulated cells, must match the board's cell_count
    dt_ms             simulated time step per sample
    max_sim_time_s    the run is stopped after this much simulated time
    discharge         load_ohm: load resistor across the pack
    charge            current_ma: charger current, voltage_limit_mv: charger voltage (pack)
    noise             gaussian noise: voltage_mv (per cell), current_ma; seed
    cell_default      the model of one cell:
                        capacity_mah, start_soc_pct (null: 100 % for discharge, 0 % for charge),
                        ocv: formula coefficients k0..k5 (see "formula"),
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
import struct
import sys
import time

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

CELL_COUNT_MAX = 7      # BD_CELL_COUNT_MAX in openbms_data.h, the sample always carries 7 cells

TIMEOUT_READ  = 1.0     # s -- register read / write / command
TIMEOUT_BOOT  = 10.0    # s -- how long to wait for the board after opening the port
PORT_POLL     = 0.02    # s -- serial read timeout
OPEN_SETTLE   = 0.5     # s -- pause after opening a port

REG_CTRL_STATE = 0x13
REG_SOC_GRID   = 0xA0
REG_MODEL_CELL = 0x9F

CTRL_STATE_NORMAL_RUN = 2
CTRL_STATE_HPPC_RUN   = 3

FET_MAIN = 0x01         # BD_FET_MAIN

HPPC_STATES = {0: "IDLE", 1: "DETECT", 2: "REST", 3: "RAMP", 4: "DONE", 5: "ABORTED"}
HPPC_STOPS = {
    0: "none", 1: "cell max", 2: "cell min", 3: "term current", 4: "no current",
    5: "no config", 6: "host", 7: "protection", 8: "current range",
}
HPPC_X_BUSY = 1 << 8
HPPC_X_DONE = 1 << 9

# Table registers per direction: OCV, R0, R1, tau1, R2, tau2 (units V, Ohm, Ohm, s, Ohm, s)
TABLE_REGS = {
    "discharge": [0xA1, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7],
    "charge":    [0xA2, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC],
}
TABLE_NAMES = ["ocv_v", "r0_mohm", "r1_mohm", "tau1_s", "r2_mohm", "tau2_s"]
TABLE_SCALE = [1.0, 1000.0, 1000.0, 1.0, 1000.0, 1.0]    # board unit -> printed unit
SOC_GRID_POINTS = 20

PRINT_EVERY_S = 60.0    # simulated seconds between progress lines

# Sample: float cell_mv[7], float current_ma, uint16 dt_ms. Reply: see Sim_Sample in openbms_comm.c
SAMPLE_FORMAT = "<7ffH"
REPLY_FORMAT  = "<HBHHBQ"


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


def send_sample(ser, cell_mv, current_ma, dt_ms):
    payload = struct.pack(SAMPLE_FORMAT, *cell_mv, current_ma, dt_ms)
    raw = transact(ser, build_frame(CC_SIM_SAMPLE, 0, payload), "sim sample")
    if raw[0] != CC_SIM_SAMPLE:
        raise SimError("sim sample: unexpected answer (is the firmware built with OPENBMS_SIMULATION?)")
    fet, state, hppc, extract, trip, errors = struct.unpack(REPLY_FORMAT, raw[3:3 + raw[2]])
    return {"fet": fet, "ctrl_state": state, "hppc_status": hppc,
            "extract": extract, "trip": trip, "errors": errors}


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
        self.soc_limit = p["ocv"]["soc_limit"]
        self.grid = p["soc_pct"]
        self.tables = {name: p[name] for name in ("r0_mohm", "r1_mohm", "tau1_s", "r2_mohm", "tau2_s")}
        for name, values in self.tables.items():
            if len(values) != len(self.grid):
                raise SimError(f"{name} has {len(values)} values, soc_pct has {len(self.grid)}")

        if p.get("start_soc_pct") is not None:
            start_soc_pct = p["start_soc_pct"]
        self.soc = start_soc_pct / 100.0
        self.v1 = 0.0       # RC1 voltage in V
        self.v2 = 0.0       # RC2 voltage in V

    def ocv(self, soc=None):
        if soc is None:
            soc = self.soc
        s = min(max(soc, self.soc_limit), 1.0 - self.soc_limit)
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

    if cfg["mode"] not in ("discharge", "charge"):
        raise SimError('mode must be "discharge" or "charge"')
    if not 1 <= cfg["cell_count"] <= CELL_COUNT_MAX:
        raise SimError(f"cell_count must be 1..{CELL_COUNT_MAX}")
    if len(cfg["cells"]) < cfg["cell_count"]:
        raise SimError(f"cells has {len(cfg['cells'])} entries, cell_count is {cfg['cell_count']}")
    return cfg


def make_cells(cfg):
    if cfg["mode"] == "discharge":
        start = 100.0
    else:
        start = 0.0

    cells = []
    for i in range(cfg["cell_count"]):
        p = dict(cfg["cell_default"])
        p.update(cfg["cells"][i])
        cells.append(Cell(p, start))
    return cells


def pack_current(cfg, cells, fet_on):
    """Current in A (+ charge) for the next step, from the load or the charger."""
    if not fet_on:
        return 0.0

    emf = sum(c.emf() for c in cells)
    r0 = sum(c.r0() for c in cells)

    if cfg["mode"] == "discharge":
        return -emf / (cfg["discharge"]["load_ohm"] + r0)

    # Charger: constant current until the pack reaches the voltage limit, then constant voltage
    i_cc = cfg["charge"]["current_ma"] / 1000.0
    v_cv = cfg["charge"]["voltage_limit_mv"] / 1000.0
    if emf + i_cc * r0 <= v_cv:
        return i_cc
    return max(0.0, (v_cv - emf) / r0)


# ---------------------------------------------------------------
# Run
# ---------------------------------------------------------------
def describe(r):
    state = HPPC_STATES.get(r["hppc_status"] & 0xFF, "?")
    stop = HPPC_STOPS.get(r["hppc_status"] >> 8, "?")
    if r["fet"] & FET_MAIN:
        fet = "on"
    else:
        fet = "off"
    return (f"ctrl {r['ctrl_state']}, hppc {state} (stop: {stop}), extract 0x{r['extract']:04X}, "
            f"FET {fet}, trip {r['trip']}, errors 0x{r['errors']:X}")


def run(ser, cfg):
    cells = make_cells(cfg)
    rng = random.Random(cfg["noise"]["seed"])
    noise_v = cfg["noise"]["voltage_mv"]
    noise_i = cfg["noise"]["current_ma"]
    dt_ms = int(cfg["dt_ms"])
    dt_s = dt_ms / 1000.0

    state = struct.unpack("<B", read_register(ser, REG_CTRL_STATE)[:1])[0]
    if state != CTRL_STATE_NORMAL_RUN:
        raise SimError(f"the board is in ctrl state {state}, it must be in normal run ({CTRL_STATE_NORMAL_RUN})")

    send_command(ser, CC_HPPC_START, "HPPC start")
    print(f"HPPC test started, {cfg['mode']}, {cfg['cell_count']} cells, dt {dt_ms} ms")

    fet_on = False
    sim_time = 0.0
    next_print = 0.0
    started = False
    wall_start = time.time()

    while sim_time < cfg["max_sim_time_s"]:
        current_a = pack_current(cfg, cells, fet_on)
        for c in cells:
            c.step(current_a, dt_s)
        sim_time += dt_s

        cell_mv = [0.0] * CELL_COUNT_MAX
        for i, c in enumerate(cells):
            cell_mv[i] = c.terminal(current_a) * 1000.0 + rng.gauss(0.0, noise_v)
        current_ma = current_a * 1000.0 + rng.gauss(0.0, noise_i)

        r = send_sample(ser, cell_mv, current_ma, dt_ms)
        fet_on = bool(r["fet"] & FET_MAIN)

        if r["ctrl_state"] == CTRL_STATE_HPPC_RUN:
            started = True
        elif started:
            print(f"{sim_time:9.1f} s  test finished: {describe(r)}")
            break

        if sim_time >= next_print:
            soc = sum(c.soc for c in cells) / len(cells) * 100.0
            rate = sim_time / max(time.time() - wall_start, 1e-3)
            print(f"{sim_time:9.1f} s  SOC {soc:5.1f} %  I {current_a:6.3f} A  V1 {cell_mv[0]:7.1f} mV  "
                  f"x{rate:5.1f}  {describe(r)}")
            next_print += PRINT_EVERY_S
    else:
        print(f"stopped after max_sim_time_s ({cfg['max_sim_time_s']} s), the test did not finish")
        return None

    if not r["extract"] & HPPC_X_DONE:
        print("the board wrote no tables, nothing to compare")
        return None
    return cells


def compare(ser, cfg, cells, name):
    grid = struct.unpack(f"<{SOC_GRID_POINTS}f", read_register(ser, REG_SOC_GRID))
    regs = TABLE_REGS[cfg["mode"]]

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"sim_compare_{name}.csv")

    header = ["cell", "soc_pct"]
    for name in TABLE_NAMES:
        header += [f"{name}_true", f"{name}_board", f"{name}_err_pct"]

    rows = []
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
            row = [ci, round(soc_pct, 2)]
            for ti, name in enumerate(TABLE_NAMES):
                b = board[ti][gi]
                err = (b - true[ti]) / true[ti] * 100.0
                worst[name] = max(worst[name], abs(err))
                row += [round(true[ti], 5), round(b, 5), round(err, 2)]
            rows.append(row)

    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

    print("Largest error per parameter, all cells and SOC points:")
    for name in TABLE_NAMES:
        print(f"  {name:8s} {worst[name]:7.2f} %")
    print(f"Table saved to {path}")


def main(argv):
    here = os.path.dirname(os.path.abspath(__file__))
    port = None
    json_path = None
    for arg in argv[1:]:
        if arg.lower().endswith(".json"):
            json_path = arg
        else:
            port = arg

    if json_path is None:
        print("Give a simulation file. Files next to this script:")
        for f in sorted(os.listdir(here)):
            if f.lower().endswith(".json"):
                print(f"  {f}")
        return 1

    # A bare file name is looked up next to this script
    if not os.path.exists(json_path) and os.path.exists(os.path.join(here, json_path)):
        json_path = os.path.join(here, json_path)
    name = os.path.splitext(os.path.basename(json_path))[0]

    try:
        print(f"Simulation file: {json_path}")
        cfg = load_json(json_path)
        if port is None:
            port = find_board()
        with serial.Serial(port, BAUD_RATE, timeout=PORT_POLL) as ser:
            time.sleep(OPEN_SETTLE)
            wait_until_ready(ser)
            cells = run(ser, cfg)
            if cells is not None:
                compare(ser, cfg, cells, name)
    except (SimError, serial.SerialException, OSError, KeyError, ValueError) as e:
        print(f"Error: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
