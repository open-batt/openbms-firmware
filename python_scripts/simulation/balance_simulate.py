#!/usr/bin/env python3
"""
balance_simulate.py -- cell balancing tests for the OpenBMS simulation build.

Usage:
    python balance_simulate.py <test_folder> [test_name] [COM_PORT]
    python balance_simulate.py <test_folder> plot [test_name]

<test_folder> is a path, or just the name of a folder next to this script, for example
7_cell_2000mAh. It must hold fuel_gauge_registers.json (the battery model) and balance.json,
which defines every balancing test. Without test_name all tests run, one after the other; with it
only that one (for example one_cell_high). Without COM_PORT the board is found by pinging every
serial port.

The board must run firmware built with OPENBMS_SIMULATION (config.h) and carry the config of the
test folder (config_tool.py w <test_folder>), with balance_enable on. For every test the script:

    1. reboots the board, checks normal run, cell_count and balance_enable, reads
       balance_resistance_ohm (0x6A) and balance_max_temp_c (0x69), writes the battery model,
    2. starts every cell at start_soc_pct plus its own cell_soc_offset_pct (the imbalance) and
       rests for the fuel gauge seed,
    3. runs the profile. The balancer bits of fet_status in every sample reply (bit 2 + cell) say
       which cells bleed: such a cell carries the pack current minus its bleed current
       (cell voltage / balance_resistance_ohm), like the real resistor across its taps. The sim
       build keeps the real pins off, so the bleed exists only in this model.
    4. checks every sample reply against the rules below and every read_every_s of simulated time
       reads the board's internal SOC (0xD0),
    5. checks the test's expect values and gives PASS or FAIL.

Rules checked in every test (a cell may bleed for at most RULE_GRACE_S in total against them, the
board's filtered current and temperature lag a step behind the samples):
    no cell bleeds while the pack current is not a charge (<= 0)
    no cell bleeds at or above balance_max_temp_c
    no cell bleeds while a protection is tripped

While a test runs, one live line shows the simulated time, current, cell voltage spread and the
bleeding cells, updated in place. New lines are printed for segments, balancer changes and trips.
At the end a PASS/FAIL summary is printed.

Log: <test_folder>/log_balance.csv, one row per read of every test run, the test name in the first
column.

Press q to stop (on Windows without Enter, elsewhere q then Enter).

plot: no test runs and no board is needed. The existing <test_folder>/log_balance.csv is plotted,
one window per test in the log (only test_name when given): the cell voltages, the true SOC of
every cell, the bleeding cells and the current.

balance.json:
    dt_ms, temperature_c, read_every_s, cell_min_mv, cell_max_mv, noise
              defaults of every test; a test can give its own
    tests     {name: test}, run in this order. A test has:
      description
      branch               OCV branch the battery starts on, "dis" or "chg"
      start_soc_pct        true SOC of every cell at the start
      cell_soc_offset_pct  list, one value per cell, added to start_soc_pct (the imbalance)
      profile              segments as in soc_simulate.py (current sign: + charge, - discharge).
                           In cccv the bleed current of the cell at the voltage limit is added, so
                           the charger keeps that cell at the limit while it bleeds.
      expect               the pass conditions, all optional:
                             cells                     cells (1..n) allowed to bleed; [] = no cell may
                             min_balance_s             the cells of "cells" bleed at least this long
                                                       (sum over those cells, simulated seconds)
                             max_final_soc_spread_pct  true SOC, highest - lowest cell at the end
                             max_error_pct             board internal SOC - true SOC, worst cell, at
                                                       every read (shows the bleed correction of the
                                                       fuel gauge)
                             trip                      true: a protection must trip in the test
                             min_capacity_updates      capacity_update_count (0x97) at the end, at least
                             max_capacity_error_pct    learned capacity (0xD4) vs the real one, worst
                                                       cell, at the end (shows the bleed is taken off
                                                       the charge counted for capacity learning)
"""

import csv
import json
import math
import os
import random
import struct
import sys
import time

# Board link, battery model and profile segments are shared with the other simulators.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hppc_simulate as hs
import soc_simulate as fs

import serial

CONFIG_FILE    = "balance.json"
LOG_FILE       = "log_balance.csv"
TEST_DEFAULTS  = ("dt_ms", "temperature_c", "read_every_s", "cell_min_mv", "cell_max_mv", "noise")
EXPECT_KEYS    = ("cells", "min_balance_s", "max_final_soc_spread_pct", "max_error_pct", "trip",
                  "min_capacity_updates", "max_capacity_error_pct")

REG_BALANCE_ENABLE   = 0x66
REG_BALANCE_MAX_TEMP = 0x69
REG_BALANCE_R_OHM    = 0x6A
FET_BAL_SHIFT        = 2         # PER_FET_BAL_SHIFT: bit 2 + cell in fet_status
FET_BAL_MASK         = 0x7F
RULE_GRACE_S         = 1.0       # bleeding against a rule for at most this long in total


class UserStop(Exception):
    pass


def pass_text(ok):
    if ok:
        return "PASS"
    return "FAIL"


def cells_text(mask):
    names = [str(i + 1) for i in range(hs.CELL_COUNT_MAX) if mask & (1 << i)]
    if not names:
        return "none"
    return ",".join(names)


def balance_mask(reply):
    if reply is None:
        return 0
    return (reply["fet"] >> FET_BAL_SHIFT) & FET_BAL_MASK


class BalanceCell(fs.TempCell):
    """Model cell with its balance resistor: while bleed_a > 0 the cell carries the pack current minus it."""
    bleed_a = 0.0

    def step(self, current_a, dt_s):
        fs.TempCell.step(self, current_a - self.bleed_a, dt_s)

    def terminal(self, current_a):
        return fs.TempCell.terminal(self, current_a - self.bleed_a)


class BalanceSegment(fs.Segment):
    def current_a(self, cells):
        """As soc_simulate, but cccv adds the bleed of each cell: the charger holds the cell at the
        voltage limit while part of its current flows through the resistor."""
        if self.seg["type"] != "cccv":
            return fs.Segment.current_a(self, cells)
        i = self.seg["current_ma"] / 1000.0
        v_lim = self.seg["cell_voltage_mv"] / 1000.0
        for c in cells:
            i_cell = (v_lim - c.emf()) / c.r0() + c.bleed_a
            if i_cell < i:
                i = i_cell
            i_full = (1.0 - c.soc) * c.capacity_as / self.dt_s + c.bleed_a
            if i_full < i:
                i = i_full
        if i < 0.0:
            i = 0.0
        return i


def load_tests(path, n):
    """balance.json as {name: test}, each test with the defaults filled in."""
    with open(path) as f:
        cfg = json.load(f)
    tests = {}
    for name, test in cfg["tests"].items():
        t = {key: cfg[key] for key in TEST_DEFAULTS}
        t.update(test)
        if t.get("branch", "chg") not in ("dis", "chg"):
            raise hs.SimError(f"{name}: branch must be dis or chg, not {t['branch']!r}")
        if not 1 <= t["dt_ms"] <= fs.DT_MS_MAX:
            raise hs.SimError(f"{name}: dt_ms must be 1..{fs.DT_MS_MAX}")
        offsets = t.get("cell_soc_offset_pct", [0.0] * n)
        if len(offsets) != n:
            raise hs.SimError(f"{name}: cell_soc_offset_pct needs {n} values, it has {len(offsets)}")
        t["cell_soc_offset_pct"] = offsets
        if not t["profile"]:
            raise hs.SimError(f"{name}: profile is empty")
        for seg in t["profile"]:
            if seg.get("type") not in fs.SEGMENT_TYPES:
                raise hs.SimError(f"{name}: unknown segment type {seg.get('type')!r}")
        for key in t.get("expect", {}):
            if key not in EXPECT_KEYS:
                raise hs.SimError(f"{name}: unknown expect key {key!r}")
        for cell in t.get("expect", {}).get("cells", []):
            if not 1 <= cell <= n:
                raise hs.SimError(f"{name}: expect cells holds {cell}, the cells are 1..{n}")
        tests[name] = t
    return tests


def check_board(ser, regs):
    """Checks the board and returns (balance_resistance_ohm, balance_max_temp_c)."""
    fs.check_board(ser, regs)
    if hs.read_register(ser, REG_BALANCE_ENABLE)[0] == 0:
        raise hs.SimError("balance_enable is off in the board config (0x66), set it in config.json and run config_tool.py w")
    r_ohm = struct.unpack("<f", hs.read_register(ser, REG_BALANCE_R_OHM)[:4])[0]
    if not r_ohm > 0.0:
        raise hs.SimError(f"balance_resistance_ohm (0x6A) is {r_ohm}, it must be above 0")
    max_temp_c = struct.unpack("<h", hs.read_register(ser, REG_BALANCE_MAX_TEMP)[:2])[0]
    return r_ohm, max_temp_c


def read_capacity(ser, n):
    """Learned capacity of every cell (Ah) and capacity_update_count."""
    capacity = []
    for i in range(n):
        hs.write_register(ser, hs.REG_MODEL_CELL, bytes([i]))
        capacity.append(struct.unpack("<f", hs.read_register(ser, fs.REG_CAPACITY_AH)[:4])[0])
    hs.write_register(ser, hs.REG_MODEL_CELL, bytes([0]))
    updates = struct.unpack("<H", hs.read_register(ser, fs.REG_CAP_UPDATES)[:2])[0]
    return capacity, updates


def read_internal(ser, n):
    soc_f = struct.unpack(f"<{hs.CELL_COUNT_MAX}f", hs.read_register(ser, fs.REG_CELL_SOC_F)[:hs.CELL_COUNT_MAX * 4])
    return [v * 100.0 for v in soc_f[:n]]


def log_header(n):
    return (["test", "sim_time_s", "segment", "segment_type", "current_ma", "temperature_c", "protection_trip",
             "balance_mask"]
            + [f"cell_mv_{i + 1}" for i in range(n)]
            + [f"true_soc_pct_{i + 1}" for i in range(n)]
            + [f"internal_soc_pct_{i + 1}" for i in range(n)]
            + [f"bleed_ma_{i + 1}" for i in range(n)])


def run(ser, regs, name, cfg, log):
    """Runs one test. Returns the measured values as a dict."""
    branch = cfg.get("branch", "chg")
    cells = fs.make_cells(regs, cfg, branch)
    n = len(cells)
    for i, c in enumerate(cells):
        c.__class__ = BalanceCell
        c.soc += cfg["cell_soc_offset_pct"][i] / 100.0
    rng = random.Random(cfg["noise"]["seed"])
    noise_v = cfg["noise"]["voltage_mv"]
    noise_i = cfg["noise"]["current_ma"]
    dt_ms = int(cfg["dt_ms"])
    dt_s = dt_ms / 1000.0
    temp_c = float(cfg["temperature_c"])
    allowed = 0
    for cell in cfg.get("expect", {}).get("cells", []):
        allowed |= 1 << (cell - 1)

    r_ohm, max_temp_c = check_board(ser, regs)
    fs.write_model(ser, regs)
    fs.set_temperature(cells, regs, temp_c)

    # Rest first: the fuel gauge seeds every cell from its own OCV
    for _ in range(int(math.ceil(fs.SEED_REST_S / dt_s))):
        hs.send_sample(ser, fs.cell_samples(cells, 0.0, rng, noise_v), 0.0, temp_c, dt_ms)

    m = {"balance_s": [0.0] * n, "other_cells_s": 0.0, "not_charging_s": 0.0, "hot_s": 0.0, "tripped_s": 0.0,
         "trips": 0, "max_error_pct": 0.0, "start_spread_pct": 0.0, "final_spread_pct": 0.0}
    true = [c.soc * 100.0 for c in cells]
    m["start_spread_pct"] = max(true) - min(true)

    sim_time = 0.0
    next_read = 0.0
    wall_start = time.time()
    wall_next = wall_start
    last_mask = 0
    last_trip = 0

    batch = hs.SampleBatch(ser)
    for index, seg_cfg in enumerate(cfg["profile"]):
        seg_no = index + 1
        seg = BalanceSegment(seg_cfg, rng, temp_c, cfg["cell_min_mv"], cfg["cell_max_mv"], dt_s)
        fs.set_temperature(cells, regs, seg.temperature_c)
        hs.event(f"{sim_time:9.1f} s  segment {seg_no}/{len(cfg['profile'])} {seg.name()}  {seg_cfg.get('note', '')}")

        while True:
            snap = (hs.snapshot(cells + [seg], rng), sim_time)
            # The cells bleed as the board's last reply says
            mask = balance_mask(batch.reply)
            for i, c in enumerate(cells):
                c.bleed_a = 0.0
                if mask & (1 << i):
                    c.bleed_a = fs.TempCell.terminal(c, 0.0) / r_ohm
            current_a = seg.current_a(cells)
            end = seg.finished(cells, current_a)

            if not end:
                for c in cells:
                    c.step(current_a, dt_s)
                sim_time += dt_s
                seg.t += dt_s
                current_ma = current_a * 1000.0 + rng.gauss(0.0, noise_i)
                cell_mv = fs.cell_samples(cells, current_a, rng, noise_v)
                full = batch.add(snap, (cell_mv, current_ma, seg.temperature_c, dt_ms),
                                 (sim_time, current_a, seg.temperature_c))
                if not full and sim_time < next_read:
                    continue

            done, back = batch.flush()
            # Every sample the board ran, with the reply after it
            for info, reply in done:
                t, i_a, t_c = info
                bal = balance_mask(reply)
                for i in range(n):
                    if bal & (1 << i):
                        m["balance_s"][i] += dt_s
                if bal & ~allowed:
                    m["other_cells_s"] += dt_s
                if bal and i_a <= 0.0:
                    m["not_charging_s"] += dt_s
                if bal and t_c >= max_temp_c:
                    m["hot_s"] += dt_s
                if bal and reply["trip"]:
                    m["tripped_s"] += dt_s
                if reply["trip"] != last_trip:
                    if reply["trip"]:
                        m["trips"] += 1
                    hs.event(f"{t:9.1f} s  PROTECTION {hs.trip_text(reply['trip'])}")
                    last_trip = reply["trip"]
                if bal != last_mask:
                    mv = [c.terminal(i_a) * 1000.0 for c in cells]
                    hs.event(f"{t:9.1f} s  BALANCE cells {cells_text(bal)}  (was {cells_text(last_mask)}), "
                             f"{i_a * 1000.0:.0f} mA, cells {min(mv):.0f} - {max(mv):.0f} mV")
                    last_mask = bal
            if back is not None:
                # The board stopped at a change of its reply: make the samples after it again
                hs.restore(back[0])
                sim_time = back[1]
                continue
            if end:
                break

            if sim_time >= next_read:
                next_read += cfg["read_every_s"]
                true = [c.soc * 100.0 for c in cells]
                internal = read_internal(ser, n)
                error = max(abs(internal[i] - true[i]) for i in range(n))
                if error > m["max_error_pct"]:
                    m["max_error_pct"] = error
                mask = balance_mask(batch.reply)
                mv = [c.terminal(current_a) * 1000.0 for c in cells]
                log.writerow([name, round(sim_time, 2), seg_no, seg.name(), round(current_a * 1000.0, 1),
                              seg.temperature_c, hs.trip_text(batch.reply["trip"]), mask]
                             + [round(v, 1) for v in mv] + [round(v, 3) for v in true]
                             + [round(v, 3) for v in internal] + [round(c.bleed_a * 1000.0, 1) for c in cells])

                if time.time() >= wall_next:
                    wall_next = time.time() + hs.READ_EVERY_S
                    rate = sim_time / max(time.time() - wall_start, 1e-3)
                    hs.live(f"{sim_time:9.1f} s  x{rate:5.0f}  I {current_a * 1000.0:6.0f} mA  cells {min(mv):6.1f} - "
                            f"{max(mv):6.1f} mV  SOC {min(true):5.1f} - {max(true):5.1f} %  bleeding {cells_text(mask)}")

            if hs.quit_pressed():
                hs.event(f"{sim_time:9.1f} s  stopped by the user")
                raise UserStop()

    true = [c.soc * 100.0 for c in cells]
    m["final_spread_pct"] = max(true) - min(true)
    capacity, m["capacity_updates"] = read_capacity(ser, n)
    m["capacity_error_pct"] = [(capacity[i] / (cells[i].capacity_as / 3600.0) - 1.0) * 100.0 for i in range(n)]
    return m


def evaluate(cfg, m):
    """Checks the rules and the test's expect block. Returns (ok, measured text, failures)."""
    expect = cfg.get("expect", {})
    failures = []
    if m["not_charging_s"] > RULE_GRACE_S:
        failures.append(f"bled {m['not_charging_s']:.1f} s while not charging")
    if m["hot_s"] > RULE_GRACE_S:
        failures.append(f"bled {m['hot_s']:.1f} s at or above balance_max_temp_c")
    if m["tripped_s"] > RULE_GRACE_S:
        failures.append(f"bled {m['tripped_s']:.1f} s with a protection tripped")

    if "cells" in expect and m["other_cells_s"] > 0.0:
        failures.append(f"cells {expect['cells']}: other cells bled {m['other_cells_s']:.1f} s")
    if "min_balance_s" in expect:
        total = sum(m["balance_s"][c - 1] for c in expect.get("cells", range(1, len(m["balance_s"]) + 1)))
        if total < expect["min_balance_s"]:
            failures.append(f"min_balance_s {expect['min_balance_s']} (bled {total:.0f} s)")
    if "max_final_soc_spread_pct" in expect and m["final_spread_pct"] > expect["max_final_soc_spread_pct"]:
        failures.append(f"max_final_soc_spread_pct {expect['max_final_soc_spread_pct']}")
    if "max_error_pct" in expect and m["max_error_pct"] > expect["max_error_pct"]:
        failures.append(f"max_error_pct {expect['max_error_pct']}")
    if expect.get("trip") and m["trips"] == 0:
        failures.append("trip (no protection tripped)")
    if "min_capacity_updates" in expect and m["capacity_updates"] < expect["min_capacity_updates"]:
        failures.append(f"min_capacity_updates {expect['min_capacity_updates']}")
    worst_capacity = max(abs(e) for e in m["capacity_error_pct"])
    if "max_capacity_error_pct" in expect and worst_capacity > expect["max_capacity_error_pct"]:
        failures.append(f"max_capacity_error_pct {expect['max_capacity_error_pct']}")

    bled = ", ".join(f"{i + 1}: {s:.0f} s" for i, s in enumerate(m["balance_s"]) if s > 0.0)
    if not bled:
        bled = "none"
    measured = (f"bled {bled}; SOC spread {m['start_spread_pct']:.2f} -> {m['final_spread_pct']:.2f} %, "
                f"fuel gauge error max {m['max_error_pct']:.2f} %, trips {m['trips']}, capacity updates "
                f"{m['capacity_updates']}, capacity error " + " ".join(f"{e:+.2f}" for e in m["capacity_error_pct"]) + " %")
    return not failures, measured, failures


def plot_log(log_path, only_test):
    """Plots log_balance.csv: one window per test (or only only_test)."""
    import matplotlib.pyplot as plt

    tests = {}
    with open(log_path, newline="") as f:
        for row in csv.DictReader(f):
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
        t = [float(r["sim_time_s"]) / 60.0 for r in rows]

        def col(key):
            return [float(r[key]) for r in rows]

        fig, ax = plt.subplots(4, 1, sharex=True, figsize=(12, 10))
        fig.canvas.manager.set_window_title(f"Balance test {name}")
        fig.suptitle(name)
        for i in range(n):
            ax[0].plot(t, col(f"cell_mv_{i + 1}"), label=f"cell {i + 1}")
            ax[1].plot(t, col(f"true_soc_pct_{i + 1}"), label=f"cell {i + 1}")
            # Bleeding cells as steps, one row per cell
            ax[2].step(t, [i + 1 + 0.8 * v / max(v, 1e-9) if v > 0.0 else i + 1 for v in col(f"bleed_ma_{i + 1}")],
                       where="post", label=f"cell {i + 1}")
        ax[0].set_ylabel("cell [mV]")
        ax[0].legend(loc="best", ncol=n)
        ax[1].set_ylabel("true SOC [%]")
        ax[2].set_ylabel("bleeding (step up)")
        ax[2].set_yticks(range(1, n + 1))
        ax[3].plot(t, col("current_ma"))
        ax[3].set_ylabel("current [mA]")
        ax[3].set_xlabel("simulated time [min]")
        ax_temp = ax[3].twinx()
        ax_temp.plot(t, col("temperature_c"), color="tab:red")
        ax_temp.set_ylabel("temperature [C]", color="tab:red")
        for a in ax:
            a.grid(True)
        fig.tight_layout()

    plt.show()
    return 0


def main(argv):
    here = os.path.dirname(os.path.abspath(__file__))
    if len(argv) < 2:
        print(__doc__.strip().split("\n\n")[1])
        fs.list_test_folders(here)
        return 1

    # A bare folder name is looked up next to this script
    folder = argv[1]
    if not os.path.isdir(folder) and os.path.isdir(os.path.join(here, folder)):
        folder = os.path.join(here, folder)
    if not os.path.isdir(folder):
        print(f"Error: no test folder called {argv[1]}")
        fs.list_test_folders(here)
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
        regs = fs.load_registers(os.path.join(folder, fs.REGISTERS_FILE))
        tests = load_tests(os.path.join(folder, CONFIG_FILE), regs["cell_count"])

        # Optional test name, then optional port
        names = list(tests)
        port = None
        rest = argv[2:]
        if rest and rest[0] in tests:
            names = [rest[0]]
            rest = rest[1:]
        if rest:
            port = rest[0]

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
                hs.reboot(ser)
                m = run(ser, regs, name, tests[name], log)
                ok, measured, failures = evaluate(tests[name], m)
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
