#!/usr/bin/env python3
"""
protection_simulate.py -- protection tests for the OpenBMS simulation build.

Usage:
    python protection_simulate.py <test_folder> [test_name] [COM_PORT]
    python protection_simulate.py <test_folder> plot [test_name]

<test_folder> is a path, or just the name of a folder next to this script, for example
7_cell_2000mAh. It must hold fuel_gauge_registers.json (the battery model) and protection.json,
which defines every protection test. Without test_name all tests run, one after the other;
with it only that one (for example ocp_charge). Without COM_PORT the board is found by pinging
every serial port.

The board must run firmware built with OPENBMS_SIMULATION (config.h) and carry the config of
the test folder (config_tool.py w <test_folder>). For every test the script:

    1. reboots the board, checks normal run and cell_count, writes the battery model,
    2. rests for the fuel gauge seed, then switches the FET on (CC_SET_FET),
    3. runs the test's profile. The simulated battery follows the board's FET: no current
       while the board holds it open, as on a real pack,
    4. passes when the expected protection, and only that one, trips in the segment marked
       expect_trip, nothing trips before it, and after reset_protection the protection is
       cleared and the FET is back on.

While a test runs, one live line shows the simulated current, cell voltages, temperature, FET and
protection, updated in place. New lines are printed only for events: segments, FET changes,
protection trips, error flags. At the end a PASS/FAIL summary is printed.

Log: <test_folder>/log_protection.csv, one row per sample (every dt_ms) of every test run, the
test name in the first column.

Press q to stop (on Windows without Enter, elsewhere q then Enter).

plot: no test runs and no board is needed. The existing <test_folder>/log_protection.csv is
plotted, one window per test in the log (only test_name when given): current, lowest and highest
cell voltage, temperature, and the FET. Dotted lines mark the segments, the red line and band
mark the time the protection is tripped.

protection.json:
    dt_ms, temperature_c, cell_min_mv, cell_max_mv, noise   defaults of every test
    tests     {name: test}, run in this order. A test has description, expect (the protection
              that must trip, a CTRL_PROT_* name such as OCP_CHARGE), branch (dis or chg, the
              model tables the simulated cells start on), start_soc_pct and profile (segments
              as in soc_simulate.py; one of them carries "expect_trip": true). Any default
              given in a test replaces the default for that test.
"""

import csv
import os
import random
import sys
import time

# Board link, battery model and profile segments are shared with the other simulators.
# soc_simulate holds the battery model and the profile segments.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hppc_simulate as hs
import soc_simulate as fs

import serial

CONFIG_FILE = "protection.json"
LOG_FILE    = "log_protection.csv"
LOG_HEADER  = ["test", "sim_time_s", "segment", "segment_type", "segment_time_s", "current_ma", "cell_min_mv",
               "cell_max_mv", "temperature_c", "fet", "protection_trip", "ctrl_state", "errors"]
TEST_DEFAULTS = ("dt_ms", "temperature_c", "cell_min_mv", "cell_max_mv", "noise")


class UserStop(Exception):
    pass


def onoff(value):
    if value:
        return "ON"
    return "OFF"


def pass_text(ok):
    if ok:
        return "PASS"
    return "FAIL"


def load_tests(path):
    """protection.json as {name: test}, each test with the defaults filled in, in the format of
    soc_simulate test."""
    with open(path) as f:
        cfg = fs.json.load(f)
    tests = {}
    for name, test in cfg["tests"].items():
        t = {key: cfg[key] for key in TEST_DEFAULTS}
        t.update(test)
        if t["expect"] not in hs.PROTECTION_NAMES:
            raise hs.SimError(f"{name}: expect must be one of {', '.join(hs.PROTECTION_NAMES)}")
        if t["branch"] not in ("dis", "chg"):
            raise hs.SimError(f"{name}: branch must be dis or chg, not {t['branch']!r}")
        if sum(1 for seg in t["profile"] if seg.get("expect_trip")) != 1:
            raise hs.SimError(f"{name}: exactly one segment must carry \"expect_trip\": true")
        for seg in t["profile"]:
            if seg.get("type") not in fs.SEGMENT_TYPES:
                raise hs.SimError(f"{name}: unknown segment type {seg.get('type')!r}")
        tests[name] = t
    return tests


def run(ser, regs, name, cfg, log):
    """Runs one test. Returns (ok, text) for the summary."""
    cells = fs.make_cells(regs, cfg, cfg["branch"])
    n = len(cells)
    rng = random.Random(cfg["noise"]["seed"])
    noise_v = cfg["noise"]["voltage_mv"]
    noise_i = cfg["noise"]["current_ma"]
    dt_ms = int(cfg["dt_ms"])
    dt_s = dt_ms / 1000.0
    temp_c = float(cfg["temperature_c"])
    expect_bit = 1 << hs.PROTECTION_NAMES.index(cfg["expect"])

    fs.check_board(ser, regs)
    fs.write_model(ser, regs)

    # Rest first: the fuel gauge seeds its SOC from the OCV
    for _ in range(int(round(fs.SEED_REST_S / dt_s))):
        hs.send_sample(ser, fs.cell_samples(cells, 0.0, rng, noise_v), 0.0, temp_c, dt_ms)
    hs.transact(ser, hs.build_frame(fs.CC_SET_FET, 0, bytes([1])), "set FET")

    sim_time = 0.0
    wall_next = time.time()
    last = None             # (fet, trip, errors) after the previous sample
    trip_seg = None         # segment of the first trip
    trip_bits = 0           # every protection that tripped during the test
    trip_at = ""

    def sample_done(info, reply):
        nonlocal last, wall_next, trip_seg, trip_bits, trip_at
        t, seg_no, seg_name, seg_t, current_ma, cell_min, cell_max, temperature_c = info
        fet = bool(reply["fet"] & hs.FET_MAIN)
        trip = reply["trip"]
        errors = reply["errors"]

        if last is None or trip != last[1]:
            if trip:
                hs.event(f"{t:9.2f} s  PROTECTION {hs.trip_text(trip)} at {current_ma:.0f} mA, cells {cell_min:.0f} - "
                         f"{cell_max:.0f} mV, {temperature_c:.1f} C  ({seg_t * 1000.0:.0f} ms into segment {seg_no})")
                if trip_seg is None:
                    trip_seg = seg_no
                    trip_at = f"segment {seg_no}, {seg_t * 1000.0:.0f} ms in"
                trip_bits |= trip
            elif last is not None:
                hs.event(f"{t:9.2f} s  protection cleared")
        if last is None or fet != last[0]:
            hs.event(f"{t:9.2f} s  FET {onoff(fet)}")
        if (last is None and errors) or (last is not None and errors != last[2]):
            hs.event(f"{t:9.2f} s  error flags 0x{errors:X}: {hs.errors_text(errors)}")
        last = (fet, trip, errors)

        log.writerow([name, round(t, 3), seg_no, seg_name, round(seg_t, 3), round(current_ma, 1), round(cell_min, 1),
                      round(cell_max, 1), temperature_c, int(fet), hs.trip_text(trip),
                      hs.CTRL_STATES.get(reply["ctrl_state"], reply["ctrl_state"]), hs.errors_text(errors)])

        if time.time() >= wall_next:
            wall_next = time.time() + hs.READ_EVERY_S
            hs.live(f"{t:9.2f} s  I {current_ma:7.0f} mA  cells {cell_min:4.0f}..{cell_max:4.0f} mV  "
                    f"T {temperature_c:5.1f} C  FET {onoff(fet):3s}  protection {hs.trip_text(trip)}")

    batch = hs.SampleBatch(ser)
    expect_seg = None
    for index, seg_cfg in enumerate(cfg["profile"]):
        seg_no = index + 1
        if seg_cfg.get("expect_trip"):
            expect_seg = seg_no
        seg = fs.Segment(seg_cfg, rng, temp_c, cfg["cell_min_mv"], cfg["cell_max_mv"], dt_s)
        hs.event(f"{sim_time:9.2f} s  segment {seg_no}/{len(cfg['profile'])} {seg.name()}  {seg_cfg.get('note', '')}")

        if seg.name() == "reset_protection":
            hs.send_command(ser, fs.CC_RESET_PROTECTION, "reset protection")
            continue

        while True:
            snap = (hs.snapshot(cells + [seg], rng), sim_time)
            current_a = seg.current_a(cells)
            # The battery follows the board's FET: an open FET carries no current
            if last is not None and not last[0]:
                current_a = 0.0
            end = seg.finished(cells, current_a)

            if not end:
                for c in cells:
                    c.step(current_a, dt_s)
                sim_time += dt_s
                seg.t += dt_s

                cell_mv = fs.cell_samples(cells, current_a, rng, noise_v)
                current_ma = current_a * 1000.0 + rng.gauss(0.0, noise_i)
                info = (sim_time, seg_no, seg.name(), seg.t, current_ma, min(cell_mv[:n]), max(cell_mv[:n]), seg.temperature_c)
                if not batch.add(snap, (cell_mv, current_ma, seg.temperature_c, dt_ms), info):
                    continue

            # The batch goes out when it is full and at the end of the segment
            done, back = batch.flush()
            for info, reply in done:
                sample_done(info, reply)
            if back is not None:
                # The board stopped at a change (e.g. the FET): make the samples after it again
                hs.restore(back[0])
                sim_time = back[1]
                continue
            if end:
                break

            if hs.quit_pressed():
                hs.event(f"{sim_time:9.2f} s  stopped by the user")
                raise UserStop()

    # Pass: the expected protection, and only that one, tripped first in the marked segment,
    # and the reset cleared it with the FET back on
    problems = []
    if trip_seg is None:
        problems.append("nothing tripped")
    else:
        if trip_seg != expect_seg:
            problems.append(f"first trip in segment {trip_seg}, expected in {expect_seg}")
        if trip_bits != expect_bit:
            problems.append(f"tripped {hs.trip_text(trip_bits)[len('TRIPPED '):]}, expected {cfg['expect']}")
    if last is None or last[1] != 0 or not last[0]:
        problems.append("not cleared with the FET on at the end")

    if problems:
        return False, "; ".join(problems)
    return True, f"{cfg['expect']} at {trip_at}"


def plot_log(log_path, only_test):
    """Plots log_protection.csv: one window per test (or only only_test)."""
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
        t = [float(r["sim_time_s"]) for r in rows]

        def col(key):
            return [float(r[key]) for r in rows]

        fig, ax = plt.subplots(4, 1, sharex=True, figsize=(12, 9))
        fig.canvas.manager.set_window_title(f"Protection test {name}")

        ax[0].plot(t, col("current_ma"))
        ax[0].set_ylabel("current [mA]")
        ax[1].plot(t, col("cell_min_mv"), label="lowest cell")
        ax[1].plot(t, col("cell_max_mv"), label="highest cell")
        ax[1].set_ylabel("cell [mV]")
        ax[1].legend(loc="best")
        ax[2].plot(t, col("temperature_c"), color="tab:red")
        ax[2].set_ylabel("temperature [C]")
        ax[3].step(t, col("fet"), where="post")
        ax[3].set_yticks([0, 1])
        ax[3].set_yticklabels(["off", "on"])
        ax[3].set_ylabel("FET")
        ax[3].set_xlabel("simulated time [s]")

        # Segment starts (dotted), trip start (red line) and the tripped time (red band)
        segment = None
        trip = "OK"
        trip_start = None
        trip_names = []
        for r, ti in zip(rows, t):
            if r["segment"] != segment:
                segment = r["segment"]
                for a in ax:
                    a.axvline(ti, color="gray", linestyle=":", linewidth=0.8)
                ax[0].annotate(f"{segment} {r['segment_type']}", (ti, 1.0), xycoords=("data", "axes fraction"),
                               fontsize=7, va="bottom", rotation=30)
            if r["protection_trip"] != trip:
                if trip == "OK":
                    trip_start = ti
                    trip_names.append(r["protection_trip"].replace("TRIPPED ", ""))
                    for a in ax:
                        a.axvline(ti, color="red", linewidth=1.0)
                elif r["protection_trip"] == "OK":
                    for a in ax:
                        a.axvspan(trip_start, ti, color="red", alpha=0.1)
                    trip_start = None
                trip = r["protection_trip"]
        if trip_start is not None:
            for a in ax:
                a.axvspan(trip_start, t[-1], color="red", alpha=0.1)

        title = name
        if trip_names:
            title += ": tripped " + ", ".join(trip_names)
        else:
            title += ": no trip"
        fig.suptitle(title)
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

        regs = fs.load_registers(os.path.join(folder, fs.REGISTERS_FILE))
        if port is None:
            port = hs.find_board()
        print(f"OpenBMS on {port}")
        print(f"Press {hs.QUIT_KEY} to stop.")
        log_path = os.path.join(folder, LOG_FILE)
        print(f"Log: {log_path}")
        with serial.Serial(port, hs.BAUD_RATE, timeout=hs.PORT_POLL) as ser, open(log_path, "w", newline="") as log_file:
            log = csv.writer(log_file)
            log.writerow(LOG_HEADER)
            time.sleep(hs.OPEN_SETTLE)
            for name in names:
                print(f"\n=== {name}: {tests[name]['description']} ===")
                # Start from a fresh board: protections, FET request and fuel gauge as after power-up
                hs.reboot(ser)
                ok, text = run(ser, regs, name, tests[name], log)
                results.append((name, ok, text))
                hs.event(f"{name}: {pass_text(ok)}, {text}")
    except UserStop:
        print("Stopped")
        return 1
    except (hs.SimError, serial.SerialException, OSError, KeyError, ValueError) as e:
        hs.event(f"Error: {e}")
        return 1

    print("\nSummary:")
    for name, ok, text in results:
        print(f"  {name:20s}  {pass_text(ok)}  {text}")

    if not all(ok for _, ok, _ in results):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
