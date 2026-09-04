r"""
kalman_soc_estimator.py
=======================
Turns the charge and discharge parameter tables from hppc_pipeline.py into the
per-cell configuration a Kalman-filter SOC estimator needs to run. No real
current/voltage log is required for this step -- that comes later.

For each cell, this script:
  1. Loads the charge and discharge hppc_soc_table_all_cells_*.csv tables and
     reads off each cell's HPPC-derived capacity (mean of the charge- and
     discharge-side Capacity_2v5_4v2_mAh columns -- the same coulomb-counting
     assumption a downstream estimator would use).
  2. Pairs that per-cell capacity with the EKF tuning constants below
     (Q_PROC / R_MEAS / P0) into one row per cell.
  3. Saves the result to a CSV, named to match the corresponding registers in
     communication-protocol.md (0xA1-0xA4 for the tuning constants) for later
     reference. Nothing is sent to the device.

This is the middle step of a 3-stage pipeline:
    hppc_pipeline.py          raw HPPC log            -> per-direction SOC/OCV/R/tau tables
    kalman_soc_estimator.py   the two tables          -> per-cell Kalman parameters   (this script)
    (a future script)         parameters + a real log -> runs open-loop + Kalman
                                                          estimation and plots the result

Run:
    python kalman_soc_estimator.py --charge-table results\hppc_soc_table_all_cells_charge.csv ^
                                    --discharge-table results\hppc_soc_table_all_cells_discharge.csv

Output (default: a "results" folder next to this script; created if missing,
existing files from other runs are kept):
    kalman_parameters.csv       one row per cell -- capacity_ah, kf_q_soc, kf_q_rc1,
                                 kf_q_rc2, kf_r_v, p0_soc, p0_vrc1, p0_vrc2
    kalman_parameters_log.txt   a plain-text copy of everything printed during the
                                 run (colors stripped), so the INFO/WARNING/ERROR
                                 trail and the final RESULT line are kept alongside
                                 the CSV, not just shown in the console

Every sanity-check finding prints directly to the console (and into the log file
above), tagged and colored by severity on-screen, same style as hppc_pipeline.py:
    INFO    - plain text   - routine status (tables loaded, capacity computed, etc.)
    WARNING - orange       - worth a look, but the run continues
    ERROR   - red          - something that likely invalidates the result
The last line of every run is a colored summary:
    === RESULT: SUCCESS (...) ===   in green, or
    === RESULT: FAILED  (...) ===   in red, if any ERROR was logged (a crash also
                                     counts as FAILED and re-raises the exception
                                     afterward, so exit codes stay non-zero).

NOTE: Q_PROC / R_MEAS / P0 below are reasonable starting points, not fitted --
there is no independent ground-truth SOC in this data to fit them against yet.
Tune them by hand if a downstream Kalman trace over- or under-reacts to noise
once the estimator step (the future script above) is built.
"""

import os
import re
import sys
import argparse
import traceback
import numpy as np
import pandas as pd

# =============================================================================
# LEVELED CONSOLE LOGGING (INFO / WARNING / ERROR) -- color-coded on screen in
# real time as it's discovered, and mirrored (colors stripped) into a plain
# text log file in the output folder -- see _Tee / main() below.
# =============================================================================
_ANSI_RE = re.compile(r"\033\[[0-9;]*m")


class _Tee:
    """Writes everything to the real console exactly as-is (so ANSI colors still
    show up live), and a color-stripped copy to a log file at the same time."""
    def __init__(self, real_stream, log_file):
        self.real_stream = real_stream
        self.log_file = log_file

    def write(self, data):
        self.real_stream.write(data)
        self.log_file.write(_ANSI_RE.sub("", data))

    def flush(self):
        self.real_stream.flush()
        self.log_file.flush()


def _enable_windows_ansi():
    """Modern Windows terminals (Windows Terminal, PowerShell, VS Code) support
    ANSI color codes once virtual terminal processing is turned on. Older
    cmd.exe windows may just show the raw escape codes -- harmless, just ugly."""
    if os.name == "nt":
        try:
            import ctypes
            kernel32 = ctypes.windll.kernel32
            handle = kernel32.GetStdHandle(-11)
            mode = ctypes.c_uint32()
            kernel32.GetConsoleMode(handle, ctypes.byref(mode))
            kernel32.SetConsoleMode(handle, mode.value | 0x0004)
        except Exception:
            pass


_enable_windows_ansi()

_RESET   = "\033[0m"
_ORANGE  = "\033[38;5;208m"   # WARNING
_RED     = "\033[1;31m"       # ERROR
_GREEN   = "\033[1;32m"       # final SUCCESS line
_BOLDRED = "\033[1;31m"       # final FAILED line

_counts = {"INFO": 0, "WARNING": 0, "ERROR": 0}


def reset_counts():
    _counts["INFO"] = 0
    _counts["WARNING"] = 0
    _counts["ERROR"] = 0


def log_info(msg):
    _counts["INFO"] += 1
    print(f"INFO: {msg}")


def log_warning(msg):
    _counts["WARNING"] += 1
    print(f"{_ORANGE}WARNING: {msg}{_RESET}")


def log_error(msg):
    _counts["ERROR"] += 1
    print(f"{_RED}ERROR: {msg}{_RESET}")


def _print_result():
    n_info, n_warn, n_err = _counts["INFO"], _counts["WARNING"], _counts["ERROR"]
    print()
    if n_err:
        print(f"{_BOLDRED}=== RESULT: FAILED  ({n_err} error(s), {n_warn} warning(s), {n_info} info) ==={_RESET}")
    elif n_warn:
        print(f"{_GREEN}=== RESULT: SUCCESS  (completed with {n_warn} warning(s), {n_info} info) ==={_RESET}")
    else:
        print(f"{_GREEN}=== RESULT: SUCCESS  (no warnings or errors, {n_info} info) ==={_RESET}")


# =============================================================================
# CONFIG
# =============================================================================
NUM_CELLS = 7

# EKF tuning -- process noise (state units: SOC fraction, volts, volts)
Q_PROC = np.diag([1e-3, 1e-3, 1e-3]) ** 2     # per-step process noise covariance
R_MEAS = (0.005) ** 2                          # measurement noise variance, (volts)^2
P0 = np.diag([0.05, 0.02, 0.02]) ** 2          # initial state covariance (SOC fraction, V, V)

MAX_ACCEPTABLE_CAPACITY_MISMATCH_PCT = 5.0   # flag if a cell's charge- vs discharge-derived
                                              # capacity disagree by more than this

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")


# =============================================================================
# PARAMETER TABLES -> PER-CELL CAPACITY
# =============================================================================
def load_capacity_mAh(table_csv):
    """Return {cell_idx: capacity_mAh} read from one hppc_soc_table_all_cells_*.csv file."""
    if not os.path.isfile(table_csv):
        raise FileNotFoundError(f"table not found: {table_csv}")

    df = pd.read_csv(table_csv)
    out = {}
    for i in range(1, NUM_CELLS + 1):
        col = f"Cell{i}_Capacity_2v5_4v2_mAh"
        if col not in df.columns:
            raise ValueError(f"{table_csv} is missing expected column: {col}")
        out[i] = float(df[col].iloc[0])
    return out


# =============================================================================
# MAIN
# =============================================================================
def main(charge_table, discharge_table, out_dir=None):
    reset_counts()
    out_dir = out_dir or OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)   # keep existing files from other runs

    log_path = os.path.join(out_dir, "kalman_parameters_log.txt")
    real_stdout = sys.stdout
    log_file = open(log_path, "w", encoding="utf-8")
    sys.stdout = _Tee(real_stdout, log_file)

    try:
        _main_body(charge_table, discharge_table, out_dir, log_path)
    finally:
        sys.stdout = real_stdout
        log_file.close()


def _main_body(charge_table, discharge_table, out_dir, log_path):
    try:
        cap_charge = load_capacity_mAh(charge_table)
        log_info(f"Loaded charge table: {charge_table}")
        cap_discharge = load_capacity_mAh(discharge_table)
        log_info(f"Loaded discharge table: {discharge_table}")

        rows = []
        for cell_idx in range(1, NUM_CELLS + 1):
            c_chg, c_dis = cap_charge[cell_idx], cap_discharge[cell_idx]

            if c_chg <= 0 or c_dis <= 0:
                log_error(f"Cell {cell_idx}: non-positive capacity (charge={c_chg:.1f} mAh, "
                          f"discharge={c_dis:.1f} mAh) -- check the source HPPC tables")

            mismatch_pct = abs(c_chg - c_dis) / ((c_chg + c_dis) / 2.0) * 100.0 if (c_chg + c_dis) else 0.0
            if mismatch_pct > MAX_ACCEPTABLE_CAPACITY_MISMATCH_PCT:
                log_warning(f"Cell {cell_idx}: charge/discharge capacity differ by {mismatch_pct:.1f}% "
                            f"(charge={c_chg:.1f} mAh, discharge={c_dis:.1f} mAh) -- "
                            f"exceeds the {MAX_ACCEPTABLE_CAPACITY_MISMATCH_PCT:.0f}% sanity threshold")

            # same averaging assumption a downstream estimator would make for coulomb counting
            capacity_ah = (c_chg + c_dis) / 2000.0
            rows.append(dict(
                cell=cell_idx,
                capacity_ah=capacity_ah,
                # matches KF_Q_SOC/KF_Q_RC1/KF_Q_RC2/KF_R_V at registers 0xA1-0xA4 (same
                # value every row -- these are global tuning constants, not per-cell)
                kf_q_soc=float(Q_PROC[0, 0]), kf_q_rc1=float(Q_PROC[1, 1]), kf_q_rc2=float(Q_PROC[2, 2]),
                kf_r_v=float(R_MEAS),
                # initial covariance seed a downstream estimator would start from
                p0_soc=float(P0[0, 0]), p0_vrc1=float(P0[1, 1]), p0_vrc2=float(P0[2, 2]),
            ))

        log_info(f"Calculated Kalman parameters for {NUM_CELLS} cells")

        result_df = pd.DataFrame(rows)
        print("\nCalculated Kalman parameters:")
        # no blanket float_format here -- kf_q_soc/kf_q_rc1/kf_q_rc2/kf_r_v are typically ~1e-6 to
        # 1e-5 and would print as a misleading 0.0000 under a fixed-decimal format; let each column
        # use its natural representation instead (the CSV always keeps full precision regardless).
        print(result_df.to_string(index=False))

        out_path = os.path.join(out_dir, "kalman_parameters.csv")
        result_df.to_csv(out_path, index=False)
        print(f"\nCalculated Kalman parameters written to: {out_path}")

    except Exception:
        log_error(f"FATAL -- run did not complete:\n{traceback.format_exc()}")
        _print_result()
        print(f"\nRun log written to: {log_path}")
        raise

    _print_result()
    print(f"\nRun log written to: {log_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Calculate per-cell Kalman filter parameters from HPPC charge/discharge "
                     "tables -- no current/voltage log needed")
    parser.add_argument("--charge-table", required=True, help="hppc_soc_table_all_cells_charge.csv")
    parser.add_argument("--discharge-table", required=True, help="hppc_soc_table_all_cells_discharge.csv")
    parser.add_argument("--outdir", default=None,
                         help="output folder (default: a 'results' folder next to this script; "
                              "created if missing, existing files from other runs are kept)")
    args = parser.parse_args()

    main(args.charge_table, args.discharge_table, out_dir=args.outdir)
