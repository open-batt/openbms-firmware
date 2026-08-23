r"""
soc_estimator.py
=================
Runs the 2-RC + Extended Kalman Filter SOC estimator against one real current/
voltage log, using the tables and tuning produced by the earlier pipeline
stages, and previews the result live in plot windows -- nothing is written
to disk.

Pipeline:
    hppc_pipeline.py          raw HPPC log            -> per-direction SOC/OCV/R/tau tables
    kalman_soc_estimator.py   the two tables          -> per-cell Kalman parameters
    soc_estimator.py          parameters + a real log -> runs the estimator, previews it live  (this script)

The only thing you have to give this script is the log file. The three input
files below default to the standard results\ locations the earlier two
stages write to, and only need to be passed if yours live somewhere else:
    --charge-table      results\hppc_soc_table_all_cells_charge.csv
    --discharge-table   results\hppc_soc_table_all_cells_discharge.csv
    --parameters        results\kalman_parameters.csv

Run:
    python soc_estimator.py logs\c_full.csv
    python soc_estimator.py logs\d_partial_1.csv --charge-table other\charge.csv

For each cell, this script builds smooth OCV(SOC) and R0/R1/R2/tau1/tau2(SOC)
lookups from the charge/discharge tables, runs the model OPEN LOOP (pure
prediction: coulomb counting + RC decay, no correction from the measured
voltage) and with an Extended Kalman Filter (predict, then correct SOC/V_RC1/
V_RC2 against the measured voltage every step, using this cell's KF_Q_SOC/
KF_Q_RC1/KF_Q_RC2/KF_R_V and initial covariance from --parameters), and opens
a plot window per cell -- real vs open-loop vs Kalman voltage, the two SOC
traces, and the pack current that drove both (same current trace on every
cell's window, since all 7 cells share the one series pack current). Close
the windows when you're done looking; this script saves
nothing (no PNGs, no CSV, no log file) -- it's for eyeballing one run, not
for keeping records. (kalman_test.py, the earlier version of this script,
saved everything instead of previewing it -- use that one if you want files.)

Console output is colored by severity:
    INFO      plain text -- routine status (tables loaded, RMSE, etc.)
    WARNING   yellow     -- worth a look, but the run continues (e.g. the
                             Kalman filter didn't actually improve on open loop)
    ERROR     red        -- something that invalidated the result
The last line is a colored summary:
    === RESULT: SUCCESS (no warnings or errors) ===   in green
    === RESULT: SUCCESS (completed with warnings) ===  in yellow
    === RESULT: FAILED  (...) ===                       in red -- re-raises the
                                                         exception, so the exit
                                                         code stays non-zero.

Innovation gating: every step, the filter checks whether the gap between the
measured and predicted voltage is plausible, two ways -- (1) is it bigger than
GATE_SIGMA (default 4) standard deviations of the filter's own uncertainty --
if so, that measurement is ignored entirely for this step (no mean update, no
covariance shrink); or (2) does correcting on it imply a SOC change faster
than MAX_SOC_RATE_PCT_PER_S (default 1.0 percentage points per second), which
coulomb counting alone could never produce -- if so, the mean correction is
scaled down to that rate cap, but the covariance still updates normally
(rate-limiting the point estimate without freezing the filter's confidence,
which would otherwise snowball into gating almost everything). Check (2)
matters because right after a long quiet rest the filter's own uncertainty
can still be loose, so a big first real residual can pass check (1) even
though it's physically absurd. When gating triggers, you'll see a WARNING
naming the cell and the worst gated sample; it also shows up as n_gated /
max_gated_mV in the results table. A cell that gates constantly is a sign its
R0 (or that voltage channel) is worth a closer look, not just a filter-tuning
issue.

IMPORTANT SIMPLIFYING ASSUMPTIONS (read before trusting the numbers):
  - SOC at t=0 is seeded by inverting the FIRST measured voltage through the
    OCV table directly (ignoring the small IR-drop already present if current
    is already flowing at t=0). This is exactly the kind of initial error the
    Kalman filter is supposed to correct out over the first several seconds --
    if you watch the Kalman SOC trace do something odd right at the very
    start, that's this seed settling in, not a bug.
  - V_RC1 = V_RC2 = 0 at t=0 (assumes a relaxed start). Same caveat as above.
  - Near-zero current (|I| < CURRENT_DEADZONE_A) holds the previously-selected
    charge/discharge parameter branch rather than flip-flopping on noise.

NOTE: plotting opens real GUI windows, so this needs a matplotlib GUI backend
(e.g. TkAgg) available on your machine -- normally already the case on a
standard Windows Python install. If no window appears, try
"python -m tkinter" to check Tk is installed, or set MPLBACKEND=TkAgg.
"""

import os
import sys
import argparse
import traceback
import numpy as np
import pandas as pd
from scipy.interpolate import PchipInterpolator

import matplotlib.pyplot as plt   # deliberately NOT matplotlib.use("Agg") -- we want a real window

# =============================================================================
# LEVELED CONSOLE LOGGING (INFO / WARNING / ERROR) -- console only; this
# script writes nothing to disk.
# =============================================================================
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

_RESET  = "\033[0m"
_GREEN  = "\033[1;32m"    # all good
_YELLOW = "\033[93m"      # warning
_RED    = "\033[1;31m"    # error / failed

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
    print(f"{_YELLOW}WARNING: {msg}{_RESET}")


def log_error(msg):
    _counts["ERROR"] += 1
    print(f"{_RED}ERROR: {msg}{_RESET}")


def _print_result():
    n_info, n_warn, n_err = _counts["INFO"], _counts["WARNING"], _counts["ERROR"]
    print()
    if n_err:
        print(f"{_RED}=== RESULT: FAILED  ({n_err} error(s), {n_warn} warning(s), {n_info} info) ==={_RESET}")
    elif n_warn:
        print(f"{_YELLOW}=== RESULT: SUCCESS  (completed with {n_warn} warning(s), {n_info} info) ==={_RESET}")
    else:
        print(f"{_GREEN}=== RESULT: SUCCESS  (no warnings or errors, {n_info} info) ==={_RESET}")


# =============================================================================
# CONFIG
# =============================================================================
NUM_CELLS = 7
VOLTAGE_SOURCE = "filtered"          # "filtered" -> cell_vf{i}, "raw" -> cell_v{i}

CURRENT_DEADZONE_A = 0.05            # |I| below this holds the last charge/discharge branch
SOC_MIN, SOC_MAX = 0.0, 1.0          # hard clip, matches the 0-100% table range

# Innovation gating (a.k.a. an outlier/NIS test): if a single sample's residual
# (measured vs. predicted voltage) is bigger than GATE_SIGMA standard deviations
# of what the filter's own uncertainty says is plausible, treat it as a glitch
# rather than real information -- fall back to the model prediction for that one
# step instead of letting the correction yank the state. This guards against
# transient measurement/filter-lag artifacts right at fast current steps (seen
# in practice: one cell's filtered voltage briefly disagreeing with its table R0
# at the exact instant of a HPPC-style pulse edge, momentarily flipping the sign
# of the SOC correction). 4-sigma is generous enough to leave normal noise alone.
GATE_SIGMA = 4.0

# Second, complementary safeguard: cap how FAST a measurement update is allowed
# to move the SOC estimate, in percentage points per second (not per sample --
# logs can be sampled at very different rates, so an absolute per-sample cap
# tuned for one log's cadence would be way too tight or too loose for another).
# This matters because right after a long quiet rest the filter's own
# covariance can still be comparatively loose, so a big first real residual can
# look "statistically plausible" to the sigma test even though it's physically
# absurd (coulomb counting alone moves SOC nowhere near this fast). 1.0 pct/s
# is generous enough to let a genuinely wrong seed catch up within a few
# seconds, while still blocking a >1 pct jump inside one ~50ms sample.
MAX_SOC_RATE_PCT_PER_S = 1.0

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_RESULTS_DIR = os.path.join(_SCRIPT_DIR, "results")
DEFAULT_CHARGE_TABLE = os.path.join(_RESULTS_DIR, "hppc_soc_table_all_cells_charge.csv")
DEFAULT_DISCHARGE_TABLE = os.path.join(_RESULTS_DIR, "hppc_soc_table_all_cells_discharge.csv")
DEFAULT_PARAMETERS = os.path.join(_RESULTS_DIR, "kalman_parameters.csv")


# =============================================================================
# 1. PARAMETER TABLES -> PER-CELL, PER-BRANCH LOOKUPS
# =============================================================================
def build_lookup(table_csv):
    """Return {cell_idx: {'OCV': callable(soc_frac)->V, 'dOCV': callable(soc_frac)->V/frac,
    'R0':.., 'R1':.., 'R2':.., 'tau1':.., 'tau2':.., 'ocv_to_soc': callable(V)->soc_frac}}
    built from one hppc_soc_table_all_cells_*.csv file."""
    if not os.path.isfile(table_csv):
        raise FileNotFoundError(f"table not found: {table_csv}")

    df = pd.read_csv(table_csv)
    soc = df["SOC_pct"].values / 100.0
    out = {}
    for i in range(1, NUM_CELLS + 1):
        p = f"Cell{i}_"
        ocv = df[p + "OCV_V"].values
        pchip = PchipInterpolator(soc, ocv)
        dpchip = pchip.derivative()

        def make_lin(col):
            y = df[p + col].values
            return lambda s, _y=y: np.interp(np.clip(s, SOC_MIN, SOC_MAX), soc, _y)

        out[i] = dict(
            OCV=lambda s, _f=pchip: float(_f(np.clip(s, SOC_MIN, SOC_MAX))),
            dOCV=lambda s, _f=dpchip: float(_f(np.clip(s, SOC_MIN, SOC_MAX))),
            R0=make_lin("R0_ohm"), R1=make_lin("R1_ohm"), R2=make_lin("R2_ohm"),
            tau1=make_lin("tau1_s"), tau2=make_lin("tau2_s"),
            ocv_to_soc=lambda v, _v=ocv, _s=soc: float(np.interp(np.clip(v, _v.min(), _v.max()), _v, _s)),
        )
    return out


# =============================================================================
# 2. KALMAN PARAMETERS (from kalman_soc_estimator.py's output)
# =============================================================================
def load_parameters(params_csv):
    """Return {cell_idx: {'capacity_ah', 'kf_q_soc', 'kf_q_rc1', 'kf_q_rc2', 'kf_r_v',
    'p0_soc', 'p0_vrc1', 'p0_vrc2'}} read from kalman_parameters.csv."""
    if not os.path.isfile(params_csv):
        raise FileNotFoundError(f"parameters file not found: {params_csv}")

    df = pd.read_csv(params_csv)
    required = ["cell", "capacity_ah", "kf_q_soc", "kf_q_rc1", "kf_q_rc2",
                "kf_r_v", "p0_soc", "p0_vrc1", "p0_vrc2"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"{params_csv} is missing expected column(s): {missing}")

    out = {}
    for _, row in df.iterrows():
        out[int(row["cell"])] = {c: float(row[c]) for c in required if c != "cell"}
    return out


# =============================================================================
# 3. WORKING-DATA LOADING + CURRENT SIGN AUTO-DETECTION
# =============================================================================
def load_working_data(path):
    if not os.path.isfile(path):
        raise FileNotFoundError(f"data log not found: {path}")

    df = pd.read_csv(path, low_memory=False)
    required = (["elapsed_s", "current_ma", "current_filtered_ma"]
                + [f"cell_v{i}" for i in range(1, 8)]
                + [f"cell_vf{i}" for i in range(1, 8)])
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"{path} is missing expected columns: {missing}")

    numeric_cols = df.select_dtypes(include=[np.number]).columns
    for col in numeric_cols:
        if df[col].isna().any():
            if col == "elapsed_s":
                df[col] = df[col].interpolate("linear").ffill().bfill()
            else:
                df[col] = df[col].ffill().bfill()

    # --- auto-detect current sign convention ---
    # Our convention (matching the HPPC tables): current > 0 while charging (voltage rising).
    # Compare net voltage trend (avg of first 10% of samples vs last 10%) against mean current
    # sign; if they disagree with "positive current -> rising voltage", flip the sign.
    cell_cols = [f"cell_v{i}" for i in range(1, 8)]
    pack_v = df[cell_cols].mean(axis=1)
    n = len(df)
    edge = max(1, n // 10)
    net_dv = pack_v.iloc[-edge:].mean() - pack_v.iloc[:edge].mean()
    mean_i = df["current_filtered_ma"].mean()

    flipped = False
    if (net_dv > 0 and mean_i < 0) or (net_dv < 0 and mean_i > 0):
        df["current_ma"] = -df["current_ma"]
        df["current_filtered_ma"] = -df["current_filtered_ma"]
        flipped = True

    direction = "charge-like (voltage rising)" if net_dv > 0 else "discharge-like (voltage falling)"
    log_info(f"[{os.path.basename(path)}] net voltage trend: {direction}, "
             f"mean current (as loaded): {mean_i:.1f} mA"
             + (" -- SIGN FLIPPED to match the charge=positive convention" if flipped else ""))

    return df


# =============================================================================
# 4. 2-RC MODEL + EKF
# =============================================================================
def select_branch(current_a, prev_branch):
    if current_a > CURRENT_DEADZONE_A:
        return "charge"
    if current_a < -CURRENT_DEADZONE_A:
        return "discharge"
    return prev_branch if prev_branch is not None else "charge"


def run_cell(cell_idx, t, i_pack_a, v_meas, lut_charge, lut_discharge,
             capacity_ah, q_proc_diag, r_meas, p0_diag):
    """Returns a dict of per-timestep arrays (v_ol, v_kf, soc_ol, soc_kf) plus the
    final EKF state (final_soc, final_vrc1, final_vrc2, final_P), and innovation-
    gating stats (n_gated, max_gated_mV, t_max_gated). capacity_ah, q_proc_diag
    ([q_soc, q_rc1, q_rc2]), r_meas and p0_diag ([p_soc, p_vrc1, p_vrc2]) come
    from this cell's row in kalman_parameters.csv."""
    lut = {"charge": lut_charge[cell_idx], "discharge": lut_discharge[cell_idx]}
    Q_PROC = np.diag(q_proc_diag)
    P0 = np.diag(p0_diag)

    n = len(t)
    v_ol = np.empty(n); v_kf = np.empty(n)
    soc_ol = np.empty(n); soc_kf = np.empty(n)

    # ---- seed initial state from the first measurement ----
    branch0 = select_branch(i_pack_a[0], None)
    soc0 = lut[branch0]["ocv_to_soc"](v_meas[0])
    x_ol = np.array([soc0, 0.0, 0.0])
    x_kf = np.array([soc0, 0.0, 0.0])
    P = P0.copy()

    branch_ol, branch_kf = branch0, branch0

    n_gated = 0
    max_gated_abs_y = 0.0
    t_max_gated = None

    for k in range(n):
        if k == 0:
            v_ol[k] = v_meas[0]
            v_kf[k] = v_meas[0]
            soc_ol[k] = soc0
            soc_kf[k] = soc0
            continue

        dt = t[k] - t[k - 1]
        i_prev = i_pack_a[k - 1]
        i_now = i_pack_a[k]

        # ---------------- open loop ----------------
        branch_ol = select_branch(i_prev, branch_ol)
        p = lut[branch_ol]
        tau1, tau2 = p["tau1"](x_ol[0]), p["tau2"](x_ol[0])
        r1, r2 = p["R1"](x_ol[0]), p["R2"](x_ol[0])
        a1, a2 = np.exp(-dt / tau1), np.exp(-dt / tau2)
        x_ol = np.array([
            np.clip(x_ol[0] + dt / (3600.0 * capacity_ah) * i_prev, SOC_MIN, SOC_MAX),
            a1 * x_ol[1] + r1 * (1 - a1) * i_prev,
            a2 * x_ol[2] + r2 * (1 - a2) * i_prev,
        ])
        p_now = lut[select_branch(i_now, branch_ol)]
        v_ol[k] = p_now["OCV"](x_ol[0]) + i_now * p_now["R0"](x_ol[0]) + x_ol[1] + x_ol[2]
        soc_ol[k] = x_ol[0]

        # ---------------- EKF: predict ----------------
        branch_kf = select_branch(i_prev, branch_kf)
        p = lut[branch_kf]
        tau1, tau2 = p["tau1"](x_kf[0]), p["tau2"](x_kf[0])
        r1, r2 = p["R1"](x_kf[0]), p["R2"](x_kf[0])
        a1, a2 = np.exp(-dt / tau1), np.exp(-dt / tau2)
        A = np.diag([1.0, a1, a2])
        B = np.array([dt / (3600.0 * capacity_ah), r1 * (1 - a1), r2 * (1 - a2)])

        x_pred = A @ x_kf + B * i_prev
        x_pred[0] = np.clip(x_pred[0], SOC_MIN, SOC_MAX)
        P_pred = A @ P @ A.T + Q_PROC

        # ---------------- EKF: measurement + update ----------------
        branch_meas = select_branch(i_now, branch_kf)
        pm = lut[branch_meas]
        r0 = pm["R0"](x_pred[0])
        v_pred = pm["OCV"](x_pred[0]) + i_now * r0 + x_pred[1] + x_pred[2]
        H = np.array([pm["dOCV"](x_pred[0]), 1.0, 1.0])

        S = H @ P_pred @ H.T + r_meas
        K = (P_pred @ H) / S
        y = v_meas[k] - v_pred

        # ---- innovation gating: is this residual plausible? Two tiers:
        # 1) a genuine statistical outlier (NIS test) -- ignore the measurement
        #    entirely this step (no mean update, no covariance shrink either;
        #    P still evolves normally via Q_PROC next step).
        # 2) not a statistical outlier, but the implied SOC jump is absurdly
        #    big for one sample -- scale the MEAN correction down to the cap,
        #    but still apply the full covariance update. This is deliberately
        #    NOT a second full skip: if it were, covariance would never shrink
        #    on a run of these, K would keep growing next step, and the cap
        #    would then trip on almost every sample after -- a runaway
        #    feedback loop. Letting the covariance narrow normally (since this
        #    measurement genuinely was informative) while only throttling how
        #    fast the point estimate is allowed to move avoids that.
        nis = (y * y) / S
        implied_dsoc = K[0] * y
        max_dsoc_this_step = MAX_SOC_RATE_PCT_PER_S * dt / 100.0

        if nis > GATE_SIGMA ** 2:
            n_gated += 1
            if abs(y) > max_gated_abs_y:
                max_gated_abs_y = abs(y)
                t_max_gated = t[k]
            x_kf = x_pred
            P = P_pred
        else:
            scale = 1.0
            if abs(implied_dsoc) > max_dsoc_this_step:
                scale = max_dsoc_this_step / abs(implied_dsoc)
                n_gated += 1
                if abs(y) > max_gated_abs_y:
                    max_gated_abs_y = abs(y)
                    t_max_gated = t[k]
            x_kf = x_pred + scale * K * y
            P = (np.eye(3) - np.outer(K, H)) @ P_pred

        x_kf[0] = np.clip(x_kf[0], SOC_MIN, SOC_MAX)

        v_kf[k] = v_pred          # a-priori prediction -- this is what the filter is judged on
        soc_kf[k] = x_kf[0]       # a-posteriori SOC -- the corrected estimate going forward
        branch_kf = branch_meas

    return dict(v_ol=v_ol, v_kf=v_kf, soc_ol=soc_ol, soc_kf=soc_kf,
                final_soc=float(x_kf[0]), final_vrc1=float(x_kf[1]), final_vrc2=float(x_kf[2]),
                final_P=P.copy(), n_gated=n_gated,
                max_gated_mV=max_gated_abs_y * 1000.0, t_max_gated=t_max_gated)


# =============================================================================
# 5. PLOTTING (live preview -- opens a window, does not save anything)
# =============================================================================
def make_plot(cell_idx, t, v_meas, i_pack_a, result, data_label):
    fig, axes = plt.subplots(3, 1, figsize=(11, 9.5), sharex=True)

    ax = axes[0]
    ax.plot(t, v_meas, "-", color="black", lw=1.2, label="Real measured voltage")
    ax.plot(t, result["v_ol"], "-", color="tab:red", lw=1.0, alpha=0.8, label="Estimated (open loop, no Kalman)")
    ax.plot(t, result["v_kf"], "-", color="tab:blue", lw=1.0, alpha=0.9, label="Estimated (with Kalman)")
    ax.set_ylabel("Voltage [V]")
    ax.set_title(f"Cell {cell_idx}: real vs estimated voltage -- {data_label}")
    ax.legend(loc="best"); ax.grid(True)

    ax = axes[1]
    ax.plot(t, result["soc_ol"] * 100, "-", color="tab:red", lw=1.0, label="SOC (open loop)")
    ax.plot(t, result["soc_kf"] * 100, "-", color="tab:blue", lw=1.0, label="SOC (Kalman)")
    ax.set_ylabel("SOC [%]")
    ax.legend(loc="best"); ax.grid(True)

    ax = axes[2]
    ax.plot(t, i_pack_a, "-", color="tab:green", lw=1.0, label="Pack current (filtered)")
    ax.axhline(0, color="gray", lw=0.8, ls="--")
    ax.set_xlabel("Time [s]"); ax.set_ylabel("Current [A]")
    ax.legend(loc="best"); ax.grid(True)

    fig.tight_layout()
    plt.show(block=False)   # open the window now; the final plt.show() in main() keeps it alive
    return fig


# =============================================================================
# MAIN
# =============================================================================
def main(log_path, charge_table=None, discharge_table=None, parameters_csv=None):
    charge_table = charge_table or DEFAULT_CHARGE_TABLE
    discharge_table = discharge_table or DEFAULT_DISCHARGE_TABLE
    parameters_csv = parameters_csv or DEFAULT_PARAMETERS

    reset_counts()

    try:
        lut_charge = build_lookup(charge_table)
        log_info(f"Loaded charge table: {charge_table}")
        lut_discharge = build_lookup(discharge_table)
        log_info(f"Loaded discharge table: {discharge_table}")
        params = load_parameters(parameters_csv)
        log_info(f"Loaded Kalman parameters: {parameters_csv}")

        missing_cells = [i for i in range(1, NUM_CELLS + 1) if i not in params]
        if missing_cells:
            raise ValueError(f"{parameters_csv} is missing rows for cell(s): {missing_cells}")

        df = load_working_data(log_path)
        stem = os.path.splitext(os.path.basename(log_path))[0]
        t = df["elapsed_s"].values
        i_pack_a = df["current_filtered_ma"].values / 1000.0

        rows = []
        for cell_idx in range(1, NUM_CELLS + 1):
            vcol = f"cell_vf{cell_idx}" if VOLTAGE_SOURCE == "filtered" else f"cell_v{cell_idx}"
            v_meas = df[vcol].values / 1000.0
            p = params[cell_idx]

            result = run_cell(cell_idx, t, i_pack_a, v_meas, lut_charge, lut_discharge,
                               p["capacity_ah"],
                               [p["kf_q_soc"], p["kf_q_rc1"], p["kf_q_rc2"]], p["kf_r_v"],
                               [p["p0_soc"], p["p0_vrc1"], p["p0_vrc2"]])

            rmse_ol = float(np.sqrt(np.mean((v_meas - result["v_ol"]) ** 2)))
            rmse_kf = float(np.sqrt(np.mean((v_meas - result["v_kf"]) ** 2)))

            if rmse_kf >= rmse_ol:
                log_warning(f"cell {cell_idx}: Kalman RMSE ({rmse_kf*1000:.1f} mV) did not improve "
                            f"on open-loop RMSE ({rmse_ol*1000:.1f} mV) -- check KF tuning "
                            f"in kalman_parameters.csv")

            if result["n_gated"]:
                log_warning(f"cell {cell_idx}: {result['n_gated']} sample(s) had an implausible "
                            f"voltage residual and were gated (either rejected outright as a "
                            f"statistical outlier, or had the SOC correction rate-limited to "
                            f"{MAX_SOC_RATE_PCT_PER_S:.1f} pct points/s) instead of being applied as-is; "
                            f"worst case {result['max_gated_mV']:.1f} mV at t={result['t_max_gated']:.2f}s. "
                            f"This usually means a fast current step and this cell's voltage momentarily "
                            f"disagreed with its table R0 -- worth a look if it happens a lot")

            fp = result["final_P"]
            rows.append(dict(
                cell=cell_idx,
                capacity_ah=p["capacity_ah"],
                kf_q_soc=p["kf_q_soc"], kf_q_rc1=p["kf_q_rc1"], kf_q_rc2=p["kf_q_rc2"], kf_r_v=p["kf_r_v"],
                cell_soc_f=result["final_soc"], cell_vrc1_v=result["final_vrc1"], cell_vrc2_v=result["final_vrc2"],
                cell_p00=fp[0, 0], cell_p01=fp[0, 1], cell_p02=fp[0, 2],
                cell_p11=fp[1, 1], cell_p12=fp[1, 2], cell_p22=fp[2, 2],
                rmse_openloop_mV=rmse_ol * 1000, rmse_kalman_mV=rmse_kf * 1000,
                n_gated=result["n_gated"], max_gated_mV=result["max_gated_mV"],
            ))

            make_plot(cell_idx, t, v_meas, i_pack_a, result, stem)
            log_info(f"cell {cell_idx}: RMSE open-loop {rmse_ol*1000:6.1f} mV, "
                     f"RMSE Kalman {rmse_kf*1000:6.1f} mV")

        result_df = pd.DataFrame(rows)
        print(f"\n{stem} results (preview only -- nothing saved):")
        # no blanket float_format here -- kf_q_soc/kf_q_rc1/kf_q_rc2/kf_r_v are typically ~1e-6 to
        # 1e-5 and would print as a misleading 0.0000 under a fixed-decimal format; let each column
        # use its natural representation instead.
        print(result_df.to_string(index=False))

    except Exception:
        log_error(f"FATAL -- run did not complete:\n{traceback.format_exc()}")
        _print_result()
        raise

    _print_result()

    if plt.get_fignums():
        print("\nClose the plot window(s) to exit.")
        plt.show()   # blocks until every open figure is closed


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run the 2-RC + EKF SOC estimator against one real log and preview it live "
                     "(nothing is saved to disk)")
    parser.add_argument("log", help="the current/voltage log to preview")
    parser.add_argument("--charge-table", default=None,
                         help=f"hppc_soc_table_all_cells_charge.csv (default: {DEFAULT_CHARGE_TABLE})")
    parser.add_argument("--discharge-table", default=None,
                         help=f"hppc_soc_table_all_cells_discharge.csv (default: {DEFAULT_DISCHARGE_TABLE})")
    parser.add_argument("--parameters", default=None,
                         help=f"kalman_parameters.csv (default: {DEFAULT_PARAMETERS})")
    args = parser.parse_args()

    main(args.log, charge_table=args.charge_table, discharge_table=args.discharge_table,
         parameters_csv=args.parameters)
