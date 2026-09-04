r"""
hppc_pipeline.py
================
Python replacement for load_data.m + soc_estimator.m, generalized from 1 cell
to all 7 series-connected cells, and from charge-only to charge OR discharge.

For each cell this script:
  1. Loads and validates the raw HPPC log (NaN repair + sanity checks).
  2. Detects REST/RAMP cycles from segment_type / commanded_fet_state.
  3. Fits a 2-RC relaxation model to every rest to get R0, R1, R2, tau1, tau2.
  4. Builds the measured OCV-vs-capacity curve and extends it from 2.5-4.2 V
     (PCHIP inside the data, slope-matched power-law tails outside it -- but
     only trusted for small gaps; see LARGE_GAP_WARNING_V).
  5. Resamples OCV and the 5 impedance parameters onto a fixed SOC grid.

Run (Windows, macOS, Linux - just needs Python 3 + numpy/pandas/scipy/matplotlib):
    python hppc_pipeline.py C c_hppc_full.csv           # analyze as a CHARGE test
    python hppc_pipeline.py D d_hppc_1.csv              # analyze as a DISCHARGE test
    python hppc_pipeline.py C file1.csv file2.csv ...   # auto time-concatenated
    python hppc_pipeline.py D d_hppc_1.csv --outdir C:\some\other\folder

Outputs (default: a "results" folder next to this script - see OUTPUT_DIR below;
created automatically if missing, and reused as-is if it already exists),
all suffixed _charge or _discharge so both directions can coexist in one folder --
running one direction does not remove the other direction's existing results:
    hppc_soc_table_all_cells_{charge|discharge}.csv   ONE file, one row per SOC
        checkpoint (ascending 2-100%), with OCV / R0 / R1 / R2 / tau1 / tau2 /
        capacity as a repeated column group per cell (Cell1_..., Cell2_..., ...)
    cell{N}_diagnostics_{charge|discharge}.png   capacity/OCV-SOC/impedance plots, one cell at a time
    all_cells_{R0,R1,R2,tau1,tau2,OCV}_{charge|discharge}.png   six diagrams, each one
        parameter vs. SOC with every cell overlaid on the same axes -- lets you spot
        cell-to-cell spread (a weak cell, a mismatched R0, ...) at a glance

No log file is written. Every sanity-check finding prints directly to the console,
tagged and colored by severity:
    INFO    - plain text   - routine status (NaN fixes, capacity breakdown, etc.)
    WARNING - orange       - worth a look, but the run continues
    ERROR   - red          - something that likely invalidates part of the result
The last line of every run is a colored summary:
    === RESULT: SUCCESS (...) ===   in green, or
    === RESULT: FAILED  (...) ===   in red, if any ERROR was logged (a crash also
                                     counts as FAILED and re-raises the exception
                                     afterward, so exit codes stay non-zero).

    Set WRITE_DETAILED_PER_CELL_CSVS = True to also get the old per-cell breakdown:
    cell_capacities_*.csv, cell{N}_hppc_params_*.csv, cell{N}_cycle_diagnostics_*.csv, summary_*.csv.
    Set WRITE_FULL_RESOLUTION_CURVES = True to also get cell{N}_ocv_soc_curve_*.csv (fine-step curve).

SOC is derived only from coulomb counting + the OCV extension model. The CSV's own
target_pct is never used for SOC - only carried through as a cross-check diagnostic.

Discharge note: cycles run high-SOC-to-low, so current is negative during RAMP and
the OCV/capacity arrays are reoriented internally before reusing the same extension
math as charge. If a discharge test stops early (e.g. a cell hits its low-voltage
cutoff, see stop_reason), the unmeasured low end is NOT reliably extrapolated over
a large gap -- it's flagged instead (LARGE_GAP_WARNING_V) rather than guessed at.
"""

import os
import sys
import argparse
import traceback
import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
from scipy.interpolate import PchipInterpolator

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# np.trapz was removed in NumPy 2.x in favour of np.trapezoid; support both.
_trapz = getattr(np, "trapezoid", None) or np.trapz

# =============================================================================
# LEVELED CONSOLE LOGGING (INFO / WARNING / ERROR) -- no log file is written;
# everything prints directly, color-coded, in real time as it's discovered.
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

# =============================================================================
# CONFIG - edit these, or override CSV paths / output dir from the command line
# =============================================================================
NUM_CELLS       = 7
VOLTAGE_SOURCE  = "filtered"     # "filtered" -> cell_vf{i} (chosen), "raw" -> cell_v{i}
CURRENT_SOURCE  = "current_filtered_ma"   # matches original MATLAB script

V_LO, V_HI      = 2.50, 4.20     # OCV bounds assigned to 0 % / 100 % SOC
Q_LO_PAD_AH     = 0.010          # small positive padding below the first point
TOP_TAPER       = 0.60           # damping applied to the linear top-end slope estimate
TABLE_DV        = 0.005          # V step of the full extended OCV-SOC curve export

TARGET_SOC_PCT  = [2, 4, 6, 8, 10, 12, 15, 20, 25, 30,
                    40, 50, 60, 70, 80, 85, 90, 95, 98, 100]   # ascending, per your confirmation

INSTANT_OFFSET      = 5     # samples after pulse start used for the R0 instantaneous jump
OCV_START_AVG_N      = 3    # samples averaged immediately before pulse start
OCV_STOP_AVG_N       = 4    # samples averaged at the end of the rest
SMOOTH_WINDOW         = 3   # gaussian smoothing window on R1,R2,tau1,tau2 (matches smoothdata)

FIT_X0 = [0.7, 5.0, 0.3, 100.0]      # [ratio1, tau1, ratio2, tau2]
FIT_LB = [0.05, 0.5, 0.05, 10.0]
FIT_UB = [0.95, 60.0, 0.95, 400.0]

OUTPUT_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")   # results land in ./results next to this script by default
MAKE_PLOTS  = True                     # one PNG per cell (not CSV clutter, kept by default)

WRITE_DETAILED_PER_CELL_CSVS = False   # True restores the old cellN_*.csv / summary.csv / cell_capacities.csv files
WRITE_FULL_RESOLUTION_CURVES = False   # True also writes cellN_ocv_soc_curve.csv (fine-step OCV-SOC curve)

EXPECTED_MIN_CELL_V = 3.5   # each cell's logged voltage should reach down to at least this
EXPECTED_MAX_CELL_V = 4.1   # ...and up to at least this, or the test likely didn't cover the full range
MAX_ACCEPTABLE_REST_FIT_RMSE_MV = 20.0   # flag any 2-RC rest fit noisier than this (good fits are usually <1 mV)
MAX_ACCEPTABLE_TARGET_SOC_ERR_PCT = 5.0  # flag if the log's own target_pct disagrees with measured SOC by more than this

# If the gap between the last measured OCV point and V_LO/V_HI exceeds this many volts,
# the exponential-tail extrapolation is being asked to cover far more range than it was
# validated for (a locally-fit slope extrapolated a long way can be wildly wrong -- e.g.
# linear-in-the-wrong-place errors of 10x+). Flag it instead of quietly reporting a number.
LARGE_GAP_WARNING_V = 0.15

REQUIRED_COLS = (
    ["elapsed_s", "uptime_ms"]
    + [f"cell_v{i}" for i in range(1, 8)]
    + [f"cell_vf{i}" for i in range(1, 8)]
    + ["pack_voltage_mv", "pack_voltage_filtered_mv",
       "current_ma", "current_filtered_ma", "temperature_c",
       "fet_status", "fet_main_enabled", "commanded_fet_state",
       "learning_status", "segment_type", "target_pct", "segment_coulomb_mah"]
)


# =============================================================================
# 1. LOAD + VALIDATE  (Python equivalent of load_data.m)
# =============================================================================
def load_and_validate(paths, direction):
    """Load one or more HPPC CSV logs, concatenate in time order, repair NaNs,
    and run sanity checks. Prints the same kind of report load_data.m did,
    plus checks that script never did (monotonic time, sign conventions,
    pack-vs-cell-sum consistency, idle current bias). direction is 'C' or 'D'."""
    if isinstance(paths, str):
        paths = [paths]

    frames = []
    t_offset = 0.0
    for p in paths:
        raw = pd.read_csv(p)
        raw["elapsed_s"] = raw["elapsed_s"] + t_offset
        frames.append(raw)
        step = raw["elapsed_s"].diff().median()
        step = step if pd.notna(step) and step > 0 else 0.1
        t_offset = raw["elapsed_s"].iloc[-1] + step
    df = pd.concat(frames, ignore_index=True)

    missing = [c for c in REQUIRED_COLS if c not in df.columns]
    if missing:
        raise ValueError(f"CSV is missing expected columns: {missing}")

    print(f"Loaded {len(df)} rows from {len(paths)} file(s)")
    print(f"Time span: {df.elapsed_s.min():.1f} s to {df.elapsed_s.max():.1f} s "
          f"({(df.elapsed_s.max() - df.elapsed_s.min()) / 60:.1f} min)")

    # ---- NaN repair, reported per column (mirrors load_data.m section 4) ----
    numeric_cols = df.select_dtypes(include=[np.number]).columns
    n_fixed_cols = 0
    for col in numeric_cols:
        n_nan = int(df[col].isna().sum())
        if n_nan == 0:
            continue
        if n_nan == len(df):
            log_info(f"column {col} is entirely empty ({n_nan} rows) -- left as-is, not used downstream.")
            continue
        if col == "elapsed_s" or "time" in col:
            df[col] = df[col].interpolate("linear").ffill().bfill()
            method = "linear interpolation"
        else:
            df[col] = df[col].ffill().bfill()
            method = "previous value (ffill)"
        log_info(f"fixed {n_nan} NaN(s) in column {col} (using {method})")
        n_fixed_cols += 1
    log_info(f"data fix complete -- fixed NaN values in {n_fixed_cols} column(s)")

    # ---- sanity checks ----
    if not df.elapsed_s.is_monotonic_increasing:
        n_bad = int((df.elapsed_s.diff() <= 0).sum())
        log_warning(f"elapsed_s is not strictly increasing ({n_bad} violation(s))")

    bad_seg = set(df.segment_type.unique()) - {"REST", "RAMP"}
    if bad_seg:
        log_info(f"segment_type includes non-REST/RAMP values: {bad_seg} (usually a benign "
                 f"end-of-test marker; see the stop_reason check below for what actually happened)")

    bad_fet = set(df.commanded_fet_state.unique()) - {"ON", "OFF"}
    if bad_fet:
        log_warning(f"unexpected commanded_fet_state values: {bad_fet}")

    mismatch = int(((df.segment_type == "RAMP") != (df.commanded_fet_state == "ON")).sum())
    if mismatch:
        log_warning(f"segment_type/commanded_fet_state disagree on {mismatch} row(s)")

    cell_cols = [f"cell_v{i}" for i in range(1, 8)]
    pack_err = (df[cell_cols].sum(axis=1) - df.pack_voltage_mv).abs()
    n_spikes = int((pack_err > 50).sum())  # mV
    if n_spikes:
        log_info(
            f"pack voltage vs sum-of-cells mismatch exceeds 50 mV on {n_spikes} row(s) "
            f"(typical mismatch {pack_err.median():.2f} mV, worst {pack_err.max():.1f} mV) -- "
            f"consistent with single-sample FET switch-on transients at RAMP starts; the "
            f"fit already skips them via INSTANT_OFFSET."
        )

    if df.temperature_c.min() < -20 or df.temperature_c.max() > 60:
        log_warning(f"temperature out of typical range: "
                    f"{df.temperature_c.min():.1f} to {df.temperature_c.max():.1f} C")

    ramp_mask = df.segment_type == "RAMP"
    ramp_cur = df.loc[ramp_mask, "current_filtered_ma"]
    if len(ramp_cur):
        # allow the single-sample FET switch-on transient right at pulse start, which can briefly
        # read oddly; flag anything beyond that as a real problem.
        if direction == "C":
            n_bad = int((ramp_cur <= 0).sum())
            bad_word = "<= 0"
        else:
            n_bad = int((ramp_cur >= 0).sum())
            bad_word = ">= 0"
        if n_bad:
            expect = "positive (charging)" if direction == "C" else "negative (discharging)"
            log_error(
                f"current is {bad_word} mA on {n_bad} sample(s) during RAMP segments -- "
                f"current should be strictly {expect} throughout; check sensor sign, a "
                f"dropped/stalled pulse, or a cable fault"
            )

    rest_cur = df.loc[df.segment_type == "REST", "current_ma"]
    idle_bias = float(rest_cur.median()) if len(rest_cur) else float("nan")
    if abs(idle_bias) > 1.0:
        log_info(f"current sensor idle bias ~{idle_bias:.2f} mA during REST -- this integrates "
                 f"into coulomb counting and is NOT auto-corrected")

    if direction == "C":
        if not df.target_pct.is_monotonic_increasing:
            drops = int((df.target_pct.diff() < 0).sum())
            if drops:
                log_warning(f"target_pct decreases at {drops} point(s) -- expected increasing for "
                            f"a charge test; check for repeated/out-of-order segments")
    else:
        tp = df.target_pct.dropna()
        if not tp.is_monotonic_decreasing:
            rises = int((tp.diff() > 0).sum())
            if rises:
                log_warning(f"target_pct increases at {rises} point(s) -- expected decreasing for "
                            f"a discharge test; check for repeated/out-of-order segments")

    if df.stop_reason.notna().any():
        sr = df.loc[df.stop_reason.notna()]
        first_idx = sr.index.min()
        reasons = sr.stop_reason.unique().tolist()
        log_warning(
            f"stop_reason is set on {len(sr)} row(s), first at elapsed_s={df.loc[first_idx,'elapsed_s']:.1f} "
            f"(row {first_idx}) -- reason(s) logged: {reasons}. This usually means the test terminated "
            f"early (e.g. a safety voltage cutoff or a normal CV-taper completion), not necessarily "
            f"that it reached its intended final checkpoint. Any target SOC below/above the last "
            f"completed cycle is extrapolated, not measured."
        )

    return df, idle_bias


# =============================================================================
# 1b. PER-CELL VOLTAGE RANGE CHECK
# =============================================================================
def check_cell_voltage_ranges(df):
    """Every cell should be logged reaching down to EXPECTED_MIN_CELL_V and up to
    EXPECTED_MAX_CELL_V at some point in the test. A cell that never does is a sign
    of a wiring/connection problem, a channel that dropped out, or a cell that
    didn't actually get cycled through the intended range."""
    for i in range(1, 8):
        col = f"cell_vf{i}" if VOLTAGE_SOURCE == "filtered" else f"cell_v{i}"
        v = df[col].values / 1000.0
        vmin, vmax = float(np.nanmin(v)), float(np.nanmax(v))
        if vmin > EXPECTED_MIN_CELL_V or vmax < EXPECTED_MAX_CELL_V:
            log_warning(
                f"Cell {i} voltage range is [{vmin:.3f}, {vmax:.3f}] V -- does not span the "
                f"expected [{EXPECTED_MIN_CELL_V}, {EXPECTED_MAX_CELL_V}] V; check wiring/connection "
                f"or whether this cell's test actually completed."
            )


# =============================================================================
# 2. CYCLE EXTRACTION  (Python equivalent of the state machine in soc_estimator.m)
# =============================================================================
def extract_cycles(df):
    """Return a list of dicts, one per HPPC pulse+rest cycle:
    {ramp_start, ramp_end, rest_start, rest_end, target_pct} (all inclusive,
    0-based row indices into df). Leading REST-only data before the first
    pulse is skipped, matching the MATLAB state machine (state starts at 0
    and only begins recording once FET turns ON). A trailing RAMP with no
    following REST is skipped (can't compute ocv_stop) and reported.
    """
    seg = df.segment_type.values
    block_id = np.zeros(len(seg), dtype=int)
    block_id[1:] = (seg[1:] != seg[:-1]).cumsum()

    blocks = []
    for b in np.unique(block_id):
        idx = np.where(block_id == b)[0]
        tp = df.target_pct.values[idx[0]]
        blocks.append({
            "type": seg[idx[0]],
            "start": int(idx[0]),
            "end": int(idx[-1]),
            "target_pct": int(tp) if np.isfinite(tp) else None,
        })

    cycles = []
    skipped_trailing = 0
    i = 0
    while i < len(blocks):
        if blocks[i]["type"] == "RAMP":
            if i + 1 < len(blocks) and blocks[i + 1]["type"] == "REST":
                cycles.append({
                    "ramp_start": blocks[i]["start"], "ramp_end": blocks[i]["end"],
                    "rest_start": blocks[i + 1]["start"], "rest_end": blocks[i + 1]["end"],
                    "target_pct": blocks[i]["target_pct"],
                })
                i += 2
                continue
            else:
                skipped_trailing += 1
        i += 1

    if skipped_trailing:
        log_info(f"{skipped_trailing} trailing RAMP segment(s) with no following REST were skipped "
                 f"(incomplete cycle, most often caused by the test stopping mid-pulse -- check "
                 f"stop_reason if this is unexpected).")
    if cycles and cycles[0]["ramp_start"] == 0:
        log_warning("first RAMP starts at row 0 -- no pre-pulse rest available for the R0 baseline; "
                    "that cycle's R0/OCV_start may be unreliable.")

    return cycles


# =============================================================================
# 3. 2-RC RELAXATION FIT  (Python equivalent of find_params.m, generalized)
# =============================================================================
def _relax_model_factory(ocv_final, v0):
    def model(t, ratio1, tau1, ratio2, tau2):
        return ocv_final + v0 * (ratio1 * np.exp(-t / tau1) + ratio2 * np.exp(-t / tau2))
    return model


def fit_cycle(cycle, t, v, i_cur):
    """Compute cc, ocv_start, ocv_stop, r0, r1, r2, tau1, tau2, fit_rmse for
    one cycle, for a single cell's voltage array v (V) and the shared pack
    current i_cur (A). t is elapsed_s (s)."""
    rs, re = cycle["ramp_start"], cycle["ramp_end"]
    ss, se = cycle["rest_start"], cycle["rest_end"]

    # --- coulomb count: charge phase + rest phase, matches find_params.m ---
    cc_charge = _trapz(i_cur[rs:re + 1], t[rs:re + 1]) / 3600.0
    cc_rest = _trapz(i_cur[ss:se + 1], t[ss:se + 1]) / 3600.0
    cc = cc_charge + cc_rest

    avg_charge_current = np.mean(i_cur[rs:re + 1])

    # --- R0 from the instantaneous jump a few samples into the pulse ---
    idx_instant = min(rs + INSTANT_OFFSET, re)
    prev_idx = max(rs - 1, 0)
    dv_inst = v[idx_instant] - v[prev_idx]
    dc_inst = i_cur[idx_instant] - i_cur[prev_idx]
    r0 = abs(dv_inst / dc_inst) if dc_inst != 0 else np.nan

    # --- OCV at the start of the pulse and at the end of the following rest ---
    a0 = max(rs - OCV_START_AVG_N, 0)
    ocv_start = np.mean(v[a0:rs])
    b0 = max(se - OCV_STOP_AVG_N + 1, ss)
    ocv_stop = np.mean(v[b0:se + 1])

    # --- 2-RC fit on the rest relaxation ---
    t_rest = t[ss:se + 1] - t[ss]
    v_rest = v[ss:se + 1]
    ocv_final = v_rest[-1]
    v0 = v_rest[0] - ocv_final

    model = _relax_model_factory(ocv_final, v0)
    try:
        popt, _ = curve_fit(model, t_rest, v_rest, p0=FIT_X0,
                             bounds=(FIT_LB, FIT_UB), maxfev=20000)
        ratio1, tau1, ratio2, tau2 = popt
        fit_rmse = np.sqrt(np.mean((v_rest - model(t_rest, *popt)) ** 2))
    except Exception:
        ratio1, tau1, ratio2, tau2 = np.nan, np.nan, np.nan, np.nan
        fit_rmse = np.nan

    r1 = v0 * ratio1 / avg_charge_current if avg_charge_current else np.nan
    r2 = v0 * ratio2 / avg_charge_current if avg_charge_current else np.nan

    return dict(cc=cc, ocv_start=ocv_start, ocv_stop=ocv_stop,
                r0=r0, r1=r1, r2=r2, tau1=tau1, tau2=tau2,
                fit_rmse_mV=fit_rmse * 1000.0 if np.isfinite(fit_rmse) else np.nan,
                target_pct=cycle["target_pct"])


def gaussian_smooth(x, window=SMOOTH_WINDOW):
    """Matches MATLAB smoothdata(x,'gaussian',window): a Gaussian-weighted
    moving average with the given window length, symmetric, edge-truncated."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n < 2:
        return x.copy()
    half = window / 2.0
    sigma = window / (2 * np.sqrt(2 * np.log(2)))  # FWHM = window, matches MATLAB's definition
    out = np.empty(n)
    idx = np.arange(n)
    for k in range(n):
        lo = max(0, k - int(np.ceil(half)) - 1)
        hi = min(n, k + int(np.ceil(half)) + 2)
        w = np.exp(-0.5 * ((idx[lo:hi] - k) / sigma) ** 2)
        out[k] = np.sum(w * x[lo:hi]) / np.sum(w)
    return out


# =============================================================================
# 4. OCV <-> SOC EXTENSION  (Python port of extend_ocv_table.m)
# =============================================================================
def extend_ocv_table(ocv, cc_ocv, v_lo=V_LO, v_hi=V_HI,
                      q_lo_pad=Q_LO_PAD_AH, taper=TOP_TAPER, dv=TABLE_DV):
    """ocv, cc_ocv: 1-D arrays, strictly increasing, cc_ocv in Ah, starting at 0.
    Returns a dict with the fine V/z grid, an OCV-SOC lookup table at step dv,
    the total extended capacity, and callables soc_from_ocv / ocv_from_soc /
    cap_from_ocv. Same math as the validated MATLAB extend_ocv_table.m."""
    ocv = np.asarray(ocv, dtype=float)
    cc_ocv = np.asarray(cc_ocv, dtype=float)
    assert np.all(np.diff(ocv) > 0), "ocv must be strictly increasing"
    assert np.all(np.diff(cc_ocv) > 0), "cc_ocv must be strictly increasing"

    # Real measured cells can legitimately land slightly outside the nominal 2.5-4.2V
    # design window (cell-to-cell variation). Expand the bound rather than crash on it,
    # and report the expansion so the caller can flag it.
    bound_expanded = False
    if ocv[0] <= v_lo:
        v_lo = ocv[0] - 0.01
        bound_expanded = True
    if ocv[-1] >= v_hi:
        v_hi = ocv[-1] + 0.01
        bound_expanded = True

    dQ = np.diff(cc_ocv)
    dV = np.diff(ocv)
    dQdV_top = dQ[-1] / dV[-1]
    q_hi_pad = taper * dQdV_top * (v_hi - ocv[-1])

    q_tot = q_lo_pad + cc_ocv[-1] + q_hi_pad
    z = (q_lo_pad + cc_ocv) / q_tot
    za, Va = z[0], ocv[0]
    zb, Vb = z[-1], ocv[-1]

    pp = PchipInterpolator(z, ocv)
    h = 1e-6
    sa = (pp(za + h) - pp(za)) / h
    sb = (pp(zb) - pp(zb - h)) / h

    p = sa * za / (Va - v_lo)
    q = sb * (1 - zb) / (v_hi - Vb)

    zg = np.linspace(0, 1, 20001)
    Vg = np.empty_like(zg)
    iL = zg < za
    iH = zg > zb
    iM = ~iL & ~iH
    Vg[iL] = v_lo + (Va - v_lo) * (zg[iL] / za) ** p
    Vg[iM] = pp(zg[iM])
    Vg[iH] = v_hi - (v_hi - Vb) * ((1 - zg[iH]) / (1 - zb)) ** q
    Vg[0], Vg[-1] = v_lo, v_hi

    Vg = np.maximum.accumulate(Vg)
    keep = np.concatenate(([True], np.diff(Vg) > 0))
    Vu, zu = Vg[keep], zg[keep]

    v_table = np.arange(v_lo, v_hi + 1e-9, dv)
    if v_table[-1] < v_hi:
        v_table = np.append(v_table, v_hi)
    z_table = np.interp(v_table, Vu, zu)

    return dict(
        q_tot=q_tot, q_lo_pad=q_lo_pad, q_hi_pad=q_hi_pad, p=p, q=q,
        z_meas=z, za=za, zb=zb, v_lo_used=v_lo, v_hi_used=v_hi, bound_expanded=bound_expanded,
        v_table=v_table, soc_table_pct=z_table * 100, cap_table_Ah=z_table * q_tot,
        soc_from_ocv=lambda v: np.interp(np.clip(v, v_lo, v_hi), Vu, zu),
        ocv_from_soc=lambda s: np.interp(np.clip(s, 0, 1), zg, Vg),
        cap_from_ocv=lambda v: np.interp(np.clip(v, v_lo, v_hi), Vu, zu) * q_tot,
    )


# =============================================================================
# 5. PER-CELL PIPELINE
# =============================================================================
def process_cell(cell_idx, df, cycles, idle_bias, direction):
    vcol = f"cell_vf{cell_idx}" if VOLTAGE_SOURCE == "filtered" else f"cell_v{cell_idx}"
    v = df[vcol].values / 1000.0          # mV -> V
    i_cur = df[CURRENT_SOURCE].values / 1000.0   # mA -> A
    t = df["elapsed_s"].values

    rows = [fit_cycle(c, t, v, i_cur) for c in cycles]
    cc = np.array([r["cc"] for r in rows])
    ocv_start = np.array([r["ocv_start"] for r in rows])
    ocv_stop = np.array([r["ocv_stop"] for r in rows])
    r0 = np.array([r["r0"] for r in rows])
    r1_raw = np.array([r["r1"] for r in rows])
    r2_raw = np.array([r["r2"] for r in rows])
    tau1_raw = np.array([r["tau1"] for r in rows])
    tau2_raw = np.array([r["tau2"] for r in rows])
    fit_rmse_mV = np.array([r["fit_rmse_mV"] for r in rows])
    target_pct = np.array([r["target_pct"] for r in rows])

    # cc_ocv_raw/ocv_raw are in original chronological (test) order. For charge this is
    # already ascending in both OCV and capacity added. For discharge it's descending in
    # both (OCV drops, capacity *removed* accumulates) -- extend_ocv_table needs ascending
    # OCV/capacity, so we reorient before calling it and reorient the result back afterward.
    cc_ocv_raw = np.concatenate(([0.0], np.cumsum(cc)))
    ocv_raw = np.concatenate((ocv_start, [ocv_stop[-1]]))

    if direction == "C":
        ocv_for_ext = ocv_raw
        cap_for_ext = cc_ocv_raw
    else:
        ocv_for_ext = ocv_raw[::-1]
        cap_for_ext = (cc_ocv_raw - cc_ocv_raw[-1])[::-1]   # rebase: 0 at the lowest OCV point reached

    if not np.all(np.diff(ocv_for_ext) > 0):
        bad = np.where(np.diff(ocv_for_ext) <= 0)[0]
        log_error(f"[cell {cell_idx}] OCV is not strictly monotonic at cycle boundary(ies) "
                  f"{bad.tolist()}; check for a mis-relaxed rest.")
    if not np.all(np.diff(cap_for_ext) > 0):
        log_error(f"[cell {cell_idx}] accumulated capacity is not strictly monotonic; check for a "
                  f"zero/wrong-sign coulomb count in a pulse.")

    n_failed = int(np.sum(np.isnan(r0) | np.isnan(r1_raw) | np.isnan(r2_raw) |
                          np.isnan(tau1_raw) | np.isnan(tau2_raw)))
    if n_failed:
        failed_cycles = np.where(np.isnan(r0) | np.isnan(r1_raw) | np.isnan(r2_raw) |
                                  np.isnan(tau1_raw) | np.isnan(tau2_raw))[0]
        log_warning(f"[cell {cell_idx}] 2-RC fit failed to converge on {n_failed} cycle(s) "
                    f"(index {failed_cycles.tolist()}); those SOC points fall back on neighboring "
                    f"values via interpolation.")

    n_noisy = int(np.nansum(fit_rmse_mV > MAX_ACCEPTABLE_REST_FIT_RMSE_MV))
    if n_noisy:
        noisy_cycles = np.where(fit_rmse_mV > MAX_ACCEPTABLE_REST_FIT_RMSE_MV)[0]
        log_warning(f"[cell {cell_idx}] {n_noisy} rest relaxation fit(s) exceed "
                    f"{MAX_ACCEPTABLE_REST_FIT_RMSE_MV:.0f} mV RMSE (cycle index {noisy_cycles.tolist()}, "
                    f"worst {np.nanmax(fit_rmse_mV):.1f} mV) -- the 2-RC model fit that rest poorly; "
                    f"check for temperature drift or an interrupted rest.")

    ext = extend_ocv_table(ocv_for_ext, cap_for_ext)

    if ext["bound_expanded"]:
        log_warning(f"[cell {cell_idx}] measured OCV [{ocv_for_ext[0]:.4f}, {ocv_for_ext[-1]:.4f}] V "
                    f"exceeded the configured window [{V_LO}, {V_HI}] V; the bound was auto-expanded "
                    f"to [{ext['v_lo_used']:.4f}, {ext['v_hi_used']:.4f}] V so the run wouldn't crash. "
                    f"This cell may be out of balance with the rest of the pack -- worth a look.")

    # bring z_meas back to original chronological order so it lines up with target_pct/r0/etc.
    z_chrono = ext["z_meas"] if direction == "C" else ext["z_meas"][::-1]
    soc_meas_pct = z_chrono[1:] * 100.0            # one value per completed cycle
    target_err_pct = soc_meas_pct - target_pct

    # discharge cycles run high-SOC-to-low, so soc_meas_pct is descending in chronological
    # order; sort everything by SOC ascending before interpolating/smoothing/tabulating.
    # For charge this sort is a no-op (already ascending).
    order = np.argsort(soc_meas_pct)
    soc_meas_pct = soc_meas_pct[order]
    target_pct = target_pct[order]
    target_err_pct = target_err_pct[order]
    ocv_stop = ocv_stop[order]
    r0 = r0[order]; r1_raw = r1_raw[order]; r2_raw = r2_raw[order]
    tau1_raw = tau1_raw[order]; tau2_raw = tau2_raw[order]
    fit_rmse_mV = fit_rmse_mV[order]

    # gap-size sanity check: is the extrapolation being asked to cover a small margin (fine)
    # or a large, unmeasured chunk of the range (not fine -- see LARGE_GAP_WARNING_V comment)?
    lo_gap_V = ocv_for_ext[0] - ext["v_lo_used"]
    hi_gap_V = ext["v_hi_used"] - ocv_for_ext[-1]
    if lo_gap_V > LARGE_GAP_WARNING_V:
        log_warning(f"[cell {cell_idx}] the low end was only measured down to {ocv_for_ext[0]:.3f} V "
                    f"({soc_meas_pct[0]:.1f}% SOC) -- a {lo_gap_V:.2f} V gap remains down to "
                    f"V_LO={ext['v_lo_used']:.3f} V. That's too large for the exponential-tail "
                    f"extrapolation to be trusted; capacity/OCV/R-values below ~{soc_meas_pct[0]:.0f}% "
                    f"SOC in this table are a rough placeholder, not real data. Get more test coverage "
                    f"down toward the true low end if you need that region.")
    if hi_gap_V > LARGE_GAP_WARNING_V:
        log_warning(f"[cell {cell_idx}] the high end was only measured up to {ocv_for_ext[-1]:.3f} V "
                    f"({soc_meas_pct[-1]:.1f}% SOC) -- a {hi_gap_V:.2f} V gap remains up to "
                    f"V_HI={ext['v_hi_used']:.3f} V. That's too large for the exponential-tail "
                    f"extrapolation to be trusted; capacity/OCV/R-values above ~{soc_meas_pct[-1]:.0f}% "
                    f"SOC in this table are a rough placeholder, not real data. Get more test coverage "
                    f"up toward the true high end if you need that region.")

    max_target_err = np.nanmax(np.abs(target_err_pct))
    if max_target_err > MAX_ACCEPTABLE_TARGET_SOC_ERR_PCT:
        worst_i = int(np.nanargmax(np.abs(target_err_pct)))
        log_warning(f"[cell {cell_idx}] the log's own target_pct disagrees with the coulomb+OCV-derived "
                    f"SOC by up to {max_target_err:.1f} percentage points (worst at "
                    f"target_pct={target_pct[worst_i]:.0f}%, measured {soc_meas_pct[worst_i]:.1f}%). "
                    f"This is larger than normal fit noise -- it usually means this test's assumed "
                    f"total capacity (used to compute target_pct on the device) doesn't match the "
                    f"capacity_2v5_to_4v2_mAh this script computed. Treat the SOC_pct values in this "
                    f"table as the ground truth from this measurement, not target_pct.")

    r1 = gaussian_smooth(r1_raw)
    r2 = gaussian_smooth(r2_raw)
    tau1 = gaussian_smooth(tau1_raw)
    tau2 = gaussian_smooth(tau2_raw)

    # ---- resample R0..tau2 and OCV onto the fixed SOC grid ----
    soc_grid = np.array(TARGET_SOC_PCT, dtype=float)
    lo, hi = soc_meas_pct.min(), soc_meas_pct.max()
    out_of_range = soc_grid[(soc_grid < lo) | (soc_grid > hi)]

    def interp_clip(x):
        return np.interp(soc_grid, soc_meas_pct, x)  # np.interp clips (holds edge value) by default

    table = pd.DataFrame({
        "SOC_pct": soc_grid,
        "OCV_V": ext["ocv_from_soc"](soc_grid / 100.0),
        "R0_ohm": interp_clip(r0),
        "R1_ohm": interp_clip(r1),
        "R2_ohm": interp_clip(r2),
        "tau1_s": interp_clip(tau1),
        "tau2_s": interp_clip(tau2),
    })

    curve = pd.DataFrame({
        "SOC_pct": ext["soc_table_pct"],
        "OCV_V": ext["v_table"],
        "Capacity_Ah": ext["cap_table_Ah"],
    })

    diag = pd.DataFrame({
        "cycle": np.arange(1, len(cycles) + 1),
        "target_pct_nominal": target_pct,
        "soc_measured_pct": soc_meas_pct,
        "target_vs_measured_err_pct": target_err_pct,
        "ocv_stop_V": ocv_stop,
        "r0_ohm": r0, "r1_ohm_raw": r1_raw, "r2_ohm_raw": r2_raw,
        "tau1_s_raw": tau1_raw, "tau2_s_raw": tau2_raw,
        "rest_fit_rmse_mV": fit_rmse_mV,
    })

    summary = dict(
        cell=cell_idx,
        n_cycles=len(cycles),
        capacity_2v5_to_4v2_Ah=ext["q_tot"],
        capacity_2v5_to_4v2_mAh=ext["q_tot"] * 1000.0,
        measured_capacity_Ah=cap_for_ext[-1],             # actually logged, between your first/last OCV points
        low_end_estimate_mAh=ext["q_lo_pad"] * 1000.0,    # below the first measured OCV point - extrapolated
        high_end_estimate_mAh=ext["q_hi_pad"] * 1000.0,   # above the last measured OCV point - extrapolated
        last_measured_soc_pct=soc_meas_pct[-1],
        max_rest_fit_rmse_mV=np.nanmax(fit_rmse_mV),
        mean_rest_fit_rmse_mV=np.nanmean(fit_rmse_mV),
        max_target_vs_measured_err_pct=np.nanmax(np.abs(target_err_pct)),
        n_target_soc_out_of_measured_range=len(out_of_range),
        idle_current_bias_mA=idle_bias,
    )

    log_info(f"[cell {cell_idx}] capacity 2.5V-4.2V (OCV): {summary['capacity_2v5_to_4v2_mAh']:.1f} mAh "
             f"= {cap_for_ext[-1]*1000:.1f} measured + {summary['low_end_estimate_mAh']:.1f} low-end estimate "
             f"+ {summary['high_end_estimate_mAh']:.1f} high-end estimate")

    if len(out_of_range):
        log_info(f"[cell {cell_idx}] {len(out_of_range)} target SOC point(s) "
                 f"{out_of_range.tolist()} fall outside the measured range "
                 f"[{lo:.2f}, {hi:.2f}] % -- held at nearest measured value.")

    return table, curve, diag, summary


def make_plots(cell_idx, curve, diag, out_dir, direction_label):
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))

    C_R0 = "tab:blue"
    C_1  = "tab:green"    # R1 and tau1
    C_2  = "tab:orange"   # R2 and tau2
    tag = f" ({direction_label})"

    ax = axes[0, 0]
    ax.plot(curve.OCV_V, curve.Capacity_Ah * 1000, "-", lw=1.3)
    ax.set_xlabel("OCV [V]"); ax.set_ylabel("Capacity [mAh]")
    ax.set_title(f"Cell {cell_idx}: extended OCV vs capacity{tag}"); ax.grid(True)

    ax = axes[0, 1]
    ax.plot(curve.SOC_pct, curve.OCV_V, "-", lw=1.3)
    ax.set_xlabel("SOC [%]"); ax.set_ylabel("OCV [V]")
    ax.set_title(f"Cell {cell_idx}: OCV vs SOC{tag}"); ax.grid(True)

    ax = axes[1, 0]
    ax.plot(diag.soc_measured_pct, diag.r0_ohm * 1000, "o-", color=C_R0, label="R0")
    ax.plot(diag.soc_measured_pct, diag.r1_ohm_raw * 1000, "o-", color=C_1, label="R1")
    ax.plot(diag.soc_measured_pct, diag.r2_ohm_raw * 1000, "o-", color=C_2, label="R2")
    ax.set_xlabel("SOC [%]"); ax.set_ylabel("R [mOhm]")
    ax.set_title(f"Cell {cell_idx}: resistances vs SOC{tag}"); ax.legend(); ax.grid(True)

    ax = axes[1, 1]
    ax.plot(diag.soc_measured_pct, diag.tau1_s_raw, "o-", color=C_1, label="tau1")
    ax.plot(diag.soc_measured_pct, diag.tau2_s_raw, "o-", color=C_2, label="tau2")
    ax.set_xlabel("SOC [%]"); ax.set_ylabel("tau [s]")
    ax.set_title(f"Cell {cell_idx}: time constants vs SOC{tag}"); ax.legend(); ax.grid(True)

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"cell{cell_idx}_diagnostics_{direction_label}.png"), dpi=140)
    plt.close(fig)


def make_all_cells_plots(tables, out_dir, direction_label):
    """One diagram per parameter -- R0, R1, R2, tau1, tau2, OCV -- with all
    cells overlaid against the shared SOC grid. Lets you spot cell-to-cell
    spread (a weak cell, a mismatched R0, ...) at a glance instead of paging
    through each cell's own cell{N}_diagnostics_{direction}.png one at a time.
    `tables` is the list of per-cell resampled tables built in main() (each
    already on the same fixed SOC grid, so the overlay lines up point-for-point)."""
    tag = f" ({direction_label})"
    cmap = plt.get_cmap("tab10")
    colors = [cmap(i % 10) for i in range(len(tables))]

    def _plot_one(column, scale, ylabel, title, fname):
        fig, ax = plt.subplots(figsize=(8, 5.5))
        for cell_idx, table in enumerate(tables, start=1):
            ax.plot(table["SOC_pct"], table[column] * scale, "o-", lw=1.3, ms=4,
                    color=colors[cell_idx - 1], label=f"Cell {cell_idx}")
        ax.set_xlabel("SOC [%]"); ax.set_ylabel(ylabel)
        ax.set_title(title + tag)
        ax.legend(ncol=2, fontsize=9); ax.grid(True)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, fname), dpi=140)
        plt.close(fig)

    _plot_one("R0_ohm", 1000.0, "R0 [mOhm]", "R0 vs SOC - all cells", f"all_cells_R0_{direction_label}.png")
    _plot_one("R1_ohm", 1000.0, "R1 [mOhm]", "R1 vs SOC - all cells", f"all_cells_R1_{direction_label}.png")
    _plot_one("R2_ohm", 1000.0, "R2 [mOhm]", "R2 vs SOC - all cells", f"all_cells_R2_{direction_label}.png")
    _plot_one("tau1_s", 1.0, "tau1 [s]", "tau1 vs SOC - all cells", f"all_cells_tau1_{direction_label}.png")
    _plot_one("tau2_s", 1.0, "tau2 [s]", "tau2 vs SOC - all cells", f"all_cells_tau2_{direction_label}.png")
    _plot_one("OCV_V", 1.0, "OCV [V]", "OCV vs SOC - all cells", f"all_cells_OCV_{direction_label}.png")


# =============================================================================
# MAIN
# =============================================================================
def build_combined_wide_table(soc_grid, tables, summaries):
    """tables[i] / summaries[i] are cell i+1's per-SOC table and summary dict.
    Returns one DataFrame: one row per SOC checkpoint, columns grouped per cell
    as Cell{n}_OCV_V, Cell{n}_Capacity_2v5_4v2_mAh, Cell{n}_R0_ohm, Cell{n}_R1_ohm,
    Cell{n}_R2_ohm, Cell{n}_tau1_s, Cell{n}_tau2_s."""
    combined = pd.DataFrame({"SOC_pct": soc_grid})
    for cell_idx, (table, summary) in enumerate(zip(tables, summaries), start=1):
        prefix = f"Cell{cell_idx}_"
        combined[prefix + "OCV_V"] = table["OCV_V"].values
        combined[prefix + "Capacity_2v5_4v2_mAh"] = summary["capacity_2v5_to_4v2_mAh"]  # constant per cell
        combined[prefix + "R0_ohm"] = table["R0_ohm"].values
        combined[prefix + "R1_ohm"] = table["R1_ohm"].values
        combined[prefix + "R2_ohm"] = table["R2_ohm"].values
        combined[prefix + "tau1_s"] = table["tau1_s"].values
        combined[prefix + "tau2_s"] = table["tau2_s"].values
    return combined


def main(paths, direction, out_dir=None):
    direction = direction.upper()
    assert direction in ("C", "D"), "direction must be 'C' (charge) or 'D' (discharge)"
    label = "charge" if direction == "C" else "discharge"

    reset_counts()
    out_dir = out_dir or OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)   # keep existing files (e.g. the other direction's charge/discharge
                                           # results) -- same-named files still get overwritten on rerun

    try:
        df, idle_bias = load_and_validate(paths, direction)
        check_cell_voltage_ranges(df)

        cycles = extract_cycles(df)
        if not cycles:
            raise RuntimeError(
                "No complete RAMP+REST cycles were detected in this log -- check the "
                "segment_type/commanded_fet_state columns, or whether the file only "
                "contains a partial/incomplete test.")
        print(f"\nDetected {len(cycles)} HPPC pulse+rest cycles ({label}) "
              f"(target_pct range {cycles[0]['target_pct']} to {cycles[-1]['target_pct']})\n")

        tables, summaries = [], []
        for cell_idx in range(1, NUM_CELLS + 1):
            print(f"--- Cell {cell_idx} ---")
            table, curve, diag, summary = process_cell(cell_idx, df, cycles, idle_bias, direction)
            tables.append(table); summaries.append(summary)

            if WRITE_DETAILED_PER_CELL_CSVS:
                table.to_csv(os.path.join(out_dir, f"cell{cell_idx}_hppc_params_{label}.csv"), index=False)
                diag.to_csv(os.path.join(out_dir, f"cell{cell_idx}_cycle_diagnostics_{label}.csv"), index=False)
            if WRITE_FULL_RESOLUTION_CURVES:
                curve.to_csv(os.path.join(out_dir, f"cell{cell_idx}_ocv_soc_curve_{label}.csv"), index=False)
            if MAKE_PLOTS:
                make_plots(cell_idx, curve, diag, out_dir, label)

            print(f"  rest-fit RMSE mean/max {summary['mean_rest_fit_rmse_mV']:.2f}/"
                  f"{summary['max_rest_fit_rmse_mV']:.2f} mV\n")

        if MAKE_PLOTS:
            make_all_cells_plots(tables, out_dir, label)

        # ---- the one combined deliverable ----
        combined = build_combined_wide_table(np.array(TARGET_SOC_PCT, dtype=float), tables, summaries)
        combined_path = os.path.join(out_dir, f"hppc_soc_table_all_cells_{label}.csv")
        combined.to_csv(combined_path, index=False)

        summary_df = pd.DataFrame(summaries)
        if WRITE_DETAILED_PER_CELL_CSVS:
            summary_df.to_csv(os.path.join(out_dir, f"summary_{label}.csv"), index=False)
            cap = summary_df[["cell", "capacity_2v5_to_4v2_mAh", "measured_capacity_Ah",
                               "low_end_estimate_mAh", "high_end_estimate_mAh"]].copy()
            cap["measured_capacity_mAh"] = cap.pop("measured_capacity_Ah") * 1000.0
            cap.to_csv(os.path.join(out_dir, f"cell_capacities_{label}.csv"), index=False)

        print(f"Capacity per cell, 2.5V-4.2V (OCV) [{label}]:")
        cap_print = summary_df[["cell", "capacity_2v5_to_4v2_mAh", "measured_capacity_Ah",
                                 "low_end_estimate_mAh", "high_end_estimate_mAh"]].copy()
        cap_print["measured_capacity_mAh"] = cap_print.pop("measured_capacity_Ah") * 1000.0
        print(cap_print.to_string(index=False, float_format=lambda x: f"{x:.1f}"))

        print(f"\nCombined table ({len(combined)} rows x {len(combined.columns)} columns): {combined_path}")
        print(f"All outputs written to {out_dir}")

    except Exception:
        log_error(f"FATAL -- run did not complete:\n{traceback.format_exc()}")
        _print_result()
        raise

    _print_result()


def _print_result():
    n_info, n_warn, n_err = _counts["INFO"], _counts["WARNING"], _counts["ERROR"]
    print()
    if n_err:
        print(f"{_BOLDRED}=== RESULT: FAILED  ({n_err} error(s), {n_warn} warning(s), {n_info} info) ==={_RESET}")
    elif n_warn:
        print(f"{_GREEN}=== RESULT: SUCCESS  (completed with {n_warn} warning(s), {n_info} info) ==={_RESET}")
    else:
        print(f"{_GREEN}=== RESULT: SUCCESS  (no warnings or errors, {n_info} info) ==={_RESET}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="HPPC 2-RC + OCV/SOC extraction pipeline")
    parser.add_argument("direction", choices=["C", "c", "D", "d"],
                         help="C = analyze as a charge test, D = analyze as a discharge test")
    parser.add_argument("csv_files", nargs="+", help="one or more HPPC log CSVs (time-concatenated in order given)")
    parser.add_argument("--outdir", default=None,
                         help="output folder (default: a 'results' folder next to this script; "
                              "created if missing, existing files from other runs are kept)")
    parser.add_argument("--detailed", action="store_true",
                         help="also write the old per-cell CSVs (cellN_hppc_params.csv, "
                              "cellN_cycle_diagnostics.csv, summary.csv, cell_capacities.csv)")
    parser.add_argument("--curves", action="store_true",
                         help="also write cellN_ocv_soc_curve.csv (fine-resolution OCV-SOC curve)")
    parser.add_argument("--no-plots", action="store_true",
                         help="skip the cellN_diagnostics.png plots and the all_cells_*.png overlay plots")
    args = parser.parse_args()

    if args.detailed:
        WRITE_DETAILED_PER_CELL_CSVS = True
    if args.curves:
        WRITE_FULL_RESOLUTION_CURVES = True
    if args.no_plots:
        MAKE_PLOTS = False

    main(args.csv_files, args.direction.upper(), out_dir=args.outdir)
