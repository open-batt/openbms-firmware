r"""
kalman_soc_estimator.py
=======================
2-RC equivalent-circuit battery model + Extended Kalman Filter for SOC, driven
by the R0/R1/R2/tau1/tau2/OCV-vs-SOC tables from hppc_pipeline.py.

For each cell, and for a real current/voltage "working data" log, this script:
  1. Builds smooth OCV(SOC) and R0/R1/R2/tau1/tau2(SOC) lookups from the charge
     and discharge parameter tables (both are used -- the model picks charge or
     discharge parameters at each timestep based on the sign of the current,
     since your own data showed real hysteresis between the two, see the
     "charge vs discharge OCV" comparison from earlier in this project).
  2. Runs the model OPEN LOOP (pure prediction: coulomb counting + RC decay,
     no correction from the measured voltage) to get an "estimated voltage".
  3. Runs the same model with an Extended Kalman Filter (predicts, then
     corrects SOC/V_RC1/V_RC2 against the real measured voltage every step)
     to get a second "estimated voltage".
  4. Plots real vs open-loop vs Kalman-corrected voltage (and the two SOC
     traces) per cell.

Run:
    python kalman_soc_estimator.py --charge-table hppc_soc_table_all_cells_charge.csv \
                                    --discharge-table hppc_soc_table_all_cells_discharge.csv \
                                    --data c_partical_1.csv --data d_partial_2.csv

Each --data file is processed independently and gets its own set of outputs,
named after that file's stem: {stem}_cell{N}_kalman.png, plus a console summary.

IMPORTANT SIMPLIFYING ASSUMPTIONS (read before trusting the numbers):
  - SOC at t=0 is seeded by inverting the FIRST measured voltage through the
    OCV table directly (ignoring the small IR-drop already present if current
    is already flowing at t=0). This is exactly the kind of initial error the
    Kalman filter is supposed to correct out over the first several seconds --
    if you watch the Kalman SOC trace do something odd right at the very
    start, that's this seed settling in, not a bug.
  - V_RC1 = V_RC2 = 0 at t=0 (assumes a relaxed start). Same caveat as above.
  - Capacity used for coulomb counting is the mean of the charge- and
    discharge-derived capacity_2v5_to_4v2_mAh for that cell (both tables
    agreed well in your real data, ~2000 mAh).
  - Near-zero current (|I| < CURRENT_DEADZONE_A) holds the previously-selected
    charge/discharge parameter branch rather than flip-flopping on noise.
  - Q_proc / R_meas below are reasonable starting points, not fitted -- there's
    no independent ground-truth SOC in this data to fit them against. Tune them
    if the Kalman trace over- or under-reacts to noise.
"""

import os
import sys
import argparse
import numpy as np
import pandas as pd
from scipy.interpolate import PchipInterpolator

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# =============================================================================
# CONFIG
# =============================================================================
NUM_CELLS = 7
VOLTAGE_SOURCE = "filtered"          # "filtered" -> cell_vf{i}, "raw" -> cell_v{i}
CURRENT_SOURCE = "current_filtered_ma"

CURRENT_DEADZONE_A = 0.05            # |I| below this holds the last charge/discharge branch
SOC_MIN, SOC_MAX = 0.0, 1.0          # hard clip, matches the 0-100% table range

# EKF tuning -- process noise (state units: SOC fraction, volts, volts)
Q_PROC = np.diag([1e-3, 1e-3, 1e-3]) ** 2     # per-step process noise covariance
R_MEAS = (0.005) ** 2                          # measurement noise variance, (volts)^2
P0 = np.diag([0.05, 0.02, 0.02]) ** 2          # initial state covariance (SOC fraction, V, V)

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))


# =============================================================================
# 1. PARAMETER TABLES -> PER-CELL, PER-BRANCH LOOKUPS
# =============================================================================
def build_lookup(table_csv):
    """Return {cell_idx: {'OCV': callable(soc_frac)->V, 'dOCV': callable(soc_frac)->V/frac,
    'R0':.., 'R1':.., 'R2':.., 'tau1':.., 'tau2':.., 'ocv_to_soc': callable(V)->soc_frac,
    'capacity_mAh': float}} built from one hppc_soc_table_all_cells_*.csv file."""
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
            capacity_mAh=float(df[p + "Capacity_2v5_4v2_mAh"].iloc[0]),
        )
    return out


# =============================================================================
# 2. WORKING-DATA LOADING + CURRENT SIGN AUTO-DETECTION
# =============================================================================
def load_working_data(path):
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
    print(f"[{os.path.basename(path)}] net voltage trend: {direction}, "
          f"mean current (as loaded): {mean_i:.1f} mA"
          + (" -- SIGN FLIPPED to match the charge=positive convention" if flipped else ""))

    return df


# =============================================================================
# 3. 2-RC MODEL + EKF
# =============================================================================
def select_branch(current_a, prev_branch):
    if current_a > CURRENT_DEADZONE_A:
        return "charge"
    if current_a < -CURRENT_DEADZONE_A:
        return "discharge"
    return prev_branch if prev_branch is not None else "charge"


def run_cell(cell_idx, t, i_pack_a, v_meas, lut_charge, lut_discharge):
    """Returns a dict of per-timestep arrays: v_ol, v_kf, soc_ol, soc_kf."""
    lut = {"charge": lut_charge[cell_idx], "discharge": lut_discharge[cell_idx]}
    capacity_ah = (lut_charge[cell_idx]["capacity_mAh"] + lut_discharge[cell_idx]["capacity_mAh"]) / 2000.0

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

        S = H @ P_pred @ H.T + R_MEAS
        K = (P_pred @ H) / S
        y = v_meas[k] - v_pred

        x_kf = x_pred + K * y
        x_kf[0] = np.clip(x_kf[0], SOC_MIN, SOC_MAX)
        P = (np.eye(3) - np.outer(K, H)) @ P_pred

        v_kf[k] = v_pred          # a-priori prediction -- this is what the filter is judged on
        soc_kf[k] = x_kf[0]       # a-posteriori SOC -- the corrected estimate going forward
        branch_kf = branch_meas

    return dict(v_ol=v_ol, v_kf=v_kf, soc_ol=soc_ol, soc_kf=soc_kf)


# =============================================================================
# 4. PLOTTING
# =============================================================================
def make_plot(cell_idx, t, v_meas, result, out_path, data_label):
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

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
    ax.set_xlabel("Time [s]"); ax.set_ylabel("SOC [%]")
    ax.legend(loc="best"); ax.grid(True)

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


# =============================================================================
# MAIN
# =============================================================================
def main(charge_table, discharge_table, data_files, out_dir=None):
    out_dir = out_dir or OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)

    lut_charge = build_lookup(charge_table)
    lut_discharge = build_lookup(discharge_table)

    for data_path in data_files:
        stem = os.path.splitext(os.path.basename(data_path))[0]
        print(f"\n=== {stem} ===")
        df = load_working_data(data_path)
        t = df["elapsed_s"].values
        i_pack_a = df["current_filtered_ma"].values / 1000.0

        rmse_rows = []
        for cell_idx in range(1, NUM_CELLS + 1):
            vcol = f"cell_vf{cell_idx}" if VOLTAGE_SOURCE == "filtered" else f"cell_v{cell_idx}"
            v_meas = df[vcol].values / 1000.0

            result = run_cell(cell_idx, t, i_pack_a, v_meas, lut_charge, lut_discharge)

            rmse_ol = float(np.sqrt(np.mean((v_meas - result["v_ol"]) ** 2)))
            rmse_kf = float(np.sqrt(np.mean((v_meas - result["v_kf"]) ** 2)))
            rmse_rows.append(dict(cell=cell_idx, rmse_openloop_mV=rmse_ol * 1000, rmse_kalman_mV=rmse_kf * 1000))

            out_path = os.path.join(out_dir, f"{stem}_cell{cell_idx}_kalman.png")
            make_plot(cell_idx, t, v_meas, result, out_path, stem)
            print(f"  Cell {cell_idx}: RMSE open-loop {rmse_ol*1000:6.1f} mV, "
                  f"RMSE Kalman {rmse_kf*1000:6.1f} mV  -> {out_path}")

        rmse_df = pd.DataFrame(rmse_rows)
        print(f"\n{stem} summary:")
        print(rmse_df.to_string(index=False, float_format=lambda x: f"{x:.1f}"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="2-RC + EKF SOC estimator using HPPC-derived parameter tables")
    parser.add_argument("--charge-table", required=True, help="hppc_soc_table_all_cells_charge.csv")
    parser.add_argument("--discharge-table", required=True, help="hppc_soc_table_all_cells_discharge.csv")
    parser.add_argument("--data", action="append", required=True,
                         help="a real current/voltage log to run the model against; repeat for multiple files")
    parser.add_argument("--outdir", default=None, help="output folder (default: folder this script is in)")
    args = parser.parse_args()

    main(args.charge_table, args.discharge_table, args.data, out_dir=args.outdir)
