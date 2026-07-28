import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def load_battery_data(file_path, sheet_name=0, show_graph=False):
    """
    Load battery data with two channel columns:
    Time_1, mA_1, mV_1 | Time_2, A_2, V_2

    Channel 1 (Time_1, A_1, V_1) — low current OCV, used for SOC/OCV curve
    Channel 2 (Time_2, A_2, V_2) — discharge and charge pulses, used for ECM parameters

    Parameters
    ----------
    file_path  : str        - path to the .xlsx file
    sheet_name : int or str - sheet index or name (default 0 = first sheet)
    show_graph : bool       - if True, plot both channels

    Returns
    -------
    Time_1, A_1, V_1 : numpy arrays - channel 1 (time s, current A, voltage V)
    Time_2, A_2, V_2 : numpy arrays - channel 2 (time s, current A, voltage V)
    """
    COLS = ["Time_1", "mA_1", "mV_1", "Time_2", "A_2", "V_2"]

    data = pd.read_excel(file_path, sheet_name=sheet_name, usecols=COLS)

    data.dropna(how="all", inplace=True)
    data.reset_index(drop=True, inplace=True)

    # channel 1 — convert mA → A and mV → V
    Time_1 = data["Time_1"].values
    A_1    = data["mA_1"].values / 1000.0
    V_1    = data["mV_1"].values / 1000.0

    # channel 2 — already in A and V (discharge and charge pulses)
    Time_2 = data["Time_2"].values
    A_2    = data["A_2"].values
    V_2    = data["V_2"].values

    # remove NaN from each channel independently
    mask_1 = np.isfinite(Time_1) & np.isfinite(A_1) & np.isfinite(V_1)
    mask_2 = np.isfinite(Time_2) & np.isfinite(A_2) & np.isfinite(V_2)

    Time_1, A_1, V_1 = Time_1[mask_1], A_1[mask_1], V_1[mask_1]
    Time_2, A_2, V_2 = Time_2[mask_2], A_2[mask_2], V_2[mask_2]

    print(f"Loaded : {file_path}  (sheet: {sheet_name})")
    print(f"Channel 1 (OCV)            — rows: {len(Time_1)}  time: {Time_1.min():.1f} → {Time_1.max():.1f} s")
    print(f"Channel 2 (dis + chg pulses) — rows: {len(Time_2)}  time: {Time_2.min():.1f} → {Time_2.max():.1f} s")
    

    if show_graph:
        fig, axes = plt.subplots(2, 2, figsize=(14, 7), sharex=False)
        fig.suptitle(f"Battery data")

        channels = [
            (Time_1/60, A_1, V_1, "OCV extraction (C/20)"),
            (Time_2/60, A_2, V_2, "HPPC test - discharge & charge pulses"),
        ]

        for i, (t, curr, volt, label) in enumerate(channels):
            axes[i, 0].plot(t, volt, color="steelblue", linewidth=0.8)
            axes[i, 0].set_ylabel("Voltage (V)")
            axes[i, 0].set_xlabel("Time (min)")
            axes[i, 0].set_title(label)
            axes[i, 0].grid(True, alpha=0.3)

            axes[i, 1].plot(t, curr, color="tomato", linewidth=0.8)
            axes[i, 1].set_ylabel("Current (A)")
            axes[i, 1].set_xlabel("Time (min)")
            axes[i, 1].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    return Time_1, A_1, V_1, Time_2, A_2, V_2

def compute_ocv_soc_curve(time, current, voltage, show_graph=False):
    """
    Compute OCV-SOC curves for both discharge and charge directions
    using Coulomb counting on low-current OCV test data (Channel 1).
    Returns lookup tables at fixed SOC grid points.

    Parameters
    ----------
    time       : numpy array - time in seconds       (Time_1)
    current    : numpy array - current in amps        (A_1, negative=discharge)
    voltage    : numpy array - voltage in volts       (V_1)
    show_graph : bool        - if True, plot OCV-SOC curves

    Returns
    -------
    soc_grid    : numpy array - SOC grid points (%)
    ocv_dis_lut : numpy array - OCV discharge values at each grid point (V)
    ocv_chg_lut : numpy array - OCV charge values at each grid point (V)
    """
    SOC_GRID = np.array([100, 98, 95, 90, 85, 80, 70, 60, 50,
                          40,  30, 25, 20, 15, 12, 10,  8,  6, 4, 2],
                        dtype=float)

    # dt between samples
    dt    = np.diff(time, prepend=time[0])
    dt[0] = dt[1]

    THRESHOLD = 0.01
    dis_mask  = current < -THRESHOLD
    chg_mask  = current >  THRESHOLD

    print(f"Discharge samples : {dis_mask.sum()}")
    print(f"Charge samples    : {chg_mask.sum()}")

    # Coulomb counting — discharge
    dis_charge_ah = np.cumsum(np.abs(current[dis_mask]) * dt[dis_mask]) / 3600.0
    Q_total_ah    = dis_charge_ah[-1]
    print(f"Total capacity    : {Q_total_ah:.4f} Ah")

    soc_dis = 100.0 * (1.0 - dis_charge_ah / Q_total_ah)
    ocv_dis = voltage[dis_mask]

    # Coulomb counting — charge
    chg_charge_ah = np.cumsum(np.abs(current[chg_mask]) * dt[chg_mask]) / 3600.0
    Q_chg_total   = chg_charge_ah[-1]

    soc_chg = 100.0 * (chg_charge_ah / Q_chg_total)
    ocv_chg = voltage[chg_mask]

    print(f"Discharge SOC range: {soc_dis.min():.1f}% → {soc_dis.max():.1f}%")
    print(f"Charge    SOC range: {soc_chg.min():.1f}% → {soc_chg.max():.1f}%")

    # interpolate onto SOC grid
    ocv_dis_lut = np.interp(SOC_GRID, soc_dis[::-1], ocv_dis[::-1])
    ocv_chg_lut = np.interp(SOC_GRID, soc_chg,       ocv_chg)

    # print lookup table
    print("\n── OCV Lookup Table ──────────────────────────────────────")
    print(f"{'SOC (%)':>8}  {'OCV_dis (V)':>12}  {'OCV_chg (V)':>12}  {'Hysteresis (mV)':>16}")
    print("-" * 54)
    for soc, vd, vc in zip(SOC_GRID, ocv_dis_lut, ocv_chg_lut):
        print(f"{soc:>8.0f}  {vd:>12.4f}  {vc:>12.4f}  {(vc-vd)*1000:>16.2f}")

    if show_graph:
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(soc_dis, ocv_dis, color="steelblue", linewidth=0.6, alpha=0.4, label="raw discharge")
        ax.plot(soc_chg, ocv_chg, color="tomato",    linewidth=0.6, alpha=0.4, label="raw charge")
        ax.plot(SOC_GRID, ocv_dis_lut, "o-", color="steelblue", linewidth=1.5, markersize=5, label="LUT discharge")
        ax.plot(SOC_GRID, ocv_chg_lut, "o-", color="tomato",    linewidth=1.5, markersize=5, label="LUT charge")
        ax.set_xlabel("SOC (%)");  ax.set_ylabel("OCV (V)")
        ax.set_title("OCV — SOC lookup table");  ax.legend()
        ax.grid(True, alpha=0.3);  ax.invert_xaxis()
        plt.tight_layout();  plt.show()

    return SOC_GRID, ocv_dis_lut, ocv_chg_lut, Q_total_ah


def _exp1_fit(ys, dt):
    """
    Fit a single exponential: y(t) = A * z^k  where z = exp(-dt/tau)
    Uses log-linear regression.
    Returns (A, tau) or None if fit fails.
    """
    N = len(ys)
    if N < 5:
        return None

    # only use positive values for log
    pos_mask = ys > 1e-6
    if pos_mask.sum() < 5:
        return None

    ys_pos = ys[pos_mask]
    ks     = np.where(pos_mask)[0].astype(float)

    # log(y) = log(A) + k * log(z)  →  linear regression
    log_y = np.log(ys_pos)
    k_mean = np.mean(ks)
    l_mean = np.mean(log_y)

    slope = np.sum((ks - k_mean) * (log_y - l_mean)) / np.sum((ks - k_mean)**2)
    intercept = l_mean - slope * k_mean

    z   = np.exp(slope)
    A   = np.exp(intercept)
    tau = -dt / np.log(z) if (0 < z < 1) else None

    if tau is None or tau <= 0 or not np.isfinite(tau):
        return None

    return A, tau


def _prony_fit(ys, dt, debug=False):
    """
    Fit sum of two exponentials using Prony's method.
    Falls back to single exponential if two-exponential fit fails.
    Returns (A1, tau1, A2, tau2) or None if both fits fail.
    """
    N = len(ys)
    if N < 20:
        if debug: print(f"    [Prony] FAIL: too few samples N={N}")
        return None

    signal_range = np.max(ys) - np.min(ys)
    if signal_range < 1e-5:
        if debug: print(f"    [Prony] FAIL: signal too flat range={signal_range:.2e}")
        return None

    # ── try two-exponential Prony first ──────────────────────────────────────
    s11, s12, s22, sb1, sb2 = 0.0, 0.0, 0.0, 0.0, 0.0
    for k in range(2, N):
        y0, y1, y2 = ys[k], ys[k-1], ys[k-2]
        s11 += y1*y1;  s12 += y1*y2;  s22 += y2*y2
        sb1 += y1*y0;  sb2 += y2*y0

    det = s11*s22 - s12*s12

    if abs(det) > 1e-28:
        c1 = (s22*sb1 - s12*sb2) / det
        c2 = (s11*sb2 - s12*sb1) / det
        disc = c1*c1 + 4*c2

        if disc >= 0:
            z1 = (c1 + np.sqrt(disc)) / 2.0
            z2 = (c1 - np.sqrt(disc)) / 2.0

            if debug:
                print(f"    [Prony2] c1={c1:.4f} c2={c2:.4f} z1={z1:.4f} z2={z2:.4f}")

            if (0 < z1 < 1 and 0 < z2 < 1 and abs(z1 - z2) > 1e-4):
                tau1 = -dt / np.log(z1)
                tau2 = -dt / np.log(z2)

                if (np.isfinite(tau1) and np.isfinite(tau2)
                        and tau1 > 0 and tau2 > 0):

                    NFIT = min(N, int(4 * max(tau1, tau2) / dt))
                    if NFIT >= 4:
                        sa11,sa12,sa22,sab1,sab2 = 0.0,0.0,0.0,0.0,0.0
                        for k in range(NFIT):
                            b1=z1**k; b2=z2**k
                            sa11+=b1*b1; sa12+=b1*b2; sa22+=b2*b2
                            sab1+=b1*ys[k]; sab2+=b2*ys[k]
                        det2 = sa11*sa22 - sa12*sa12
                        if abs(det2) > 1e-28:
                            A1 = (sa22*sab1 - sa12*sab2) / det2
                            A2 = (sa11*sab2 - sa12*sab1) / det2
                            if tau1 < tau2:
                                A1,tau1,A2,tau2 = A2,tau2,A1,tau1
                            if debug:
                                print(f"    [Prony2] SUCCESS tau1={tau1:.1f}s tau2={tau2:.1f}s")
                            return A1, tau1, A2, tau2

    # ── fall back to single exponential ──────────────────────────────────────
    if debug:
        print(f"    [Prony2] failed — trying single exponential fit")

    result = _exp1_fit(ys, dt)
    if result is None:
        if debug: print(f"    [Exp1] FAIL")
        return None

    A1, tau1 = result

    # represent as 2RC with tiny second component so downstream code works
    # set tau2 = tau1 / 10 and A2 = small fraction of A1
    tau2 = tau1 / 10.0
    A2   = A1  * 0.01

    if debug:
        print(f"    [Exp1] SUCCESS tau1={tau1:.1f}s  (tau2={tau2:.1f}s synthetic)")

    return A1, tau1, A2, tau2


def extract_ecm_parameters(time, current, voltage, Q_nom_ah,
                            direction="discharge", show_graph=False):
    """
    Extract 2RC ECM parameters from pulse-rest data (Channel 2).
    Channel 2 contains both discharge (negative) and charge (positive) pulses.
    Pass direction="discharge" or direction="charge" to filter which pulses
    are processed.

    Parameters
    ----------
    time      : numpy array - time in seconds
    current   : numpy array - current in amps (negative=discharge, positive=charge)
    voltage   : numpy array - voltage in volts
    Q_nom_ah  : float       - nominal cell capacity in Ah
    direction : str         - "discharge" or "charge"
    show_graph: bool        - if True, plot parameters vs SOC

    Returns
    -------
    params : list of dicts, each containing:
             soc, V_ocv, R0, R1, C1, R2, C2, tau1, tau2
    """
    THRESHOLD   = 0.05
    MIN_PULSE_S = 10
    MIN_REST_S  = 60

    # ── clean NaN rows ────────────────────────────────────────────────────────
    valid_mask = np.isfinite(time) & np.isfinite(current) & np.isfinite(voltage)
    time    = time[valid_mask]
    current = current[valid_mask]
    voltage = voltage[valid_mask]

    if len(time) == 0:
        print(f"No valid data found for {direction} channel.")
        return []

    print(f"\n── {direction.upper()} parameter extraction ─────────────────")
    print(f"Valid rows : {len(time)}  time: {time.min():.1f} → {time.max():.1f} s")

    # ── detect all pulse transitions ──────────────────────────────────────────
    active       = (np.abs(current) > THRESHOLD).astype(int)
    transitions  = np.diff(active, prepend=0)
    pulse_starts = np.where(transitions ==  1)[0]
    pulse_ends   = np.where(transitions == -1)[0]

    if len(pulse_ends) < len(pulse_starts):
        pulse_ends = np.append(pulse_ends, len(current) - 1)

    print(f"Total pulses detected : {len(pulse_starts)}")

    # ── SOC trace across entire channel (single running Coulomb counter) ──────
    # channel 2 starts fully charged (100%) — SOC decreases during discharge
    # and increases during charge
    dt        = np.diff(time, prepend=time[0])
    dt[0]     = dt[1]
    charge_ah = np.cumsum(current * dt) / 3600.0
    soc_trace = np.clip(100.0 + (charge_ah / Q_nom_ah) * 100.0, 0.0, 100.0)

    params = []

    for idx, (ps, pe) in enumerate(zip(pulse_starts, pulse_ends)):

        # ── check pulse duration ──────────────────────────────────────────────
        pulse_duration = time[pe] - time[ps]
        if pulse_duration < MIN_PULSE_S:
            continue

        # ── check rest window after pulse ─────────────────────────────────────
        next_start    = pulse_starts[idx + 1] if idx + 1 < len(pulse_starts) else len(current)
        rest_end      = min(next_start, len(current) - 1)
        rest_duration = time[rest_end] - time[pe]
        if rest_duration < MIN_REST_S:
            continue

        # ── check pulse direction matches requested direction ─────────────────
        I_pulse = np.mean(current[ps:pe])
        if np.abs(I_pulse) < THRESHOLD:
            continue

        is_discharge = I_pulse < 0
        if direction == "discharge" and not is_discharge:
            continue
        if direction == "charge" and is_discharge:
            continue

        # ── R0: instantaneous drop at pulse start ─────────────────────────────
        V_before = voltage[ps - 1] if ps > 0 else voltage[ps]
        V_after  = voltage[ps + 1]
        R0       = np.abs(V_before - V_after) / np.abs(I_pulse)

        # ── V_OCV: settled voltage at end of rest ─────────────────────────────
        n_settle = max(1, int((rest_end - pe) * 0.05))
        V_ocv    = np.mean(voltage[rest_end - n_settle : rest_end])

        soc_at_pulse = soc_trace[ps]

        # ── Prony fit on relaxation window ────────────────────────────────────
        V_rel  = voltage[pe:rest_end]
        t_rel  = time[pe:rest_end]

        if len(t_rel) < 2:
            continue

        dt_rel = np.mean(np.diff(t_rel))
        if dt_rel <= 0 or not np.isfinite(dt_rel):
            continue

        # use abs() so both discharge (V rises to OCV) and
        # charge (V drops to OCV) give a positive decaying signal
        Vtilde = np.abs(V_ocv - V_rel)

        fit = _prony_fit(Vtilde, dt_rel)

        if fit is None:
            print(f"  Pulse {idx+1:2d} [{direction:3s}]: Prony fit failed  "
                  f"SOC={soc_at_pulse:.1f}%  I={I_pulse:.3f}A")
            continue

        A1, tau1, A2, tau2 = fit

        # ── recover R1, R2 from amplitudes ────────────────────────────────────
        e1 = 1 - np.exp(-pulse_duration / tau1)
        e2 = 1 - np.exp(-pulse_duration / tau2)
        R1 = np.abs(A1) / (np.abs(I_pulse) * e1) if e1 > 0.01 else np.nan
        R2 = np.abs(A2) / (np.abs(I_pulse) * e2) if e2 > 0.01 else np.nan
        C1 = tau1 / R1 if (np.isfinite(R1) and R1 > 0) else np.nan
        C2 = tau2 / R2 if (np.isfinite(R2) and R2 > 0) else np.nan

        entry = {
            "soc"  : round(soc_at_pulse, 1),
            "V_ocv": round(V_ocv,        4),
            "R0"   : round(R0,           6),
            "R1"   : round(R1, 6) if np.isfinite(R1) else None,
            "C1"   : round(C1, 2) if np.isfinite(C1) else None,
            "R2"   : round(R2, 6) if np.isfinite(R2) else None,
            "C2"   : round(C2, 2) if np.isfinite(C2) else None,
            "tau1" : round(tau1, 2),
            "tau2" : round(tau2, 2),
        }
        params.append(entry)

        R1_str = f"{R1*1000:.1f}" if np.isfinite(R1) else "nan"
        print(f"  Pulse {idx+1:2d} [{direction[:3]}]: SOC={soc_at_pulse:5.1f}%  "
              f"R0={R0*1000:5.1f}mΩ  R1={R1_str}mΩ  "
              f"tau1={tau1:6.1f}s  tau2={tau2:6.1f}s  V_ocv={V_ocv:.4f}V")

    print(f"Successfully extracted {len(params)} parameter sets")

    if show_graph and params:
        _plot_ecm_params(params, direction)

    return params


def _plot_ecm_params(params, direction="discharge"):
    """
    Plot extracted ECM parameters vs SOC.
    """
    color = "steelblue" if direction == "discharge" else "tomato"

    def get(key):
        x = [p["soc"] for p in params if p[key] is not None]
        y = [p[key]   for p in params if p[key] is not None]
        return x, y

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(f"2RC ECM Parameters vs SOC — {direction}")

    for ax, key, label, scale in [
        (axes[0, 0], "R0",    "R0 (mΩ)",   1000),
        (axes[0, 1], "R1",    "R1 (mΩ)",   1000),
        (axes[1, 0], "R2",    "R2 (mΩ)",   1000),
        (axes[1, 1], "tau1",  "τ1 (s)",       1),
        (axes[2, 0], "tau2",  "τ2 (s)",       1),
        (axes[2, 1], "V_ocv", "V_OCV (V)",    1),
    ]:
        x, y = get(key)
        ax.plot(x, [v * scale for v in y], "o-", color=color, linewidth=1.5, markersize=5)
        ax.set_title(label);  ax.set_xlabel("SOC (%)")
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

def build_ecm_lut(dis_params, chg_params, soc_grid):
    """
    Interpolate extracted ECM parameters onto the same SOC grid as the OCV LUT.

    Parameters
    ----------
    dis_params : list of dicts  - from extract_ecm_parameters direction="discharge"
    chg_params : list of dicts  - from extract_ecm_parameters direction="charge"
    soc_grid   : numpy array    - SOC grid points (%) from compute_ocv_soc_curve

    Returns
    -------
    lut : dict with keys:
          soc_grid,
          R0_dis, R1_dis, tau1_dis, R2_dis, tau2_dis,
          R0_chg, R1_chg, tau1_chg, R2_chg, tau2_chg
    """

    def interp_param(params, key, soc_grid):
        """
        Collect valid (soc, value) pairs for key and interpolate onto soc_grid.
        Returns NaN where outside the measured SOC range.
        """
        soc_vals = np.array([p["soc"] for p in params if p.get(key) is not None])
        par_vals = np.array([p[key]   for p in params if p.get(key) is not None])

        if len(soc_vals) < 2:
            return np.full(len(soc_grid), np.nan)

        sort_idx    = np.argsort(soc_vals)
        soc_sorted  = soc_vals[sort_idx]
        par_sorted  = par_vals[sort_idx]

        interpolated = np.interp(soc_grid, soc_sorted, par_sorted)

        # mark out-of-range as NaN instead of extrapolating
        interpolated[soc_grid < soc_sorted[0]]  = np.nan
        interpolated[soc_grid > soc_sorted[-1]] = np.nan

        return interpolated

    lut = {"soc_grid": soc_grid}

    for key in ["R0", "R1", "tau1", "R2", "tau2"]:
        lut[f"{key}_dis"] = interp_param(dis_params, key, soc_grid)
        lut[f"{key}_chg"] = interp_param(chg_params, key, soc_grid)

    # print lookup table
    print("\n── ECM Lookup Table ──────────────────────────────────────────────────────────────────────────────────────────")
    print(f"{'SOC%':>5}  "
          f"{'R0_dis':>9}  {'R1_dis':>9}  {'tau1_dis':>9}  {'R2_dis':>9}  {'tau2_dis':>9}  "
          f"{'R0_chg':>9}  {'R1_chg':>9}  {'tau1_chg':>9}  {'R2_chg':>9}  {'tau2_chg':>9}")
    print("-" * 113)

    for i, soc in enumerate(soc_grid):

        def fmt_r(val):
            return f"{val*1000:>9.2f}" if np.isfinite(val) else f"{'N/A':>9}"

        def fmt_t(val):
            return f"{val:>9.1f}" if np.isfinite(val) else f"{'N/A':>9}"

        print(f"{soc:>5.0f}  "
              f"{fmt_r(lut['R0_dis'][i])}  "
              f"{fmt_r(lut['R1_dis'][i])}  "
              f"{fmt_t(lut['tau1_dis'][i])}  "
              f"{fmt_r(lut['R2_dis'][i])}  "
              f"{fmt_t(lut['tau2_dis'][i])}  "
              f"{fmt_r(lut['R0_chg'][i])}  "
              f"{fmt_r(lut['R1_chg'][i])}  "
              f"{fmt_t(lut['tau1_chg'][i])}  "
              f"{fmt_r(lut['R2_chg'][i])}  "
              f"{fmt_t(lut['tau2_chg'][i])}")

    print("\nR0, R1, R2 in mΩ  |  tau1, tau2 in seconds  |  N/A = outside measured range")

    return lut


def plot_ecm_lut(lut):
    """
    Plot ECM LUT parameters vs SOC — discharge and charge on same axes.
    """
    soc = lut["soc_grid"]

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("ECM Parameter Lookup Table — Discharge vs Charge")

    plots = [
        (axes[0, 0], "R0",   "R0 (mΩ)",  1000),
        (axes[0, 1], "R1",   "R1 (mΩ)",  1000),
        (axes[1, 0], "tau1", "τ1 (s)",      1),
        (axes[1, 1], "R2",   "R2 (mΩ)",  1000),
        (axes[2, 0], "tau2", "τ2 (s)",      1),
    ]

    for ax, key, ylabel, scale in plots:
        dis_vals = lut[f"{key}_dis"] * scale
        chg_vals = lut[f"{key}_chg"] * scale

        ax.plot(soc, dis_vals, "o-", color="steelblue", linewidth=1.5,
                markersize=4, label="discharge")
        ax.plot(soc, chg_vals, "o-", color="tomato", linewidth=1.5,
                markersize=4, label="charge")
        ax.set_title(ylabel)
        ax.set_xlabel("SOC (%)")
        ax.invert_xaxis()
        ax.legend()
        ax.grid(True, alpha=0.3)

    axes[2, 1].axis("off")

    plt.tight_layout()
    plt.show()

# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":

    FILE_SAMPLE_DATA = "battery_sample_data.xlsx"

    # load all three channels
    Time_1, A_1, V_1, Time_2, A_2, V_2 = load_battery_data(FILE_SAMPLE_DATA, show_graph=True)

    # Channel 1 — compute OCV-SOC lookup tables and get Q_nom
    soc_grid, ocv_dis_lut, ocv_chg_lut, Q_nom_ah = compute_ocv_soc_curve(Time_1, A_1, V_1, show_graph=False)

    print(f"\nQ_nom : {Q_nom_ah:.4f} Ah")

    
    # Channel 2 — discharge pulses (negative current)
    dis_params = extract_ecm_parameters(Time_2, A_2, V_2, Q_nom_ah,direction="discharge",show_graph=False)

    # Channel 2 — charge pulses (positive current)
    chg_params = extract_ecm_parameters(Time_2, A_2, V_2, Q_nom_ah,direction="charge",show_graph=False)
    
    # build aligned lookup table on same SOC grid as OCV
    ecm_lut = build_ecm_lut(dis_params, chg_params, soc_grid)

    # plot everything
    plot_ecm_lut(ecm_lut)