#!/usr/bin/env python3
"""
preview.py — offline viewer for OpenBMS CSV logs produced by
monitor.py / charging.py / test.py.

Plots, against elapsed time, whichever of the following are
requested (and present in the CSV):
  pv  - pack voltage in V (filtered, falls back to raw)
  cv  - filtered cell voltages in V (cell_vf1..cell_vf7)
  pc  - pack current in A (filtered, falls back to raw)
  fet - main FET state (ON/OFF)
  t   - temperature in C

Works with any of the CSV formats produced so far (continuous,
pulse, or HPPC logs) — it only looks for the columns it needs and
skips a chart if its column isn't present in the file.

Usage:
    python preview.py <csv_log_file> [pv] [cv] [pc] [fet] [t]

If no chart arguments are given, the default is: pv pc

Examples:
    python preview.py openbms_charging_20260706_142233.csv
    python preview.py openbms_charging_20260706_142233.csv cv fet
    python preview.py openbms_charging_20260706_142233.csv pv cv pc fet t
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt


VALID_ARGS = ('pv', 'cv', 'pc', 'fet', 't')
DEFAULT_ARGS = ('pv', 'pc')


def load_data(path):
    return pd.read_csv(path)


def get_time_axis(df):
    """Prefer elapsed_s; fall back to uptime_ms; fall back to row index."""
    if 'elapsed_s' in df.columns:
        return df['elapsed_s'].astype(float), 'Elapsed time (s)'

    if 'uptime_ms' in df.columns:
        up = df['uptime_ms'].astype(float)
        first_valid = up.dropna()
        if not first_valid.empty:
            t = (up - first_valid.iloc[0]) / 1000.0
            return t, 'Elapsed time (s, from device uptime)'

    return pd.Series(range(len(df))), 'Sample index'


def get_pack_voltage(df):
    """Return the pack voltage series (V), preferring the filtered
    column over the raw one, or None if neither is present."""
    if 'pack_voltage_filtered_mv' in df.columns:
        return df['pack_voltage_filtered_mv'].astype(float) / 1000.0

    if 'pack_voltage_mv' in df.columns:
        return df['pack_voltage_mv'].astype(float) / 1000.0

    return None


def get_pack_current(df):
    """Return the pack current series (A), preferring the filtered
    column over the raw one, or None if neither is present."""
    if 'current_filtered_ma' in df.columns:
        return df['current_filtered_ma'].astype(float) / 1000.0

    if 'current_ma' in df.columns:
        return df['current_ma'].astype(float) / 1000.0

    return None


def get_temperature(df):
    """Return the temperature series (C), or None if not present."""
    if 'temperature_c' in df.columns:
        return df['temperature_c'].astype(float)

    return None


def get_fet_state(df):
    """Return a 0/1 series for the main FET state, or None if no
    suitable column exists. Prefers the actually-measured
    fet_main_enabled / fet_status over the merely-commanded state."""
    if 'fet_main_enabled' in df.columns:
        s = df['fet_main_enabled'].astype(str).str.strip().str.lower()
        return s.map({'true': 1.0, 'false': 0.0})

    if 'fet_status' in df.columns:
        def bit0(v):
            try:
                return 1.0 if (int(float(v)) & 0x01) else 0.0
            except (TypeError, ValueError):
                return float('nan')
        return df['fet_status'].apply(bit0)

    if 'commanded_fet_state' in df.columns:
        s = df['commanded_fet_state'].astype(str).str.strip().str.upper()
        return s.map({'ON': 1.0, 'OFF': 0.0})

    return None


def print_usage():
    print("Usage: python preview.py <csv_log_file> [pv] [cv] [pc] [fet] [t]")
    print("  pv  - pack voltage")
    print("  cv  - cell voltages")
    print("  pc  - pack current")
    print("  fet - fet state")
    print("  t   - temperature")
    print("If no chart arguments are given, the default is: pv pc")
    print("Example: python preview.py openbms_charging_20260706_142233.csv cv fet")


def main():
    if len(sys.argv) < 2:
        print_usage()
        sys.exit(1)

    path = sys.argv[1]
    requested = sys.argv[2:]

    if not requested:
        requested = list(DEFAULT_ARGS)

    invalid = [a for a in requested if a not in VALID_ARGS]
    if invalid:
        print(f"ERR: unknown argument(s): {', '.join(invalid)}")
        print_usage()
        sys.exit(1)

    want_pv = 'pv' in requested
    want_cv = 'cv' in requested
    want_pc = 'pc' in requested
    want_fet = 'fet' in requested
    want_t = 't' in requested

    try:
        df = load_data(path)
    except Exception as e:
        print(f"ERR: failed to read '{path}': {e}")
        sys.exit(1)

    if df.empty:
        print(f"ERR: '{path}' contains no data rows.")
        sys.exit(1)

    t, t_label = get_time_axis(df)

    pack_v = None
    if want_pv:
        pack_v = get_pack_voltage(df)
        if pack_v is None:
            print("WARNING: no pack voltage column (pack_voltage_filtered_mv / pack_voltage_mv) found — skipping that chart.")

    cell_cols = []
    if want_cv:
        cell_cols = [f"cell_vf{i}" for i in range(1, 8) if f"cell_vf{i}" in df.columns]
        if not cell_cols:
            print("WARNING: no filtered cell voltage columns (cell_vf1..cell_vf7) found — skipping that chart.")

    pack_c = None
    if want_pc:
        pack_c = get_pack_current(df)
        if pack_c is None:
            print("WARNING: no pack current column (current_filtered_ma / current_ma) found — skipping that chart.")

    fet_state = None
    if want_fet:
        fet_state = get_fet_state(df)
        if fet_state is None:
            print("WARNING: no FET state column found — skipping that chart.")

    temp = None
    if want_t:
        temp = get_temperature(df)
        if temp is None:
            print("WARNING: no temperature column (temperature_c) found — skipping that chart.")

    n_plots = sum([pack_v is not None, bool(cell_cols), pack_c is not None,
                   fet_state is not None, temp is not None])
    if n_plots == 0:
        print("ERR: none of the requested columns were found in this file — nothing to plot.")
        sys.exit(1)

    fig, axes = plt.subplots(n_plots, 1, figsize=(12, 3.2 * n_plots), sharex=True)
    if n_plots == 1:
        axes = [axes]
    ax_idx = 0

    if pack_v is not None:
        ax = axes[ax_idx]; ax_idx += 1
        ax.plot(t, pack_v, color='tab:blue', linewidth=1)
        ax.set_ylabel("Pack voltage (V)")
        ax.set_title("Pack Voltage")
        ax.grid(True, alpha=0.3)

    if cell_cols:
        ax = axes[ax_idx]; ax_idx += 1
        for i, col in enumerate(cell_cols, start=1):
            ax.plot(t, df[col] / 1000.0, label=f"Cell {i}", linewidth=1)
        ax.set_ylabel("Filtered voltage (V)")
        ax.set_title("Filtered Cell Voltages")
        ax.legend(loc='upper right', ncol=min(len(cell_cols), 7), fontsize=8)
        ax.grid(True, alpha=0.3)

    if pack_c is not None:
        ax = axes[ax_idx]; ax_idx += 1
        ax.plot(t, pack_c, color='tab:orange', linewidth=1)
        ax.axhline(0, color='gray', linewidth=0.5)
        ax.set_ylabel("Pack current (A)")
        ax.set_title("Pack Current")
        ax.grid(True, alpha=0.3)

    if fet_state is not None:
        ax = axes[ax_idx]; ax_idx += 1
        ax.step(t, fet_state, where='post', color='tab:green', linewidth=1.5)
        ax.set_ylabel("FET state")
        ax.set_yticks([0, 1])
        ax.set_yticklabels(["OFF", "ON"])
        ax.set_ylim(-0.2, 1.2)
        ax.set_title("Main FET State")
        ax.grid(True, alpha=0.3)

    if temp is not None:
        ax = axes[ax_idx]; ax_idx += 1
        ax.plot(t, temp, color='tab:red', linewidth=1)
        ax.set_ylabel("Temperature (C)")
        ax.set_title("Temperature")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel(t_label)
    fig.suptitle(f"OpenBMS Preview — {path}", fontsize=13, fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()


if __name__ == "__main__":
    main()
