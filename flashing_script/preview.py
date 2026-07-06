#!/usr/bin/env python3
"""
preview.py — offline viewer for OpenBMS CSV logs produced by
monitor.py / charging.py / test.py.

Plots, against elapsed time:
  1. Filtered cell voltages (cell_vf1..cell_vf7)
  2. Filtered pack current (current_filtered_ma)
  3. Main FET state (ON/OFF)

Works with any of the CSV formats produced so far (continuous,
pulse, or HPPC logs) — it only looks for the columns it needs and
skips a chart if its column isn't present in the file.

Usage:
    python preview.py <csv_log_file>
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt


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


def main():
    if len(sys.argv) != 2:
        print("Usage: python preview.py <csv_log_file>")
        print("Example: python preview.py openbms_charging_20260706_142233.csv")
        sys.exit(1)

    path = sys.argv[1]

    try:
        df = load_data(path)
    except Exception as e:
        print(f"ERR: failed to read '{path}': {e}")
        sys.exit(1)

    if df.empty:
        print(f"ERR: '{path}' contains no data rows.")
        sys.exit(1)

    t, t_label = get_time_axis(df)

    cell_cols = [f"cell_vf{i}" for i in range(1, 8) if f"cell_vf{i}" in df.columns]
    if not cell_cols:
        print("WARNING: no filtered cell voltage columns (cell_vf1..cell_vf7) found — skipping that chart.")

    curr_col = 'current_filtered_ma' if 'current_filtered_ma' in df.columns else None
    if curr_col is None:
        print("WARNING: no 'current_filtered_ma' column found — skipping that chart.")

    fet_state = get_fet_state(df)
    if fet_state is None:
        print("WARNING: no FET state column found — skipping that chart.")

    n_plots = sum([bool(cell_cols), curr_col is not None, fet_state is not None])
    if n_plots == 0:
        print("ERR: none of the expected columns were found in this file — nothing to plot.")
        sys.exit(1)

    fig, axes = plt.subplots(n_plots, 1, figsize=(12, 3.2 * n_plots), sharex=True)
    if n_plots == 1:
        axes = [axes]
    ax_idx = 0

    if cell_cols:
        ax = axes[ax_idx]; ax_idx += 1
        for i, col in enumerate(cell_cols, start=1):
            ax.plot(t, df[col], label=f"Cell {i}", linewidth=1)
        ax.set_ylabel("Filtered voltage (mV)")
        ax.set_title("Filtered Cell Voltages")
        ax.legend(loc='upper right', ncol=min(len(cell_cols), 7), fontsize=8)
        ax.grid(True, alpha=0.3)

    if curr_col is not None:
        ax = axes[ax_idx]; ax_idx += 1
        ax.plot(t, df[curr_col], color='tab:orange', linewidth=1)
        ax.axhline(0, color='gray', linewidth=0.5)
        ax.set_ylabel("Filtered current (mA)")
        ax.set_title("Filtered Pack Current")
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

    axes[-1].set_xlabel(t_label)
    fig.suptitle(f"OpenBMS Preview — {path}", fontsize=13, fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()


if __name__ == "__main__":
    main()
