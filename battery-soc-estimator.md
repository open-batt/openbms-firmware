# 🔬 Battery SOC Estimator

Toolchain that turns a real battery pack into a set of model parameters the firmware can use, and then checks how well those parameters actually predict the pack's behaviour.

For the theory behind the 2-RC model and what each parameter means, see [battery-model.md](battery-model.md). For the UART protocol these scripts speak, see [communication-protocol.md](communication-protocol.md).

## How the whole thing works

Parameter extraction happens **offline on a PC**, not on the STM32. The board is used as a data logger and a switch: the host tells it when to connect the load/charger and reads voltages and current back at 20 Hz. Everything else — fitting, curve building, filter validation — runs in Python.

```
  ┌──────────────┐   UART 230400   ┌──────────────┐
  │  OpenBMS     │◄───────────────►│   test.py    │  Stage 1 — log
  │  board       │  read registers │              │
  │  + pack      │  FET on/off     └──────┬───────┘
  └──────────────┘                        │  openbms_*hppc_*.csv
                                          ▼
                                  ┌────────────────────┐
                                  │ hppc_pipeline.py   │  Stage 2 — extract
                                  └────────┬───────────┘
                                           │  hppc_soc_table_all_cells_{charge,discharge}.csv
                                           ▼
                                  ┌──────────────────────────┐
                                  │ kalman_soc_estimator.py  │  Stage 3 — validate
                                  └────────┬─────────────────┘
                                           │  RMSE per cell + PNG plots
                                           ▼
                                    parameters → firmware registers 0x86–0x95
```

| Stage | Script | Runs | Input | Output |
|-------|--------|------|-------|--------|
| 1 — Log | `test.py` | on hardware | live pack | CSV log at 20 Hz |
| 2 — Extract | `hppc_pipeline.py` | offline | HPPC CSV log(s) | OCV/SOC + 2-RC parameter table, diagnostic plots |
| 3 — Validate | `kalman_soc_estimator.py` | offline | parameter tables + a normal CSV log | open-loop vs EKF voltage RMSE, plots |

Stage 2 must be run **twice** — once on a charge HPPC log and once on a discharge HPPC log — because the OCV curve and the impedances differ measurably between the two directions. Stage 3 needs both tables and picks the charge or discharge branch at each timestep from the sign of the current.

## Requirements

```
pip install pyserial numpy pandas scipy matplotlib
```

`test.py` and `monitor.py` use `msvcrt` for keypress handling and are **Windows-only**. `hppc_pipeline.py`, `kalman_soc_estimator.py` and `preview.py` are pure Python and run anywhere.

## Stage 1 — Logging data from the device

`test.py` drives the board over UART. It requests **learning mode** (the only mode in which the firmware accepts FET commands), then switches the main FETs on and off according to the selected test profile while logging every register it needs at 20 Hz.

```
python test.py <MODE> <COM_PORT>
```

| Mode | Name | What it does |
|------|------|--------------|
| `M` | monitor | Read-only. No mode change, no FET commands. Safe first check. |
| `C` | charging | Continuous charge until a stop condition |
| `D` | discharging | Continuous discharge until a stop condition |
| `CP` | charging_pulse | Pulsed charge, 2 s on / 2 s off |
| `DP` | discharging_pulse | Pulsed discharge, 2 s on / 2 s off |
| `CHPPC` | charging_hppc | **Charge HPPC test** — feeds Stage 2 |
| `DHPPC` | discharging_hppc | **Discharge HPPC test** — feeds Stage 2 |

Logs are written to `openbms_<mode>_<YYYYmmdd_HHMMSS>.csv` in the working directory. Press `q` to stop cleanly — the script disables the FETs on the way out.

### What an HPPC run does

The board is walked through 20 SOC checkpoints, resting at each one long enough for the cell to relax:

```
REST 450s at 100%  →  RAMP to 98%  →  REST 450s  →  RAMP to 95%  →  REST 450s  →  ...  →  REST at 2%
```

Checkpoints are `100, 98, 95, 90, 85, 80, 70, 60, 50, 40, 30, 25, 20, 15, 12, 10, 8, 6, 4, 2` % — run downward for `DHPPC`, upward for `CHPPC`. Each RAMP is **coulomb-counted**: the FET stays on until `CellCapacity() × Δ% / 100` mAh has passed, then off for the rest. The 450 s rest is what makes the fit possible — the relaxation curve after each pulse is where R1, τ1, R2 and τ2 come from, so cutting the rest short costs you the slow time constant.

A full HPPC run is long: 20 checkpoints × 450 s of rest alone is ~2.5 hours before any ramp time. Runs can be split across several files and re-joined in Stage 2.

### Safety limits enforced by the host

These are host-side limits in `test.py`, on top of whatever the firmware's own protections do:

| Limit | Value | Applies to |
|-------|-------|------------|
| Hard cell ceiling | 4300 mV filtered — stops immediately | `C`, `CP`, `CHPPC` |
| Termination-current window | Only evaluated once a cell reaches 4100 mV | `C`, `CP`, `CHPPC` |
| Termination current | `ChargingTerminationCurrent()` (`0x35`) | `C`, `CP`, `CHPPC` |
| Low cell cutoff | `MinCellVoltage()` (`0x34`) | `D`, `DP`, `DHPPC` |

Whatever ended the run is written into the `stop_reason` column, so Stage 2 can tell a clean finish from an early cutoff.

### Log format

Both log formats share the same core columns: `elapsed_s`, `uptime_ms`, `cell_v1`–`cell_v7` (raw mV), `cell_vf1`–`cell_vf7` (filtered mV), `pack_voltage_mv`, `pack_voltage_filtered_mv`, `current_ma`, `current_filtered_ma`, `temperature_c`, `fet_status`, `fet_main_enabled`, `commanded_fet_state`, `learning_status`, `stop_reason`.

HPPC logs add three columns that Stage 2 depends on:

| Column | Meaning |
|--------|---------|
| `segment_type` | `REST` or `RAMP` — this is what the cycle detector segments on |
| `target_pct` | The checkpoint this segment is heading to or resting at |
| `segment_coulomb_mah` | Accumulated charge for the active RAMP segment |

`preview.py <log.csv>` gives a quick look at any log — filtered cell voltages, current and FET state against time — before committing to a full analysis.

## Stage 2 — Parameter extraction

`hppc_pipeline.py` turns an HPPC log into the parameter tables. Per cell, it:

1. **Loads and validates** the log — repairs NaNs, checks time monotonicity, current sign convention, pack-vs-cell-sum consistency, idle current bias, and whether the voltage range actually covered the cell.
2. **Detects cycles** from `segment_type`, pairing each RAMP with the REST that follows it. A trailing RAMP with no rest is skipped and reported.
3. **Fits a 2-RC relaxation model** to every rest: R0 comes from the instantaneous jump a few samples into the pulse, and R1/τ1/R2/τ2 from a bounded least-squares fit of `OCV + V₀·(a₁·e^(−t/τ₁) + a₂·e^(−t/τ₂))` to the relaxation curve. Good fits are typically under 1 mV RMSE; anything over 20 mV is flagged.
4. **Builds the OCV-vs-capacity curve** from the rest end-points and extends it out to 2.50 V and 4.20 V — PCHIP inside the measured data, slope-matched power-law tails outside it. Extrapolating a long way is unreliable, so a gap larger than 0.15 V to either bound is **flagged rather than silently reported**.
5. **Resamples** OCV and all five impedance parameters onto the fixed SOC grid.

SOC is derived from coulomb counting plus the OCV extension model only. The log's own `target_pct` is never used to compute SOC — it is carried through as a cross-check and flagged if it disagrees by more than 5 %.

```
python hppc_pipeline.py C c_hppc_full.csv                    # analyze as a charge test
python hppc_pipeline.py D d_hppc_1.csv                       # analyze as a discharge test
python hppc_pipeline.py C part1.csv part2.csv part3.csv      # split run, time-concatenated in order
python hppc_pipeline.py D d_hppc_full.csv --outdir results   # choose an output folder
```

| Flag | Effect |
|------|--------|
| `--outdir <dir>` | Output folder; default is the folder the script lives in |
| `--detailed` | Also write the per-cell CSVs — `cellN_hppc_params`, `cellN_cycle_diagnostics`, `summary`, `cell_capacities` |
| `--curves` | Also write `cellN_ocv_soc_curve.csv`, the fine-step OCV-SOC curve |
| `--no-plots` | Skip the per-cell diagnostic PNGs |

**Outputs** — everything suffixed `_charge` or `_discharge` so both directions coexist in one folder:

| File | Contents |
|------|----------|
| `hppc_soc_table_all_cells_{charge\|discharge}.csv` | The deliverable. One row per SOC checkpoint, columns grouped per cell: `Cell{N}_OCV_V`, `Cell{N}_Capacity_2v5_4v2_mAh`, `Cell{N}_R0_ohm`, `Cell{N}_R1_ohm`, `Cell{N}_R2_ohm`, `Cell{N}_tau1_s`, `Cell{N}_tau2_s` |
| `cell{N}_diagnostics_{charge\|discharge}.png` | Capacity, OCV-SOC and impedance plots per cell |

No log file is written — findings print to the console as `INFO` / `WARNING` (orange) / `ERROR` (red), and the last line is either `=== RESULT: SUCCESS ===` in green or `=== RESULT: FAILED ===` in red. A FAILED result means at least one finding likely invalidates part of the output; read the errors before using the table.

## Stage 3 — Validation with the Kalman filter

`kalman_soc_estimator.py` answers the question Stage 2 can't: *do these parameters actually predict this pack?* It replays a real log through the 2-RC model twice and compares both against the measured voltage.

- **Open loop** — pure prediction. Coulomb counting plus RC decay, with no correction from the measured voltage. This shows the raw quality of the parameter tables and how fast the estimate drifts.
- **Extended Kalman Filter** — the same model, but SOC, V_RC1 and V_RC2 are corrected against the measured voltage at every step. This is what the firmware will eventually do on-device.

At each timestep the charge or discharge branch is chosen from the sign of the current; a dead zone of ±50 mA holds the previous branch rather than flip-flopping on noise. Initial SOC is seeded by inverting the first measured voltage through the OCV table, and V_RC1 = V_RC2 = 0 — so a small settling transient at the very start of the Kalman trace is expected, not a bug.

```
python kalman_soc_estimator.py \
    --charge-table    hppc_soc_table_all_cells_charge.csv \
    --discharge-table hppc_soc_table_all_cells_discharge.csv \
    --data c_partical_1.csv \
    --data d_partial_2.csv
```

Repeat `--data` for each log you want to check; each is processed independently. Use a **normal charge/discharge log** here (`C`/`D`/`CP`/`DP`), not the HPPC log the tables were fitted from — testing against the fitting data tells you nothing about generalisation.

**Outputs:** `{log_stem}_cell{N}_kalman.png` per cell — measured vs open-loop vs Kalman voltage on top, the two SOC traces below — plus a per-cell RMSE table on the console:

```
 cell  rmse_openloop_mV  rmse_kalman_mV
    1              12.4             3.1
    ...
```

Open-loop RMSE tells you how good the parameters are. Kalman RMSE tells you how well the filter tracks with those parameters. If the Kalman trace over- or under-reacts to noise, tune `Q_PROC` and `R_MEAS` at the top of the script — they are sensible starting points, not fitted values, since this data has no independent ground-truth SOC to fit them against.

## End-to-end walkthrough

1. **Sanity check the link.** `python test.py M COM3` — confirm cell voltages, current and temperature look right before doing anything that moves charge.
2. **Run a discharge HPPC.** Charge the pack full, then `python test.py DHPPC COM3`. Expect several hours. Keep the resulting CSV.
3. **Run a charge HPPC.** From empty: `python test.py CHPPC COM3`. Keep that CSV too.
4. **Extract both directions.**
   ```
   python hppc_pipeline.py D openbms_discharging_hppc_<stamp>.csv
   python hppc_pipeline.py C openbms_charging_hppc_<stamp>.csv
   ```
   Read the console output. Fix and re-run before trusting a `FAILED` result. Check the per-cell diagnostic PNGs — a bad OCV tail is obvious on the plot.
5. **Record a separate validation log.** `python test.py D COM3` (or `C`/`CP`/`DP`) — something with realistic current, distinct from the HPPC logs.
6. **Validate.** Run `kalman_soc_estimator.py` with both tables and that log. Compare open-loop against Kalman RMSE and inspect the plots.
7. **Load the parameters onto the board** once you are satisfied — see below.

## Getting the parameters onto the device

The columns of `hppc_soc_table_all_cells_*.csv` map onto the fuel gauge registers described in [communication-protocol.md](communication-protocol.md). All are `float[20]` little-endian except where noted:

| Table column | Charge register | Discharge register |
|--------------|-----------------|--------------------|
| `SOC_pct` | `SOC_Grid()` `0x86` | *(shared)* |
| `Cell{N}_OCV_V` | `OCV_Charge()` `0x88` | `OCV_Discharge()` `0x87` |
| `Cell{N}_R0_ohm` | `R0_Charge()` `0x8E` | `R0_Discharge()` `0x89` |
| `Cell{N}_R1_ohm` | `R1_Charge()` `0x8F` | `R1_Discharge()` `0x8A` |
| `Cell{N}_tau1_s` | `Tau1_Charge()` `0x90` | `Tau1_Discharge()` `0x8B` |
| `Cell{N}_R2_ohm` | `R2_Charge()` `0x91` | `R2_Discharge()` `0x8C` |
| `Cell{N}_tau2_s` | `Tau2_Charge()` `0x92` | `Tau2_Discharge()` `0x8D` |
| `Cell{N}_Capacity_2v5_4v2_mAh` | `Q_Nom()` `0x95` (Ah, `float`) and `CellCapacity()` `0x32` (mAh, `uint32`) | *(shared)* |

Three things to be aware of before writing these:

- **The pipeline produces per-cell tables; the firmware holds one shared table.** The device has a single `float[20]` per parameter, not one per cell. Pick a representative cell or average across cells — per-cell divergence is visible in the diagnostic plots and is itself useful information.
- **The SOC grids don't currently match.** The pipeline emits 20 checkpoints from 2 % to 100 %; the firmware's default `SOC_Grid()` is 0 % to 95 % in 5 % steps. Write `SOC_Grid()` from the table's own `SOC_pct` column rather than assuming the default.
- **Writes don't stick yet.** As noted in [communication-protocol.md](communication-protocol.md), register writes are currently copied into a module-local snapshot in `openbms_comm.c` with no path back to the owning module and no EEPROM persistence. Until that is wired up, loading parameters onto the board has no effect — Stage 3 in Python is the only place these tables are exercised.
