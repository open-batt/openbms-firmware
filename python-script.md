# 🛠️ Python Scripts

Quick reference for every script in `python_scripts/` — what each one does, what it expects as input, and what it produces. For the theory behind the numbers these scripts work with, see [battery-model.md](battery-model.md); for the wire protocol they all speak to the board, see [communication-protocol.md](communication-protocol.md).

## Requirements

```
pip install pyserial numpy pandas scipy matplotlib
```

`flash.py`, `read_all_registers.py`, and `battery_cycler.py` only need `pyserial` (`battery_cycler.py` also needs `msvcrt`, which ships with Windows Python — it won't run on Linux/macOS). The rest are pure Python and don't touch a serial port at all.

## Which script do I run?

| I want to... | Script |
|---|---|
| Flash new firmware onto the board | `flash.py` |
| Sanity-check every register at once | `read_all_registers.py` |
| Run a charge / discharge / HPPC test on real hardware | `battery_cycler.py` |
| Eyeball a CSV log before analyzing it | `preview_logs.py` |
| Extract HPPC parameters from a log | `hppc_pipeline.py` |
| Turn those parameters into Kalman tuning | `kalman_soc_estimator.py` |
| Test the estimator against a real log | `soc_estimator.py` |

## flash.py

Flashes a compiled `.hex` file onto the board over UART, driving the bootloader's line-based ASCII protocol described in [communication-protocol.md](communication-protocol.md#bootloader-entry) end to end: ping, erase, program line by line, verify by CRC-32/MPEG-2, set the application flag, reset.

| | |
|---|---|
| **Input** | A COM port and an Intel HEX file (the STM32CubeIDE build output) |
| **Output** | Nothing saved — progress and pass/fail print to the console at each step |

```
python flash.py <COM_PORT> <HEX_FILE>
python flash.py COM3 firmware.hex
```

If the board is already running the application rather than the bootloader, the script waits for it to reset into the bootloader on its own before continuing. Any failed step (erase, a program line, CRC mismatch) aborts with a non-zero exit code rather than continuing on a device in an unknown state.

## read_all_registers.py

Reads and prints every register in the UART register map — peripheral, control, and fuel-gauge banks — once. A quick "is the board alive and does its configuration look right" check, not a logger.

| | |
|---|---|
| **Input** | A COM port |
| **Output** | A printed table (Address / Register / Value / Unit) plus an error count — nothing saved to disk |

```
python read_all_registers.py <COM_PORT>
python read_all_registers.py COM3
```

The register list is kept in sync with the register map in [communication-protocol.md](communication-protocol.md#register-map) — if that changes, this script's three register lists need updating to match.

## battery_cycler.py

The live serial test harness — the only script here that actually drives the board through a real charge/discharge/HPPC test, as opposed to `read_all_registers.py`'s one-shot check or `flash.py`'s bootloader upload. Runs one of monitor, continuous charge/discharge, pulsed charge/discharge, or a full HPPC checkpoint run, logging every 20Hz read to a timestamped CSV and rendering a live terminal dashboard at 5Hz. This is what produces the logs that `preview_logs.py`, `hppc_pipeline.py`, and `soc_estimator.py` all consume.

| | |
|---|---|
| **Input** | A mode code and a COM port |
| **Output** | `openbms_<mode>_<timestamp>.csv` in the current directory, plus a live terminal dashboard — nothing else saved |

```
python battery_cycler.py <MODE> <COM_PORT>
python battery_cycler.py DHPPC COM3
```

| Mode | Effect |
|------|--------|
| `M` | Monitor — read-only, 20Hz dashboard/log, never sends a mode or FET command |
| `C` / `D` | Continuous charge / discharge, FET held on until a stop condition |
| `CP` / `DP` | Same as `C`/`D` but pulsed 2s ON / 2s OFF |
| `CHPPC` / `DHPPC` | HPPC run: alternates 450s rests with coulomb-counted ramps through the checkpoint list, then an open-ended ramp to the safety stop |

Charging stops at a hard per-cell ceiling or on taper current once past a monitor threshold; discharging stops at the device's `voltage_cell_min` register — see [communication-protocol.md](communication-protocol.md#register-map) for those registers. Hitting a stop condition never ends the script: it disables the FET and keeps logging/displaying passively until you press `q`, so the CSV always ends cleanly, and the FET is guaranteed disabled on any exit path. Windows-only (uses `msvcrt` for keypress detection).

## preview_logs.py

Offline chart viewer for any OpenBMS CSV log — continuous, pulse, or HPPC. Only looks for the columns it needs and skips a chart if that column isn't in the file, so it works across log formats without any flags telling it what kind of log it's looking at.

| | |
|---|---|
| **Input** | A CSV log path, plus optional chart selectors |
| **Output** | A live matplotlib window — nothing saved |

| Selector | Chart |
|----------|-------|
| `pv` | Pack voltage (V), filtered — falls back to raw |
| `cv` | Filtered per-cell voltages (V) |
| `pc` | Pack current (A), filtered — falls back to raw |
| `fet` | Main FET state (ON/OFF) |
| `t` | Temperature (°C) |

```
python preview_logs.py <csv_log_file> [pv] [cv] [pc] [fet] [t]
python preview_logs.py openbms_charging_20260706_142233.csv cv fet
```

If no selectors are given, the default is `pv pc`.

## hppc_pipeline.py

Stage 1 of the parameter-extraction pipeline. Turns a raw HPPC log into per-cell OCV, R0, R1, τ1, R2, τ2 and capacity, fitted from the relaxation curve after every rest and resampled onto a fixed SOC grid. Charge and discharge are analyzed separately since the extracted parameters differ by direction — see [battery-model.md](battery-model.md#example-of-battery-extracted-parameters).

| | |
|---|---|
| **Input** | A direction flag (`C` or `D`) and one or more HPPC CSV logs (multiple files are time-concatenated in the order given) |
| **Output** | `hppc_soc_table_all_cells_{charge\|discharge}.csv` plus per-cell diagnostic PNGs, written to a `results` folder next to the script (created if missing, existing files from other runs kept) |

```
python hppc_pipeline.py C c_hppc_full.csv
python hppc_pipeline.py D d_hppc_1.csv
python hppc_pipeline.py C part1.csv part2.csv part3.csv
```

| Flag | Effect |
|------|--------|
| `--outdir <dir>` | Output folder; default is the `results` folder next to the script |
| `--detailed` | Also write the per-cell breakdown CSVs (`cellN_hppc_params`, `cellN_cycle_diagnostics`, `summary`, `cell_capacities`) |
| `--curves` | Also write `cellN_ocv_soc_curve.csv`, the fine-step OCV-SOC curve |
| `--no-plots` | Skip the per-cell diagnostic PNGs |

Findings print to the console as `INFO` / `WARNING` / `ERROR`, ending in a colored `RESULT: SUCCESS` or `RESULT: FAILED` line — read the errors before trusting a failed run's output.

## kalman_soc_estimator.py

Stage 2. Takes the charge and discharge tables from stage 1 and pairs each cell's HPPC-derived capacity with the Kalman filter's tuning constants — no current/voltage log needed for this step, since it's just preparing configuration, not running the filter. See [battery-model.md](battery-model.md#filter-tuning-parameters) for what these numbers mean.

| | |
|---|---|
| **Input** | `--charge-table` and `--discharge-table` (both required) |
| **Output** | `kalman_parameters.csv` (one row per cell) and `kalman_parameters_log.txt`, written to a `results` folder next to the script |

```
python kalman_soc_estimator.py --charge-table results\hppc_soc_table_all_cells_charge.csv --discharge-table results\hppc_soc_table_all_cells_discharge.csv
```

Same `INFO` / `WARNING` / `ERROR` console convention as `hppc_pipeline.py`, including the final colored `RESULT` line.

## soc_estimator.py

Stage 3. Runs the actual 2-RC model plus Extended Kalman Filter against one real current/voltage log and previews the result live — real vs. open-loop vs. Kalman-corrected voltage, the two SOC traces, and the pack current that drove them, one plot window per cell. Nothing is saved; this script is for looking at one run, not for keeping records. See [battery-model.md](battery-model.md#example-of-soc-estimation) for example output.

| | |
|---|---|
| **Input** | A log file path (the only required argument); `--charge-table`, `--discharge-table` and `--parameters` are optional and default to the `results\` files the earlier two stages produce |
| **Output** | A live matplotlib window per cell plus a console RMSE table — nothing saved to disk |

```
python soc_estimator.py <log.csv>
python soc_estimator.py logs\d_partial_1.csv
python soc_estimator.py logs\d_partial_1.csv --charge-table other\charge.csv
```

Includes innovation gating — corrections that are statistically implausible or imply an unrealistically fast SOC change are scaled back or blocked, and SOC is never corrected while the pack is at rest. A cell that triggers this a lot (flagged as a `WARNING` naming the cell) is worth a second look on the hardware side.
