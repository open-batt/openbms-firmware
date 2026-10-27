# 🛠️ Python Scripts

Quick reference for every script in `python_scripts/` — what each one does, what it expects as input, and what it produces. For the theory behind the numbers these scripts work with, see [battery-model.md](battery-model.md); for the wire protocol they all speak to the board, see [communication-protocol.md](communication-protocol.md).

## Requirements

Python 3. Every script installs the modules it is missing on its first run (`pyserial`, and `matplotlib` for the simulation scripts). To install them by hand:

```
pip install pyserial matplotlib
```

Every script that talks to the board takes an optional `COM_PORT`. Without it, the script pings every serial port and uses the one where an OpenBMS answers.

## Which script do I run?

| I want to... | Script |
|---|---|
| Flash new firmware onto the board | `flash.py` |
| Send a command or check the status by hand | `bms_cmd.py` |
| Start, stop and watch the HPPC test on a real pack, with a log | `hppc_run.py` |
| Sanity-check every register at once | `read_all_registers.py` |
| Write the config to the board, or read it back | `config_tool.py` |
| Test the HPPC test and parameter extraction | `simulation/hppc_simulate.py` (simulation build only*) |
| Test the SOC estimation and its corrections | `simulation/soc_simulate.py` (simulation build only*) |
| Test the protections | `simulation/protection_simulate.py` (simulation build only*) |
| Test cell balancing | `simulation/balance_simulate.py` (simulation build only*) |

\* Works only with firmware built with `OPENBMS_SIMULATION` defined (uncomment `#define OPENBMS_SIMULATION` in `firmware/Core/Inc/config.h`), see [Simulation scripts](#simulation-scripts). A normal build answers these scripts with `CE_WRONG_CMD`.

## flash.py

Flashes a compiled `.hex` file onto the board over UART, driving the bootloader's line-based ASCII protocol described in [communication-protocol.md](communication-protocol.md#bootloader-entry) end to end: ping, erase, program line by line, verify by CRC-32/MPEG-2, set the application flag, reset.

| | |
|---|---|
| **Input** | Optional COM port and Intel HEX file. Without a file, the newest `.hex` in `firmware/build/Release` is used |
| **Output** | Nothing saved — progress and pass/fail print to the console at each step |

```
python flash.py [COM_PORT] [HEX_FILE]
python flash.py COM3 firmware.hex
```

If the board is running the application, it is sent into the bootloader first. Any failed step (erase, a program line, CRC mismatch) aborts with a non-zero exit code.

Flashing does not touch the config saved in the EEPROM. If a new firmware changes the config layout, the board refuses the old config and waits in WAIT_CONFIG; write the config again with `config_tool.py w`.

## bms_cmd.py

Sends commands to the board by typing them, either at an interactive `bms>` prompt or as one command on the command line.

| | |
|---|---|
| **Input** | Optional COM port, optional command |
| **Output** | The reply printed to the console — nothing saved |

```
python bms_cmd.py [COM_PORT]              interactive
python bms_cmd.py [COM_PORT] <command>    one command, then exit
python bms_cmd.py fet on
```

| Command | Effect |
|---------|--------|
| `ping` | Check the board answers |
| `reset` | Reboot the STM32 |
| `fet on` / `fet off` | Main FET (normal run only) |
| `prefet on` / `prefet off` | Pre-charge FET (normal run only) |
| `hppc start` | Start an HPPC test (normal run only) |
| `hppc stop` | Stop a running HPPC test |
| `save` | Save the config to the EEPROM |
| `reset protection` | Clear a protection trip |
| `clear errors` | Clear the error flags |
| `clear learning` | Clear the learning status bits |
| `clear fault` | Clear the fault snapshot (`0x70`–`0x75`) |
| `status` | Read voltages, current, temperatures, FET, states and errors |
| `read <addr>` | Read one register as raw bytes, e.g. `read 0x13` |
| `help` | List the commands |
| `exit` / `quit` | Leave |

## hppc_run.py

Starts, stops and watches the on-board HPPC test on a real pack, and logs it. Normal firmware build only: with `OPENBMS_SIMULATION` the board's uptime does not move and the script stops.

| | |
|---|---|
| **Input** | Optional COM port and a mode, or `plot` with a log file |
| **Output** | `python_scripts/logs/hppc_<date>_<time>.csv`, a new file per run, one row per second |

```
python hppc_run.py [COM_PORT] monitor
python hppc_run.py [COM_PORT] start
python hppc_run.py [COM_PORT] stop
python hppc_run.py plot <csv_log>
```

| Mode | Effect |
|------|--------|
| `monitor` | Watch the board, change nothing |
| `start` | Check the board is in NORMAL_RUN with no protection trip, no error flags and the pack temperature within 15–35 °C. If not, say why and don't start. Otherwise count down 5 s (`q` cancels), start the test, then watch |
| `stop` | Stop a running HPPC test, then watch. The FET stays off until a reboot |
| `plot` | Plot a log: cell voltages, pack current, temperature and the HPPC state, with a red line where the test stopped. No board needed |

While watching, one live line shows the HPPC state, direction, pack voltage, current, temperature, every cell voltage and `OK` or `FAIL`. New lines are printed for HPPC state changes, the end of the test with its stop reason, and failures: protection trips, new error flags, an aborted test, extraction warnings and the board not answering. Press `q` to leave; this never changes the HPPC test, a running test keeps running.

The extracted model is not read out; it stays in the board's RAM until EEPROM storage for it exists.

## read_all_registers.py

Reads and prints every register in the UART register map once — peripheral, system, config, fault snapshot and fuel gauge. The per-cell model registers are read for every cell by stepping `model_cell` (`0x9F`). A quick "is the board alive and does its configuration look right" check, not a logger.

| | |
|---|---|
| **Input** | Optional COM port |
| **Output** | A printed table (Address / Register / Value / Unit) plus an error count — nothing saved |

```
python read_all_registers.py [COM_PORT]
python read_all_registers.py COM3
```

The register lists in the script follow the register map in [communication-protocol.md](communication-protocol.md#register-map); if the map changes, the lists need updating to match.

## Simulation scripts

The scripts in `python_scripts/simulation/` test the firmware without a real battery. The board runs a **simulation build**: uncomment `#define OPENBMS_SIMULATION` in `firmware/Core/Inc/config.h`, build and flash. The script then simulates the battery (cells with a 2-RC model, charger or load, temperature) on the PC and sends every sample to the board with `CC_SIM_SAMPLE` (see [communication-protocol.md](communication-protocol.md#simulation-sample-cc_sim_sample)). The board runs its real control loop, protections, HPPC test, fuel gauge and balancing on these samples, and the script checks what the board does. The FET pins stay off in this build, so no current flows on the board.

`config_tool.py` sits in `python_scripts/` because a real board needs it too; it finds the test folders in `simulation/` by name.

Everything for one battery lives in a **test folder**, e.g. `simulation/7_cell_2000mAh/`:

| File | Content |
|---|---|
| `config.json` | The board config (keys are the `Config_Data_t` field names), written by `config_tool.py w` |
| `hppc.json` | The simulated battery and settings for `hppc_simulate.py` |
| `fuel_gauge_registers.json` | The battery model (fuel gauge registers), written by `hppc_simulate.py` and loaded by the other scripts |
| `soc.json` | The SOC tests |
| `protection.json` | The protection tests |
| `balance.json` | The balancing tests |
| `log_*.csv` | The log of the last run of each script |

`<test_folder>` can be a path or just the name of a folder next to the scripts. Usual order:

```
python config_tool.py w 7_cell_2000mAh
python hppc_simulate.py 7_cell_2000mAh
python soc_simulate.py 7_cell_2000mAh
python protection_simulate.py 7_cell_2000mAh
python balance_simulate.py 7_cell_2000mAh
```

Each test script reboots the board before every test, shows one live line updated in place plus a line for every event, and ends with a PASS/FAIL summary. Press `q` to stop (on Windows without Enter, elsewhere `q` then Enter). With `plot` instead of a COM port, no test runs and no board is needed: the log of the last run is plotted, one window per test.

### Profiles

The SOC, protection and balancing tests describe the current as a list of segments. Current sign is the firmware's: + charge, − discharge. Every segment may also carry `temperature_c` and `note`.

| Segment | Keys |
|---|---|
| `rest` | `duration_s` |
| `constant` | `current_ma`, `duration_s` |
| `resistor` | `load_ohm`, `duration_s` — a load resistor across the pack |
| `pulse` | `high_ma`, `low_ma`, `period_s`, `duty`, `duration_s` |
| `sine` | `offset_ma`, `amplitude_ma`, `period_s`, `duration_s` |
| `random` | `min_ma`, `max_ma`, `hold_s`, `duration_s` |
| `cccv` | `current_ma`, `cell_voltage_mv`, `end_current_ma`, `duration_s` — constant current until the highest cell reaches `cell_voltage_mv`, then held there until the current falls to `end_current_ma` |
| `reset_protection` | Sends `CC_RESET_PROTECTION` (protection tests) |

## config_tool.py

Reads or writes the board config over UART.

| | |
|---|---|
| **Input** | A mode, a test folder for `w`, optional COM port |
| **Output** | Console only |

```
python config_tool.py r [COM_PORT]
python config_tool.py w <test_folder> [COM_PORT]
python config_tool.py p [COM_PORT]
```

| Mode | Effect |
|------|--------|
| `r` | Read the whole config and print it |
| `w` | Write every value from `<test_folder>/config.json` (settings and device info strings), save to the EEPROM, reboot, then read everything back and check it against the file |
| `p` | Ping five times and print the raw bytes sent and received with timing — for finding connection problems. Without a COM port, on every serial port |

`w` is also how a real board gets its config: a new board waits in WAIT_CONFIG until a config is written and saved.

## hppc_simulate.py

Runs the on-board HPPC test against a simulated battery, both directions, and saves the extracted model.

```
python hppc_simulate.py <test_folder> [c|d] [COM_PORT]
python hppc_simulate.py <test_folder> plot [c|d] [run]
```

`c` runs only the charge direction, `d` only the discharge direction; without it, both run. Without arguments the test folders are listed.

For each direction it runs five tests, each after a reboot and from a fresh simulated battery:

| Run | Must end with |
|---|---|
| `high_current` | Stop reason current range (current above 2 C) |
| `low_current` | Stop reason current range (current below 0.5 C) |
| `high_temperature` | Stop reason temperature, after 5 min outside 15–35 °C |
| `low_temperature` | Stop reason temperature |
| `good_run` | DONE, with tables written. The tables are compared with the simulated cells |

The failure runs start their fault `faults.start_after_s` into the test. When the good runs write tables, the registers are saved to `fuel_gauge_registers.json`: charge tables and `model_temp_chg_c` from the charge run, everything else from the discharge run. A single-direction run updates only its own part of the file. Log: `log_hppc.csv`, one row per real second.

**`hppc.json`:**

| Key | Meaning |
|---|---|
| `cell_count` | Number of simulated cells, must match the board |
| `dt_ms` | Simulated time step per sample |
| `temperature_c` | Pack temperature |
| `max_sim_time_s` | A good run is stopped after this much simulated time |
| `start_soc_pct` | `{"charge": .., "discharge": ..}` — SOC the cells start each run at |
| `discharge` | `load_ohm` — load resistor across the pack |
| `charge` | `current_ma`, `voltage_limit_mv` (pack) — the charger |
| `faults` | `start_after_s`, `timeout_s`, `high_current_c`, `low_current_c`, `high_temperature_c`, `low_temperature_c` |
| `noise` | `voltage_mv`, `current_ma`, `seed` — gaussian noise |
| `cell_default` | The model of one cell: `capacity_mah`, OCV formula coefficients, and per 2 % SOC `r0_mohm`, `r1_mohm`, `tau1_s`, `r2_mohm`, `tau2_s` |
| `cells` | One entry per cell; any key given there replaces `cell_default` for that cell |

## soc_simulate.py

Tests the fuel gauge: SOC estimation, the rest and full-charge corrections, capacity learning and the temperature model.

```
python soc_simulate.py <test_folder> [test_name] [COM_PORT]
python soc_simulate.py <test_folder> plot [test_name]
```

For every test the script writes the battery model from `fuel_gauge_registers.json` to the board, so the board and the simulated cells use the same model. It then runs the profile and reads the board's internal SOC, shown SOC, remaining capacity, capacity, SOH, capacity update count and cycle count every `read_every_s`. Log: `log_soc.csv`.

Tests in `soc.json`: `discharge`, `charge`, `high_current_discharge`, `rest_correction`, `full_charge`, `capacity_learning`, `temperature`, `temperature_off`.

**`soc.json`:** defaults (`dt_ms`, `temperature_c`, `read_every_s`, `cell_min_mv`, `cell_max_mv`, `repeat`, `noise`) that each test can override, and `tests`, each with:

| Key | Meaning |
|---|---|
| `branch` | OCV branch the battery starts on, `dis` or `chg` |
| `start_soc_pct` | True SOC of every cell at the start |
| `soc_offset_pct` | After the seed, the true SOC is moved by this much, so the board starts off by that amount |
| `capacity_scale` | Real cell capacity / the board's `capacity_ah`, e.g. 0.95 |
| `voltage_offset_mv` | Added to every cell voltage sent to the board (a measurement error) |
| `board_temperature_model` | `false` writes `ea_*` = 0 to the board; the simulated cells still follow the temperature |
| `repeat` | How many times the profile runs |
| `profile` | Segments, see [Profiles](#profiles) |
| `expect` | Pass conditions, all optional: `max_error_pct`, `max_final_error_pct`, `max_mean_flow_error_pct`, `min_mean_flow_error_pct`, `min_rest_corrections`, `min_full_charges`, `min_capacity_updates`, `max_final_capacity_error_pct`, `final_shown_pct`, `remaining_factor` |

Errors are board internal SOC − true SOC, worst cell, in % SOC.

## protection_simulate.py

Tests every protection: the expected one, and only that one, must trip in the segment marked `expect_trip`, and after `reset_protection` the protection must be cleared and the FET back on. The simulated battery follows the board's FET: no current while the FET is off.

```
python protection_simulate.py <test_folder> [test_name] [COM_PORT]
python protection_simulate.py <test_folder> plot [test_name]
```

Tests in `protection.json`: `ovp_slow`, `ovp_fast`, `uvp_slow`, `uvp_fast`, `ocp_charge`, `ocp_discharge_slow`, `ocp_discharge_fast`, `otp`, `utp`, `otp_charge`, `utp_charge`.

**`protection.json`:** defaults (`dt_ms`, `temperature_c`, `cell_min_mv`, `cell_max_mv`, `noise`) and `tests`, each with `expect` (the protection that must trip, e.g. `OCP_CHARGE`), `branch`, `start_soc_pct` and `profile`. Log: `log_protection.csv`, one row per sample.

## balance_simulate.py

Tests cell balancing. The balancing bits in `fet_status` of every sample reply say which cells bleed; such a simulated cell then carries the pack current minus its bleed current (V / `balance_resistance_ohm`). The board must have `balance_enable` on.

```
python balance_simulate.py <test_folder> [test_name] [COM_PORT]
python balance_simulate.py <test_folder> plot [test_name]
```

In every test, no cell may bleed while the pack is not charging, at or above `balance_max_temp_c`, or while a protection is tripped.

Tests in `balance.json`: `one_cell_high`, `two_cells_high`, `balanced_pack`, `below_start`, `discharge_and_rest`, `hot_and_trip`, `balance_capacity`.

**`balance.json`:** defaults (`dt_ms`, `temperature_c`, `read_every_s`, `cell_min_mv`, `cell_max_mv`, `noise`) and `tests`, each with:

| Key | Meaning |
|---|---|
| `branch`, `start_soc_pct` | As in `soc.json` |
| `cell_soc_offset_pct` | One value per cell, added to `start_soc_pct` (the imbalance) |
| `profile` | Segments, see [Profiles](#profiles). In `cccv` the bleed current of the cell at the limit is added, so the charger keeps it there |
| `expect` | All optional: `cells` (cells allowed to bleed, `[]` = none), `min_balance_s`, `max_final_soc_spread_pct`, `max_error_pct`, `trip`, `min_capacity_updates`, `max_capacity_error_pct` |

Log: `log_balance.csv`.

## logs and results

`python_scripts/logs/` holds real charge, discharge and HPPC logs recorded on the bench, and `python_scripts/results/` the parameter plots used in [battery-model.md](battery-model.md#example-estimating-a-7-cell-battery). The scripts that made them have been replaced by the on-board HPPC test and fuel gauge.
