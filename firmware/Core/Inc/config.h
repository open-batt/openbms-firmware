#ifndef CONFIG_H
#define CONFIG_H

// The project-wide settings in one place: build options, the limits of the system,
// the EEPROM layout, the HPPC test and the timeouts. The tuning of the algorithms
// (fuel gauge filter, HPPC parameter fit) stays next to the code that uses it.

// -----------------------------------------------------------------------------
// Build options
// -----------------------------------------------------------------------------

// Simulation build. The host (python_scripts/simulation/hppc_simulate.py, fuelgauge_simulate.py)
// simulates the battery and the load, and sends the cell voltages, pack current, temperature
// and time step with CC_SIM_SAMPLE. Each sample runs Ctrl_Run once on the simulated time, so the
// HPPC test, the parameter extraction, the fuel gauge and the protections all run on the host's data.
// The FET pins always stay off, fet_status still shows what the firmware asked for.
// #define OPENBMS_SIMULATION

// -----------------------------------------------------------------------------
// System
// -----------------------------------------------------------------------------

#define SYS_CELL_COUNT_MIN                   (2)             // lowest supported number of series cells
#define SYS_CELL_COUNT_MAX                   (7)             // highest supported number of series cells (ADC channels), sets every per-cell array
#define SYS_LOOP_PERIOD_MS                   (10UL)          // main loop period: Ctrl_Run / Comm_Run run once per TIM7 tick (100 Hz)
#define SYS_SOC_GRID_POINTS                  (20)            // points of the SOC grid and of every fuel gauge table

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------

#define PER_CELL_VOLTAGE_RESISTANCE_FACTOR   29.3333333f     // cell voltage divider: cell mV = ADC mV x this factor

// -----------------------------------------------------------------------------
// EEPROM layout
// -----------------------------------------------------------------------------

// Config_Data_t is stored here: a version byte, the config, then its CRC-32 (see Periph_EepromWriteChecked)
#define CTRL_CONFIG_EEPROM_ADDRESS               0x0000U
#define CTRL_CONFIG_EEPROM_VERSION               1U              // kept at 1, the config is written again after a layout change

// -----------------------------------------------------------------------------
// HPPC test (the same values as battery_cycler.py)
// -----------------------------------------------------------------------------

#define HPPC_REST_MS                        450000UL        // rest before every pulse, HPPC_REST_S
#define HPPC_DETECT_MS                      500UL           // FET on this long to see which way the current flows
#define HPPC_DETECT_MIN_MA                  50.0f           // less than this and nothing is connected
#define HPPC_CURRENT_MIN_C                  0.5f            // ramp current range in C of cell_capacity
#define HPPC_CURRENT_MAX_C                  2.0f
#define HPPC_CURRENT_FAULT_MS               2000UL          // out of range this long stops the test, covers the charger or load start
#define HPPC_MAX_DT_MS                      1000UL          // longest time step counted in one pass (e.g. after a debugger halt)
#define HPPC_TEMP_MIN_C                     15.0f           // pack temperature window of the test: outside it the test is not started
#define HPPC_TEMP_MAX_C                     35.0f
#define HPPC_TEMP_OUT_MS                    300000UL        // outside the window this long stops a running test

// -----------------------------------------------------------------------------
// Timeouts
// -----------------------------------------------------------------------------

#define PER_ADS_DATA_TIMEOUT_MS             (100U)          // no good ADS131M08 sample for this long: measurement lost (~25 samples)
#define COMM_TX_TIMEOUT_MS                  50U             // a reply still sending after this is aborted
#define COMM_I2C_TRANSFER_TIMEOUT_MS        100U            // an I2C transfer with no STOP after this restarts the I2C (a full frame is ~25 ms at 100 kHz)

// -----------------------------------------------------------------------------
// CAN (11-bit standard IDs, values little-endian)
// -----------------------------------------------------------------------------

#define COMM_CAN_ID_PACK                    0x6B0U          // uint32 pack voltage mV, int32 pack current mA (+ charge)
#define COMM_CAN_ID_CELLS_LOW               0x6B1U          // uint16 cell 1..4 mV
#define COMM_CAN_ID_CELLS_HIGH              0x6B2U          // uint16 cell 5..7 mV
#define COMM_CAN_ID_SOC_TEMP                0x6B3U          // uint16 SOC 0.1 %, int16 temperature 0.1 degC
#define COMM_CAN_ID_STATE                   0x6B4U          // uint8 ctrl_state, uint16 fet_status, uint8 protection_trigger, uint32 error_flags low 32 bits
#define COMM_CAN_ID_REPLY                   0x6BEU          // uint8 command, uint8 result (CE_*)
#define COMM_CAN_ID_COMMAND                 0x6BFU          // uint8 command (CC_*), then its arguments
#define COMM_CAN_BROADCAST_PERIOD_MS        100U            // status messages are sent this often while the broadcast is on
#define COMM_CAN_BROADCAST_DEFAULT          true            // broadcast state after boot, CC_CAN_BROADCAST changes it until the next reboot
#define COMM_CAN_RESTART_MS                 1000U           // a stopped CAN is restarted at most this often

#endif
