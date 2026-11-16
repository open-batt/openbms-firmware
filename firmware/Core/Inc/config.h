#ifndef CONFIG_H
#define CONFIG_H

// #define OPENBMS_TEST                                     // test build: adds CC_SOC_RESET (0x13), CC_ADS_STANDBY (0x17), CC_MEAS_OFF (0x18)

#define SYS_CELL_COUNT_MIN                   (2)             // lowest supported number of series cells
#define SYS_CELL_COUNT_MAX                   (7)             // highest supported number of series cells (ADC channels), sets every per-cell array
#define SYS_LOOP_PERIOD_MS                   (10UL)          // main loop period: Ctrl_Run / Comm_Run run once per TIM7 tick (100 Hz)
#define SYS_SOC_GRID_POINTS                  (20)            // points of the SOC grid and of every fuel gauge table

#define PER_CELL_DIVIDER_OHM                 70000.0f        // each cell tap's divider to battery minus (68k + 2k): drains the cells while cell measuring is on
#define PER_CELL_VOLTAGE_RESISTANCE_FACTOR   35.0f           // cell voltage divider (RevB 68k / 2k, 0.1 %): cell mV = ADC mV x (68 + 2) / 2
#define PER_CHG_PRES_DIVIDER_FACTOR          11.0f           // CHG_PRES (PA3, ADC1 IN8) divider from SYS_VCC (RevB 100k / 10k): SYS_VCC mV = ADC mV x (100 + 10) / 10
#define CTRL_CHARGER_PRESENT_MV              7000.0f         // SYS_VCC above this sets charger_present (0x1C)

#define EEPROM_SIZE                              32768U          // M24256 (RevB), 256 kbit
#define EEPROM_PAGE_SIZE                         64U             // a page write must not cross a page boundary

#define EEPROM_CONFIG_ADDRESS                    0x0000U         // version byte, Config_Data_t, CRC-32
#define EEPROM_CONFIG_VERSION                    1U              // kept at 1, the config is written again after a layout change

#define EEPROM_FUELGAUGE_MODEL_ADDRESS           0x0400U         // FuelGauge_Data_t (model of every cell), CRC-32

#define EEPROM_FUELGAUGE_STATE_ADDRESS           0x3000U         // FuelGauge_State_t, two slots written in turn
#define EEPROM_FUELGAUGE_STATE_SLOT_SIZE         256U            // 4 pages: one slot, the state with its CRC

#define HPPC_REST_MS                        450000UL        // rest before every pulse
#define HPPC_SETTLE_MIN_MS                  120000UL        // rest before the test (FET off): at least this long, so two windows can be compared
#define HPPC_SETTLE_MAX_MS                  1800000UL       // ... and at most this long, then the test goes on even if the cells still move
#define HPPC_SETTLE_WINDOW_MS               60000UL         // cell voltages averaged over windows this long
#define HPPC_SETTLE_DVDT_MAX_V_PER_S        4e-6f           // relaxed once two windows in a row differ by less than this on every cell
#define HPPC_DETECT_MS                      2000UL          // FET on this long to see which way the current flows
#define HPPC_DETECT_MIN_MA                  50.0f           // less than this and nothing is connected
#define HPPC_CURRENT_MIN_C                  0.5f            // ramp current range in C of cell_capacity
#define HPPC_CURRENT_MAX_C                  2.0f
#define HPPC_CURRENT_FAULT_MS               2000UL          // out of range this long stops the test, covers the charger or load start
#define HPPC_MAX_DT_MS                      1000UL          // longest time step counted in one pass (e.g. after a debugger halt)
#define HPPC_TEMP_MIN_C                     15.0f           // pack temperature window of the test: outside it the test is not started
#define HPPC_TEMP_MAX_C                     35.0f
#define HPPC_TEMP_OUT_MS                    300000UL        // outside the window this long stops a running test
#define HPPC_DIS_START_MIN_MV               4100.0f         // discharge test: every cell at or above this at the start (pack full)
#define HPPC_CHG_START_MAX_MV               3450.0f         // charge test: every cell at or below this at the start (pack empty)

#define SYS_DEVICE_STRING_SIZE              32U             // every device string register: up to 31 ASCII characters, null-padded
#define SYS_FIRMWARE_VERSION                "1.0.0"
#define SYS_HARDWARE_VERSION                "rev B"
#define SYS_MANUFACTURER_NAME               "OpenBatt Team"
#define SYS_DEVICE_NAME                     "OpenBMS"
#define SYS_MANUFACTURER_DATA               "Year 2026"

#define PER_ADS_DATA_TIMEOUT_MS             (100U)          // no good ADS131M08 sample for this long: measurement lost (~25 samples)
#define COMM_TX_TIMEOUT_MS                  50U             // a reply still sending after this is aborted
#define COMM_I2C_TRANSFER_TIMEOUT_MS        100U            // an I2C transfer with no STOP after this restarts the I2C (a full frame is ~25 ms at 100 kHz)

#define COMM_CAN_ID_PACK                    0x6B0U          // uint32 pack voltage mV, int32 pack current mA (+ charge)
#define COMM_CAN_ID_CELLS_LOW               0x6B1U          // uint16 cell 1..4 mV
#define COMM_CAN_ID_CELLS_HIGH              0x6B2U          // uint16 cell 5..7 mV
#define COMM_CAN_ID_SOC_TEMP                0x6B3U          // uint16 SOC 0.1 %, int16 temperature 0.1 degC, uint8 battery_status, uint8 charger_present
#define COMM_CAN_ID_STATE                   0x6B4U          // uint8 ctrl_state, uint16 fet_status, uint16 protection_trigger, error_flags low 24 bits
#define COMM_CAN_ID_REPLY                   0x6BEU          // uint8 command, uint8 result (CE_*)
#define COMM_CAN_ID_COMMAND                 0x6BFU          // uint8 command (CC_*), then its arguments
#define COMM_CAN_BROADCAST_PERIOD_MS        100U            // status messages are sent this often while the broadcast is on
#define COMM_CAN_BROADCAST_DEFAULT          true            // broadcast state after boot, CC_CAN_BROADCAST changes it until the next reboot
#define COMM_CAN_RESTART_MS                 1000U           // a stopped CAN is restarted at most this often

#endif
