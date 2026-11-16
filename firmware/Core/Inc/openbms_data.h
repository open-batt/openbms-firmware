#ifndef OPENBMS_DATA_H
#define OPENBMS_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "config.h"

#define CTRL_MODE_NORMAL             (0x00)          // Normal mode
#define CTRL_MODE_HPPC               (0x01)          // HPPC run

#define CTRL_STATE_STARTUP               (0)             // boot check of the ADS and the cells
#define CTRL_STATE_NORMAL_RUN            (1)             // normal operation: protections, FET and balancing with a loaded config, the fuel gauge with a loaded model
#define CTRL_STATE_HPPC_RUN              (2)             // HPPC operation (needs a config), kept after the test ends until a reboot
#define CTRL_STATE_FAULT                 (3)             // boot check failed or an ERROR_FAULT_FLAGS error, outputs off until reboot
#define CTRL_STATE_SHUTDOWN              (4)             // CC_SHUTDOWN: everything off, fuel gauge state saved, then PWR_ON low (power stays while WAKE_UP is high)
#define CTRL_STATE_RESET                 (5)             // CC_REBOOT / CC_BOOTLOADER: everything off, state saved, then reset

#define CTRL_PROT_OVP_SLOW               (1U << 0)       // cell over-voltage, slow
#define CTRL_PROT_OVP_FAST               (1U << 1)       // cell over-voltage, fast
#define CTRL_PROT_UVP_SLOW               (1U << 2)       // cell under-voltage, slow
#define CTRL_PROT_UVP_FAST               (1U << 3)       // cell under-voltage, fast
#define CTRL_PROT_OCP_CHARGE             (1U << 4)       // charge over-current
#define CTRL_PROT_OCP_DISCHARGE_SLOW     (1U << 5)       // discharge over-current, slow
#define CTRL_PROT_OCP_DISCHARGE_FAST     (1U << 6)       // discharge over-current, fast
#define CTRL_PROT_OTP                    (1U << 7)       // over-temperature
#define CTRL_PROT_UTP                    (1U << 8)       // under-temperature
#define CTRL_PROT_OTP_CHARGE             (1U << 9)       // over-temperature while charging
#define CTRL_PROT_UTP_CHARGE             (1U << 10)      // under-temperature while charging

#define CTRL_BATT_FULL                   (1U << 0)       // fully charged: top cell >= charge_monitor_cell_mv, charge current <= charging_term_current
#define CTRL_BATT_EMPTY                  (1U << 1)       // fully discharged: a cell <= voltage_cell_min

#define PER_FET_MAIN                     (1 << 0)        // Bit 0     - main FETs state
#define PER_FET_PRE                      (1 << 1)        // Bit 1     - pre-FET state
#define PER_FET_BAL_SHIFT                (2)             // Bits 8:2  - balancer FET of cell 1 to 7 (bit 2 = cell 1)

typedef struct __attribute__((packed))
{
    uint8_t  cell_count;                               // number of series cells, SYS_CELL_COUNT_MIN to SYS_CELL_COUNT_MAX
    bool     use_i2c;                                  // I2C interface enabled
    bool     use_can;                                  // CAN interface enabled
    bool     use_uart;                                 // UART interface enabled
    bool     volt_protection;                          // voltage protection enabled
    bool     current_protection;                       // current protection enabled
    bool     temp_protection;                          // temperature protection enabled
    uint32_t cell_capacity;                            // factory capacity in mAh
    uint16_t voltage_cell_max;                         // max designed cell voltage in mV, HPPC charge stops here
    uint16_t voltage_cell_min;                         // min designed cell voltage in mV, HPPC discharge stops here
    uint16_t charging_term_current;                    // Termination current in mA

    uint16_t uvp_slow_threshold_mv;                    // slow UVP threshold in mV
    uint16_t uvp_slow_time_ms;                         // slow UVP detection time in ms
    uint16_t uvp_fast_threshold_mv;                    // fast UVP threshold in mV
    uint16_t uvp_fast_time_ms;                         // fast UVP detection time in ms
    uint16_t ovp_slow_threshold_mv;                    // slow OVP threshold in mV
    uint16_t ovp_slow_time_ms;                         // slow OVP detection time in ms
    uint16_t ovp_fast_threshold_mv;                    // fast OVP threshold in mV
    uint16_t ovp_fast_time_ms;                         // fast OVP detection time in ms

    uint16_t ocp_charge_threshold_ma;                  // charge OCP threshold in mA
    uint16_t ocp_charge_time_ms;                       // charge OCP detection time in ms
    uint16_t ocp_discharge_slow_threshold_ma;          // slow discharge OCP threshold in mA
    uint16_t ocp_discharge_slow_time_ms;               // slow discharge OCP detection time in ms
    uint16_t ocp_discharge_fast_threshold_ma;          // fast discharge OCP threshold in mA
    uint16_t ocp_discharge_fast_time_ms;               // fast discharge OCP detection time in ms

    uint16_t otp_threshold_c;                          // OTP threshold in degC
    uint16_t otp_time_ms;                              // OTP detection time in ms

    uint16_t charge_monitor_cell_mv;                   // start checking the termination current above this cell voltage in mV
    int16_t  utp_threshold_c;                          // under-temperature threshold in degC
    uint16_t utp_time_ms;                              // UTP detection time in ms
    int16_t  otp_charge_threshold_c;                   // over-temperature threshold while charging in degC (time: otp_time_ms)
    int16_t  utp_charge_threshold_c;                   // under-temperature threshold while charging in degC (time: utp_time_ms)
    uint16_t ocv_v_lo_mv;                              // cell voltage treated as 0 % SOC in mV
    uint16_t ocv_v_hi_mv;                              // cell voltage treated as 100 % SOC in mV

    bool     balance_enable;                           // cell balancing enabled
    uint16_t balance_start_mv;                         // balancing starts when the highest cell is above this in mV
    uint16_t balance_delta_mv;                         // a cell bleeds when it is this far above the lowest cell in mV
    int16_t  balance_max_temp_c;                       // no balancing at or above this pack temperature in degC
    bool     balance_rest_enable;                      // also balance at rest (|current| <= 50 mA), not only while charging
    uint16_t balance_stop_mv;                          // a bleeding cell stops when it is this close to the lowest cell in mV (below balance_delta_mv)

    float    current_sensor_gain;                      // current sensor gain calibration
    float    voltage_gain[SYS_CELL_COUNT_MAX];                          // per-cell ADC gain calibration
    float    ntc_beta;                                 // NTC Beta value from datasheet (e.g. 3950.0)
    float    ntc_r_nominal;                            // NTC nominal resistance at 25 degC in Ohm
    float    ntc_r_fixed;                              // Fixed resistor value in Ohm (e.g. 10000.0)
    float    ntc_t_nominal;                            // NTC nominal temperature in Kelvin (e.g. 298.15 = 25 degC)
    float    temperature_offset;                       // temperature sensor offset in degC
    float    shunt_resistance_mohm;                    // current shunt resistance in mOhm
    float    current_offset_ma;                        // user current offset added to every reading in mA
    float    voltage_offset_mv[SYS_CELL_COUNT_MAX];                     // per-cell user voltage offset added to every reading in mV
    float    balance_resistance_ohm;                   // balance resistor across each cell in Ohm (50 on RevB), for the fuel gauge

    char     device_chemistry[SYS_DEVICE_STRING_SIZE];   // "Li-Ion" - ASCII null-terminated

} Config_Data_t;

typedef struct __attribute__((packed))
{
    uint16_t main_control;                             // Bit 0: 0 - normal run, 1 - HPPC run (read-only)
    bool     fet_request;                              // FET wanted by the host (CC_SET_FET), used in normal run only
    bool     pre_fet_request;                          // pre-FET wanted by the host (CC_SET_PRE_FET), used in normal run only
    uint32_t last_communication_timestamp;             // timestamp of last host communication in ms
    uint16_t protection_trigger;                       // CTRL_PROT_* bits of the protections that tripped, 0 - none
    uint8_t  ctrl_state;                               // CTRL_STATE_*
    uint64_t error_flags;                              // bit n = OpenBMS_Error_t n: latched errors and live hardware pin bits
    uint32_t session_uptime_ms;                        // uptime since this boot in ms
    uint8_t  hppc_direction;                           // HPPC_DIR_* - detected by the board, never written by the host
    uint16_t hppc_status;                              // low byte HPPC_STATE_*, high byte stop reason
    uint16_t hppc_extract_status;                      // HX_STATUS_* bits
    uint8_t  battery_status;                           // CTRL_BATT_* bits: fully charged / fully discharged
    uint8_t  charger_present;                          // 1 - output voltage SYS_VCC above CTRL_CHARGER_PRESENT_MV, 0 - not
    uint8_t  eeprom_status;                            // EEPROM_STATUS_* (openbms_eeprom.h): what was loaded at boot, the last config / model save

} System_Data_t;

typedef struct __attribute__((packed))
{
    uint16_t fault_snapshot_voltage[SYS_CELL_COUNT_MAX];                // per-cell voltage at the fault in mV
    int16_t  fault_snapshot_current;                   // current at the fault in mA (+ charge)
    int8_t   fault_snapshot_temperature;               // temperature at the fault in degC
    uint8_t  fault_snapshot_soc;                       // pack SoC (relative_soc) at the fault in %
    uint16_t fault_code;                               // protection_trigger at the fault, CTRL_PROT_* bits
    uint32_t fault_timestamp;                          // session_uptime_ms at the fault

} Stats_Data_t;

typedef struct __attribute__((packed, aligned(4)))
{
    float       ocv_dis[SYS_SOC_GRID_POINTS];                              // OCV discharge curve in V
    float       ocv_chg[SYS_SOC_GRID_POINTS];                              // OCV charge curve in V
    float       r0_dis[SYS_SOC_GRID_POINTS];                               // R0 discharge in Ohm
    float       r1_dis[SYS_SOC_GRID_POINTS];                               // R1 discharge in Ohm
    float       tau1_dis[SYS_SOC_GRID_POINTS];                             // tau1 discharge in s
    float       r2_dis[SYS_SOC_GRID_POINTS];                               // R2 discharge in Ohm
    float       tau2_dis[SYS_SOC_GRID_POINTS];                             // tau2 discharge in s
    float       r0_chg[SYS_SOC_GRID_POINTS];                               // R0 charge in Ohm
    float       r1_chg[SYS_SOC_GRID_POINTS];                               // R1 charge in Ohm
    float       tau1_chg[SYS_SOC_GRID_POINTS];                             // tau1 charge in s
    float       r2_chg[SYS_SOC_GRID_POINTS];                               // R2 charge in Ohm
    float       tau2_chg[SYS_SOC_GRID_POINTS];                             // tau2 charge in s
    float       capacity_ah;                              // capacity of this cell in Ah

} FuelGauge_Cell_t;

typedef struct __attribute__((packed, aligned(4)))
{
    uint8_t     relative_soc;                             // % of FullChargeCapacity
    uint8_t     cell_soc[SYS_CELL_COUNT_MAX];                              // per-cell SoC in %
    uint8_t     cell_soh[SYS_CELL_COUNT_MAX];                              // per-cell SoH in %
    uint16_t    cell_remaining_capacity[SYS_CELL_COUNT_MAX];               // per-cell remaining capacity in mAh

    uint8_t     reserved[3];                              // padding - keeps the float block below 4-byte aligned

    float       soc_grid[SYS_SOC_GRID_POINTS];                             // SOC setpoints in %
    FuelGauge_Cell_t cell[SYS_CELL_COUNT_MAX];                    // per-cell model: tables and capacity
    float       q_temp_c[5];                              // temperature setpoints in degC, rising
    float       q_temp_factor[5];                         // usable capacity at each temperature / capacity at the model temperature: scales remaining capacity only
    float       q_nom_ah;                                 // pack capacity in Ah: the HPPC discharge test, then the weakest cell's learned capacity
    float       coulombic_efficiency;                     // typically 0.995 to 0.999
    float       ea_r0;                                    // activation energy for R0 in J/mol
    float       ea_r1;                                    // activation energy for R1 in J/mol
    float       ea_tau1;                                  // activation energy for tau1 in J/mol
    float       ea_r2;                                    // activation energy for R2 in J/mol
    float       ea_tau2;                                  // activation energy for tau2 in J/mol
    float       model_temp_dis_c;                         // temperature the discharge tables were measured at in degC (HPPC average): R and tau scale from it with ea_*
    float       model_temp_chg_c;                         // the same for the charge tables
    float       kf_r_v;                                   // measurement noise - voltage sensor V^2
    float       cell_soc_f[SYS_CELL_COUNT_MAX];                            // last estimated SOC per cell (0.0 to 1.0)
    float       cell_soc_shown_f[SYS_CELL_COUNT_MAX];                      // shown SOC per cell (0.0 to 1.0), corrections spread over the next charge/discharge
    float       cell_vrc1[SYS_CELL_COUNT_MAX];                             // last estimated V_RC1 per cell in V
    float       cell_vrc2[SYS_CELL_COUNT_MAX];                             // last estimated V_RC2 per cell in V
    float       cell_p[SYS_CELL_COUNT_MAX][6];                             // full covariance upper triangle per cell
    uint16_t    cycle_count;                              // full cycles: discharged charge / q_nom_ah, saved with the fuel gauge state
    uint16_t    learning_status;                          // FG_LEARN_* bits: tables per direction (HPPC or host), fuel gauge runs only with FG_LEARN_COMPLETE
    uint16_t    capacity_update_count;                    // times the learned capacity changed, saved with the fuel gauge state

} FuelGauge_Data_t;

typedef struct __attribute__((packed))
{
    float    capacity_ah[SYS_CELL_COUNT_MAX];                          // learned capacity per cell in Ah
    float    q_nom_ah;                                                 // pack capacity in Ah (the weakest cell)
    float    cell_soc_f[SYS_CELL_COUNT_MAX];                           // SOC per cell (0.0 to 1.0)
    float    cell_soc_shown_f[SYS_CELL_COUNT_MAX];                     // shown SOC per cell (0.0 to 1.0)
    uint16_t cycle_count;                                              // full cycles
    uint16_t capacity_update_count;                                    // times the learned capacity changed
    uint8_t  cell_soh[SYS_CELL_COUNT_MAX];                             // SOH per cell in %
    float    bal_target_ah[SYS_CELL_COUNT_MAX];                        // SOC balancing: charge each cell still has to bleed, Ah
    uint8_t  last_charge;                                              // direction of the last current: 1 charge, 0 discharge, 2 not known (OCV table at the next rest)
    float    cycle_ah;                                                 // discharged charge toward the next full cycle, Ah
} FuelGauge_State_t;

typedef struct __attribute__((packed))
{
    uint8_t        version;
    Config_Data_t  config;
} Config_Stored_t;

typedef struct __attribute__((packed))
{
    uint32_t           model_crc;
    uint32_t           sequence;
    FuelGauge_State_t  state;
} FuelGauge_StateStored_t;

typedef struct
{
    float       cell_voltage[SYS_CELL_COUNT_MAX];                        // per-cell voltage in mV (held while the cell's balancer bleeds)
    float       cell_voltage_live[SYS_CELL_COUNT_MAX];                   // per-cell voltage in mV, never held (~10 mV low while bleeding): protections only
    float       pack_voltage;                           // pack voltage in mV
    float       pack_current;                           // instantaneous current in mA
    float       temperature_package;                    // NTC battery temperature degC
    float       temperature_stm32;                      // Temperature read out from STM32 in degC
    float       main_vdd_voltage_mv;                    // VDD voltage in the system (usually 3300mV)
    float       sys_vcc_mv;                             // output terminal voltage SYS_VCC in mV, from CHG_PRES

    uint16_t    fet_status;                             // Bit 8:2: balancer FETs, Bit 1: pre-FET state, Bit 0: main FETs state

} Peripheral_Data_t;

#ifdef __cplusplus
}
#endif

#endif
