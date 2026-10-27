#ifndef OPENBMS_DATA_H
#define OPENBMS_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "config.h"

// SYS_CELL_COUNT_MIN / MAX, SYS_LOOP_PERIOD_MS and SYS_SOC_GRID_POINTS are in config.h

// Time base of the control code (HPPC, fuel gauge, protections, uptime). In the simulation
// build it is the simulated time from the CC_SIM_SAMPLE time steps, otherwise the HAL tick.
#ifdef OPENBMS_SIMULATION
uint32_t Ctrl_SimGetTick(void);
#define SYS_GET_TICK()                       Ctrl_SimGetTick()
#else
#define SYS_GET_TICK()                       HAL_GetTick()
#endif

// -------------------------------------------------------
// 0x10 — MainControl register bit definitions (read-only, the mode is switched by CC_NORMAL_RUN / CC_HPPC_START)
// -------------------------------------------------------
#define CTRL_MODE_NORMAL             (0x00)          // Normal mode
#define CTRL_MODE_HPPC               (0x01)          // HPPC run

// System_Data_t.ctrl_state - device lifecycle, set by the control module
#define CTRL_STATE_WAIT_CONFIG           (1)             // no valid saved config - waiting for the host to set the device up and save
#define CTRL_STATE_NORMAL_RUN            (2)             // configured - normal operation
#define CTRL_STATE_HPPC_RUN              (3)             // configured - HPPC operation, kept after the test ends until a reboot

// protection_trigger bits (register 0x12): 0 - nothing tripped, every set bit is a protection that tripped
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

// -------------------------------------------------------
// 0x09 — FETStatus register bit definitions (read-only)
// -------------------------------------------------------
#define PER_FET_MAIN                     (1 << 0)        // Bit 0     — main FETs state
#define PER_FET_PRE                      (1 << 1)        // Bit 1     — pre-FET state
#define PER_FET_BAL_SHIFT                (2)             // Bits 8:2  — balancer FET of cell 1 to 7 (bit 2 = cell 1)

// -----------------------------------------------------------------------------
// Config_Data_t — settings the user chooses. Written by the host in any state and
// saved to EEPROM with CC_EEPROM_CONFIG. Nothing in here is updated by the device itself.
// -----------------------------------------------------------------------------
typedef struct __attribute__((packed))
{
    // -------------------------------------------------------------------------
    // Configuration registers
    // -------------------------------------------------------------------------

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

    // -------------------------------------------------------------------------
    // Voltage protection configuration (per-cell)
    // -------------------------------------------------------------------------

    uint16_t uvp_slow_threshold_mv;                    // slow UVP threshold in mV
    uint16_t uvp_slow_time_ms;                         // slow UVP detection time in ms
    uint16_t uvp_fast_threshold_mv;                    // fast UVP threshold in mV
    uint16_t uvp_fast_time_ms;                         // fast UVP detection time in ms
    uint16_t ovp_slow_threshold_mv;                    // slow OVP threshold in mV
    uint16_t ovp_slow_time_ms;                         // slow OVP detection time in ms
    uint16_t ovp_fast_threshold_mv;                    // fast OVP threshold in mV
    uint16_t ovp_fast_time_ms;                         // fast OVP detection time in ms

    // -------------------------------------------------------------------------
    // Current protection configuration
    // -------------------------------------------------------------------------

    uint16_t ocp_charge_threshold_ma;                  // charge OCP threshold in mA
    uint16_t ocp_charge_time_ms;                       // charge OCP detection time in ms
    uint16_t ocp_discharge_slow_threshold_ma;          // slow discharge OCP threshold in mA
    uint16_t ocp_discharge_slow_time_ms;               // slow discharge OCP detection time in ms
    uint16_t ocp_discharge_fast_threshold_ma;          // fast discharge OCP threshold in mA
    uint16_t ocp_discharge_fast_time_ms;               // fast discharge OCP detection time in ms

    // -------------------------------------------------------------------------
    // Temperature protection configuration
    // -------------------------------------------------------------------------

    uint16_t otp_threshold_c;                          // OTP threshold in °C
    uint16_t otp_time_ms;                              // OTP detection time in ms

    // -------------------------------------------------------------------------
    // Charge limits, charge temperature limits and OCV window
    // -------------------------------------------------------------------------

    uint16_t charge_monitor_cell_mv;                   // start checking the termination current above this cell voltage in mV
    int16_t  utp_threshold_c;                          // under-temperature threshold in °C
    uint16_t utp_time_ms;                              // UTP detection time in ms
    int16_t  otp_charge_threshold_c;                   // over-temperature threshold while charging in °C (time: otp_time_ms)
    int16_t  utp_charge_threshold_c;                   // under-temperature threshold while charging in °C (time: utp_time_ms)
    uint16_t ocv_v_lo_mv;                              // cell voltage treated as 0 % SOC in mV
    uint16_t ocv_v_hi_mv;                              // cell voltage treated as 100 % SOC in mV

    // -------------------------------------------------------------------------
    // Cell balancing (passive, only while charging)
    // -------------------------------------------------------------------------

    bool     balance_enable;                           // cell balancing enabled
    uint16_t balance_start_mv;                         // balancing starts when the highest cell is above this in mV
    uint16_t balance_delta_mv;                         // a cell bleeds when it is this far above the lowest cell in mV
    int16_t  balance_max_temp_c;                       // no balancing at or above this pack temperature in °C

    // -------------------------------------------------------------------------
    // Sensor calibration (the ADC offsets are measured at every boot inside the peripheral
    // code; the user offsets below are added on top of them)
    // -------------------------------------------------------------------------

    float    current_sensor_gain;                      // current sensor gain calibration
    float    voltage_gain[SYS_CELL_COUNT_MAX];                          // per-cell ADC gain calibration
    float    ntc_beta;                                 // NTC Beta value from datasheet (e.g. 3950.0)
    float    ntc_r_nominal;                            // NTC nominal resistance at 25°C in Ohms
    float    ntc_r_fixed;                              // Fixed resistor value in Ohms (e.g. 10000.0)
    float    ntc_t_nominal;                            // NTC nominal temperature in Kelvin (e.g. 298.15 = 25°C)
    float    temperature_offset;                       // temperature sensor offset in °C
    float    shunt_resistance_mohm;                    // current shunt resistance in mOhm
    float    current_offset_ma;                        // user current offset added to every reading in mA
    float    voltage_offset_mv[SYS_CELL_COUNT_MAX];                     // per-cell user voltage offset added to every reading in mV
    float    balance_resistance_ohm;                   // balance resistor across each cell in Ohm (50 on RevB), for the fuel gauge

    // -------------------------------------------------------------------------
    // Device information — written by the host and saved with the config
    // -------------------------------------------------------------------------

    char     firmware_version[32];                     // firmware version string — ASCII null-terminated
    char     hardware_version[32];                     // hardware version string — ASCII null-terminated
    char     manufacturer_name[32];                    // "OpenBatt Team" — ASCII null-terminated
    char     device_name[32];                          // "OpenBMS" — ASCII null-terminated
    char     device_chemistry[32];                     // "Li-Ion" — ASCII null-terminated
    char     manufacturer_data[32];                    // "Year 2026" — ASCII null-terminated

} Config_Data_t;

// -----------------------------------------------------------------------------
// System_Data_t — runtime state of the device. Lives in RAM only and is never
// saved, so the device always boots into normal mode with no trip flags set.
// -----------------------------------------------------------------------------
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
    uint8_t  hppc_direction;                           // HPPC_DIR_* — detected by the board, never written by the host
    uint16_t hppc_status;                              // low byte HPPC_STATE_*, high byte stop reason
    uint16_t hppc_extract_status;                      // HX_STATUS_* bits

} System_Data_t;

// -----------------------------------------------------------------------------
// Stats_Data_t — the fault snapshot. Written only by the device and read-only
// to the host. Nothing persists yet, so it restarts from zero at boot for now.
// -----------------------------------------------------------------------------
typedef struct __attribute__((packed))
{
    // -------------------------------------------------------------------------
    // Fault registers
    // -------------------------------------------------------------------------

    // Taken at the first protection trip and kept until the host clears it (CC_CLEAR_FAULT);
    // later trips do not overwrite it. fault_code 0 - no fault stored.
    uint16_t fault_snapshot_voltage[SYS_CELL_COUNT_MAX];                // per-cell voltage at the fault in mV
    int16_t  fault_snapshot_current;                   // current at the fault in mA (+ charge)
    int8_t   fault_snapshot_temperature;               // temperature at the fault in °C
    uint8_t  fault_snapshot_soc;                       // pack SoC (relative_soc) at the fault in %
    uint16_t fault_code;                               // protection_trigger at the fault, CTRL_PROT_* bits
    uint32_t fault_timestamp;                          // session_uptime_ms at the fault

} Stats_Data_t;

// One cell's battery model: OCV and the 2-RC parameters on the shared SOC grid
// for both directions, plus its capacity. Every cell has its own copy, so a cell
// that is weaker or has a different OCV curve is described by its own numbers.
typedef struct __attribute__((packed, aligned(4)))
{
    float       ocv_dis[SYS_SOC_GRID_POINTS];                              // OCV discharge curve in V
    float       ocv_chg[SYS_SOC_GRID_POINTS];                              // OCV charge curve in V
    float       r0_dis[SYS_SOC_GRID_POINTS];                               // R0 discharge in Ω
    float       r1_dis[SYS_SOC_GRID_POINTS];                               // R1 discharge in Ω
    float       tau1_dis[SYS_SOC_GRID_POINTS];                             // tau1 discharge in s
    float       r2_dis[SYS_SOC_GRID_POINTS];                               // R2 discharge in Ω
    float       tau2_dis[SYS_SOC_GRID_POINTS];                             // tau2 discharge in s
    float       r0_chg[SYS_SOC_GRID_POINTS];                               // R0 charge in Ω
    float       r1_chg[SYS_SOC_GRID_POINTS];                               // R1 charge in Ω
    float       tau1_chg[SYS_SOC_GRID_POINTS];                             // tau1 charge in s
    float       r2_chg[SYS_SOC_GRID_POINTS];                               // R2 charge in Ω
    float       tau2_chg[SYS_SOC_GRID_POINTS];                             // tau2 charge in s
    float       capacity_ah;                              // capacity of this cell in Ah

} FuelGauge_Cell_t;

// The fuel gauge estimator takes float pointers into this struct and
// dereferences them directly, so every float member has to sit on a 4-byte
// boundary — on Cortex-M4 an unaligned VLDR raises a UsageFault rather than
// just being slow. Packing alone gives the struct an alignment of 1 and would
// land soc_grid on offset 29, so the reserved bytes below realign the float
// block and aligned(4) pins the struct itself. openbms_fuelgauge.c holds this
// with static assertions; adding a field above soc_grid without adjusting the
// padding will fail the build rather than fault on hardware.
typedef struct __attribute__((packed, aligned(4)))
{
    // -------------------------------------------------------------------------
    // Fuel gauge registers
    // ------------------------------------------------------------------------

    uint8_t     relative_soc;                             // % of FullChargeCapacity
    uint8_t     cell_soc[SYS_CELL_COUNT_MAX];                              // per-cell SoC in %
    uint8_t     cell_soh[SYS_CELL_COUNT_MAX];                              // per-cell SoH in %
    uint16_t    cell_remaining_capacity[SYS_CELL_COUNT_MAX];               // per-cell remaining capacity in mAh

    uint8_t     reserved[3];                              // padding — keeps the float block below 4-byte aligned

    float       soc_grid[SYS_SOC_GRID_POINTS];                             // SOC setpoints in %
    FuelGauge_Cell_t cell[SYS_CELL_COUNT_MAX];                    // per-cell model: tables and capacity
    float       q_temp_c[5];                              // temperature setpoints in °C, rising
    float       q_temp_factor[5];                         // usable capacity at each temperature / capacity at the model temperature: scales remaining capacity only
    float       q_nom_ah;                                 // pack capacity in Ah: the HPPC discharge test, then the weakest cell's learned capacity
    float       coulombic_efficiency;                     // typically 0.995 to 0.999
    float       ea_r0;                                    // activation energy for R0 in J/mol
    float       ea_r1;                                    // activation energy for R1 in J/mol
    float       ea_tau1;                                  // activation energy for tau1 in J/mol
    float       ea_r2;                                    // activation energy for R2 in J/mol
    float       ea_tau2;                                  // activation energy for tau2 in J/mol
    float       model_temp_dis_c;                         // temperature the discharge tables were measured at in °C (HPPC average): R and tau scale from it with ea_*
    float       model_temp_chg_c;                         // the same for the charge tables
    float       kf_q_soc;                                 // process noise — SOC state
    float       kf_q_rc1;                                 // process noise — V_RC1 state
    float       kf_q_rc2;                                 // process noise — V_RC2 state
    float       kf_r_v;                                   // measurement noise — voltage sensor V²
    float       cell_soc_f[SYS_CELL_COUNT_MAX];                            // last estimated SOC per cell (0.0 to 1.0)
    float       cell_soc_shown_f[SYS_CELL_COUNT_MAX];                      // shown SOC per cell (0.0 to 1.0): follows cell_soc_f, a rest correction is spread over the next charge/discharge
    float       cell_vrc1[SYS_CELL_COUNT_MAX];                             // last estimated V_RC1 per cell in V
    float       cell_vrc2[SYS_CELL_COUNT_MAX];                             // last estimated V_RC2 per cell in V
    float       cell_p[SYS_CELL_COUNT_MAX][6];                             // full covariance upper triangle per cell
    uint16_t    cycle_count;                              // full cycles: discharged charge / q_nom_ah (RAM only for now)
    uint16_t    learning_status;                          // FG_LEARN_* bits: tables per direction (HPPC or host), fuel gauge runs only with FG_LEARN_COMPLETE
    uint16_t    capacity_update_count;                    // times the learned capacity changed (RAM only for now), cleared with CC_CLEAR_LEARNING

} FuelGauge_Data_t;

typedef struct
{
    float       cell_voltage[SYS_CELL_COUNT_MAX];                        // per-cell voltage in mV
    float       pack_voltage;                           // pack voltage in mV
    float       pack_current;                           // instantaneous current in mA
    float       temperature_package;                    // NTC battery temperature °C
    float       temperature_stm32;                      // Temperature read out from STM32 in °C
    float       main_vdd_voltage_mv;                    // VDD voltage in the system (usually 3300mV)

    uint16_t    fet_status;                             // Bit 8:2: balancer FETs, Bit 1: pre-FET state, Bit 0: main FETs state

} Peripheral_Data_t;

#ifdef __cplusplus
}
#endif

#endif