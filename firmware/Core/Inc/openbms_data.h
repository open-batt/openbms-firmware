#ifndef OPENBMS_DATA_H
#define OPENBMS_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "config.h"

#define BD_CELL_COUNT_MIN                   (2)             // Lowest supported number of series cells
#define BD_CELL_COUNT_MAX                   (7)             // Highest supported number of series cells
#define BD_LOOP_PERIOD_MS                   (10UL)          // Main loop period: Ctrl_Run / Comm_Run run once per TIM7 tick (100 Hz)

// Time base of the control code (HPPC, fuel gauge, protections, uptime). In the simulation
// build it is the simulated time from the CC_SIM_SAMPLE time steps, otherwise the HAL tick.
#ifdef OPENBMS_SIMULATION
uint32_t Ctrl_SimGetTick(void);
#define BD_GET_TICK()                       Ctrl_SimGetTick()
#else
#define BD_GET_TICK()                       HAL_GetTick()
#endif

// -------------------------------------------------------
// 0x10 — MainControl register bit definitions (read-only, the mode is switched by CC_NORMAL_RUN / CC_HPPC_START)
// -------------------------------------------------------
#define BD_MAIN_CTR_MODE_NORMAL             (0x00)          // Normal mode
#define BD_MAIN_CTR_MODE_HPPC               (0x01)          // HPPC run

// System_Data_t.ctrl_state - device lifecycle, set by the control module
#define BD_CTRL_STATE_WAIT_CONFIG           (1)             // no valid saved config - waiting for the host to set the device up and save
#define BD_CTRL_STATE_NORMAL_RUN            (2)             // configured - normal operation
#define BD_CTRL_STATE_HPPC_RUN              (3)             // configured - HPPC operation

// -------------------------------------------------------
// 0x09 — FETStatus register bit definitions (read-only)
// -------------------------------------------------------
#define BD_FET_MAIN                     (1 << 0)        // Bit 0     — main FETs state
#define BD_FET_PRE                      (1 << 1)        // Bit 1     — pre-FET state

// -----------------------------------------------------------------------------
// Config_Data_t — settings the user chooses. Written by the host in any state and
// saved to EEPROM with CC_EEPROM_CONFIG. Nothing in here is updated by the device itself.
// -----------------------------------------------------------------------------
typedef struct __attribute__((packed))
{
    // -------------------------------------------------------------------------
    // Configuration registers
    // -------------------------------------------------------------------------

    uint8_t  cell_count;                               // number of series cells, BD_CELL_COUNT_MIN to BD_CELL_COUNT_MAX
    bool     use_i2c;                                  // I2C interface enabled
    bool     use_can;                                  // CAN interface enabled
    bool     use_uart;                                 // UART interface enabled
    bool     volt_protection;                          // voltage protection enabled
    bool     current_protection;                       // current protection enabled
    bool     temp_protection;                          // temperature protection enabled
    uint32_t cell_capacity;                            // factory capacity in mAh
    uint16_t voltage_cell_max;                         // max designed cell voltage in mV
    uint16_t voltage_cell_min;                         // min designed cell voltage in mV
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
    // Charge limits, under-temperature and OCV window
    // (stored for the HPPC sequencer and protections, not acted on yet)
    // -------------------------------------------------------------------------

    uint16_t charge_stop_cell_mv;                      // stop charging when any cell reaches this voltage in mV
    uint16_t charge_monitor_cell_mv;                   // start checking the termination current above this cell voltage in mV
    int16_t  utp_threshold_c;                          // under-temperature threshold in °C
    uint16_t utp_time_ms;                              // UTP detection time in ms
    int16_t  otp_charge_threshold_c;                   // over-temperature threshold while charging in °C
    uint16_t ocv_v_lo_mv;                              // cell voltage treated as 0 % SOC in mV
    uint16_t ocv_v_hi_mv;                              // cell voltage treated as 100 % SOC in mV

    // -------------------------------------------------------------------------
    // Sensor calibration (the offsets are measured at every boot, see Calibration_Data_t)
    // -------------------------------------------------------------------------

    float    current_sensor_gain;                      // current sensor gain calibration
    float    voltage_gain[7];                          // per-cell ADC gain calibration
    float    ntc_beta;                                 // NTC Beta value from datasheet (e.g. 3950.0)
    float    ntc_r_nominal;                            // NTC nominal resistance at 25°C in Ohms
    float    ntc_r_fixed;                              // Fixed resistor value in Ohms (e.g. 10000.0)
    float    ntc_t_nominal;                            // NTC nominal temperature in Kelvin (e.g. 298.15 = 25°C)
    float    temperature_offset;                       // temperature sensor offset in °C
    float    shunt_resistance_mohm;                    // current shunt resistance in mOhm
    float    current_bias_ma;                          // constant current error subtracted from every reading in mA

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
    bool     protection_trigger;                       // true once any protection has tripped
    uint8_t  ctrl_state;                               // BD_CTRL_STATE_*
    uint64_t error_flags;                              // bit n = OpenBMS_Error_t n: latched errors and live hardware pin bits
    uint32_t session_uptime_ms;                        // uptime since this boot in ms
    uint8_t  hppc_direction;                           // BD_HPPC_DIR_* — detected by the board, never written by the host
    uint16_t hppc_status;                              // low byte BD_HPPC_STATE_*, high byte stop reason
    uint16_t hppc_extract_status;                      // BD_HPPC_X_* bits

} System_Data_t;

// -----------------------------------------------------------------------------
// Stats_Data_t — lifetime statistics and the fault log. Written only by the
// device and read-only to the host. Meant to be saved periodically; nothing
// persists yet, so every value restarts from zero at boot for now.
// -----------------------------------------------------------------------------
typedef struct __attribute__((packed))
{
    // -------------------------------------------------------------------------
    // Fault registers
    // -------------------------------------------------------------------------

    uint16_t fault_snapshot_voltage[7];                // per-cell voltage at last fault in mV
    int16_t  fault_snapshot_current;                   // current at last fault in mA
    uint8_t  fault_snapshot_temperature;               // temperature at last fault in °C
    uint8_t  fault_snapshot_soc;                       // SoC at last fault in %
    uint8_t  fault_code[8];                            // last 8 fault codes
    uint32_t fault_timestamp[8];                       // last 8 fault Unix timestamps

    // -------------------------------------------------------------------------
    // Lifetime history registers — block
    // -------------------------------------------------------------------------

    uint16_t cell_balancing_energy[7];                 // per-cell accumulated balancing energy in mWh
    uint16_t cell_balancing_time[7];                   // per-cell accumulated balancing time in minutes
    uint8_t  cell_deepest_discharge[7];                // per-cell lowest SoC ever recorded in %
    uint8_t  cell_max_temperature[7];                  // per-cell highest temperature ever recorded in °C

} Stats_Data_t;

// One cell's battery model: OCV and the 2-RC parameters on the shared SOC grid
// for both directions, plus its capacity. Every cell has its own copy, so a cell
// that is weaker or has a different OCV curve is described by its own numbers.
typedef struct __attribute__((packed, aligned(4)))
{
    float       ocv_dis[20];                              // OCV discharge curve in V
    float       ocv_chg[20];                              // OCV charge curve in V
    float       r0_dis[20];                               // R0 discharge in Ω
    float       r1_dis[20];                               // R1 discharge in Ω
    float       tau1_dis[20];                             // tau1 discharge in s
    float       r2_dis[20];                               // R2 discharge in Ω
    float       tau2_dis[20];                             // tau2 discharge in s
    float       r0_chg[20];                               // R0 charge in Ω
    float       r1_chg[20];                               // R1 charge in Ω
    float       tau1_chg[20];                             // tau1 charge in s
    float       r2_chg[20];                               // R2 charge in Ω
    float       tau2_chg[20];                             // tau2 charge in s
    float       capacity_ah;                              // capacity of this cell in Ah

} FuelGauge_Cell_t;

// The fuel gauge estimator takes float pointers into this struct and
// dereferences them directly, so every float member has to sit on a 4-byte
// boundary — on Cortex-M4 an unaligned VLDR raises a UsageFault rather than
// just being slow. Packing alone gives the struct an alignment of 1 and would
// land soc_grid on offset 58, so the reserved bytes below realign the float
// block and aligned(4) pins the struct itself. openbms_fuelgauge.c holds this
// with static assertions; adding a field above soc_grid without adjusting the
// padding will fail the build rather than fault on hardware.
typedef struct __attribute__((packed, aligned(4)))
{
    // -------------------------------------------------------------------------
    // Fuel gauge registers
    // ------------------------------------------------------------------------

    uint8_t     relative_soc;                             // % of FullChargeCapacity
    uint8_t     absolute_soc;                             // % of DesignCapacity
    uint8_t     cell_soc[7];                              // per-cell SoC in %
    uint8_t     cell_soh[7];                              // per-cell SoH in %
    uint16_t    cell_remaining_capacity[7];               // per-cell remaining capacity in mAh
    uint16_t    cell_self_discharge[7];                   // per-cell self-discharge rate in mAh/month
    uint16_t    cell_qmax[7];                             // per-cell learned maximum capacity in mAh

    uint8_t     reserved[2];                              // padding — keeps the float block below 4-byte aligned

    float       soc_grid[20];                             // SOC setpoints in %
    FuelGauge_Cell_t cell[BD_CELL_COUNT_MAX];                    // per-cell model: tables and capacity
    float       q_nom_temp_c[5];                          // temperature setpoints in °C
    float       q_nom_temp_ah[5];                         // capacity at each temperature in Ah
    float       q_nom_ah;                                 // nominal capacity at 25°C in Ah
    float       coulombic_efficiency;                     // typically 0.995 to 0.999
    float       ea_r0;                                    // activation energy for R0 in J/mol
    float       ea_r1;                                    // activation energy for R1 in J/mol
    float       ea_tau1;                                  // activation energy for tau1 in J/mol
    float       ea_r2;                                    // activation energy for R2 in J/mol
    float       ea_tau2;                                  // activation energy for tau2 in J/mol
    float       kf_q_soc;                                 // process noise — SOC state
    float       kf_q_rc1;                                 // process noise — V_RC1 state
    float       kf_q_rc2;                                 // process noise — V_RC2 state
    float       kf_r_v;                                   // measurement noise — voltage sensor V²
    float       cell_soc_f[7];                            // last estimated SOC per cell (0.0 to 1.0)
    float       cell_vrc1[7];                             // last estimated V_RC1 per cell in V
    float       cell_vrc2[7];                             // last estimated V_RC2 per cell in V
    float       cell_p[7][6];                             // full covariance upper triangle per cell
    uint16_t    cycle_count;                              // charge/discharge cycle counter
    uint16_t    learning_status;                          // Kalman filter convergence and learning state flags

} FuelGauge_Data_t;

typedef struct
{
    float       cell_voltage[7];                        // per-cell voltage in mV
    float       pack_voltage;                           // pack voltage in mV
    float       pack_current;                           // instantaneous current in mA
    float       temperature_package;                    // NTC battery temperature °C
    float       temperature_stm32;                      // Temperature read out from STM32 in °C
    float       main_vdd_voltage_mv;                    // VDD voltage in the system (usually 3300mV)

    uint16_t    fet_status;                             // Bit 8:2: balancer FETs, Bit 1: pre-FET state, Bit 0: main FETs state

} Peripheral_Data_t;

// -----------------------------------------------------------------------------
// Calibration_Data_t — what the peripheral code measures with. The gains, NTC,
// shunt and bias values are a copy of the ones in Config_Data_t (Periph_SetCalibration),
// the two offsets are measured at every boot and are read-only to the host.
// current_sensor_offset is the voltage across the shunt in mV, not a current.
// -----------------------------------------------------------------------------
typedef struct
{
    float       current_sensor_offset;                  // current sensor offset, measured, in mV across the shunt
    float       current_sensor_gain;                    // current sensor gain calibration
    float       voltage_offset[7];                      // per-cell ADC offset, measured
    float       voltage_gain[7];                        // per-cell ADC gain calibration
    float       ntc_beta;                               // NTC Beta value from datasheet (e.g. 3950.0)
    float       ntc_r_nominal;                          // NTC nominal resistance at 25°C
    float       ntc_r_fixed;                            // Fixed resistor value in Ohms (e.g. 10000.0)
    float       ntc_t_nominal;                          // NTC nominal temperature in Kelvin (e.g. 298.15 = 25°C)
    float       temperature_offset;                     // temperature sensor offset in °C
    float       shunt_resistance_mohm;                  // current shunt resistance in mOhm
    float       current_bias_ma;                        // constant current error subtracted from every reading in mA

} Calibration_Data_t;

#ifdef __cplusplus
}
#endif

#endif