#ifndef OPENBMS_DATA_H
#define OPENBMS_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// ---------------------------------------------------------
// 0x00 — Configuration register bit definitions (read-only)
// ---------------------------------------------------------
#define BD_CONFIG_CELL_COUNT_MASK           (0x0F)          // Bits 3-0  — cell count value
#define BD_CONFIG_USE_I2C                   (1 << 4)        // Bit 4     — I2C enabled
#define BD_CONFIG_USE_CAN                   (1 << 5)        // Bit 5     — CAN enabled
#define BD_CONFIG_USE_UART                  (1 << 6)        // Bit 6     — UART enabled
#define BD_CONFIG_USE_NTC                   (1 << 7)        // Bit 7     — NTC temperature sensor
#define BD_CONFIG_VOLT_PROT                 (1 << 8)        // Bit 8     — voltage protection
#define BD_CONFIG_CURR_PROT                 (1 << 9)        // Bit 9     — current protection
#define BD_CONFIG_TEMP_PROT                 (1 << 10)       // Bit 10    — temperature protection

// -------------------------------------------------------
// 0x01 — MainControl register bit definitions
// -------------------------------------------------------
#define BD_MAIN_CTR_MODE_SHIFT              (0)             // Bits 1-0  — mode shift
#define BD_MAIN_CTR_MODE_MASK               (0x03)          // Bits 1-0  — mode mask
#define BD_MAIN_CTR_MODE_NORMAL             (0x00)          // Bits 1-0  — normal mode
#define BD_MAIN_CTR_MODE_CONFIG             (0x01)          // Bits 1-0  — config mode
#define BD_MAIN_CTR_MODE_LEARNING           (0x02)          // Bits 1-0  — learning mode

// -------------------------------------------------------
// 0x39 — FETStatus register bit definitions (read-only)
// -------------------------------------------------------
#define BD_FET_MAIN                     (1 << 0)        // Bit 0     — main FETs state
#define BD_FET_PRE                      (1 << 1)        // Bit 1     — pre-FET state

#define BD_FET_BAL_MASK                 (0x7F << 2)     // Bits 8:2  — balancer FETs mask
#define BD_FET_BAL_1                    (1 << 2)        // Bit 2     — balancer FET cell 1
#define BD_FET_BAL_2                    (1 << 3)        // Bit 3     — balancer FET cell 2
#define BD_FET_BAL_3                    (1 << 4)        // Bit 4     — balancer FET cell 3
#define BD_FET_BAL_4                    (1 << 5)        // Bit 5     — balancer FET cell 4
#define BD_FET_BAL_5                    (1 << 6)        // Bit 6     — balancer FET cell 5
#define BD_FET_BAL_6                    (1 << 7)        // Bit 7     — balancer FET cell 6
#define BD_FET_BAL_7                    (1 << 8)        // Bit 8     — balancer FET cell 7

typedef struct __attribute__((packed))
{
    // -------------------------------------------------------------------------
    // Control and configuration registers
    // -------------------------------------------------------------------------

    uint16_t configuration;                            // Bit 15-11: reserved
                                                       // Bit 10: temp prot
                                                       // Bit 9: curr prot
                                                       // Bit 8: volt prot
                                                       // Bit 7: Use NTC sesnor
                                                       // Bit 6: UART
                                                       // Bit 5: CAN
                                                       // Bit 4: I2C
                                                       // Bits 3-0: cell count
    uint16_t main_control;                             // Bit 2-0: 0 - normal mode, 1 - config mode, 2 - learning mode
    uint32_t pack_capacity;                            // factory capacity in mAh
    uint16_t voltage_pack_max;                         // max designed pack voltage in mV
    uint16_t voltage_pack_min;                         // min designed pack voltage in mV
    uint16_t charging_term_current;                    // Termination current in mA

    // -------------------------------------------------------------------------
    // Voltage protection configuration
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

    // -------------------------------------------------------------------------
    // System information registers
    // -------------------------------------------------------------------------

    uint32_t last_communication_timestamp;             // timestamp of last host communication in ms
    uint64_t uptime_counter;                           // total uptime since first boot in milliseconds
    char     firmware_version[32];                     // firmware version string — ASCII null-terminated
    char     hardware_version[32];                     // hardware version string — ASCII null-terminated
    char     manufacturer_name[32];                    // "OpenBatt Team" — ASCII null-terminated
    char     device_name[32];                          // "OpenBMS" — ASCII null-terminated
    char     device_chemistry[32];                     // "Li-Ion" — ASCII null-terminated
    char    manufacturer_data[32];                     // "Year 2026" — ASCII null-terminated

} Control_Data_t;

typedef struct __attribute__((packed))
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

    float       soc_grid[20];                             // SOC setpoints in %
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
    float       q_nom_temp_c[5];                          // temperature setpoints in °C
    float       q_nom_temp_ah[5];                         // capacity at each temperature in Ah
    float       q_nom_ah;                                 // nominal capacity at 25°C in Ah
    float       coulombic_efficiency;                     // typically 0.995 to 0.999
    float       r0_ref;                                   // R0 reference at 25°C in Ω
    float       r1_ref;                                   // R1 reference at 25°C in Ω
    float       tau1_ref;                                 // tau1 reference at 25°C in s
    float       r2_ref;                                   // R2 reference at 25°C in Ω
    float       tau2_ref;                                 // tau2 reference at 25°C in s
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
    float       cell_q_nom_ah[7];                         // current capacity after aging in Ah
    float       cell_r0_scale[7];                         // R0 growth factor per cell (1.0 = nominal)
    uint16_t    cycle_count;                              // charge/discharge cycle counter
    uint16_t    learning_status;                          // Kalman filter convergence and learning state flags

} FuelGauge_Data_t;

typedef struct __attribute__((packed))
{
    float       cell_voltage[7];                        // per-cell voltage in mV
    float       cell_voltage_filtered[7];               // per-cell filtered voltage in mV
    float       pack_voltage;                           // pack voltage in mV
    float       pack_voltage_filtered;                  // filtered pack voltage in mV
    float       pack_current;                           // instantaneous current in mA
    float       pack_current_filtered;                  // filtered current in mA
    float       temperature_package;                    // NTC battery temperature °C
    float       temperature_stm32;                      // Temperature read out from STM32 in °C
    float       main_vdd_voltage_mv;                    // VDD voltage in the system (usually 3300mV)

    float       voltage_offset[7];                      // per-cell ADC offset calibration
    float       voltage_gain[7];                        // per-cell ADC gain calibration
    float       current_sensor_offset;                  // current sensor offset calibration
    float       current_sensor_gain;                    // current sensor gain calibration
    float       ntc_beta;                               // NTC Beta value from datasheet (e.g. 3950.0)
    float       ntc_r_nominal;                          // NTC nominal resistance at 25°C
    float       ntc_r_fixed;                            // Fixed resistor value in Ohms (e.g. 10000.0)
    float       ntc_t_nominal;                          // NTC nominal temperature in Kelvin (e.g. 298.15 = 25°C)
    float       temperature_offset;                     // temperature sensor offset in °C

    uint16_t    fet_status;                             // Bit 8:2: balancer FETs, Bit 1: pre-FET state, Bit 0: main FETs state

} Peripheral_Data_t;

#ifdef __cplusplus
}
#endif

#endif