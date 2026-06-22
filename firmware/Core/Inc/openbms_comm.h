#ifndef OPENBMS_COMM_H
#define OPENBMS_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "main.h"

#define UART_RX_BUFFER_SIZE     2048U
#define UART_TX_BUFFER_SIZE     2048U

typedef enum
{
    TYPE_UINT8,
    TYPE_UINT16,
    TYPE_UINT32,
    TYPE_INT8,
    TYPE_INT16,
    TYPE_INT32,
    TYPE_FLOAT,
    TYPE_DOUBLE,
    TYPE_CHAR,
    TYPE_STRING,

} SBS_RegType_t;

typedef struct
{
    uint8_t  rx_buffer[UART_RX_BUFFER_SIZE];
    uint8_t  tx_buffer[UART_TX_BUFFER_SIZE];
    uint16_t rx_length;
    uint8_t  uart_rx_byte;
    bool     frame_ready;

} UART_Command_t;

typedef struct __attribute__((packed))
{
    // -------------------------------------------------------------------------
    // 0x00 — ManufacturerAccess - read-only
    // -------------------------------------------------------------------------
    const uint16_t manufacturer_access;                // default 0x21

    // -------------------------------------------------------------------------
    // 0x01 — RemainingCapacityAlarm
    // 0x02 — RemainingTimeAlarm
    // -------------------------------------------------------------------------
    uint16_t remaining_capacity_alarm;                 // mAh or 10mWh; 0 disables
    uint16_t remaining_time_alarm;                     // minutes; 0 disables

    // -------------------------------------------------------------------------
    // 0x03 — BatteryMode
    // -------------------------------------------------------------------------
    uint16_t battery_mode;                             // config flags bitfield

    // -------------------------------------------------------------------------
    // 0x04 — AtRate
    // 0x05 — AtRateTimeToFull - read-only
    // 0x06 — AtRateTimeToEmpty - read-only
    // 0x07 — AtRateOK - read-only
    // -------------------------------------------------------------------------
    int16_t  at_rate;                                  // signed mA or 10mW
    uint16_t at_rate_time_to_full;                     // minutes; 65535 = not charging
    uint16_t at_rate_time_to_empty;                    // minutes; 65535 = not discharging
    uint16_t at_rate_ok;                               // boolean

    // -------------------------------------------------------------------------
    // 0x08 — Temperature - read-only
    // 0x09 — Voltage - read-only
    // 0x0A — Current - read-only
    // 0x0B — AverageCurrent - read-only
    // -------------------------------------------------------------------------
    uint16_t temperature;                              // 0.1K units; NTC battery temperature
    uint16_t voltage;                                  // pack terminal voltage in mV
    int16_t  current;                                  // signed mA; positive=charge, negative=discharge
    int16_t  average_current;                          // signed mA; 1-minute rolling average

    // -------------------------------------------------------------------------
    // 0x0C — MaxError - read-only
    // 0x0D — RelativeStateOfCharge - read-only
    // 0x0E — AbsoluteStateOfCharge - read-only
    // -------------------------------------------------------------------------
    uint16_t max_error;                                // % gauge uncertainty; 0-100
    uint16_t relative_soc;                             // % of FullChargeCapacity
    uint16_t absolute_soc;                             // % of DesignCapacity

    // -------------------------------------------------------------------------
    // 0x0F — RemainingCapacity - read-only
    // 0x10 — FullChargeCapacity - read-only
    // -------------------------------------------------------------------------
    uint16_t remaining_capacity;                       // mAh or 10mWh
    uint16_t full_charge_capacity;                     // mAh or 10mWh; learned

    // -------------------------------------------------------------------------
    // 0x11 — RunTimeToEmpty - read-only
    // 0x12 — AverageTimeToEmpty - read-only
    // 0x13 — AverageTimeToFull - read-only
    // -------------------------------------------------------------------------
    uint16_t run_time_to_empty;                        // minutes; 65535 = not discharging
    uint16_t average_time_to_empty;                    // minutes; 65535 = not discharging
    uint16_t average_time_to_full;                     // minutes; 65535 = not charging

    // -------------------------------------------------------------------------
    // 0x14 — ChargingCurrent - r/w
    // 0x15 — ChargingVoltage - r/w
    // -------------------------------------------------------------------------
    uint16_t charging_current;                         // mA requested to charger
    uint16_t charging_voltage;                         // mV requested to charger

    // -------------------------------------------------------------------------
    // 0x16 — BatteryStatus / AlarmWarning
    // -------------------------------------------------------------------------
    uint16_t battery_status;                           // alarm + status flags + error code nibble

    // -------------------------------------------------------------------------
    // 0x17 — CycleCount - read-only
    // -------------------------------------------------------------------------
    uint16_t cycle_count;                              // charge/discharge cycle counter

    // -------------------------------------------------------------------------
    // 0x18 — DesignCapacity
    // 0x19 — DesignVoltage
    // -------------------------------------------------------------------------
    uint16_t design_capacity;                          // mAh or 10mWh; nominal factory capacity
    uint16_t design_voltage;                           // mV; nominal pack voltage

    // -------------------------------------------------------------------------
    // 0x1A — SpecificationInfo - read-only
    // 0x1B — ManufactureDate - read-only
    // 0x1C — SerialNumber - read-only
    // -------------------------------------------------------------------------
    const uint16_t specification_info;                 // 0x0011 — Version=1, Revision=1, VScale=0, IPScale=0
    uint16_t       manufacture_date;                   // packed: (year-1980)*512 + month*32 + day
    uint16_t       serial_number;                      // 16-bit unique serial number

    // -------------------------------------------------------------------------
    // 0x20 — ManufacturerName - read-only
    // 0x21 — DeviceName - read-only
    // 0x22 — DeviceChemistry - read-only
    // 0x23 — ManufacturerData - read-only
    // -------------------------------------------------------------------------
    const char manufacturer_name[32];                  // "OpenBatt Team"
    const char device_name[32];                        // "OpenBMS"
    const char device_chemistry[32];                   // "Li-Ion"
    const char manufacturer_data[32];                  // "Year 2026"

    // -------------------------------------------------------------------------
    // Control registers
    // -------------------------------------------------------------------------

    // 0x40 — Configuration
    uint16_t configuration;                            // Bit 6: UART, Bit 5: CAN, Bit 4: I2C, Bits 3-0: cell count

    // 0x41 — MainControl
    uint16_t main_control;                             // Bit 5: temp prot, Bit 4: curr prot, Bit 3: volt prot, Bit 1: test mode, Bit 0: OpenBMS state

    // 0x42 — FETState
    uint16_t fet_state;                                // Bit 1: aux FET, Bit 0: main FETs

    // 0x43 — FETStatus - read-only
    uint16_t fet_status;                               // Bit 1: aux FET state, Bit 0: main FETs state

    // -------------------------------------------------------------------------
    // Voltage protection configuration
    // -------------------------------------------------------------------------

    // 0x44 — UVP slow threshold
    uint16_t uvp_slow_threshold_mv;                    // slow UVP threshold in mV

    // 0x45 — UVP slow time
    uint16_t uvp_slow_time_ms;                         // slow UVP detection time in ms

    // 0x46 — UVP fast threshold
    uint16_t uvp_fast_threshold_mv;                    // fast UVP threshold in mV

    // 0x47 — UVP fast time
    uint16_t uvp_fast_time_ms;                         // fast UVP detection time in ms

    // 0x48 — OVP slow threshold
    uint16_t ovp_slow_threshold_mv;                    // slow OVP threshold in mV

    // 0x49 — OVP slow time
    uint16_t ovp_slow_time_ms;                         // slow OVP detection time in ms

    // 0x4A — OVP fast threshold
    uint16_t ovp_fast_threshold_mv;                    // fast OVP threshold in mV

    // 0x4B — OVP fast time
    uint16_t ovp_fast_time_ms;                         // fast OVP detection time in ms

    // -------------------------------------------------------------------------
    // Current protection configuration
    // -------------------------------------------------------------------------

    // 0x4C — Charge OCP threshold
    uint16_t ocp_charge_threshold_ma;                  // charge OCP threshold in mA

    // 0x4D — Charge OCP time
    uint16_t ocp_charge_time_ms;                       // charge OCP detection time in ms

    // 0x4E — Slow discharge OCP threshold
    uint16_t ocp_discharge_slow_threshold_ma;          // slow discharge OCP threshold in mA

    // 0x4F — Slow discharge OCP time
    uint16_t ocp_discharge_slow_time_ms;               // slow discharge OCP detection time in ms

    // 0x50 — Fast discharge OCP threshold
    uint16_t ocp_discharge_fast_threshold_ma;          // fast discharge OCP threshold in mA

    // 0x51 — Fast discharge OCP time
    uint16_t ocp_discharge_fast_time_ms;               // fast discharge OCP detection time in ms

    // -------------------------------------------------------------------------
    // Temperature protection configuration
    // -------------------------------------------------------------------------

    // 0x52 — OTP threshold
    uint16_t otp_threshold_c;                          // OTP threshold in °C

    // 0x53 — OTP time
    uint16_t otp_time_ms;                              // OTP detection time in ms

    // -------------------------------------------------------------------------
    // Analog measurement registers — block
    // -------------------------------------------------------------------------

    // 0x54 — CellVoltage - read-only
    uint16_t cell_voltage[7];                          // per-cell voltage in mV

    // 0x55 — CellTemperature - read-only
    int16_t  cell_temperature[7];                      // per-cell temperature in 0.1°C

    // -------------------------------------------------------------------------
    // Fuel gauge registers — block
    // -------------------------------------------------------------------------

    // 0x56 — CellSoC - read-only
    uint8_t  cell_soc[7];                              // per-cell SoC in %

    // 0x57 — CellSoH - read-only
    uint8_t  cell_soh[7];                              // per-cell SoH in %

    // 0x58 — CellRemainingCapacity - read-only
    uint16_t cell_remaining_capacity[7];               // per-cell remaining capacity in mAh

    // 0x59 — CellSelfDischarge - read-only
    uint16_t cell_self_discharge[7];                   // per-cell self-discharge rate in mAh/month

    // 0x5A — CellQmax - read-only
    uint16_t cell_qmax[7];                             // per-cell learned maximum capacity in mAh

    // -------------------------------------------------------------------------
    // Fault registers — block
    // -------------------------------------------------------------------------

    // 0x5B — FaultSnapshotVoltage - read-only
    uint16_t fault_snapshot_voltage[7];                // per-cell voltage at last fault in mV

    // 0x5C — FaultSnapshotCurrent - read-only
    int16_t  fault_snapshot_current;                   // current at last fault in mA

    // 0x5D — FaultSnapshotTemperature - read-only
    uint8_t  fault_snapshot_temperature;               // temperature at last fault in °C

    // 0x5E — FaultSnapshotSoC - read-only
    uint8_t  fault_snapshot_soc;                       // SoC at last fault in %

    // 0x5F — FaultCode - read-only
    uint8_t  fault_code[8];                            // last 8 fault codes

    // 0x60 — FaultTimestamp - read-only
    uint32_t fault_timestamp[8];                       // last 8 fault Unix timestamps

    // -------------------------------------------------------------------------
    // Protection event counters — block per protection type
    // -------------------------------------------------------------------------

    // 0x61 — OVP counters - read-only
    uint16_t ovp_counter[7];                           // per-cell OVP trigger count

    // 0x62 — UVP counters - read-only
    uint16_t uvp_counter[7];                           // per-cell UVP trigger count

    // 0x63 — OCP counters - read-only
    uint16_t ocp_counter[7];                           // per-cell OCP trigger count

    // 0x64 — OTP counters - read-only
    uint16_t otp_counter[7];                           // per-cell OTP trigger count

    // 0x65 — UTP counters - read-only
    uint16_t utp_counter[7];                           // per-cell UTP trigger count

    // -------------------------------------------------------------------------
    // Calibration registers
    // -------------------------------------------------------------------------

    // 0x66 — Current sensor offset
    int16_t  current_sensor_offset;                    // current sensor offset calibration

    // 0x67 — Current sensor gain
    int16_t  current_sensor_gain;                      // current sensor gain calibration

    // 0x68 — Voltage offset calibration — block
    int16_t  voltage_offset[7];                        // per-cell ADC offset calibration

    // 0x69 — Voltage gain calibration — block
    int16_t  voltage_gain[7];                          // per-cell ADC gain calibration

    // 0x6A — Temperature offset
    uint8_t  temperature_offset;                       // temperature sensor offset in °C

    // -------------------------------------------------------------------------
    // Lifetime history registers — block
    // -------------------------------------------------------------------------

    // 0x6B — CellBalancingEnergy - read-only
    uint16_t cell_balancing_energy[7];                 // per-cell accumulated balancing energy in mWh

    // 0x6C — CellBalancingTime - read-only
    uint16_t cell_balancing_time[7];                   // per-cell accumulated balancing time in minutes

    // 0x6D — CellDeepestDischarge - read-only
    uint8_t  cell_deepest_discharge[7];                // per-cell lowest SoC ever recorded in %

    // 0x6E — CellMaxTemperature - read-only
    uint8_t  cell_max_temperature[7];                  // per-cell highest temperature ever recorded in °C

    // -------------------------------------------------------------------------
    // Balancing registers
    // -------------------------------------------------------------------------

    // 0x6F — BalancingStatus - read-only
    uint16_t balancing_status;                         // bitmask — bit N = cell N+1 balancing

    // 0x70 — BalancingControl
    uint16_t balancing_control;                        // bitmask — bit N = cell N+1 force balancing

    // -------------------------------------------------------------------------
    // System information registers
    // -------------------------------------------------------------------------

    // 0x71 — FirmwareVersion - read-only
    char     firmware_version[32];                     // firmware version string — ASCII null-terminated

    // 0x72 — HardwareVersion - read-only
    char     hardware_version[32];                     // hardware version string — ASCII null-terminated

    // 0x73 — LastCommunicationTimestamp - read-only
    uint32_t last_communication_timestamp;             // Unix timestamp of last host communication

    // 0x74 — UptimeCounter - read-only
    uint32_t uptime_counter;                           // total uptime since first boot in seconds

    // -------------------------------------------------------------------------
    // 0xA0 — SOC grid
    // -------------------------------------------------------------------------
    float soc_grid[20];                                // SOC setpoints in %

    // -------------------------------------------------------------------------
    // 0xA1 — OCV discharge curve
    // 0xA2 — OCV charge curve
    // -------------------------------------------------------------------------
    float ocv_dis[20];                                 // OCV discharge curve in V
    float ocv_chg[20];                                 // OCV charge curve in V

    // -------------------------------------------------------------------------
    // 0xA3 — R0 discharge
    // 0xA4 — R1 discharge
    // 0xA5 — tau1 discharge
    // 0xA6 — R2 discharge
    // 0xA7 — tau2 discharge
    // -------------------------------------------------------------------------
    float r0_dis[20];                                  // R0 discharge in Ω
    float r1_dis[20];                                  // R1 discharge in Ω
    float tau1_dis[20];                                // tau1 discharge in s
    float r2_dis[20];                                  // R2 discharge in Ω
    float tau2_dis[20];                                // tau2 discharge in s

    // -------------------------------------------------------------------------
    // 0xA8 — R0 charge
    // 0xA9 — R1 charge
    // 0xAA — tau1 charge
    // 0xAB — R2 charge
    // 0xAC — tau2 charge
    // -------------------------------------------------------------------------
    float r0_chg[20];                                  // R0 charge in Ω
    float r1_chg[20];                                  // R1 charge in Ω
    float tau1_chg[20];                                // tau1 charge in s
    float r2_chg[20];                                  // R2 charge in Ω
    float tau2_chg[20];                                // tau2 charge in s

    // -------------------------------------------------------------------------
    // 0xAD — Q_nom temperature setpoints
    // 0xAE — Q_nom at each temperature
    // -------------------------------------------------------------------------
    float q_nom_temp_c[5];                             // temperature setpoints in °C
    float q_nom_temp_ah[5];                            // capacity at each temperature in Ah

    // -------------------------------------------------------------------------
    // 0xAF — Nominal capacity at 25°C
    // 0xB0 — Coulombic efficiency
    // -------------------------------------------------------------------------
    float q_nom_ah;                                    // nominal capacity at 25°C in Ah
    float coulombic_efficiency;                        // typically 0.995 to 0.999

    // -------------------------------------------------------------------------
    // 0xB1 — R0 reference at 25°C
    // 0xB2 — R1 reference at 25°C
    // 0xB3 — tau1 reference at 25°C
    // 0xB4 — R2 reference at 25°C
    // 0xB5 — tau2 reference at 25°C
    // -------------------------------------------------------------------------
    float r0_ref;                                      // R0 reference at 25°C in Ω
    float r1_ref;                                      // R1 reference at 25°C in Ω
    float tau1_ref;                                    // tau1 reference at 25°C in s
    float r2_ref;                                      // R2 reference at 25°C in Ω
    float tau2_ref;                                    // tau2 reference at 25°C in s

    // -------------------------------------------------------------------------
    // 0xB6 — Ea for R0
    // 0xB7 — Ea for R1
    // 0xB8 — Ea for tau1
    // 0xB9 — Ea for R2
    // 0xBA — Ea for tau2
    // -------------------------------------------------------------------------
    float ea_r0;                                       // activation energy for R0 in J/mol
    float ea_r1;                                       // activation energy for R1 in J/mol
    float ea_tau1;                                     // activation energy for tau1 in J/mol
    float ea_r2;                                       // activation energy for R2 in J/mol
    float ea_tau2;                                     // activation energy for tau2 in J/mol

    // -------------------------------------------------------------------------
    // 0xBB — Kalman process noise SOC
    // 0xBC — Kalman process noise RC1
    // 0xBD — Kalman process noise RC2
    // 0xBE — Kalman measurement noise
    // -------------------------------------------------------------------------
    float kf_q_soc;                                    // process noise — SOC state
    float kf_q_rc1;                                    // process noise — V_RC1 state
    float kf_q_rc2;                                    // process noise — V_RC2 state
    float kf_r_v;                                      // measurement noise — voltage sensor V²

    // -------------------------------------------------------------------------
    // 0xBF — Per-cell SOC float (7 cells) - read-only
    // 0xC0 — Per-cell V_RC1 (7 cells) - read-only
    // 0xC1 — Per-cell V_RC2 (7 cells) - read-only
    // -------------------------------------------------------------------------
    float cell_soc_f[7];                               // last estimated SOC per cell (0.0 to 1.0)
    float cell_vrc1[7];                                // last estimated V_RC1 per cell in V
    float cell_vrc2[7];                                // last estimated V_RC2 per cell in V

    // -------------------------------------------------------------------------
    // 0xC2 — Per-cell covariance matrix upper triangle (7 cells) - read-only
    //        P layout per cell: [P00, P01, P02, P11, P12, P22]
    // -------------------------------------------------------------------------
    float cell_p[7][6];                                // full covariance upper triangle per cell

    // -------------------------------------------------------------------------
    // 0xC3 — Per-cell capacity after aging (7 cells) - read-only
    // 0xC4 — Per-cell R0 growth factor (7 cells) - read-only
    // 0xC5 — Per-cell cycle count (7 cells) - read-only
    // -------------------------------------------------------------------------
    float    cell_q_nom_ah[7];                         // current capacity after aging in Ah
    float    cell_r0_scale[7];                         // R0 growth factor per cell (1.0 = nominal)
    uint16_t cell_cycle_count[7];                      // cycle count per cell

    // -------------------------------------------------------------------------
    // 0xC6 — LearningStatus - read-only
    // -------------------------------------------------------------------------
    uint16_t learning_status;                          // Kalman filter convergence and learning state flags

} OpenBMS_SBS_Data_t;


void OpenBMS_Comm_Init(void);
void OpenBMS_Comm_Run(void);

#endif