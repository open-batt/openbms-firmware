#ifndef OPENBMS_CTRL_H
#define OPENBMS_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "main.h"

#define EEPROM_I2C_ADDRESS          0xA0
#define EEPROM_SIZE                 4096
#define EEPROM_I2C_ID_TIMEOUT       50
#define EEPROM_I2C_READ_TIMEOUT     10
#define EEPROM_I2C_WRITE_TIMEOUT    20

#define ADS131M0_SPI_TIMEOUT        100

typedef struct
{
    bool main_fet_enable;
    bool pre_fet_enable;
    bool meas_cell_voltage_enable;
    bool meas_pack_voltage_enable;
    bool cell_balancer_enable[7];
    bool pwr_on;

    bool fet_driver_fault;
    bool fet_driver_gate_fault;

} OpenBMS_Ctrl_t;

typedef struct
{
    float cell_voltage[7];
    bool  cell_voltage_updated;

    float pack_voltage;
    bool  pack_voltage_updated;

    float pack_current;
    bool  pack_current_updated;
    
    float temperature_ntc;
    bool  temperature_ntc_updated;

    float vdd_mv;
    bool  vdd_mv_updated;

    float temperature_stm32;
    bool  temperature_stm32_updated;

} OpenBMS_Data_t;

typedef struct __attribute__((packed))
{
    // -------------------------------------------------------------------------
    // Hardware configuration & calibration
    // -------------------------------------------------------------------------
    float    cell_voltage_resistance_factor;        // Factor to convert raw ADC value to voltage, cell 1-7
    float    batt_voltage_resistance_factor;        // Factor to convert raw ADC value to battery (pack) voltage
    float    shunt_resistance_mohms;                // Shunt resistance in milliohms for current measurement
    float    current_sense_offset_mv;               // Offset in mV to be subtracted from current sense voltage
    float    current_sense_gain;                    // Gain factor to convert current sense voltage to current
    uint8_t  num_cells;                             // Number of cells in the battery pack

    // -------------------------------------------------------------------------
    // Battery identity
    // -------------------------------------------------------------------------
    char     serial_number[16];                     // Pack serial number, ASCII null-terminated
    char     cell_chemistry[8];                     // Cell chemistry identifier e.g. "LION", "LFP"
    uint16_t manufacture_date;                      // Packed: (year-1980)*512 + month*32 + day
    uint16_t design_capacity_mah;                   // Nominal design capacity in mAh
    uint16_t design_voltage_mv;                     // Nominal design voltage in mV

    // -------------------------------------------------------------------------
    // Versions
    // -------------------------------------------------------------------------
    char     firmware_version[8];                   // Firmware version string e.g. "1.0.0"
    char     hardware_version[8];                   // Hardware version string e.g. "RevA"

    // -------------------------------------------------------------------------
    // Communication configuration
    // -------------------------------------------------------------------------
    uint8_t  node_id;                               // CAN/Modbus node ID, default 0x01
    uint8_t  interfaces_enabled;                    // Bitmask — bit0=I2C, bit1=CAN, bit2=UART

    // -------------------------------------------------------------------------
    // Lookup tables
    // -------------------------------------------------------------------------
    uint16_t ocv_mv[51][4];                         // OCV table — 51 SoC points, 4 temperatures (-10C, 0C, 25C, 45C)
    uint16_t impedance_mohm_c1[51][4];              // Cell 1 impedance — 51 SoC points, 4 temperatures
    uint16_t impedance_mohm_c2[51][4];              // Cell 2 impedance — 51 SoC points, 4 temperatures
    uint16_t impedance_mohm_c3[51][4];              // Cell 3 impedance — 51 SoC points, 4 temperatures
    uint16_t impedance_mohm_c4[51][4];              // Cell 4 impedance — 51 SoC points, 4 temperatures
    uint16_t impedance_mohm_c5[51][4];              // Cell 5 impedance — 51 SoC points, 4 temperatures
    uint16_t impedance_mohm_c6[51][4];              // Cell 6 impedance — 51 SoC points, 4 temperatures
    uint16_t impedance_mohm_c7[51][4];              // Cell 7 impedance — 51 SoC points, 4 temperatures

    // -------------------------------------------------------------------------
    // Protection thresholds
    // -------------------------------------------------------------------------
    uint16_t uvp_slow_threshold_mv;                 // Slow undervoltage protection threshold in mV
    uint16_t uvp_slow_time_ms;                      // Slow undervoltage protection detection time in ms
    uint16_t uvp_fast_threshold_mv;                 // Fast undervoltage protection threshold in mV
    uint16_t uvp_fast_time_ms;                      // Fast undervoltage protection detection time in ms
    uint16_t ovp_slow_threshold_mv;                 // Slow overvoltage protection threshold in mV
    uint16_t ovp_slow_time_ms;                      // Slow overvoltage protection detection time in ms
    uint16_t ovp_fast_threshold_mv;                 // Fast overvoltage protection threshold in mV
    uint16_t ovp_fast_time_ms;                      // Fast overvoltage protection detection time in ms

    uint16_t ocp_charge_threshold_ma;               // Charge overcurrent protection threshold in mA
    uint16_t ocp_charge_time_ms;                    // Charge overcurrent protection detection time in ms
    uint16_t ocp_discharge_slow_threshold_ma;       // Slow discharge overcurrent protection threshold in mA
    uint16_t ocp_discharge_slow_time_ms;            // Slow discharge overcurrent protection detection time in ms
    uint16_t ocp_discharge_fast_threshold_ma;       // Fast discharge overcurrent protection threshold in mA
    uint16_t ocp_discharge_fast_time_ms;            // Fast discharge overcurrent protection detection time in ms

    uint16_t otp_threshold_c;                       // Overtemperature protection threshold in °C
    uint16_t otp_time_ms;                           // Overtemperature protection detection time in ms

    // -------------------------------------------------------------------------
    // Learned fuel gauge data
    // -------------------------------------------------------------------------
    uint16_t qmax_mah[7];                           // Per-cell learned maximum capacity in mAh
    uint16_t self_discharge_rate_mah_month[7];      // Per-cell self-discharge rate in mAh/month
    uint8_t  last_soc_percent[7];                   // Per-cell last known SoC in % — restored after power loss

    // -------------------------------------------------------------------------
    // Lifetime history
    // -------------------------------------------------------------------------
    uint32_t total_cycle_count;                     // Total pack cycle count
    uint16_t cell_cycle_count[7];                   // Per-cell cycle count
    uint8_t  cell_deepest_discharge_percent[7];     // Per-cell lowest SoC ever recorded in %
    uint8_t  cell_max_temperature_c[7];             // Per-cell highest temperature ever recorded in °C
    uint16_t cell_balancing_energy_mwh[7];          // Per-cell accumulated balancing energy in mWh
    uint16_t cell_balancing_time_min[7];            // Per-cell accumulated balancing time in minutes

    // -------------------------------------------------------------------------
    // Protection event counters
    // -------------------------------------------------------------------------
    uint16_t ovp_counter[7];                        // Per-cell OVP trigger count
    uint16_t uvp_counter[7];                        // Per-cell UVP trigger count
    uint16_t ocp_counter[7];                        // Per-cell OCP trigger count
    uint16_t otp_counter[7];                        // Per-cell OTP trigger count
    uint16_t utp_counter[7];                        // Per-cell UTP trigger count

    // -------------------------------------------------------------------------
    // NTC temperature sensor constants
    // -------------------------------------------------------------------------
    float    ntc_beta;                              // NTC Beta value — from datasheet (e.g. 3950.0)
    float    ntc_r_nominal;                         // NTC nominal resistance at 25°C in Ohms (e.g. 10000.0)
    float    ntc_r_fixed;                           // Fixed resistor value in Ohms (e.g. 10000.0)
    float    ntc_t_nominal;                         // NTC nominal temperature in Kelvin (e.g. 298.15 = 25°C)

    // -------------------------------------------------------------------------
    // Fault log
    // -------------------------------------------------------------------------
    uint8_t  fault_code[8];                         // Last 8 fault codes
    uint32_t fault_timestamp[8];                    // Last 8 fault timestamps — Unix timestamp
    uint8_t  fault_soc[8];                          // SoC at each fault event
    int16_t  fault_current_ma[8];                   // Current at each fault event
    uint16_t fault_cell_voltage_mv[8][7];           // Per-cell voltages at each fault event
    uint8_t  fault_temperature_c[8];                // Temperature at each fault event

} OpenBMS_Config_t;

typedef enum 
{
    OPENBMS_EEPROM_ID_READ_FAIL,
    OPENBMS_EEPROM_ID_VAL_FAIL,
    OPENBMS_EEPROM_READ_FAIL,
    OPENBMS_EEPROM_WRITE_FAIL,
    OPENBMS_EEPROM_CRC_FAIL,

    OPENBMS_ADS131M08_READ_FAIL,
    OPENBMS_ADS131M08_INIT_FAIL,
    OPENBMS_ADS131M08_ID_FAIL,
    
} OpenBMS_Status_t;

void OpenBMS_Ctrl_Init(void);
void OpenBMS_Ctrl_Run(void);

#ifdef __cplusplus
}
#endif

#endif // OPENBMS_CTRL_H
