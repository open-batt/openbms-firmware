/**
  ******************************************************************************
  * @file           : openbms_comm.c
  * @brief          : OpenBMS Communication Module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 OpenBatt.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "openbms_comm.h"
#include "openbms_periph.h"
#include "openbms_ctrl.h"

#define APP_FLAG_FLASH_LAGE  19U

static UART_HandleTypeDef   *debug_uart             = &huart1;
static UART_Command_t       uart_cmd                = {0};
static Peripheral_Data_t    pd                      = {0};
static Control_Data_t       cd                      = {0};
static FuelGauge_Data_t     fg                      = {0};

static CommErrorType_t      Controls_Set(uint8_t address, uint8_t *cmd, uint8_t length);
static CommErrorType_t      Data_ReadWriteRegister(uint8_t address, uint8_t *raw_data, uint8_t *length, bool write); 

static void                 RemoveAppFlag(void);
static void                 ProcessCommand(void);
static uint8_t              Checksum(uint8_t *data, uint16_t length);
static void                 SendResponse(uint8_t *data, uint8_t length);
static void                 SendError(CommErrorType_t err);

/*
static void Data_SetDefaults(void)
{
    
    // -------------------------------------------------------------------------
    // 0x00 — Configuration
    // 0x01 — MainControl
    // -------------------------------------------------------------------------
    bd.configuration                              = 0x0002;
    bd.main_control                               = 0x0000;

    // -------------------------------------------------------------------------
    // 0x02 — PackCapacity
    // 0x03 — MaxPackVoltage
    // 0x04 — MinPackVoltage
    // 0x05 — ChargingTerminationCurrent
    // -------------------------------------------------------------------------
    bd.pack_capacity                              = 35000;
    bd.voltage_pack_max                           = 29400;
    bd.voltage_pack_min                           = 19600;
    bd.charging_term_current                      = 0;

 

    // -------------------------------------------------------------------------
    // 0x20 — UVP slow threshold      0x21 — UVP slow time
    // 0x22 — UVP fast threshold      0x23 — UVP fast time
    // 0x24 — OVP slow threshold      0x25 — OVP slow time
    // 0x26 — OVP fast threshold      0x27 — OVP fast time
    // -------------------------------------------------------------------------
    bd.uvp_slow_threshold_mv                      = 2800;
    bd.uvp_slow_time_ms                           = 5000;
    bd.uvp_fast_threshold_mv                      = 2500;
    bd.uvp_fast_time_ms                           = 100;
    bd.ovp_slow_threshold_mv                      = 4200;
    bd.ovp_slow_time_ms                           = 2000;
    bd.ovp_fast_threshold_mv                      = 4250;
    bd.ovp_fast_time_ms                           = 50;

    // -------------------------------------------------------------------------
    // 0x28 — Charge OCP threshold    0x29 — Charge OCP time
    // 0x2A — Slow OCP threshold      0x2B — Slow OCP time
    // 0x2C — Fast OCP threshold      0x2D — Fast OCP time
    // -------------------------------------------------------------------------
    bd.ocp_charge_threshold_ma                    = 5000;
    bd.ocp_charge_time_ms                         = 1000;
    bd.ocp_discharge_slow_threshold_ma            = 10000;
    bd.ocp_discharge_slow_time_ms                 = 2000;
    bd.ocp_discharge_fast_threshold_ma            = 20000;
    bd.ocp_discharge_fast_time_ms                 = 50;

    // -------------------------------------------------------------------------
    // 0x2E — OTP threshold
    // 0x2F — OTP time
    // -------------------------------------------------------------------------
    bd.otp_threshold_c                            = 60;
    bd.otp_time_ms                                = 3000;



    // -------------------------------------------------------------------------
    // 0x3A — LastCommunicationTimestamp
    // 0x3B — UptimeCounter
    // -------------------------------------------------------------------------
    
    bd.last_communication_timestamp               = 0;
    bd.uptime_counter                             = 0;

    // -------------------------------------------------------------------------
    // 0x50 — FaultSnapshotVoltage
    // 0x51 — FaultSnapshotCurrent
    // 0x52 — FaultSnapshotTemperature
    // 0x53 — FaultSnapshotSoC
    // 0x54 — FaultCode
    // 0x55 — FaultTimestamp
    // -------------------------------------------------------------------------
    memset(bd.fault_snapshot_voltage,     0, sizeof(bd.fault_snapshot_voltage));
    bd.fault_snapshot_current             = 0;
    bd.fault_snapshot_temperature         = 0;
    bd.fault_snapshot_soc                 = 0;
    memset(bd.fault_code,                 0, sizeof(bd.fault_code));
    memset(bd.fault_timestamp,            0, sizeof(bd.fault_timestamp));

    // -------------------------------------------------------------------------
    // 0x60 — CellBalancingEnergy
    // 0x61 — CellBalancingTime
    // 0x62 — CellDeepestDischarge
    // 0x63 — CellMaxTemperature
    // -------------------------------------------------------------------------
    memset(bd.cell_balancing_energy,      0, sizeof(bd.cell_balancing_energy));
    memset(bd.cell_balancing_time,        0, sizeof(bd.cell_balancing_time));
    memset(bd.cell_deepest_discharge,     0, sizeof(bd.cell_deepest_discharge));
    memset(bd.cell_max_temperature,       0, sizeof(bd.cell_max_temperature));

    // -------------------------------------------------------------------------
    // 0x70 — FirmwareVersion
    // 0x71 — HardwareVersion
    // 0x72 — ManufacturerName
    // 0x73 — DeviceName
    // 0x74 — DeviceChemistry
    // 0x75 — ManufacturerData
    // -------------------------------------------------------------------------
    strncpy(bd.firmware_version,                  "1.0.0",         32);
    strncpy(bd.hardware_version,                  "RevA",          32);
    strncpy(bd.manufacturer_name,                 "OpenBatt Team", 32);
    strncpy(bd.device_name,                       "OpenBMS",       32);
    strncpy(bd.device_chemistry,                  "Li-Ion",        32);
    strncpy(bd.manufacturer_data,                 "Year 2026",     32);

    // -------------------------------------------------------------------------
    // 0xB0 — RelativeSoC
    // 0xB1 — CellSoC
    // 0xB2 — CellSoH
    // 0xB3 — CellRemainingCapacity
    // 0xB4 — CellSelfDischarge
    // 0xB5 — CellQmax
    // -------------------------------------------------------------------------
    bd.relative_soc                               = 0;
    memset(bd.cell_soc,                   0, sizeof(bd.cell_soc));
    memset(bd.cell_soh,                   0, sizeof(bd.cell_soh));
    memset(bd.cell_remaining_capacity,    0, sizeof(bd.cell_remaining_capacity));
    memset(bd.cell_self_discharge,        0, sizeof(bd.cell_self_discharge));
    memset(bd.cell_qmax,                  0, sizeof(bd.cell_qmax));

    // -------------------------------------------------------------------------
    // 0xB6 — SOC grid (0% to 95% in 5% steps)
    // -------------------------------------------------------------------------
    float soc_grid_vals[20] = {
         0.0f,  5.0f, 10.0f, 15.0f, 20.0f,
        25.0f, 30.0f, 35.0f, 40.0f, 45.0f,
        50.0f, 55.0f, 60.0f, 65.0f, 70.0f,
        75.0f, 80.0f, 85.0f, 90.0f, 95.0f
    };
    memcpy(bd.soc_grid, soc_grid_vals, sizeof(soc_grid_vals));

    // -------------------------------------------------------------------------
    // 0xB7 — OCV discharge curve (V per cell)
    // -------------------------------------------------------------------------
    float ocv_dis_vals[20] = {
        3.00f, 3.20f, 3.40f, 3.48f, 3.52f,
        3.55f, 3.58f, 3.60f, 3.62f, 3.65f,
        3.68f, 3.70f, 3.73f, 3.75f, 3.78f,
        3.82f, 3.87f, 3.93f, 4.00f, 4.10f
    };
    memcpy(bd.ocv_dis, ocv_dis_vals, sizeof(ocv_dis_vals));

    // -------------------------------------------------------------------------
    // 0xB8 — OCV charge curve (V per cell)
    // -------------------------------------------------------------------------
    float ocv_chg_vals[20] = {
        3.03f, 3.23f, 3.43f, 3.51f, 3.55f,
        3.58f, 3.61f, 3.63f, 3.65f, 3.68f,
        3.71f, 3.73f, 3.76f, 3.78f, 3.81f,
        3.85f, 3.90f, 3.96f, 4.03f, 4.13f
    };
    memcpy(bd.ocv_chg, ocv_chg_vals, sizeof(ocv_chg_vals));

    // -------------------------------------------------------------------------
    // 0xB9 — R0 discharge (Ω)
    // -------------------------------------------------------------------------
    float r0_dis_vals[20] = {
        0.030f, 0.025f, 0.022f, 0.020f, 0.018f,
        0.016f, 0.015f, 0.014f, 0.014f, 0.013f,
        0.013f, 0.013f, 0.014f, 0.014f, 0.015f,
        0.015f, 0.016f, 0.017f, 0.018f, 0.020f
    };
    memcpy(bd.r0_dis, r0_dis_vals, sizeof(r0_dis_vals));

    // -------------------------------------------------------------------------
    // 0xBA — R1 discharge (Ω)
    // -------------------------------------------------------------------------
    float r1_dis_vals[20] = {
        0.012f, 0.010f, 0.009f, 0.008f, 0.008f,
        0.007f, 0.007f, 0.006f, 0.006f, 0.006f,
        0.006f, 0.006f, 0.006f, 0.007f, 0.007f,
        0.007f, 0.008f, 0.008f, 0.009f, 0.010f
    };
    memcpy(bd.r1_dis, r1_dis_vals, sizeof(r1_dis_vals));

    // -------------------------------------------------------------------------
    // 0xBB — tau1 discharge (s)
    // -------------------------------------------------------------------------
    float tau1_dis_vals[20] = {
        45.0f, 42.0f, 40.0f, 38.0f, 37.0f,
        36.0f, 35.0f, 35.0f, 34.0f, 34.0f,
        34.0f, 35.0f, 35.0f, 36.0f, 37.0f,
        38.0f, 40.0f, 42.0f, 44.0f, 46.0f
    };
    memcpy(bd.tau1_dis, tau1_dis_vals, sizeof(tau1_dis_vals));

    // -------------------------------------------------------------------------
    // 0xBC — R2 discharge (Ω)
    // -------------------------------------------------------------------------
    float r2_dis_vals[20] = {
        0.005f, 0.004f, 0.004f, 0.003f, 0.003f,
        0.003f, 0.003f, 0.003f, 0.003f, 0.003f,
        0.003f, 0.003f, 0.003f, 0.003f, 0.003f,
        0.004f, 0.004f, 0.004f, 0.005f, 0.005f
    };
    memcpy(bd.r2_dis, r2_dis_vals, sizeof(r2_dis_vals));

    // -------------------------------------------------------------------------
    // 0xBD — tau2 discharge (s)
    // -------------------------------------------------------------------------
    float tau2_dis_vals[20] = {
        450.0f, 420.0f, 400.0f, 380.0f, 360.0f,
        350.0f, 340.0f, 330.0f, 330.0f, 320.0f,
        320.0f, 330.0f, 330.0f, 340.0f, 350.0f,
        360.0f, 380.0f, 400.0f, 420.0f, 450.0f
    };
    memcpy(bd.tau2_dis, tau2_dis_vals, sizeof(tau2_dis_vals));

    // -------------------------------------------------------------------------
    // 0xBE — R0 charge (Ω)
    // -------------------------------------------------------------------------
    float r0_chg_vals[20] = {
        0.028f, 0.023f, 0.020f, 0.018f, 0.016f,
        0.015f, 0.014f, 0.013f, 0.013f, 0.012f,
        0.012f, 0.012f, 0.013f, 0.013f, 0.014f,
        0.014f, 0.015f, 0.016f, 0.017f, 0.019f
    };
    memcpy(bd.r0_chg, r0_chg_vals, sizeof(r0_chg_vals));

    // -------------------------------------------------------------------------
    // 0xBF — R1 charge (Ω)
    // -------------------------------------------------------------------------
    float r1_chg_vals[20] = {
        0.011f, 0.009f, 0.008f, 0.007f, 0.007f,
        0.006f, 0.006f, 0.006f, 0.005f, 0.005f,
        0.005f, 0.006f, 0.006f, 0.006f, 0.007f,
        0.007f, 0.007f, 0.008f, 0.008f, 0.009f
    };
    memcpy(bd.r1_chg, r1_chg_vals, sizeof(r1_chg_vals));

    // -------------------------------------------------------------------------
    // 0xC0 — tau1 charge (s)
    // -------------------------------------------------------------------------
    float tau1_chg_vals[20] = {
        40.0f, 38.0f, 36.0f, 35.0f, 34.0f,
        33.0f, 32.0f, 32.0f, 31.0f, 31.0f,
        31.0f, 32.0f, 32.0f, 33.0f, 34.0f,
        35.0f, 37.0f, 39.0f, 41.0f, 43.0f
    };
    memcpy(bd.tau1_chg, tau1_chg_vals, sizeof(tau1_chg_vals));

    // -------------------------------------------------------------------------
    // 0xC1 — R2 charge (Ω)
    // -------------------------------------------------------------------------
    float r2_chg_vals[20] = {
        0.005f, 0.004f, 0.003f, 0.003f, 0.003f,
        0.002f, 0.002f, 0.002f, 0.002f, 0.002f,
        0.002f, 0.002f, 0.002f, 0.003f, 0.003f,
        0.003f, 0.003f, 0.004f, 0.004f, 0.005f
    };
    memcpy(bd.r2_chg, r2_chg_vals, sizeof(r2_chg_vals));

    // -------------------------------------------------------------------------
    // 0xC2 — tau2 charge (s)
    // -------------------------------------------------------------------------
    float tau2_chg_vals[20] = {
        420.0f, 400.0f, 380.0f, 360.0f, 340.0f,
        330.0f, 320.0f, 310.0f, 310.0f, 300.0f,
        300.0f, 310.0f, 310.0f, 320.0f, 330.0f,
        340.0f, 360.0f, 380.0f, 400.0f, 430.0f
    };
    memcpy(bd.tau2_chg, tau2_chg_vals, sizeof(tau2_chg_vals));

    // -------------------------------------------------------------------------
    // 0xC3 — Q_nom temperature setpoints
    // 0xC4 — Q_nom capacity at each temperature
    // -------------------------------------------------------------------------
    bd.q_nom_temp_c[0]                            = -20.0f;
    bd.q_nom_temp_c[1]                            = -10.0f;
    bd.q_nom_temp_c[2]                            =   0.0f;
    bd.q_nom_temp_c[3]                            =  25.0f;
    bd.q_nom_temp_c[4]                            =  45.0f;

    bd.q_nom_temp_ah[0]                           =   3.50f;
    bd.q_nom_temp_ah[1]                           =   4.00f;
    bd.q_nom_temp_ah[2]                           =   4.50f;
    bd.q_nom_temp_ah[3]                           =   5.00f;
    bd.q_nom_temp_ah[4]                           =   4.80f;

    // -------------------------------------------------------------------------
    // 0xC5 — Nominal capacity at 25°C
    // 0xC6 — Coulombic efficiency
    // -------------------------------------------------------------------------
    bd.q_nom_ah                                   =   5.00f;
    bd.coulombic_efficiency                       =   0.998f;

    // -------------------------------------------------------------------------
    // 0xC7 — R0 ref    0xC8 — R1 ref    0xC9 — tau1 ref
    // 0xCA — R2 ref    0xCB — tau2 ref
    // -------------------------------------------------------------------------
    bd.r0_ref                                     =   0.014f;
    bd.r1_ref                                     =   0.006f;
    bd.tau1_ref                                   =  35.0f;
    bd.r2_ref                                     =   0.003f;
    bd.tau2_ref                                   = 330.0f;

    // -------------------------------------------------------------------------
    // 0xCC — Ea R0    0xCD — Ea R1    0xCE — Ea tau1
    // 0xCF — Ea R2    0xD0 — Ea tau2
    // -------------------------------------------------------------------------
    bd.ea_r0                                      =  30000.0f;
    bd.ea_r1                                      =  25000.0f;
    bd.ea_tau1                                    =  20000.0f;
    bd.ea_r2                                      =  20000.0f;
    bd.ea_tau2                                    =  15000.0f;

    // -------------------------------------------------------------------------
    // 0xD1 — Kalman process noise SOC
    // 0xD2 — Kalman process noise RC1
    // 0xD3 — Kalman process noise RC2
    // 0xD4 — Kalman measurement noise
    // -------------------------------------------------------------------------
    bd.kf_q_soc                                   =  1e-6f;
    bd.kf_q_rc1                                   =  1e-4f;
    bd.kf_q_rc2                                   =  1e-5f;
    bd.kf_r_v                                     =  1e-4f;

    // -------------------------------------------------------------------------
    // 0xD5 — Per-cell SOC float
    // 0xD6 — Per-cell V_RC1
    // 0xD7 — Per-cell V_RC2
    // -------------------------------------------------------------------------
    for(uint8_t i = 0; i < 7; i++)
    {
        bd.cell_soc_f[i]                          =  0.0f;
        bd.cell_vrc1[i]                           =  0.0f;
        bd.cell_vrc2[i]                           =  0.0f;
    }

    // -------------------------------------------------------------------------
    // 0xD8 — Per-cell covariance matrix upper triangle
    // P layout: [P00, P01, P02, P11, P12, P22]
    // -------------------------------------------------------------------------
    for(uint8_t i = 0; i < 7; i++)
    {
        bd.cell_p[i][0]                           =  0.01f;   // P00 — 1% SoC uncertainty
        bd.cell_p[i][1]                           =  0.0f;    // P01
        bd.cell_p[i][2]                           =  0.0f;    // P02
        bd.cell_p[i][3]                           =  0.001f;  // P11 — V_RC1 variance
        bd.cell_p[i][4]                           =  0.0f;    // P12
        bd.cell_p[i][5]                           =  0.001f;  // P22 — V_RC2 variance
    }

    // -------------------------------------------------------------------------
    // 0xD9 — Per-cell capacity after aging
    // 0xDA — Per-cell R0 growth factor
    // -------------------------------------------------------------------------
    for(uint8_t i = 0; i < 7; i++)
    {
        bd.cell_q_nom_ah[i]                       =  5.00f;
        bd.cell_r0_scale[i]                       =  1.00f;
    }

    // -------------------------------------------------------------------------
    // 0xDC — CycleCount
    // -------------------------------------------------------------------------
    bd.cycle_count                                =  0;

    // -------------------------------------------------------------------------
    // 0xDE — LearningStatus
    // -------------------------------------------------------------------------
    bd.learning_status                            =  0x0000;
    
}
*/
static CommErrorType_t Controls_Set(uint8_t address, uint8_t *cmd, uint8_t length)
{
    CommErrorType_t res = CE_OK;

    // Load data
    Ctrl_GetData(&cd);

    if(length != 1)
    {
        res = CE_WRONG_CMD;
    }
    else 
    {
        switch(address)
        {
            case 0x00:
            {
                if(cmd[0] >= 0x03)
                {
                    res = CE_WRONG_CMD;
                }
                else 
                {
                    Ctrl_SetMode((uint16_t) cmd[0]);
                }
            }
            break;

            case 0x01: 
            {
                if(cmd[0] != 0x00 && cmd[0] != 0x01) 
                {
                    res = CE_WRONG_CMD;
                }
                else 
                {
                    // Allow FET control only in learning mode
                    if(cd.main_control & BD_MAIN_CTR_MODE_MASK & BD_MAIN_CTR_MODE_LEARNING)
                    {
                        Periph_SetFET((bool) cmd[0]);
                    }
                    else 
                    {
                        res = CE_RO;
                    }
                }
            }
            break;

            default: 
            {
                res = CE_NO_REG; 
            }
            break;
        }
    }


    return res;
}
static CommErrorType_t Data_ReadWriteRegister(uint8_t address, uint8_t *raw_data, uint8_t *length, bool write)
{
    void    *data_point = NULL;
    bool     ro         = false;

    if(address <= 0x2F)
    {
        // Load data
        Periph_GetData(&pd);
        switch (address)
        {
            // -------------------------------------------------------
            // 0x00 — CellVoltage[7] (read-only)
            // 0x01 — CellVoltageFiltered[7] (read-only)
            // 0x02 — PackVoltage (read-only)
            // 0x03 — PackVoltageFiltered (read-only)
            // 0x04 — PackCurrent (read-only)
            // 0x05 — PackCurrentFiltered (read-only)
            // 0x06 — TemperaturePackage (read-only)
            // 0x07 — TemperatureSTM32 (read-only)
            // 0x08 — MainVddVoltage (read-only)
            // 0x09 — FETStatus (read-only)
            // 0x0A — CurrentSensorOffset
            // 0x0B — CurrentSensorGain
            // 0x0C — VoltageOffset[7]
            // 0x0D — VoltageGain[7]
            // 0x0E — NTC_Beta
            // 0x0F — NTC_R_Nominal
            // 0x10 — NTC_R_Fixed
            // 0x11 — NTC_T_Nominal
            // 0x12 — TemperatureOffset
            // -------------------------------------------------------
            case 0x00: { data_point = (void *)pd.cell_voltage;                                *length = sizeof(pd.cell_voltage);                               ro = true; } break;
            case 0x01: { data_point = (void *)pd.cell_voltage_filtered;                       *length = sizeof(pd.cell_voltage_filtered);                      ro = true; } break;
            case 0x02: { data_point = (void *)&pd.pack_voltage;                               *length = sizeof(pd.pack_voltage);                               ro = true; } break;
            case 0x03: { data_point = (void *)&pd.pack_voltage_filtered;                      *length = sizeof(pd.pack_voltage_filtered);                      ro = true; } break;
            case 0x04: { data_point = (void *)&pd.pack_current;                               *length = sizeof(pd.pack_current);                               ro = true; } break;
            case 0x05: { data_point = (void *)&pd.pack_current_filtered;                      *length = sizeof(pd.pack_current_filtered);                      ro = true; } break;
            case 0x06: { data_point = (void *)&pd.temperature_package;                        *length = sizeof(pd.temperature_package);                        ro = true; } break;
            case 0x07: { data_point = (void *)&pd.temperature_stm32;                          *length = sizeof(pd.temperature_stm32);                          ro = true; } break;
            case 0x08: { data_point = (void *)&pd.main_vdd_voltage_mv;                        *length = sizeof(pd.main_vdd_voltage_mv);                        ro = true; } break;
            case 0x09: { data_point = (void *)&pd.fet_status;                                 *length = sizeof(pd.fet_status);                                 ro = true; } break;
            case 0x0A: { data_point = (void *)&pd.current_sensor_offset;                      *length = sizeof(pd.current_sensor_offset);                      } break;
            case 0x0B: { data_point = (void *)&pd.current_sensor_gain;                        *length = sizeof(pd.current_sensor_gain);                        } break;
            case 0x0C: { data_point = (void *)pd.voltage_offset;                              *length = sizeof(pd.voltage_offset);                             } break;
            case 0x0D: { data_point = (void *)pd.voltage_gain;                                *length = sizeof(pd.voltage_gain);                               } break;
            case 0x0E: { data_point = (void *)&pd.ntc_beta;                                   *length = sizeof(pd.ntc_beta);                                   } break;
            case 0x0F: { data_point = (void *)&pd.ntc_r_nominal;                              *length = sizeof(pd.ntc_r_nominal);                              } break;
            case 0x10: { data_point = (void *)&pd.ntc_r_fixed;                                *length = sizeof(pd.ntc_r_fixed);                                } break;
            case 0x11: { data_point = (void *)&pd.ntc_t_nominal;                              *length = sizeof(pd.ntc_t_nominal);                              } break;
            case 0x12: { data_point = (void *)&pd.temperature_offset;                         *length = sizeof(pd.temperature_offset);                         } break;
            default: break;
        }
    }
    else if(address >= 0x30 && address <= 0x7F)
    {
        // Load data
        Ctrl_GetData(&cd);
        switch (address)
        {
            // -------------------------------------------------------
            // 0x30 — Configuration
            // 0x31 — MainControl
            // 0x32 — PackCapacity
            // 0x33 — MaxPackVoltage
            // 0x34 — MinPackVoltage
            // 0x35 — ChargingTerminationCurrent
            // 0x36 — UVP slow threshold      
            // 0x37 — UVP slow time
            // 0x38 — UVP fast threshold      
            // 0x39 — UVP fast time
            // 0x3A — OVP slow threshold      
            // 0x3B — OVP slow time
            // 0x3C — OVP fast threshold      
            // 0x3D — OVP fast time
            // 0x3E — Charge OCP threshold    
            // 0x3F — Charge OCP time
            // 0x40 — Slow OCP threshold      
            // 0x41 — Slow OCP time
            // 0x42 — Fast OCP threshold      
            // 0x43 — Fast OCP time
            // 0x44 — OTP threshold
            // 0x45 — OTP time
            // 0x46 — FaultSnapshotVoltage[7] (read-only)
            // 0x47 — FaultSnapshotCurrent (read-only)
            // 0x48 — FaultSnapshotTemperature (read-only)
            // 0x49 — FaultSnapshotSoC (read-only)
            // 0x4A — FaultCode[8] (read-only)
            // 0x4B — FaultTimestamp[8] (read-only)
            // 0x4C — CellBalancingEnergy[7] (read-only)
            // 0x4D — CellBalancingTime[7] (read-only)
            // 0x4E — CellDeepestDischarge[7] (read-only)
            // 0x4F — CellMaxTemperature[7] (read-only)
            // 0x50 — LastCommunicationTimestamp (read-only)
            // 0x51 — UptimeCounter (read-only)
            // 0x52 — FirmwareVersion[32] (read-only)
            // 0x53 — HardwareVersion[32] (read-only)
            // 0x54 — ManufacturerName[32] (read-only)
            // 0x55 — DeviceName[32] (read-only)
            // 0x56 — DeviceChemistry[32] (read-only)
            // 0x57 — ManufacturerData[32] (read-only)
            // -------------------------------------------------------
            case 0x30: { data_point = (void *)&cd.configuration;                              *length = sizeof(cd.configuration);                              } break;
            case 0x31: { data_point = (void *)&cd.main_control;                               *length = sizeof(cd.main_control);                               } break;
            case 0x32: { data_point = (void *)&cd.cell_capacity;                              *length = sizeof(cd.cell_capacity);                              } break;
            case 0x33: { data_point = (void *)&cd.voltage_cell_max;                           *length = sizeof(cd.voltage_cell_max);                           } break;
            case 0x34: { data_point = (void *)&cd.voltage_cell_min;                           *length = sizeof(cd.voltage_cell_min);                           } break;
            case 0x35: { data_point = (void *)&cd.charging_term_current;                      *length = sizeof(cd.charging_term_current);                      } break;
            case 0x36: { data_point = (void *)&cd.uvp_slow_threshold_mv;                      *length = sizeof(cd.uvp_slow_threshold_mv);                      } break;
            case 0x37: { data_point = (void *)&cd.uvp_slow_time_ms;                           *length = sizeof(cd.uvp_slow_time_ms);                           } break;
            case 0x38: { data_point = (void *)&cd.uvp_fast_threshold_mv;                      *length = sizeof(cd.uvp_fast_threshold_mv);                      } break;
            case 0x39: { data_point = (void *)&cd.uvp_fast_time_ms;                           *length = sizeof(cd.uvp_fast_time_ms);                           } break;
            case 0x3A: { data_point = (void *)&cd.ovp_slow_threshold_mv;                      *length = sizeof(cd.ovp_slow_threshold_mv);                      } break;
            case 0x3B: { data_point = (void *)&cd.ovp_slow_time_ms;                           *length = sizeof(cd.ovp_slow_time_ms);                           } break;
            case 0x3C: { data_point = (void *)&cd.ovp_fast_threshold_mv;                      *length = sizeof(cd.ovp_fast_threshold_mv);                      } break;
            case 0x3D: { data_point = (void *)&cd.ovp_fast_time_ms;                           *length = sizeof(cd.ovp_fast_time_ms);                           } break;
            case 0x3E: { data_point = (void *)&cd.ocp_charge_threshold_ma;                    *length = sizeof(cd.ocp_charge_threshold_ma);                    } break;
            case 0x3F: { data_point = (void *)&cd.ocp_charge_time_ms;                         *length = sizeof(cd.ocp_charge_time_ms);                         } break;
            case 0x40: { data_point = (void *)&cd.ocp_discharge_slow_threshold_ma;             *length = sizeof(cd.ocp_discharge_slow_threshold_ma);            } break;
            case 0x41: { data_point = (void *)&cd.ocp_discharge_slow_time_ms;                 *length = sizeof(cd.ocp_discharge_slow_time_ms);                 } break;
            case 0x42: { data_point = (void *)&cd.ocp_discharge_fast_threshold_ma;            *length = sizeof(cd.ocp_discharge_fast_threshold_ma);            } break;
            case 0x43: { data_point = (void *)&cd.ocp_discharge_fast_time_ms;                 *length = sizeof(cd.ocp_discharge_fast_time_ms);                 } break;
            case 0x44: { data_point = (void *)&cd.otp_threshold_c;                            *length = sizeof(cd.otp_threshold_c);                            } break;
            case 0x45: { data_point = (void *)&cd.otp_time_ms;                                *length = sizeof(cd.otp_time_ms);                                } break;
            case 0x46: { data_point = (void *)cd.fault_snapshot_voltage;                      *length = sizeof(cd.fault_snapshot_voltage);                     ro = true; } break;
            case 0x47: { data_point = (void *)&cd.fault_snapshot_current;                     *length = sizeof(cd.fault_snapshot_current);                     ro = true; } break;
            case 0x48: { data_point = (void *)&cd.fault_snapshot_temperature;                 *length = sizeof(cd.fault_snapshot_temperature);                 ro = true; } break;
            case 0x49: { data_point = (void *)&cd.fault_snapshot_soc;                         *length = sizeof(cd.fault_snapshot_soc);                         ro = true; } break;
            case 0x4A: { data_point = (void *)cd.fault_code;                                  *length = sizeof(cd.fault_code);                                 ro = true; } break;
            case 0x4B: { data_point = (void *)cd.fault_timestamp;                             *length = sizeof(cd.fault_timestamp);                            ro = true; } break;
            case 0x4C: { data_point = (void *)cd.cell_balancing_energy;                       *length = sizeof(cd.cell_balancing_energy);                      ro = true; } break;
            case 0x4D: { data_point = (void *)cd.cell_balancing_time;                         *length = sizeof(cd.cell_balancing_time);                        ro = true; } break;
            case 0x4E: { data_point = (void *)cd.cell_deepest_discharge;                      *length = sizeof(cd.cell_deepest_discharge);                     ro = true; } break;
            case 0x4F: { data_point = (void *)cd.cell_max_temperature;                        *length = sizeof(cd.cell_max_temperature);                       ro = true; } break;
            case 0x50: { data_point = (void *)&cd.last_communication_timestamp;               *length = sizeof(cd.last_communication_timestamp);               ro = true; } break;
            case 0x51: { data_point = (void *)&cd.uptime_counter;                             *length = sizeof(cd.uptime_counter);                             ro = true; } break;
            case 0x52: { data_point = (void *)cd.firmware_version;                            *length = sizeof(cd.firmware_version);                           ro = true; } break;
            case 0x53: { data_point = (void *)cd.hardware_version;                            *length = sizeof(cd.hardware_version);                           ro = true; } break;
            case 0x54: { data_point = (void *)cd.manufacturer_name;                           *length = sizeof(cd.manufacturer_name);                          ro = true; } break;
            case 0x55: { data_point = (void *)cd.device_name;                                 *length = sizeof(cd.device_name);                                ro = true; } break;
            case 0x56: { data_point = (void *)cd.device_chemistry;                            *length = sizeof(cd.device_chemistry);                           ro = true; } break;
            case 0x57: { data_point = (void *)cd.manufacturer_data;                           *length = sizeof(cd.manufacturer_data);                          ro = true; } break;
            default: break;
        }
    }
    else if(address >= 0x80 && address <= 0xD9)
    {
        switch (address)
        {
            // -------------------------------------------------------
            // 0x80 — RelativeSoC (read-only)
            // 0x81 — CellSoC[7] (read-only)
            // 0x82 — CellSoH[7] (read-only)
            // 0x83 — CellRemainingCapacity[7] (read-only)
            // 0x84 — CellSelfDischarge[7] (read-only)
            // 0x85 — CellQmax[7] (read-only)
            // 0xB6 — SOC grid[20]
            // 0xB7 — OCV discharge[20]
            // 0xB8 — OCV charge[20]
            // 0xB9 — R0 discharge[20]    
            // 0xBA — R1 discharge[20]
            // 0xBB — tau1 discharge[20]  
            // 0xBC — R2 discharge[20]
            // 0xBD — tau2 discharge[20]
            // 0xBE — R0 charge[20]    
            // 0xBF — R1 charge[20]
            // 0x90 — tau1 charge[20]  
            // 0x91 — R2 charge[20]
            // 0x92 — tau2 charge[20]
            // 0x93 — Q_nom temperature setpoints[5]
            // 0x94 — Q_nom at each temperature[5]
            // 0x95 — Nominal capacity at 25°C
            // 0x96 — Coulombic efficiency
            // 0x97 — R0 ref    
            // 0x98 — R1 ref    
            // 0x99 — tau1 ref
            // 0x9A — R2 ref    
            // 0x9B — tau2 ref
            // 0x9C — Ea R0    
            // 0x9D — Ea R1    
            // 0x9E — Ea tau1
            // 0x9F — Ea R2    
            // 0xA0 — Ea tau2
            // 0xA1 — KF process noise SOC    
            // 0xA2 — KF process noise RC1
            // 0xA3 — KF process noise RC2    
            // 0xA4 — KF measurement noise
            // 0xA5 — cell_soc_f[7] (read-only)
            // 0xA6 — cell_vrc1[7] (read-only)
            // 0xA7 — cell_vrc2[7] (read-only)
            // 0xA8 — cell_p[7][6]
            // 0xA9 — cell_q_nom_ah[7] (read-only)
            // 0xAA — cell_r0_scale[7] (read-only)
            // 0xAB — CycleCount (read-only)
            // 0xAC — LearningStatus (read-only)
            // -------------------------------------------------------
            case 0x80: { data_point = (void *)&fg.relative_soc;                               *length = sizeof(fg.relative_soc);                               ro = true; } break;
            case 0x81: { data_point = (void *)&fg.cell_soc[0];                                *length = sizeof(fg.cell_soc);                                   ro = true; } break;
            case 0x82: { data_point = (void *)&fg.cell_soh[0];                                *length = sizeof(fg.cell_soh);                                   ro = true; } break;
            case 0x83: { data_point = (void *)&fg.cell_remaining_capacity[0];                 *length = sizeof(fg.cell_remaining_capacity);                    ro = true; } break;
            case 0x84: { data_point = (void *)&fg.cell_self_discharge[0];                     *length = sizeof(fg.cell_self_discharge);                        ro = true; } break;
            case 0x85: { data_point = (void *)&fg.cell_qmax[0];                               *length = sizeof(fg.cell_qmax);                                  ro = true; } break;
            case 0x86: { data_point = (void *)fg.soc_grid;                                    *length = sizeof(fg.soc_grid);                                   } break;
            case 0x87: { data_point = (void *)fg.ocv_dis;                                     *length = sizeof(fg.ocv_dis);                                    } break;
            case 0x88: { data_point = (void *)fg.ocv_chg;                                     *length = sizeof(fg.ocv_chg);                                    } break;
            case 0x89: { data_point = (void *)fg.r0_dis;                                      *length = sizeof(fg.r0_dis);                                     } break;
            case 0x8A: { data_point = (void *)fg.r1_dis;                                      *length = sizeof(fg.r1_dis);                                     } break;
            case 0x8B: { data_point = (void *)fg.tau1_dis;                                    *length = sizeof(fg.tau1_dis);                                   } break;
            case 0x8C: { data_point = (void *)fg.r2_dis;                                      *length = sizeof(fg.r2_dis);                                     } break;
            case 0x8D: { data_point = (void *)fg.tau2_dis;                                    *length = sizeof(fg.tau2_dis);                                   } break;
            case 0x8E: { data_point = (void *)fg.r0_chg;                                      *length = sizeof(fg.r0_chg);                                     } break;
            case 0x8F: { data_point = (void *)fg.r1_chg;                                      *length = sizeof(fg.r1_chg);                                     } break;
            case 0x90: { data_point = (void *)fg.tau1_chg;                                    *length = sizeof(fg.tau1_chg);                                   } break;
            case 0x91: { data_point = (void *)fg.r2_chg;                                      *length = sizeof(fg.r2_chg);                                     } break;
            case 0x92: { data_point = (void *)fg.tau2_chg;                                    *length = sizeof(fg.tau2_chg);                                   } break;
            case 0x93: { data_point = (void *)fg.q_nom_temp_c;                                *length = sizeof(fg.q_nom_temp_c);                               } break;
            case 0x94: { data_point = (void *)fg.q_nom_temp_ah;                               *length = sizeof(fg.q_nom_temp_ah);                              } break;
            case 0x95: { data_point = (void *)&fg.q_nom_ah;                                   *length = sizeof(fg.q_nom_ah);                                   } break;
            case 0x96: { data_point = (void *)&fg.coulombic_efficiency;                       *length = sizeof(fg.coulombic_efficiency);                       } break;
            case 0x97: { data_point = (void *)&fg.r0_ref;                                     *length = sizeof(fg.r0_ref);                                     } break;
            case 0x98: { data_point = (void *)&fg.r1_ref;                                     *length = sizeof(fg.r1_ref);                                     } break;
            case 0x99: { data_point = (void *)&fg.tau1_ref;                                   *length = sizeof(fg.tau1_ref);                                   } break;
            case 0x9A: { data_point = (void *)&fg.r2_ref;                                     *length = sizeof(fg.r2_ref);                                     } break;
            case 0x9B: { data_point = (void *)&fg.tau2_ref;                                   *length = sizeof(fg.tau2_ref);                                   } break;
            case 0x9C: { data_point = (void *)&fg.ea_r0;                                      *length = sizeof(fg.ea_r0);                                      } break;
            case 0x9D: { data_point = (void *)&fg.ea_r1;                                      *length = sizeof(fg.ea_r1);                                      } break;
            case 0x9E: { data_point = (void *)&fg.ea_tau1;                                    *length = sizeof(fg.ea_tau1);                                    } break;
            case 0x9F: { data_point = (void *)&fg.ea_r2;                                      *length = sizeof(fg.ea_r2);                                      } break;
            case 0xA0: { data_point = (void *)&fg.ea_tau2;                                    *length = sizeof(fg.ea_tau2);                                    } break;
            case 0xA1: { data_point = (void *)&fg.kf_q_soc;                                   *length = sizeof(fg.kf_q_soc);                                   } break;
            case 0xA2: { data_point = (void *)&fg.kf_q_rc1;                                   *length = sizeof(fg.kf_q_rc1);                                   } break;
            case 0xA3: { data_point = (void *)&fg.kf_q_rc2;                                   *length = sizeof(fg.kf_q_rc2);                                   } break;
            case 0xA4: { data_point = (void *)&fg.kf_r_v;                                     *length = sizeof(fg.kf_r_v);                                     } break;
            case 0xA5: { data_point = (void *)fg.cell_soc_f;                                  *length = sizeof(fg.cell_soc_f);                                 ro = true; } break;
            case 0xA6: { data_point = (void *)fg.cell_vrc1;                                   *length = sizeof(fg.cell_vrc1);                                  ro = true; } break;
            case 0xA7: { data_point = (void *)fg.cell_vrc2;                                   *length = sizeof(fg.cell_vrc2);                                  ro = true; } break;
            case 0xA8: { data_point = (void *)fg.cell_p;                                      *length = sizeof(fg.cell_p);                                     } break;
            case 0xA9: { data_point = (void *)fg.cell_q_nom_ah;                               *length = sizeof(fg.cell_q_nom_ah);                              ro = true; } break;
            case 0xAA: { data_point = (void *)fg.cell_r0_scale;                               *length = sizeof(fg.cell_r0_scale);                              ro = true; } break;
            case 0xAB: { data_point = (void *)&fg.cycle_count;                                *length = sizeof(fg.cycle_count);                                ro = true; } break;
            case 0xAC: { data_point = (void *)&fg.learning_status;                            *length = sizeof(fg.learning_status);                            ro = true; } break;
            default: break;
        }
    }

    if(data_point == NULL)
    {
        return CE_NO_REG;
    }

    if(write && ro)
    {
        return CE_RO;
    }
    else if(write && !ro)
    {
        memcpy(data_point, raw_data, *length);
    }
    else
    {
        memcpy(raw_data, data_point, *length);
    }

    return CE_OK;
}
static void RemoveAppFlag(void)
{
    HAL_StatusTypeDef      status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t               page_error = 0;

    status = HAL_FLASH_Unlock();
    if(status != HAL_OK)
    {
        //return ERROR_ERRASE_FAIL;
    }

    erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase_init.Banks       = FLASH_BANK_1;
    erase_init.Page        = APP_FLAG_FLASH_LAGE;
    erase_init.NbPages     = 1;

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);

    HAL_FLASH_Lock();

    if(status != HAL_OK)
    {
        //return ERROR_ERRASE_FAIL;
    }

    //return OK;
}
static void ProcessCommand()
{
    uint8_t             *rx_buf   = uart_cmd.rx_buffer;
    uint8_t             *tx_buf   = uart_cmd.tx_buffer;
    uint8_t             cmd;
    uint8_t             length;
    uint8_t             address;
    uint8_t             crc_calc, crc_rec;
    bool                write = false;
    CommErrorType_t     res;

    // -------------------------------------------------------
    // Parse command type
    // -------------------------------------------------------
    cmd = rx_buf[0];
    if( cmd != CC_WRITE && 
        cmd != CC_READ && 
        cmd != CC_BOOTLOADER &&
        cmd != CC_CMD)
    {
        SendError(CE_WRONG_CMD);
        return;
    }

    // -------------------------------------------------------
    // Perform Bootloader Running or Command Execution
    // -------------------------------------------------------
    if(cmd == CC_BOOTLOADER)
    {
        RemoveAppFlag();
        SendResponse((uint8_t*)"1\n",2);
        HAL_Delay(100);
        NVIC_SystemReset();
    }

    // If it's not bootloader running, then just continue

    // -------------------------------------------------------
    // Parse address and length
    // -------------------------------------------------------
    address = rx_buf[1];
    length  = rx_buf[2];

    // -------------------------------------------------------
    // Validate CRC
    // -------------------------------------------------------
    crc_calc = Checksum(rx_buf, length + 3);
    crc_rec  = rx_buf[length + 3];

    if(crc_calc != crc_rec)
    {
        SendError(CE_BAD_CRC);
        return;
    }

    if(cmd == CC_CMD)
    {
        res = Controls_Set(address, &rx_buf[3], length);

        if(res == CE_OK)
        {
            // ACK: [CC_ACK][code][crc]
            tx_buf[0] = CC_ACK;
            tx_buf[1] = 0;
            tx_buf[2] = 0;
            tx_buf[3] = Checksum(tx_buf, 3);
            length = 0;

        }
        else
        {
            SendError( res);
            return;
        }
    }
    else if(cmd == CC_WRITE || cmd == CC_READ)
    {
        // -------------------------------------------------------
        // Execute read or write
        // -------------------------------------------------------
        if     (cmd == 0x01) write = true;
        else if(cmd == 0x02) write = false;

        if(write)
        {
            res = Data_ReadWriteRegister(address, &rx_buf[3], &length, true);
        }
        else
        {
            // During read operation, disable interrupts from ADS to prevent data corruption
            HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);    // ADS131 DRDY
            HAL_NVIC_DisableIRQ(ADC1_IRQn);         // STM32 internal ADC
            res = Data_ReadWriteRegister(address, &tx_buf[3], &length, false);
            HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); 
            HAL_NVIC_EnableIRQ(ADC1_IRQn);

        }

        if(res != CE_OK)
        {
            SendError( res);
            return;
        }

        // -------------------------------------------------------
        // Respond
        // -------------------------------------------------------
        if(write)
        {
            // ACK: [CC_ACK][code][crc]
            tx_buf[0] = CC_ACK;
            tx_buf[1] = 0;
            tx_buf[2] = 0;
            tx_buf[3] = Checksum(tx_buf, 3);
            length = 0;
        }
        else
        {
            // Read response: [CC_READ][address][length][data...][crc]
            tx_buf[0] = CC_READ;
            tx_buf[1] = address;
            tx_buf[2] = length;
            // Data is already store at this moment
            tx_buf[length + 3] = Checksum(tx_buf, length + 3);
        }
    }

    SendResponse(tx_buf, length + 4);
}
static uint8_t Checksum(uint8_t *data, uint16_t length)
{
    uint8_t sum = 0;
    for(uint16_t i = 0; i < length; i++)
    {
        sum += (uint8_t)data[i];
    }
    return sum;
}
static void SendResponse(uint8_t *data, uint8_t length)
{
    HAL_UART_Transmit_IT(&huart1, (uint8_t *)data, length);
}
static void SendError(CommErrorType_t err)
{
    uint8_t buff[5];

    buff[0] = CC_ERROR;
    buff[1] = 0;    // Address
    buff[2] = 1;    // Lenght
    buff[3] = err; // Error type
    buff[4] = Checksum(buff, 4);

    SendResponse(buff, sizeof(buff));
}
void Comm_Init(void)
{
    // Start data receiving on UART
    HAL_UARTEx_ReceiveToIdle_DMA(debug_uart, uart_cmd.rx_buffer, UART_RX_BUFFER_SIZE);
}
void Comm_Run(void)
{
  if(uart_cmd.frame_ready)
  {
    ProcessCommand();
    uart_cmd.frame_ready = false;
    uart_cmd.rx_length   = 0;
  }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if(huart->Instance == USART1)
    {
        uart_cmd.rx_length       = size;
        uart_cmd.frame_ready     = true;
        HAL_UARTEx_ReceiveToIdle_DMA(debug_uart, uart_cmd.rx_buffer, UART_RX_BUFFER_SIZE);
    }
}