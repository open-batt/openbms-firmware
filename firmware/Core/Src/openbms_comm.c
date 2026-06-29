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
#include "stm32l431xx.h"
#include <stdint.h>

#define APP_FLAG_FLASH_LAGE     19U

UART_HandleTypeDef        *debug_uart           = &huart1;
static UART_Command_t     uart_cmd              = {0};
OpenBMS_Data_t OpenBMS_data      = {0};

static CommErrorType_t  SBS_ReadWriteRegister(OpenBMS_Data_t *sbs, uint8_t address, uint8_t *raw_data, uint8_t *length, bool write); 
static void             SBS_SetDefaults(OpenBMS_Data_t *sbs);

static void             RemoveAppFlag(void);
static void             ProcessCommand(void);
static uint8_t          Checksum(uint8_t *data, uint16_t length);
static void             SendResponse(uint8_t *data, uint8_t length);
static void             SendError(CommErrorType_t err);

static CommErrorType_t SBS_ReadWriteRegister(OpenBMS_Data_t *sbs, uint8_t address, uint8_t *raw_data, uint8_t *length, bool write)
{
    void    *data_point = NULL;
    bool     ro         = false;

    switch (address)
    {
        // -------------------------------------------------------
        // 0x00 — ManufacturerAccess (read-only)
        // -------------------------------------------------------
        case 0x00: { data_point = (void *)&sbs->manufacturer_access;                        *length = sizeof(sbs->manufacturer_access);                        ro = true; } break;

        // -------------------------------------------------------
        // 0x01 — RemainingCapacityAlarm
        // 0x02 — RemainingTimeAlarm
        // -------------------------------------------------------
        case 0x01: { data_point = (void *)&sbs->remaining_capacity_alarm;                   *length = sizeof(sbs->remaining_capacity_alarm);                   } break;
        case 0x02: { data_point = (void *)&sbs->remaining_time_alarm;                       *length = sizeof(sbs->remaining_time_alarm);                       } break;

        // -------------------------------------------------------
        // 0x03 — BatteryMode
        // -------------------------------------------------------
        case 0x03: { data_point = (void *)&sbs->battery_mode;                               *length = sizeof(sbs->battery_mode);                               } break;

        // -------------------------------------------------------
        // 0x04 — AtRate
        // 0x05 — AtRateTimeToFull (read-only)
        // 0x06 — AtRateTimeToEmpty (read-only)
        // 0x07 — AtRateOK (read-only)
        // -------------------------------------------------------
        case 0x04: { data_point = (void *)&sbs->at_rate;                                    *length = sizeof(sbs->at_rate);                                    } break;
        case 0x05: { data_point = (void *)&sbs->at_rate_time_to_full;                       *length = sizeof(sbs->at_rate_time_to_full);                       ro = true; } break;
        case 0x06: { data_point = (void *)&sbs->at_rate_time_to_empty;                      *length = sizeof(sbs->at_rate_time_to_empty);                      ro = true; } break;
        case 0x07: { data_point = (void *)&sbs->at_rate_ok;                                 *length = sizeof(sbs->at_rate_ok);                                 ro = true; } break;

        // -------------------------------------------------------
        // 0x08 — Temperature (read-only)
        // 0x09 — Voltage (read-only)
        // 0x0A — Current (read-only)
        // 0x0B — AverageCurrent (read-only)
        // -------------------------------------------------------
        case 0x08: { data_point = (void *)&sbs->temperature_package;                        *length = sizeof(sbs->temperature_package);                        ro = true; } break;
        case 0x09: { data_point = (void *)&sbs->voltage;                                    *length = sizeof(sbs->voltage);                                    ro = true; } break;
        case 0x0A: { data_point = (void *)&sbs->current;                                    *length = sizeof(sbs->current);                                    ro = true; } break;
        case 0x0B: { data_point = (void *)&sbs->average_current;                            *length = sizeof(sbs->average_current);                            ro = true; } break;

        // -------------------------------------------------------
        // 0x0C — MaxError (read-only)
        // 0x0D — RelativeStateOfCharge (read-only)
        // 0x0E — AbsoluteStateOfCharge (read-only)
        // -------------------------------------------------------
        case 0x0C: { data_point = (void *)&sbs->max_error;                                  *length = sizeof(sbs->max_error);                                  ro = true; } break;
        case 0x0D: { data_point = (void *)&sbs->relative_soc;                               *length = sizeof(sbs->relative_soc);                               ro = true; } break;
        case 0x0E: { data_point = (void *)&sbs->absolute_soc;                               *length = sizeof(sbs->absolute_soc);                               ro = true; } break;

        // -------------------------------------------------------
        // 0x0F — RemainingCapacity (read-only)
        // 0x10 — FullChargeCapacity (read-only)
        // -------------------------------------------------------
        case 0x0F: { data_point = (void *)&sbs->remaining_capacity;                         *length = sizeof(sbs->remaining_capacity);                         ro = true; } break;
        case 0x10: { data_point = (void *)&sbs->full_charge_capacity;                       *length = sizeof(sbs->full_charge_capacity);                       ro = true; } break;

        // -------------------------------------------------------
        // 0x11 — RunTimeToEmpty (read-only)
        // 0x12 — AverageTimeToEmpty (read-only)
        // 0x13 — AverageTimeToFull (read-only)
        // -------------------------------------------------------
        case 0x11: { data_point = (void *)&sbs->run_time_to_empty;                          *length = sizeof(sbs->run_time_to_empty);                          ro = true; } break;
        case 0x12: { data_point = (void *)&sbs->average_time_to_empty;                      *length = sizeof(sbs->average_time_to_empty);                      ro = true; } break;
        case 0x13: { data_point = (void *)&sbs->average_time_to_full;                       *length = sizeof(sbs->average_time_to_full);                       ro = true; } break;

        // -------------------------------------------------------
        // 0x14 — ChargingCurrent
        // 0x15 — ChargingVoltage
        // -------------------------------------------------------
        case 0x14: { data_point = (void *)&sbs->charging_current;                           *length = sizeof(sbs->charging_current);                           } break;
        case 0x15: { data_point = (void *)&sbs->charging_voltage;                           *length = sizeof(sbs->charging_voltage);                           } break;

        // -------------------------------------------------------
        // 0x16 — BatteryStatus
        // -------------------------------------------------------
        case 0x16: { data_point = (void *)&sbs->battery_status;                             *length = sizeof(sbs->battery_status);                             } break;

        // -------------------------------------------------------
        // 0x17 — CycleCount (read-only)
        // -------------------------------------------------------
        case 0x17: { data_point = (void *)&sbs->cycle_count;                                *length = sizeof(sbs->cycle_count);                                ro = true; } break;

        // -------------------------------------------------------
        // 0x18 — DesignCapacity
        // 0x19 — DesignVoltage
        // -------------------------------------------------------
        case 0x18: { data_point = (void *)&sbs->design_capacity;                            *length = sizeof(sbs->design_capacity);                            } break;
        case 0x19: { data_point = (void *)&sbs->design_voltage;                             *length = sizeof(sbs->design_voltage);                             } break;

        // -------------------------------------------------------
        // 0x1A — SpecificationInfo (read-only)
        // 0x1B — ManufactureDate (read-only)
        // 0x1C — SerialNumber (read-only)
        // -------------------------------------------------------
        case 0x1A: { data_point = (void *)&sbs->specification_info;                         *length = sizeof(sbs->specification_info);                         ro = true; } break;
        case 0x1B: { data_point = (void *)&sbs->manufacture_date;                           *length = sizeof(sbs->manufacture_date);                           ro = true; } break;
        case 0x1C: { data_point = (void *)&sbs->serial_number;                              *length = sizeof(sbs->serial_number);                              ro = true; } break;

        // -------------------------------------------------------
        // 0x20 — ManufacturerName (read-only)
        // 0x21 — DeviceName (read-only)
        // 0x22 — DeviceChemistry (read-only)
        // 0x23 — ManufacturerData (read-only)
        // -------------------------------------------------------
        case 0x20: { data_point = (void *)sbs->manufacturer_name;                           *length = sizeof(sbs->manufacturer_name);                          ro = true; } break;
        case 0x21: { data_point = (void *)sbs->device_name;                                 *length = sizeof(sbs->device_name);                                ro = true; } break;
        case 0x22: { data_point = (void *)sbs->device_chemistry;                            *length = sizeof(sbs->device_chemistry);                           ro = true; } break;
        case 0x23: { data_point = (void *)sbs->manufacturer_data;                           *length = sizeof(sbs->manufacturer_data);                          ro = true; } break;

        // -------------------------------------------------------
        // 0x40 — Configuration
        // 0x41 — MainControl
        // -------------------------------------------------------
        case 0x40: { data_point = (void *)&sbs->configuration;                              *length = sizeof(sbs->configuration);                              } break;
        case 0x41: { data_point = (void *)&sbs->main_control;                               *length = sizeof(sbs->main_control);                               } break;

        // -------------------------------------------------------
        // 0x42 — PackCapacity
        // 0x43 — MaxPackVoltage
        // 0x44 — MinPackVoltage
        // -------------------------------------------------------
        case 0x42: { data_point = (void *)&sbs->pack_capacity;                              *length = sizeof(sbs->pack_capacity);                              } break;
        case 0x43: { data_point = (void *)&sbs->voltage_pack_max;                           *length = sizeof(sbs->voltage_pack_max);                           } break;
        case 0x44: { data_point = (void *)&sbs->voltage_pack_min;                           *length = sizeof(sbs->voltage_pack_min);                           } break;

        // -------------------------------------------------------
        // 0x45 — CurrentSensorOffset
        // 0x46 — CurrentSensorGain
        // -------------------------------------------------------
        case 0x45: { data_point = (void *)&sbs->current_sensor_offset;                      *length = sizeof(sbs->current_sensor_offset);                      } break;
        case 0x46: { data_point = (void *)&sbs->current_sensor_gain;                        *length = sizeof(sbs->current_sensor_gain);                        } break;

        // -------------------------------------------------------
        // 0x47 — VoltageOffset[7]
        // 0x48 — VoltageGain[7]
        // -------------------------------------------------------
        case 0x47: { data_point = (void *)sbs->voltage_offset;                              *length = sizeof(sbs->voltage_offset);                             } break;
        case 0x48: { data_point = (void *)sbs->voltage_gain;                                *length = sizeof(sbs->voltage_gain);                               } break;

        // -------------------------------------------------------
        // 0x49 — NTC_Beta
        // 0x4A — NTC_R_Nominal
        // 0x4B — NTC_R_Fixed
        // 0x4C — NTC_T_Nominal
        // 0x4D — TemperatureOffset
        // -------------------------------------------------------
        case 0x49: { data_point = (void *)&sbs->ntc_beta;                                   *length = sizeof(sbs->ntc_beta);                                   } break;
        case 0x4A: { data_point = (void *)&sbs->ntc_r_nominal;                              *length = sizeof(sbs->ntc_r_nominal);                              } break;
        case 0x4B: { data_point = (void *)&sbs->ntc_r_fixed;                                *length = sizeof(sbs->ntc_r_fixed);                                } break;
        case 0x4C: { data_point = (void *)&sbs->ntc_t_nominal;                              *length = sizeof(sbs->ntc_t_nominal);                              } break;
        case 0x4D: { data_point = (void *)&sbs->temperature_offset;                         *length = sizeof(sbs->temperature_offset);                         } break;

        // -------------------------------------------------------
        // 0x60 — UVP slow threshold      0x61 — UVP slow time
        // 0x62 — UVP fast threshold      0x63 — UVP fast time
        // -------------------------------------------------------
        case 0x60: { data_point = (void *)&sbs->uvp_slow_threshold_mv;                      *length = sizeof(sbs->uvp_slow_threshold_mv);                      } break;
        case 0x61: { data_point = (void *)&sbs->uvp_slow_time_ms;                           *length = sizeof(sbs->uvp_slow_time_ms);                           } break;
        case 0x62: { data_point = (void *)&sbs->uvp_fast_threshold_mv;                      *length = sizeof(sbs->uvp_fast_threshold_mv);                      } break;
        case 0x63: { data_point = (void *)&sbs->uvp_fast_time_ms;                           *length = sizeof(sbs->uvp_fast_time_ms);                           } break;

        // -------------------------------------------------------
        // 0x64 — OVP slow threshold      0x65 — OVP slow time
        // 0x66 — OVP fast threshold      0x67 — OVP fast time
        // -------------------------------------------------------
        case 0x64: { data_point = (void *)&sbs->ovp_slow_threshold_mv;                      *length = sizeof(sbs->ovp_slow_threshold_mv);                      } break;
        case 0x65: { data_point = (void *)&sbs->ovp_slow_time_ms;                           *length = sizeof(sbs->ovp_slow_time_ms);                           } break;
        case 0x66: { data_point = (void *)&sbs->ovp_fast_threshold_mv;                      *length = sizeof(sbs->ovp_fast_threshold_mv);                      } break;
        case 0x67: { data_point = (void *)&sbs->ovp_fast_time_ms;                           *length = sizeof(sbs->ovp_fast_time_ms);                           } break;

        // -------------------------------------------------------
        // 0x68 — Charge OCP threshold    0x69 — Charge OCP time
        // 0x6A — Slow OCP threshold      0x6B — Slow OCP time
        // 0x6C — Fast OCP threshold      0x6D — Fast OCP time
        // -------------------------------------------------------
        case 0x68: { data_point = (void *)&sbs->ocp_charge_threshold_ma;                    *length = sizeof(sbs->ocp_charge_threshold_ma);                    } break;
        case 0x69: { data_point = (void *)&sbs->ocp_charge_time_ms;                         *length = sizeof(sbs->ocp_charge_time_ms);                         } break;
        case 0x6A: { data_point = (void *)&sbs->ocp_discharge_slow_threshold_ma;            *length = sizeof(sbs->ocp_discharge_slow_threshold_ma);            } break;
        case 0x6B: { data_point = (void *)&sbs->ocp_discharge_slow_time_ms;                 *length = sizeof(sbs->ocp_discharge_slow_time_ms);                 } break;
        case 0x6C: { data_point = (void *)&sbs->ocp_discharge_fast_threshold_ma;            *length = sizeof(sbs->ocp_discharge_fast_threshold_ma);            } break;
        case 0x6D: { data_point = (void *)&sbs->ocp_discharge_fast_time_ms;                 *length = sizeof(sbs->ocp_discharge_fast_time_ms);                 } break;

        // -------------------------------------------------------
        // 0x6E — OTP threshold
        // 0x6F — OTP time
        // -------------------------------------------------------
        case 0x6E: { data_point = (void *)&sbs->otp_threshold_c;                            *length = sizeof(sbs->otp_threshold_c);                            } break;
        case 0x6F: { data_point = (void *)&sbs->otp_time_ms;                                *length = sizeof(sbs->otp_time_ms);                                } break;

        // -------------------------------------------------------
        // 0x80 — FETStatus (read-only)
        // 0x81 — MainVddVoltage (read-only)
        // 0x82 — TemperatureSTM32 (read-only)
        // -------------------------------------------------------
        case 0x80: { data_point = (void *)&sbs->fet_status;                                 *length = sizeof(sbs->fet_status);                                 ro = true; } break;
        case 0x81: { data_point = (void *)&sbs->main_vdd_voltage_mv;                        *length = sizeof(sbs->main_vdd_voltage_mv);                        ro = true; } break;
        case 0x82: { data_point = (void *)&sbs->temperature_stm32;                          *length = sizeof(sbs->temperature_stm32);                          ro = true; } break;

        // -------------------------------------------------------
        // 0x90 — FaultSnapshotVoltage[7] (read-only)
        // 0x91 — FaultSnapshotCurrent (read-only)
        // 0x92 — FaultSnapshotTemperature (read-only)
        // 0x93 — FaultSnapshotSoC (read-only)
        // -------------------------------------------------------
        case 0x90: { data_point = (void *)sbs->fault_snapshot_voltage;                      *length = sizeof(sbs->fault_snapshot_voltage);                     ro = true; } break;
        case 0x91: { data_point = (void *)&sbs->fault_snapshot_current;                     *length = sizeof(sbs->fault_snapshot_current);                     ro = true; } break;
        case 0x92: { data_point = (void *)&sbs->fault_snapshot_temperature;                 *length = sizeof(sbs->fault_snapshot_temperature);                 ro = true; } break;
        case 0x93: { data_point = (void *)&sbs->fault_snapshot_soc;                         *length = sizeof(sbs->fault_snapshot_soc);                         ro = true; } break;

        // -------------------------------------------------------
        // 0x94 — FaultCode[8] (read-only)
        // 0x95 — FaultTimestamp[8] (read-only)
        // -------------------------------------------------------
        case 0x94: { data_point = (void *)sbs->fault_code;                                  *length = sizeof(sbs->fault_code);                                 ro = true; } break;
        case 0x95: { data_point = (void *)sbs->fault_timestamp;                             *length = sizeof(sbs->fault_timestamp);                            ro = true; } break;

        // -------------------------------------------------------
        // 0x96 — CellBalancingEnergy[7] (read-only)
        // 0x97 — CellBalancingTime[7] (read-only)
        // 0x98 — CellDeepestDischarge[7] (read-only)
        // 0x99 — CellMaxTemperature[7] (read-only)
        // -------------------------------------------------------
        case 0x96: { data_point = (void *)sbs->cell_balancing_energy;                       *length = sizeof(sbs->cell_balancing_energy);                      ro = true; } break;
        case 0x97: { data_point = (void *)sbs->cell_balancing_time;                         *length = sizeof(sbs->cell_balancing_time);                        ro = true; } break;
        case 0x98: { data_point = (void *)sbs->cell_deepest_discharge;                      *length = sizeof(sbs->cell_deepest_discharge);                     ro = true; } break;
        case 0x99: { data_point = (void *)sbs->cell_max_temperature;                        *length = sizeof(sbs->cell_max_temperature);                       ro = true; } break;

        // -------------------------------------------------------
        // 0xA0 — CellVoltageResistanceFactor
        // 0xA1 — BattVoltageResistanceFactor
        // -------------------------------------------------------
        case 0xA0: { data_point = (void *)&sbs->cell_voltage_resistance_factor;             *length = sizeof(sbs->cell_voltage_resistance_factor);             } break;
        case 0xA1: { data_point = (void *)&sbs->batt_voltage_resistance_factor;             *length = sizeof(sbs->batt_voltage_resistance_factor);             } break;

        // -------------------------------------------------------
        // 0xA2 — ShuntResistance
        // 0xA3 — CurrentSenseOffsetMv
        // 0xA4 — CurrentSenseGain
        // -------------------------------------------------------
        case 0xA2: { data_point = (void *)&sbs->shunt_resistance_mohms;                     *length = sizeof(sbs->shunt_resistance_mohms);                     } break;
        case 0xA3: { data_point = (void *)&sbs->current_sense_offset_mv;                    *length = sizeof(sbs->current_sense_offset_mv);                    } break;
        case 0xA4: { data_point = (void *)&sbs->current_sense_gain;                         *length = sizeof(sbs->current_sense_gain);                         } break;

        // -------------------------------------------------------
        // 0xA5 — BalancerResistor
        // -------------------------------------------------------
        case 0xA5: { data_point = (void *)&sbs->balancer_resistor;                          *length = sizeof(sbs->balancer_resistor);                          } break;

        // -------------------------------------------------------
        // 0xA6 — FirmwareVersion[32] (read-only)
        // 0xA7 — HardwareVersion[32] (read-only)
        // -------------------------------------------------------
        case 0xA6: { data_point = (void *)sbs->firmware_version;                            *length = sizeof(sbs->firmware_version);                           ro = true; } break;
        case 0xA7: { data_point = (void *)sbs->hardware_version;                            *length = sizeof(sbs->hardware_version);                           ro = true; } break;

        // -------------------------------------------------------
        // 0xA8 — LastCommunicationTimestamp (read-only)
        // 0xA9 — UptimeCounter (read-only)
        // -------------------------------------------------------
        case 0xA8: { data_point = (void *)&sbs->last_communication_timestamp;               *length = sizeof(sbs->last_communication_timestamp);               ro = true; } break;
        case 0xA9: { data_point = (void *)&sbs->uptime_counter;                             *length = sizeof(sbs->uptime_counter);                             ro = true; } break;

        // -------------------------------------------------------
        // 0xB0 — CellVoltage[7] (read-only)
        // 0xB1 — CellSoC[7] (read-only)
        // 0xB2 — CellSoH[7] (read-only)
        // 0xB3 — CellRemainingCapacity[7] (read-only)
        // 0xB4 — CellSelfDischarge[7] (read-only)
        // 0xB5 — CellQmax[7] (read-only)
        // -------------------------------------------------------
        case 0xB0: { data_point = (void *)sbs->cell_voltage;                                *length = sizeof(sbs->cell_voltage);                               ro = true; } break;
        case 0xB1: { data_point = (void *)sbs->cell_soc;                                    *length = sizeof(sbs->cell_soc);                                   ro = true; } break;
        case 0xB2: { data_point = (void *)sbs->cell_soh;                                    *length = sizeof(sbs->cell_soh);                                   ro = true; } break;
        case 0xB3: { data_point = (void *)sbs->cell_remaining_capacity;                     *length = sizeof(sbs->cell_remaining_capacity);                    ro = true; } break;
        case 0xB4: { data_point = (void *)sbs->cell_self_discharge;                         *length = sizeof(sbs->cell_self_discharge);                        ro = true; } break;
        case 0xB5: { data_point = (void *)sbs->cell_qmax;                                   *length = sizeof(sbs->cell_qmax);                                  ro = true; } break;

        // -------------------------------------------------------
        // 0xB6 — SOC grid[20]
        // 0xB7 — OCV discharge[20]
        // 0xB8 — OCV charge[20]
        // -------------------------------------------------------
        case 0xB6: { data_point = (void *)sbs->soc_grid;                                    *length = sizeof(sbs->soc_grid);                                   } break;
        case 0xB7: { data_point = (void *)sbs->ocv_dis;                                     *length = sizeof(sbs->ocv_dis);                                    } break;
        case 0xB8: { data_point = (void *)sbs->ocv_chg;                                     *length = sizeof(sbs->ocv_chg);                                    } break;

        // -------------------------------------------------------
        // 0xB9 — R0 discharge[20]    0xBA — R1 discharge[20]
        // 0xBB — tau1 discharge[20]  0xBC — R2 discharge[20]
        // 0xBD — tau2 discharge[20]
        // -------------------------------------------------------
        case 0xB9: { data_point = (void *)sbs->r0_dis;                                      *length = sizeof(sbs->r0_dis);                                     } break;
        case 0xBA: { data_point = (void *)sbs->r1_dis;                                      *length = sizeof(sbs->r1_dis);                                     } break;
        case 0xBB: { data_point = (void *)sbs->tau1_dis;                                    *length = sizeof(sbs->tau1_dis);                                   } break;
        case 0xBC: { data_point = (void *)sbs->r2_dis;                                      *length = sizeof(sbs->r2_dis);                                     } break;
        case 0xBD: { data_point = (void *)sbs->tau2_dis;                                    *length = sizeof(sbs->tau2_dis);                                   } break;

        // -------------------------------------------------------
        // 0xBE — R0 charge[20]    0xBF — R1 charge[20]
        // 0xC0 — tau1 charge[20]  0xC1 — R2 charge[20]
        // 0xC2 — tau2 charge[20]
        // -------------------------------------------------------
        case 0xBE: { data_point = (void *)sbs->r0_chg;                                      *length = sizeof(sbs->r0_chg);                                     } break;
        case 0xBF: { data_point = (void *)sbs->r1_chg;                                      *length = sizeof(sbs->r1_chg);                                     } break;
        case 0xC0: { data_point = (void *)sbs->tau1_chg;                                    *length = sizeof(sbs->tau1_chg);                                   } break;
        case 0xC1: { data_point = (void *)sbs->r2_chg;                                      *length = sizeof(sbs->r2_chg);                                     } break;
        case 0xC2: { data_point = (void *)sbs->tau2_chg;                                    *length = sizeof(sbs->tau2_chg);                                   } break;

        // -------------------------------------------------------
        // 0xC3 — Q_nom temperature setpoints[5]
        // 0xC4 — Q_nom at each temperature[5]
        // -------------------------------------------------------
        case 0xC3: { data_point = (void *)sbs->q_nom_temp_c;                                *length = sizeof(sbs->q_nom_temp_c);                               } break;
        case 0xC4: { data_point = (void *)sbs->q_nom_temp_ah;                               *length = sizeof(sbs->q_nom_temp_ah);                              } break;

        // -------------------------------------------------------
        // 0xC5 — Nominal capacity at 25°C
        // 0xC6 — Coulombic efficiency
        // -------------------------------------------------------
        case 0xC5: { data_point = (void *)&sbs->q_nom_ah;                                   *length = sizeof(sbs->q_nom_ah);                                   } break;
        case 0xC6: { data_point = (void *)&sbs->coulombic_efficiency;                       *length = sizeof(sbs->coulombic_efficiency);                       } break;

        // -------------------------------------------------------
        // 0xC7 — R0 ref    0xC8 — R1 ref    0xC9 — tau1 ref
        // 0xCA — R2 ref    0xCB — tau2 ref
        // -------------------------------------------------------
        case 0xC7: { data_point = (void *)&sbs->r0_ref;                                     *length = sizeof(sbs->r0_ref);                                     } break;
        case 0xC8: { data_point = (void *)&sbs->r1_ref;                                     *length = sizeof(sbs->r1_ref);                                     } break;
        case 0xC9: { data_point = (void *)&sbs->tau1_ref;                                   *length = sizeof(sbs->tau1_ref);                                   } break;
        case 0xCA: { data_point = (void *)&sbs->r2_ref;                                     *length = sizeof(sbs->r2_ref);                                     } break;
        case 0xCB: { data_point = (void *)&sbs->tau2_ref;                                   *length = sizeof(sbs->tau2_ref);                                   } break;

        // -------------------------------------------------------
        // 0xCC — Ea R0    0xCD — Ea R1    0xCE — Ea tau1
        // 0xCF — Ea R2    0xD0 — Ea tau2
        // -------------------------------------------------------
        case 0xCC: { data_point = (void *)&sbs->ea_r0;                                      *length = sizeof(sbs->ea_r0);                                      } break;
        case 0xCD: { data_point = (void *)&sbs->ea_r1;                                      *length = sizeof(sbs->ea_r1);                                      } break;
        case 0xCE: { data_point = (void *)&sbs->ea_tau1;                                    *length = sizeof(sbs->ea_tau1);                                    } break;
        case 0xCF: { data_point = (void *)&sbs->ea_r2;                                      *length = sizeof(sbs->ea_r2);                                      } break;
        case 0xD0: { data_point = (void *)&sbs->ea_tau2;                                    *length = sizeof(sbs->ea_tau2);                                    } break;

        // -------------------------------------------------------
        // 0xD1 — KF process noise SOC    0xD2 — KF process noise RC1
        // 0xD3 — KF process noise RC2    0xD4 — KF measurement noise
        // -------------------------------------------------------
        case 0xD1: { data_point = (void *)&sbs->kf_q_soc;                                   *length = sizeof(sbs->kf_q_soc);                                   } break;
        case 0xD2: { data_point = (void *)&sbs->kf_q_rc1;                                   *length = sizeof(sbs->kf_q_rc1);                                   } break;
        case 0xD3: { data_point = (void *)&sbs->kf_q_rc2;                                   *length = sizeof(sbs->kf_q_rc2);                                   } break;
        case 0xD4: { data_point = (void *)&sbs->kf_r_v;                                     *length = sizeof(sbs->kf_r_v);                                     } break;

        // -------------------------------------------------------
        // 0xD5 — cell_soc_f[7] (read-only)
        // 0xD6 — cell_vrc1[7] (read-only)
        // 0xD7 — cell_vrc2[7] (read-only)
        // -------------------------------------------------------
        case 0xD5: { data_point = (void *)sbs->cell_soc_f;                                  *length = sizeof(sbs->cell_soc_f);                                 ro = true; } break;
        case 0xD6: { data_point = (void *)sbs->cell_vrc1;                                   *length = sizeof(sbs->cell_vrc1);                                  ro = true; } break;
        case 0xD7: { data_point = (void *)sbs->cell_vrc2;                                   *length = sizeof(sbs->cell_vrc2);                                  ro = true; } break;

        // -------------------------------------------------------
        // 0xD8 — cell_p[7][6] (read-only)
        // -------------------------------------------------------
        case 0xD8: { data_point = (void *)sbs->cell_p;                                      *length = sizeof(sbs->cell_p);                                     ro = true; } break;

        // -------------------------------------------------------
        // 0xD9 — cell_q_nom_ah[7] (read-only)
        // 0xDA — cell_r0_scale[7] (read-only)
        // -------------------------------------------------------
        case 0xD9: { data_point = (void *)sbs->cell_q_nom_ah;                               *length = sizeof(sbs->cell_q_nom_ah);                              ro = true; } break;
        case 0xDA: { data_point = (void *)sbs->cell_r0_scale;                               *length = sizeof(sbs->cell_r0_scale);                              ro = true; } break;

        // -------------------------------------------------------
        // 0xDC — LearningStatus (read-only)
        // -------------------------------------------------------
        case 0xDC: { data_point = (void *)&sbs->learning_status;                            *length = sizeof(sbs->learning_status);                            ro = true; } break;

        default: break;
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
static void SBS_SetDefaults(OpenBMS_Data_t *sbs)
{
    // -------------------------------------------------------------------------
    // 0x00 — ManufacturerAccess
    // -------------------------------------------------------------------------
    *(uint16_t *)&sbs->manufacturer_access          = 0x0021;

    // -------------------------------------------------------------------------
    // 0x01 — RemainingCapacityAlarm
    // 0x02 — RemainingTimeAlarm
    // -------------------------------------------------------------------------
    sbs->remaining_capacity_alarm                   = 0;
    sbs->remaining_time_alarm                       = 0;

    // -------------------------------------------------------------------------
    // 0x03 — BatteryMode
    // -------------------------------------------------------------------------
    sbs->battery_mode                               = 0x0000;

    // -------------------------------------------------------------------------
    // 0x04 — AtRate
    // 0x05 — AtRateTimeToFull
    // 0x06 — AtRateTimeToEmpty
    // 0x07 — AtRateOK
    // -------------------------------------------------------------------------
    sbs->at_rate                                    = 0;
    sbs->at_rate_time_to_full                       = 0xFFFF;
    sbs->at_rate_time_to_empty                      = 0xFFFF;
    sbs->at_rate_ok                                 = 1;

    // -------------------------------------------------------------------------
    // 0x08 — Temperature
    // 0x09 — Voltage
    // 0x0A — Current
    // 0x0B — AverageCurrent
    // -------------------------------------------------------------------------
    sbs->temperature_package                        = 0;
    sbs->voltage                                    = 0;
    sbs->current                                    = 0;
    sbs->average_current                            = 0;

    // -------------------------------------------------------------------------
    // 0x0C — MaxError
    // 0x0D — RelativeStateOfCharge
    // 0x0E — AbsoluteStateOfCharge
    // -------------------------------------------------------------------------
    sbs->max_error                                  = 100;
    sbs->relative_soc                               = 0;
    sbs->absolute_soc                               = 0;

    // -------------------------------------------------------------------------
    // 0x0F — RemainingCapacity
    // 0x10 — FullChargeCapacity
    // -------------------------------------------------------------------------
    sbs->remaining_capacity                         = 0;
    sbs->full_charge_capacity                       = 0;

    // -------------------------------------------------------------------------
    // 0x11 — RunTimeToEmpty
    // 0x12 — AverageTimeToEmpty
    // 0x13 — AverageTimeToFull
    // -------------------------------------------------------------------------
    sbs->run_time_to_empty                          = 0xFFFF;
    sbs->average_time_to_empty                      = 0xFFFF;
    sbs->average_time_to_full                       = 0xFFFF;

    // -------------------------------------------------------------------------
    // 0x14 — ChargingCurrent
    // 0x15 — ChargingVoltage
    // -------------------------------------------------------------------------
    sbs->charging_current                           = 0;
    sbs->charging_voltage                           = 0;

    // -------------------------------------------------------------------------
    // 0x16 — BatteryStatus
    // -------------------------------------------------------------------------
    sbs->battery_status                             = 0x0080;   // INITIALIZED bit set

    // -------------------------------------------------------------------------
    // 0x17 — CycleCount
    // -------------------------------------------------------------------------
    sbs->cycle_count                                = 0;

    // -------------------------------------------------------------------------
    // 0x18 — DesignCapacity
    // 0x19 — DesignVoltage
    // -------------------------------------------------------------------------
    sbs->design_capacity                            = 0;
    sbs->design_voltage                             = 0;

    // -------------------------------------------------------------------------
    // 0x1A — SpecificationInfo
    // 0x1B — ManufactureDate
    // 0x1C — SerialNumber
    // -------------------------------------------------------------------------
    *(uint16_t *)&sbs->specification_info           = 0x0011;
    sbs->manufacture_date                           = 0;
    sbs->serial_number                              = 0;

    // -------------------------------------------------------------------------
    // 0x20 — ManufacturerName
    // 0x21 — DeviceName
    // 0x22 — DeviceChemistry
    // 0x23 — ManufacturerData
    // -------------------------------------------------------------------------
    strncpy((char *)sbs->manufacturer_name,         "OpenBatt Team",  32);
    strncpy((char *)sbs->device_name,               "OpenBMS",        32);
    strncpy((char *)sbs->device_chemistry,          "Li-Ion",         32);
    strncpy((char *)sbs->manufacturer_data,         "Year 2026",      32);

    // -------------------------------------------------------------------------
    // 0x40 — Configuration
    // 0x41 — MainControl
    // -------------------------------------------------------------------------
    sbs->configuration                              = 0x0077;   // UART+CAN+I2C enabled, 7 cells
    sbs->main_control                               = 0x0001;   // OpenBMS enabled, protections off

    // -------------------------------------------------------------------------
    // 0x42 — PackCapacity
    // 0x43 — MaxPackVoltage
    // 0x44 — MinPackVoltage
    // -------------------------------------------------------------------------
    sbs->pack_capacity                              = 35000;    // 7 cells × 5000mAh
    sbs->voltage_pack_max                           = 29400;    // 7 cells × 4200mV
    sbs->voltage_pack_min                           = 19600;    // 7 cells × 2800mV

    // -------------------------------------------------------------------------
    // 0x45 — CurrentSensorOffset
    // 0x46 — CurrentSensorGain
    // -------------------------------------------------------------------------
    sbs->current_sensor_offset                      = 0.0f;
    sbs->current_sensor_gain                        = 1.0f;

    // -------------------------------------------------------------------------
    // 0x47 — VoltageOffset
    // 0x48 — VoltageGain
    // -------------------------------------------------------------------------
    for(uint8_t i = 0; i < 7; i++)
    {
        sbs->voltage_offset[i]                      = 0.0f;
    }
    sbs->voltage_gain[0]                            = 1.02622576f;
    sbs->voltage_gain[1]                            = 1.00699300f;
    sbs->voltage_gain[2]                            = 1.01580135f;
    sbs->voltage_gain[3]                            = 0.99365166f;
    sbs->voltage_gain[4]                            = 1.02243680f;
    sbs->voltage_gain[5]                            = 1.00000000f;
    sbs->voltage_gain[6]                            = 1.00446428f;

    // -------------------------------------------------------------------------
    // 0x49 — NTC_Beta
    // 0x4A — NTC_R_Nominal
    // 0x4B — NTC_R_Fixed
    // 0x4C — NTC_T_Nominal
    // 0x4D — TemperatureOffset
    // -------------------------------------------------------------------------
    sbs->ntc_beta                                   = 3950.0f;
    sbs->ntc_r_nominal                              = 10000.0f;
    sbs->ntc_r_fixed                                = 10000.0f;
    sbs->ntc_t_nominal                              = 298.15f;
    sbs->temperature_offset                         = 0.0f;

    // -------------------------------------------------------------------------
    // 0x60 — UVP slow threshold
    // 0x61 — UVP slow time
    // 0x62 — UVP fast threshold
    // 0x63 — UVP fast time
    // 0x64 — OVP slow threshold
    // 0x65 — OVP slow time
    // 0x66 — OVP fast threshold
    // 0x67 — OVP fast time
    // -------------------------------------------------------------------------
    sbs->uvp_slow_threshold_mv                      = 2800;
    sbs->uvp_slow_time_ms                           = 5000;
    sbs->uvp_fast_threshold_mv                      = 2500;
    sbs->uvp_fast_time_ms                           = 100;
    sbs->ovp_slow_threshold_mv                      = 4200;
    sbs->ovp_slow_time_ms                           = 2000;
    sbs->ovp_fast_threshold_mv                      = 4250;
    sbs->ovp_fast_time_ms                           = 50;

    // -------------------------------------------------------------------------
    // 0x68 — Charge OCP threshold
    // 0x69 — Charge OCP time
    // 0x6A — Slow discharge OCP threshold
    // 0x6B — Slow discharge OCP time
    // 0x6C — Fast discharge OCP threshold
    // 0x6D — Fast discharge OCP time
    // -------------------------------------------------------------------------
    sbs->ocp_charge_threshold_ma                    = 5000;
    sbs->ocp_charge_time_ms                         = 1000;
    sbs->ocp_discharge_slow_threshold_ma            = 10000;
    sbs->ocp_discharge_slow_time_ms                 = 2000;
    sbs->ocp_discharge_fast_threshold_ma            = 20000;
    sbs->ocp_discharge_fast_time_ms                 = 50;

    // -------------------------------------------------------------------------
    // 0x6E — OTP threshold
    // 0x6F — OTP time
    // -------------------------------------------------------------------------
    sbs->otp_threshold_c                            = 60;
    sbs->otp_time_ms                                = 3000;

    // -------------------------------------------------------------------------
    // 0x80 — FETStatus
    // 0x81 — MainVddVoltage
    // 0x82 — TemperatureSTM32
    // -------------------------------------------------------------------------
    sbs->fet_status                                 = 0x0000;
    sbs->main_vdd_voltage_mv                        = 0.0f;
    sbs->temperature_stm32                          = 0.0f;

    // -------------------------------------------------------------------------
    // 0x90 — FaultSnapshotVoltage
    // 0x91 — FaultSnapshotCurrent
    // 0x92 — FaultSnapshotTemperature
    // 0x93 — FaultSnapshotSoC
    // -------------------------------------------------------------------------
    memset(sbs->fault_snapshot_voltage,     0, sizeof(sbs->fault_snapshot_voltage));
    sbs->fault_snapshot_current             = 0;
    sbs->fault_snapshot_temperature         = 0;
    sbs->fault_snapshot_soc                 = 0;

    // -------------------------------------------------------------------------
    // 0x94 — FaultCode
    // 0x95 — FaultTimestamp
    // -------------------------------------------------------------------------
    memset(sbs->fault_code,                 0, sizeof(sbs->fault_code));
    memset(sbs->fault_timestamp,            0, sizeof(sbs->fault_timestamp));

    // -------------------------------------------------------------------------
    // 0x96 — CellBalancingEnergy
    // 0x97 — CellBalancingTime
    // 0x98 — CellDeepestDischarge
    // 0x99 — CellMaxTemperature
    // -------------------------------------------------------------------------
    memset(sbs->cell_balancing_energy,      0, sizeof(sbs->cell_balancing_energy));
    memset(sbs->cell_balancing_time,        0, sizeof(sbs->cell_balancing_time));
    memset(sbs->cell_deepest_discharge,     0, sizeof(sbs->cell_deepest_discharge));
    memset(sbs->cell_max_temperature,       0, sizeof(sbs->cell_max_temperature));

    // -------------------------------------------------------------------------
    // 0xA0 — CellVoltageResistanceFactor
    // 0xA1 — BattVoltageResistanceFactor
    // -------------------------------------------------------------------------
    sbs->cell_voltage_resistance_factor             = 21.6060606f;
    sbs->batt_voltage_resistance_factor             = 42.2121212f;

    // -------------------------------------------------------------------------
    // 0xA2 — ShuntResistance
    // 0xA3 — CurrentSenseOffsetMv
    // 0xA4 — CurrentSenseGain
    // -------------------------------------------------------------------------
    sbs->shunt_resistance_mohms                     = 1.0f;
    sbs->current_sense_offset_mv                    = 0.0f;
    sbs->current_sense_gain                         = 1.0f;

    // -------------------------------------------------------------------------
    // 0xA5 — BalancerResistor
    // -------------------------------------------------------------------------
    sbs->balancer_resistor                          = 100;      // 100 mOhms

    // -------------------------------------------------------------------------
    // 0xA6 — FirmwareVersion
    // 0xA7 — HardwareVersion
    // -------------------------------------------------------------------------
    strncpy(sbs->firmware_version,                  "1.0.0",    32);
    strncpy(sbs->hardware_version,                  "RevA",     32);

    // -------------------------------------------------------------------------
    // 0xA8 — LastCommunicationTimestamp
    // 0xA9 — UptimeCounter
    // -------------------------------------------------------------------------
    sbs->last_communication_timestamp               = 0;
    sbs->uptime_counter                             = 0;

    // -------------------------------------------------------------------------
    // 0xB0 — CellVoltage
    // 0xB1 — CellSoC
    // 0xB2 — CellSoH
    // 0xB3 — CellRemainingCapacity
    // 0xB4 — CellSelfDischarge
    // 0xB5 — CellQmax
    // -------------------------------------------------------------------------
    memset(sbs->cell_voltage,               0, sizeof(sbs->cell_voltage));
    memset(sbs->cell_soc,                   0, sizeof(sbs->cell_soc));
    memset(sbs->cell_soh,                   0, sizeof(sbs->cell_soh));
    memset(sbs->cell_remaining_capacity,    0, sizeof(sbs->cell_remaining_capacity));
    memset(sbs->cell_self_discharge,        0, sizeof(sbs->cell_self_discharge));
    memset(sbs->cell_qmax,                  0, sizeof(sbs->cell_qmax));

    // -------------------------------------------------------------------------
    // 0xB6 — SOC grid (0% to 95% in 5% steps)
    // -------------------------------------------------------------------------
    float soc_grid_vals[20] = {
         0.0f,  5.0f, 10.0f, 15.0f, 20.0f,
        25.0f, 30.0f, 35.0f, 40.0f, 45.0f,
        50.0f, 55.0f, 60.0f, 65.0f, 70.0f,
        75.0f, 80.0f, 85.0f, 90.0f, 95.0f
    };
    memcpy(sbs->soc_grid, soc_grid_vals, sizeof(soc_grid_vals));

    // -------------------------------------------------------------------------
    // 0xB7 — OCV discharge curve (V per cell)
    // -------------------------------------------------------------------------
    float ocv_dis_vals[20] = {
        3.00f, 3.20f, 3.40f, 3.48f, 3.52f,
        3.55f, 3.58f, 3.60f, 3.62f, 3.65f,
        3.68f, 3.70f, 3.73f, 3.75f, 3.78f,
        3.82f, 3.87f, 3.93f, 4.00f, 4.10f
    };
    memcpy(sbs->ocv_dis, ocv_dis_vals, sizeof(ocv_dis_vals));

    // -------------------------------------------------------------------------
    // 0xB8 — OCV charge curve (V per cell)
    // -------------------------------------------------------------------------
    float ocv_chg_vals[20] = {
        3.03f, 3.23f, 3.43f, 3.51f, 3.55f,
        3.58f, 3.61f, 3.63f, 3.65f, 3.68f,
        3.71f, 3.73f, 3.76f, 3.78f, 3.81f,
        3.85f, 3.90f, 3.96f, 4.03f, 4.13f
    };
    memcpy(sbs->ocv_chg, ocv_chg_vals, sizeof(ocv_chg_vals));

    // -------------------------------------------------------------------------
    // 0xB9 — R0 discharge (Ω)
    // -------------------------------------------------------------------------
    float r0_dis_vals[20] = {
        0.030f, 0.025f, 0.022f, 0.020f, 0.018f,
        0.016f, 0.015f, 0.014f, 0.014f, 0.013f,
        0.013f, 0.013f, 0.014f, 0.014f, 0.015f,
        0.015f, 0.016f, 0.017f, 0.018f, 0.020f
    };
    memcpy(sbs->r0_dis, r0_dis_vals, sizeof(r0_dis_vals));

    // -------------------------------------------------------------------------
    // 0xBA — R1 discharge (Ω)
    // -------------------------------------------------------------------------
    float r1_dis_vals[20] = {
        0.012f, 0.010f, 0.009f, 0.008f, 0.008f,
        0.007f, 0.007f, 0.006f, 0.006f, 0.006f,
        0.006f, 0.006f, 0.006f, 0.007f, 0.007f,
        0.007f, 0.008f, 0.008f, 0.009f, 0.010f
    };
    memcpy(sbs->r1_dis, r1_dis_vals, sizeof(r1_dis_vals));

    // -------------------------------------------------------------------------
    // 0xBB — tau1 discharge (s)
    // -------------------------------------------------------------------------
    float tau1_dis_vals[20] = {
        45.0f, 42.0f, 40.0f, 38.0f, 37.0f,
        36.0f, 35.0f, 35.0f, 34.0f, 34.0f,
        34.0f, 35.0f, 35.0f, 36.0f, 37.0f,
        38.0f, 40.0f, 42.0f, 44.0f, 46.0f
    };
    memcpy(sbs->tau1_dis, tau1_dis_vals, sizeof(tau1_dis_vals));

    // -------------------------------------------------------------------------
    // 0xBC — R2 discharge (Ω)
    // -------------------------------------------------------------------------
    float r2_dis_vals[20] = {
        0.005f, 0.004f, 0.004f, 0.003f, 0.003f,
        0.003f, 0.003f, 0.003f, 0.003f, 0.003f,
        0.003f, 0.003f, 0.003f, 0.003f, 0.003f,
        0.004f, 0.004f, 0.004f, 0.005f, 0.005f
    };
    memcpy(sbs->r2_dis, r2_dis_vals, sizeof(r2_dis_vals));

    // -------------------------------------------------------------------------
    // 0xBD — tau2 discharge (s)
    // -------------------------------------------------------------------------
    float tau2_dis_vals[20] = {
        450.0f, 420.0f, 400.0f, 380.0f, 360.0f,
        350.0f, 340.0f, 330.0f, 330.0f, 320.0f,
        320.0f, 330.0f, 330.0f, 340.0f, 350.0f,
        360.0f, 380.0f, 400.0f, 420.0f, 450.0f
    };
    memcpy(sbs->tau2_dis, tau2_dis_vals, sizeof(tau2_dis_vals));

    // -------------------------------------------------------------------------
    // 0xBE — R0 charge (Ω)
    // -------------------------------------------------------------------------
    float r0_chg_vals[20] = {
        0.028f, 0.023f, 0.020f, 0.018f, 0.016f,
        0.015f, 0.014f, 0.013f, 0.013f, 0.012f,
        0.012f, 0.012f, 0.013f, 0.013f, 0.014f,
        0.014f, 0.015f, 0.016f, 0.017f, 0.019f
    };
    memcpy(sbs->r0_chg, r0_chg_vals, sizeof(r0_chg_vals));

    // -------------------------------------------------------------------------
    // 0xBF — R1 charge (Ω)
    // -------------------------------------------------------------------------
    float r1_chg_vals[20] = {
        0.011f, 0.009f, 0.008f, 0.007f, 0.007f,
        0.006f, 0.006f, 0.006f, 0.005f, 0.005f,
        0.005f, 0.006f, 0.006f, 0.006f, 0.007f,
        0.007f, 0.007f, 0.008f, 0.008f, 0.009f
    };
    memcpy(sbs->r1_chg, r1_chg_vals, sizeof(r1_chg_vals));

    // -------------------------------------------------------------------------
    // 0xC0 — tau1 charge (s)
    // -------------------------------------------------------------------------
    float tau1_chg_vals[20] = {
        40.0f, 38.0f, 36.0f, 35.0f, 34.0f,
        33.0f, 32.0f, 32.0f, 31.0f, 31.0f,
        31.0f, 32.0f, 32.0f, 33.0f, 34.0f,
        35.0f, 37.0f, 39.0f, 41.0f, 43.0f
    };
    memcpy(sbs->tau1_chg, tau1_chg_vals, sizeof(tau1_chg_vals));

    // -------------------------------------------------------------------------
    // 0xC1 — R2 charge (Ω)
    // -------------------------------------------------------------------------
    float r2_chg_vals[20] = {
        0.005f, 0.004f, 0.003f, 0.003f, 0.003f,
        0.002f, 0.002f, 0.002f, 0.002f, 0.002f,
        0.002f, 0.002f, 0.002f, 0.003f, 0.003f,
        0.003f, 0.003f, 0.004f, 0.004f, 0.005f
    };
    memcpy(sbs->r2_chg, r2_chg_vals, sizeof(r2_chg_vals));

    // -------------------------------------------------------------------------
    // 0xC2 — tau2 charge (s)
    // -------------------------------------------------------------------------
    float tau2_chg_vals[20] = {
        420.0f, 400.0f, 380.0f, 360.0f, 340.0f,
        330.0f, 320.0f, 310.0f, 310.0f, 300.0f,
        300.0f, 310.0f, 310.0f, 320.0f, 330.0f,
        340.0f, 360.0f, 380.0f, 400.0f, 430.0f
    };
    memcpy(sbs->tau2_chg, tau2_chg_vals, sizeof(tau2_chg_vals));

    // -------------------------------------------------------------------------
    // 0xC3 — Q_nom temperature setpoints
    // 0xC4 — Q_nom capacity at each temperature
    // -------------------------------------------------------------------------
    sbs->q_nom_temp_c[0]                            = -20.0f;
    sbs->q_nom_temp_c[1]                            = -10.0f;
    sbs->q_nom_temp_c[2]                            =   0.0f;
    sbs->q_nom_temp_c[3]                            =  25.0f;
    sbs->q_nom_temp_c[4]                            =  45.0f;

    sbs->q_nom_temp_ah[0]                           =   3.50f;
    sbs->q_nom_temp_ah[1]                           =   4.00f;
    sbs->q_nom_temp_ah[2]                           =   4.50f;
    sbs->q_nom_temp_ah[3]                           =   5.00f;
    sbs->q_nom_temp_ah[4]                           =   4.80f;

    // -------------------------------------------------------------------------
    // 0xC5 — Nominal capacity at 25°C
    // 0xC6 — Coulombic efficiency
    // -------------------------------------------------------------------------
    sbs->q_nom_ah                                   =   5.00f;
    sbs->coulombic_efficiency                       =   0.998f;

    // -------------------------------------------------------------------------
    // 0xC7 — R0 reference at 25°C
    // 0xC8 — R1 reference at 25°C
    // 0xC9 — tau1 reference at 25°C
    // 0xCA — R2 reference at 25°C
    // 0xCB — tau2 reference at 25°C
    // -------------------------------------------------------------------------
    sbs->r0_ref                                     =   0.014f;
    sbs->r1_ref                                     =   0.006f;
    sbs->tau1_ref                                   =  35.0f;
    sbs->r2_ref                                     =   0.003f;
    sbs->tau2_ref                                   = 330.0f;

    // -------------------------------------------------------------------------
    // 0xCC — Ea for R0
    // 0xCD — Ea for R1
    // 0xCE — Ea for tau1
    // 0xCF — Ea for R2
    // 0xD0 — Ea for tau2
    // -------------------------------------------------------------------------
    sbs->ea_r0                                      =  30000.0f;
    sbs->ea_r1                                      =  25000.0f;
    sbs->ea_tau1                                    =  20000.0f;
    sbs->ea_r2                                      =  20000.0f;
    sbs->ea_tau2                                    =  15000.0f;

    // -------------------------------------------------------------------------
    // 0xD1 — Kalman process noise SOC
    // 0xD2 — Kalman process noise RC1
    // 0xD3 — Kalman process noise RC2
    // 0xD4 — Kalman measurement noise
    // -------------------------------------------------------------------------
    sbs->kf_q_soc                                   =  1e-6f;
    sbs->kf_q_rc1                                   =  1e-4f;
    sbs->kf_q_rc2                                   =  1e-5f;
    sbs->kf_r_v                                     =  1e-4f;

    // -------------------------------------------------------------------------
    // 0xD5 — Per-cell SOC float
    // 0xD6 — Per-cell V_RC1
    // 0xD7 — Per-cell V_RC2
    // -------------------------------------------------------------------------
    for(uint8_t i = 0; i < 7; i++)
    {
        sbs->cell_soc_f[i]                          =  0.0f;
        sbs->cell_vrc1[i]                           =  0.0f;
        sbs->cell_vrc2[i]                           =  0.0f;
    }

    // -------------------------------------------------------------------------
    // 0xD8 — Per-cell covariance matrix upper triangle
    // P layout: [P00, P01, P02, P11, P12, P22]
    // -------------------------------------------------------------------------
    for(uint8_t i = 0; i < 7; i++)
    {
        sbs->cell_p[i][0]                           =  0.01f;   // P00 — 1% SoC uncertainty
        sbs->cell_p[i][1]                           =  0.0f;    // P01
        sbs->cell_p[i][2]                           =  0.0f;    // P02
        sbs->cell_p[i][3]                           =  0.001f;  // P11 — V_RC1 variance
        sbs->cell_p[i][4]                           =  0.0f;    // P12
        sbs->cell_p[i][5]                           =  0.001f;  // P22 — V_RC2 variance
    }

    // -------------------------------------------------------------------------
    // 0xD9 — Per-cell capacity after aging
    // 0xDA — Per-cell R0 growth factor
    // 0xDB — Per-cell cycle count
    // -------------------------------------------------------------------------
    for(uint8_t i = 0; i < 7; i++)
    {
        sbs->cell_q_nom_ah[i]                       =  5.00f;
        sbs->cell_r0_scale[i]                       =  1.00f;
    }

    // -------------------------------------------------------------------------
    // 0xDC — LearningStatus
    // -------------------------------------------------------------------------
    sbs->learning_status                            =  0x0000;
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
static void ProcessCommand(void)
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
            res = SBS_ReadWriteRegister(&OpenBMS_data, address, &rx_buf[3], &length, true);
        }
        else
        {
            res = SBS_ReadWriteRegister(&OpenBMS_data, address, &tx_buf[3], &length, false);
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
            // ACK: [0x03][code][crc]
            tx_buf[0] = 0x03;
            tx_buf[1] = 0;
            tx_buf[2] = 0;
            tx_buf[2] = Checksum(tx_buf, 3);
            length = 0;
        }
        else
        {
            // Read response: [0x02][address][length][data...][crc]
            tx_buf[0] = 0x02;
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
void OpenBMS_Comm_Init(void)
{
  //SBS_SetTestValues(&OpenBMS_data);
  SBS_SetDefaults(&OpenBMS_data);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_cmd.rx_buffer, UART_RX_BUFFER_SIZE);
}
void OpenBMS_Comm_Run(void)
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
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_cmd.rx_buffer, UART_RX_BUFFER_SIZE);
    }
}