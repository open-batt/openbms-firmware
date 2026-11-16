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
#include "openbms_error.h"
#include "openbms_fuelgauge.h"
#include "openbms_hppc.h"
#include "openbms_eeprom.h"


static UART_HandleTypeDef   *debug_uart             = &huart1;
static UART_Command_t       uart_cmd                = {0};
static I2C_HandleTypeDef    *host_i2c               = &hi2c2;
static I2C_Command_t        i2c_cmd                 = {0};
static CAN_HandleTypeDef    *host_can               = &hcan1;
static CAN_Command_t        can_cmd                 = {0};
static Peripheral_Data_t    pd                      = {0};
static uint8_t              model_cell              = 0;    // cell the fuel gauge model registers (0xA1 to 0xAC, 0xD4) act on
static uint8_t              fg_state_chunk          = 0;    // part of the fuel gauge state register 0x9C shows (COMM_FG_STATE_CHUNK_SIZE bytes each)
static FuelGauge_State_t    fg_state_image          = {0};  // the fuel gauge state for the host: taken when chunk 0 is read, filled by writes to 0x9C

#define COMM_FG_STATE_CHUNKS      ((sizeof(FuelGauge_State_t) + COMM_FG_STATE_CHUNK_SIZE - 1U) / COMM_FG_STATE_CHUNK_SIZE)
_Static_assert(COMM_FG_STATE_CHUNK_SIZE <= 255U, "a 0x9C chunk must fit one frame");

static const char           comm_firmware_version[SYS_DEVICE_STRING_SIZE]  = SYS_FIRMWARE_VERSION;
static const char           comm_hardware_version[SYS_DEVICE_STRING_SIZE]  = SYS_HARDWARE_VERSION;
static const char           comm_manufacturer_name[SYS_DEVICE_STRING_SIZE] = SYS_MANUFACTURER_NAME;
static const char           comm_device_name[SYS_DEVICE_STRING_SIZE]       = SYS_DEVICE_NAME;
static const char           comm_manufacturer_data[SYS_DEVICE_STRING_SIZE] = SYS_MANUFACTURER_DATA;

static CommErrorType_t      Comm_ControlsSet(uint8_t cmd, uint8_t *data, uint8_t length);
static CommErrorType_t      Comm_ReadWriteRegister(uint8_t address, uint8_t *raw_data, uint8_t *length, bool write); 

static void                 Comm_UartWaitTxIdle(void);
static void                 Comm_ProcessCommand(CommPort_t port);
static uint8_t              Comm_Checksum(uint8_t *data, uint16_t length);
static void                 Comm_SendResponse(CommPort_t port, uint8_t *data, uint16_t length);
static void                 Comm_SendError(CommPort_t port, CommErrorType_t err);
static void                 Comm_SendReply(CommPort_t port, CommErrorType_t res, uint8_t cmd, uint8_t address, uint8_t length);
static void                 Comm_I2cRestart(void);
static void                 Comm_I2cCheck(void);
static void                 Comm_I2cReceiveNext(void);
static void                 Comm_I2cTransmitNext(void);
static bool                 Comm_CanStart(void);
static void                 Comm_CanRestart(void);
static void                 Comm_CanCheck(void);
static bool                 Comm_CanQueue(uint32_t id, const uint8_t *data, uint8_t length);
static void                 Comm_CanSendNext(void);
static void                 Comm_CanBroadcast(void);
static void                 Comm_CanProcessCommand(void);


static CommErrorType_t Comm_ControlsSet(uint8_t cmd, uint8_t *data, uint8_t length)
{
    CommErrorType_t res = CE_OK;

    const System_Data_t *sys = Ctrl_GetSystem();

    switch(cmd)
    {
        case CC_PING:
        {
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
        }
        break;

        case CC_EEPROM_CONFIG:
        {
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else if((sys->ctrl_state != CTRL_STATE_STARTUP && sys->ctrl_state != CTRL_STATE_NORMAL_RUN &&
                     sys->ctrl_state != CTRL_STATE_FAULT) || !EEPROM_SaveConfig())
            {
                res = CE_STATE;
            }
        }
        break;

        case CC_EEPROM_FUELGAUGE:
        {
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else if(sys->ctrl_state != CTRL_STATE_NORMAL_RUN || !EEPROM_SaveFuelGaugeModel())
            {
                res = CE_STATE;
            }
        }
        break;

        case CC_EEPROM_ERASE:
        {
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else if((sys->ctrl_state != CTRL_STATE_STARTUP && sys->ctrl_state != CTRL_STATE_NORMAL_RUN &&
                     sys->ctrl_state != CTRL_STATE_FAULT) || !EEPROM_Erase())
            {
                res = CE_STATE;
            }
        }
        break;

        case CC_EEPROM_FG_STATE:
        {
            if(length != 1)
            {
                res = CE_WRONG_CMD;
            }
            else if(data[0] == COMM_FG_STATE_STORE)
            {
                if(!FuelGauge_StateValid(&fg_state_image))
                {
                    res = CE_WRONG_CMD;
                }
                else if(sys->ctrl_state != CTRL_STATE_NORMAL_RUN || !EEPROM_SaveFuelGaugeState(&fg_state_image))
                {
                    res = CE_STATE;
                }
            }
            else if(data[0] == COMM_FG_STATE_ERASE)
            {
                if((sys->ctrl_state != CTRL_STATE_STARTUP && sys->ctrl_state != CTRL_STATE_NORMAL_RUN &&
                    sys->ctrl_state != CTRL_STATE_FAULT) || !EEPROM_EraseFuelGaugeState())
                {
                    res = CE_STATE;
                }
            }
            else
            {
                res = CE_WRONG_CMD;
            }
        }
        break;

        case CC_HPPC_START:
        {
            if(length != 1 || (data[0] != HPPC_DIR_CHARGE && data[0] != HPPC_DIR_DISCHARGE))
            {
                res = CE_WRONG_CMD;
            }
#ifdef OPENBMS_TEST
            else if(Periph_AdsTestStandbyOn() || Periph_MeasTestOffOn())
            {
                res = CE_STATE;
            }
#endif
            else if((sys->ctrl_state != CTRL_STATE_NORMAL_RUN && sys->ctrl_state != CTRL_STATE_HPPC_RUN) ||
                    !EEPROM_ConfigLoaded() || !Ctrl_StartHppc(data[0]))
            {
                res = CE_STATE;
            }
        }
        break;

        case CC_NORMAL_RUN:
        {
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else if(sys->ctrl_state == CTRL_STATE_HPPC_RUN)
            {
                Ctrl_SetMode(CTRL_MODE_NORMAL);
            }
            else
            {
                res = CE_STATE;
            }
        }
        break;

        case CC_SET_FET:
        {
            if(length != 1 || (data[0] != 0x00 && data[0] != 0x01))
            {
                res = CE_WRONG_CMD;
            }
            else if(sys->ctrl_state != CTRL_STATE_NORMAL_RUN || !EEPROM_ConfigLoaded())
            {
                res = CE_STATE;
            }
#ifdef OPENBMS_TEST
            else if(data[0] == 0x01 && (Periph_AdsTestStandbyOn() || Periph_MeasTestOffOn()))
            {
                res = CE_STATE;
            }
#endif
            else
            {
                Ctrl_SetFetRequest((bool) data[0]);
            }
        }
        break;

        case CC_SET_PRE_FET:
        {
            if(length != 1 || (data[0] != 0x00 && data[0] != 0x01))
            {
                res = CE_WRONG_CMD;
            }
            else if(sys->ctrl_state != CTRL_STATE_NORMAL_RUN || !EEPROM_ConfigLoaded())
            {
                res = CE_STATE;
            }
#ifdef OPENBMS_TEST
            else if(data[0] == 0x01 && (Periph_AdsTestStandbyOn() || Periph_MeasTestOffOn()))
            {
                res = CE_STATE;
            }
#endif
            else
            {
                Ctrl_SetPreFetRequest((bool) data[0]);
            }
        }
        break;

        case CC_RESET_PROTECTION:
        {
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else
            {
                Ctrl_ResetProtection();
            }
        }
        break;

        case CC_CLEAR_ERRORS:
        {
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else
            {
                Ctrl_ClearErrors();
            }
        }
        break;

        case CC_CLEAR_LEARNING:
        {
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else if(sys->ctrl_state != CTRL_STATE_NORMAL_RUN || EEPROM_SaveRunning())
            {
                res = CE_STATE;
            }
            else
            {
                FuelGauge_ClearLearning();
                EEPROM_SaveFuelGaugeModel();
            }
        }
        break;

        case CC_CLEAR_FAULT:
        {
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else
            {
                Ctrl_ClearFault();
            }
        }
        break;

        case CC_SHUTDOWN:
        {
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else if(sys->ctrl_state != CTRL_STATE_NORMAL_RUN && sys->ctrl_state != CTRL_STATE_HPPC_RUN &&
                    sys->ctrl_state != CTRL_STATE_FAULT)
            {
                res = CE_STATE;
            }
            else
            {
                Ctrl_Shutdown();
            }
        }
        break;

#ifdef OPENBMS_TEST
        case CC_SOC_RESET:
        {
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else if(sys->ctrl_state != CTRL_STATE_NORMAL_RUN || !EEPROM_FuelGaugeLoaded() || EEPROM_SaveRunning() ||
                    !FuelGauge_ResetSocFromOcv())
            {
                res = CE_STATE;
            }
        }
        break;

        case CC_ADS_STANDBY:
        {
            if(length != 1 || (data[0] != 0x00 && data[0] != 0x01))
            {
                res = CE_WRONG_CMD;
            }
            else if(data[0] == 0x01 &&
                    (sys->ctrl_state != CTRL_STATE_NORMAL_RUN || sys->fet_request || sys->pre_fet_request))
            {
                res = CE_STATE;
            }
            else if(!Periph_AdsTestStandby((bool) data[0]))
            {
                res = CE_STATE;
            }
        }
        break;

        case CC_MEAS_OFF:
        {
            if(length != 1 || (data[0] != 0x00 && data[0] != 0x01))
            {
                res = CE_WRONG_CMD;
            }
            else if(data[0] == 0x01 &&
                    (sys->ctrl_state != CTRL_STATE_NORMAL_RUN || sys->fet_request || sys->pre_fet_request))
            {
                res = CE_STATE;
            }
            else
            {
                Periph_MeasTestOff((bool) data[0]);
            }
        }
        break;
#endif

        default:
        {
            res = CE_WRONG_CMD;
        }
        break;
    }

    return res;
}
static CommErrorType_t Comm_ReadWriteRegister(uint8_t address, uint8_t *raw_data, uint8_t *length, bool write)
{
    void    *data_point    = NULL;
    bool     ro            = false;
    bool     is_fuel_gauge = false;
    bool     is_config     = false;
    uint8_t  frame_length  = *length;

    const System_Data_t    *sys;
    const Config_Data_t    *cfg;
    const Stats_Data_t     *st;
    const FuelGauge_Data_t *fg;

    if(address <= 0x0F)
    {
        Periph_GetData(&pd);
        switch (address)
        {
            case 0x00: { data_point = (void *)pd.cell_voltage;                                   *length = sizeof(pd.cell_voltage);                             ro = true; } break;
            case 0x01: { data_point = (void *)&pd.pack_voltage;                                  *length = sizeof(pd.pack_voltage);                             ro = true; } break;
            case 0x02: { data_point = (void *)&pd.pack_current;                                  *length = sizeof(pd.pack_current);                             ro = true; } break;
            case 0x03: { data_point = (void *)&pd.temperature_package;                           *length = sizeof(pd.temperature_package);                      ro = true; } break;
            case 0x04: { data_point = (void *)&pd.temperature_stm32;                             *length = sizeof(pd.temperature_stm32);                        ro = true; } break;
            case 0x05: { data_point = (void *)&pd.main_vdd_voltage_mv;                           *length = sizeof(pd.main_vdd_voltage_mv);                      ro = true; } break;
            case 0x06: { data_point = (void *)&pd.fet_status;                                    *length = sizeof(pd.fet_status);                               ro = true; } break;
            case 0x07: { data_point = (void *)&pd.sys_vcc_mv;                                    *length = sizeof(pd.sys_vcc_mv);                               ro = true; } break;
#ifdef OPENBMS_TEST
            case 0x08: { data_point = (void *)Periph_TestBootVoltageOffset();                    *length = SYS_CELL_COUNT_MAX * sizeof(float);                  ro = true; } break;
#endif
            default: break;
        }
    }
    else if(address >= 0x10 && address <= 0x1F)
    {
        sys = Ctrl_GetSystem();
        switch (address)
        {
            case 0x10: { data_point = (void *)&sys->main_control;                                 *length = sizeof(sys->main_control);                            ro = true; } break;
            case 0x11: { data_point = (void *)&sys->last_communication_timestamp;                 *length = sizeof(sys->last_communication_timestamp);            ro = true; } break;
            case 0x12: { data_point = (void *)&sys->protection_trigger;                           *length = sizeof(sys->protection_trigger);                      ro = true; } break;
            case 0x13: { data_point = (void *)&sys->ctrl_state;                                   *length = sizeof(sys->ctrl_state);                              ro = true; } break;
            case 0x15: { data_point = (void *)&sys->error_flags;                                  *length = sizeof(sys->error_flags);                             ro = true; } break;
            case 0x17: { data_point = (void *)&sys->session_uptime_ms;                            *length = sizeof(sys->session_uptime_ms);                       ro = true; } break;
            case 0x18: { data_point = (void *)&sys->hppc_direction;                               *length = sizeof(sys->hppc_direction);                          ro = true; } break;
            case 0x19: { data_point = (void *)&sys->hppc_status;                                  *length = sizeof(sys->hppc_status);                             ro = true; } break;
            case 0x1A: { data_point = (void *)&sys->hppc_extract_status;                          *length = sizeof(sys->hppc_extract_status);                     ro = true; } break;
            case 0x1B: { data_point = (void *)&sys->battery_status;                               *length = sizeof(sys->battery_status);                          ro = true; } break;
            case 0x1C: { data_point = (void *)&sys->charger_present;                              *length = sizeof(sys->charger_present);                         ro = true; } break;
            case 0x1D: { data_point = (void *)&sys->eeprom_status;                                *length = sizeof(sys->eeprom_status);                           ro = true; } break;
#ifdef OPENBMS_TEST
            case 0x1E: { data_point = (void *)Ctrl_TestLoopTime();                                *length = 2 * sizeof(uint32_t);                                 ro = true; } break;
#endif
            default: break;
        }
    }
    else if(address >= 0x30 && address <= 0x6F)
    {
        cfg = Ctrl_GetConfig();
        is_config = true;
        switch (address)
        {
            case 0x30: { data_point = (void *)&cfg->cell_count;                                   *length = sizeof(cfg->cell_count);                             } break;
            case 0x31: { data_point = (void *)&cfg->use_i2c;                                      *length = sizeof(cfg->use_i2c);                                } break;
            case 0x32: { data_point = (void *)&cfg->use_can;                                      *length = sizeof(cfg->use_can);                                } break;
            case 0x33: { data_point = (void *)&cfg->use_uart;                                     *length = sizeof(cfg->use_uart);                               } break;
            case 0x34: { data_point = (void *)&cfg->volt_protection;                              *length = sizeof(cfg->volt_protection);                        } break;
            case 0x35: { data_point = (void *)&cfg->current_protection;                           *length = sizeof(cfg->current_protection);                     } break;
            case 0x36: { data_point = (void *)&cfg->temp_protection;                              *length = sizeof(cfg->temp_protection);                        } break;
            case 0x37: { data_point = (void *)&cfg->cell_capacity;                                *length = sizeof(cfg->cell_capacity);                          } break;
            case 0x38: { data_point = (void *)&cfg->voltage_cell_max;                             *length = sizeof(cfg->voltage_cell_max);                       } break;
            case 0x39: { data_point = (void *)&cfg->voltage_cell_min;                             *length = sizeof(cfg->voltage_cell_min);                       } break;
            case 0x3A: { data_point = (void *)&cfg->charging_term_current;                        *length = sizeof(cfg->charging_term_current);                  } break;
            case 0x3B: { data_point = (void *)&cfg->uvp_slow_threshold_mv;                        *length = sizeof(cfg->uvp_slow_threshold_mv);                  } break;
            case 0x3C: { data_point = (void *)&cfg->uvp_slow_time_ms;                             *length = sizeof(cfg->uvp_slow_time_ms);                       } break;
            case 0x3D: { data_point = (void *)&cfg->uvp_fast_threshold_mv;                        *length = sizeof(cfg->uvp_fast_threshold_mv);                  } break;
            case 0x3E: { data_point = (void *)&cfg->uvp_fast_time_ms;                             *length = sizeof(cfg->uvp_fast_time_ms);                       } break;
            case 0x3F: { data_point = (void *)&cfg->ovp_slow_threshold_mv;                        *length = sizeof(cfg->ovp_slow_threshold_mv);                  } break;
            case 0x40: { data_point = (void *)&cfg->ovp_slow_time_ms;                             *length = sizeof(cfg->ovp_slow_time_ms);                       } break;
            case 0x41: { data_point = (void *)&cfg->ovp_fast_threshold_mv;                        *length = sizeof(cfg->ovp_fast_threshold_mv);                  } break;
            case 0x42: { data_point = (void *)&cfg->ovp_fast_time_ms;                             *length = sizeof(cfg->ovp_fast_time_ms);                       } break;
            case 0x43: { data_point = (void *)&cfg->ocp_charge_threshold_ma;                      *length = sizeof(cfg->ocp_charge_threshold_ma);                } break;
            case 0x44: { data_point = (void *)&cfg->ocp_charge_time_ms;                           *length = sizeof(cfg->ocp_charge_time_ms);                     } break;
            case 0x45: { data_point = (void *)&cfg->ocp_discharge_slow_threshold_ma;              *length = sizeof(cfg->ocp_discharge_slow_threshold_ma);        } break;
            case 0x46: { data_point = (void *)&cfg->ocp_discharge_slow_time_ms;                   *length = sizeof(cfg->ocp_discharge_slow_time_ms);             } break;
            case 0x47: { data_point = (void *)&cfg->ocp_discharge_fast_threshold_ma;              *length = sizeof(cfg->ocp_discharge_fast_threshold_ma);        } break;
            case 0x48: { data_point = (void *)&cfg->ocp_discharge_fast_time_ms;                   *length = sizeof(cfg->ocp_discharge_fast_time_ms);             } break;
            case 0x49: { data_point = (void *)&cfg->otp_threshold_c;                              *length = sizeof(cfg->otp_threshold_c);                        } break;
            case 0x4A: { data_point = (void *)&cfg->otp_time_ms;                                  *length = sizeof(cfg->otp_time_ms);                            } break;
            case 0x4C: { data_point = (void *)&cfg->charge_monitor_cell_mv;                       *length = sizeof(cfg->charge_monitor_cell_mv);                 } break;
            case 0x4D: { data_point = (void *)&cfg->utp_threshold_c;                              *length = sizeof(cfg->utp_threshold_c);                        } break;
            case 0x4E: { data_point = (void *)&cfg->utp_time_ms;                                  *length = sizeof(cfg->utp_time_ms);                            } break;
            case 0x4F: { data_point = (void *)&cfg->otp_charge_threshold_c;                       *length = sizeof(cfg->otp_charge_threshold_c);                 } break;
            case 0x50: { data_point = (void *)&cfg->ocv_v_lo_mv;                                  *length = sizeof(cfg->ocv_v_lo_mv);                            } break;
            case 0x51: { data_point = (void *)&cfg->ocv_v_hi_mv;                                  *length = sizeof(cfg->ocv_v_hi_mv);                            } break;
            case 0x52: { data_point = (void *)&cfg->current_sensor_gain;                          *length = sizeof(cfg->current_sensor_gain);                    } break;
            case 0x53: { data_point = (void *)cfg->voltage_gain;                                  *length = sizeof(cfg->voltage_gain);                           } break;
            case 0x54: { data_point = (void *)&cfg->ntc_beta;                                     *length = sizeof(cfg->ntc_beta);                               } break;
            case 0x55: { data_point = (void *)&cfg->ntc_r_nominal;                                *length = sizeof(cfg->ntc_r_nominal);                          } break;
            case 0x56: { data_point = (void *)&cfg->ntc_r_fixed;                                  *length = sizeof(cfg->ntc_r_fixed);                            } break;
            case 0x57: { data_point = (void *)&cfg->ntc_t_nominal;                                *length = sizeof(cfg->ntc_t_nominal);                          } break;
            case 0x58: { data_point = (void *)&cfg->temperature_offset;                           *length = sizeof(cfg->temperature_offset);                     } break;
            case 0x59: { data_point = (void *)&cfg->shunt_resistance_mohm;                        *length = sizeof(cfg->shunt_resistance_mohm);                  } break;
            case 0x5A: { data_point = (void *)&cfg->current_offset_ma;                            *length = sizeof(cfg->current_offset_ma);                      } break;
            case 0x5B: { data_point = (void *)cfg->voltage_offset_mv;                             *length = sizeof(cfg->voltage_offset_mv);                      } break;
            case 0x5C: { data_point = (void *)&cfg->utp_charge_threshold_c;                       *length = sizeof(cfg->utp_charge_threshold_c);                 } break;
            case 0x60: { data_point = (void *)comm_firmware_version;                                *length = sizeof(comm_firmware_version);                       ro = true; } break;
            case 0x61: { data_point = (void *)comm_hardware_version;                                *length = sizeof(comm_hardware_version);                       ro = true; } break;
            case 0x62: { data_point = (void *)comm_manufacturer_name;                               *length = sizeof(comm_manufacturer_name);                      ro = true; } break;
            case 0x63: { data_point = (void *)comm_device_name;                                     *length = sizeof(comm_device_name);                            ro = true; } break;
            case 0x64: { data_point = (void *)cfg->device_chemistry;                              *length = sizeof(cfg->device_chemistry);                                  } break;
            case 0x65: { data_point = (void *)comm_manufacturer_data;                               *length = sizeof(comm_manufacturer_data);                      ro = true; } break;
            case 0x66: { data_point = (void *)&cfg->balance_enable;                               *length = sizeof(cfg->balance_enable);                                    } break;
            case 0x67: { data_point = (void *)&cfg->balance_start_mv;                             *length = sizeof(cfg->balance_start_mv);                                  } break;
            case 0x68: { data_point = (void *)&cfg->balance_delta_mv;                             *length = sizeof(cfg->balance_delta_mv);                                  } break;
            case 0x69: { data_point = (void *)&cfg->balance_max_temp_c;                           *length = sizeof(cfg->balance_max_temp_c);                                } break;
            case 0x6A: { data_point = (void *)&cfg->balance_resistance_ohm;                       *length = sizeof(cfg->balance_resistance_ohm);                            } break;
            case 0x6B: { data_point = (void *)&cfg->balance_rest_enable;                          *length = sizeof(cfg->balance_rest_enable);                               } break;
            case 0x6C: { data_point = (void *)&cfg->balance_stop_mv;                              *length = sizeof(cfg->balance_stop_mv);                                   } break;
            default: break;
        }
    }
    else if(address >= 0x70 && address <= 0x7F)
    {
        st = Ctrl_GetStats();
        switch (address)
        {
            case 0x70: { data_point = (void *)st->fault_snapshot_voltage;                         *length = sizeof(st->fault_snapshot_voltage);                   ro = true; } break;
            case 0x71: { data_point = (void *)&st->fault_snapshot_current;                        *length = sizeof(st->fault_snapshot_current);                   ro = true; } break;
            case 0x72: { data_point = (void *)&st->fault_snapshot_temperature;                    *length = sizeof(st->fault_snapshot_temperature);               ro = true; } break;
            case 0x73: { data_point = (void *)&st->fault_snapshot_soc;                            *length = sizeof(st->fault_snapshot_soc);                       ro = true; } break;
            case 0x74: { data_point = (void *)&st->fault_code;                                     *length = sizeof(st->fault_code);                               ro = true; } break;
            case 0x75: { data_point = (void *)&st->fault_timestamp;                                *length = sizeof(st->fault_timestamp);                          ro = true; } break;
            default: break;
        }
    }
    else if(address >= 0x90 && address <= 0xDF)
    {
        fg = Ctrl_GetFuelGauge();
        is_fuel_gauge = true;
        switch (address)
        {
            case 0x90: { data_point = (void *)&fg->relative_soc;                                  *length = sizeof(fg->relative_soc);                             ro = true; } break;
            case 0x92: { data_point = (void *)fg->cell_soc;                                       *length = sizeof(fg->cell_soc);                                 ro = true; } break;
            case 0x93: { data_point = (void *)fg->cell_soh;                                       *length = sizeof(fg->cell_soh);                                 ro = true; } break;
            case 0x94: { data_point = (void *)fg->cell_remaining_capacity;                        *length = sizeof(fg->cell_remaining_capacity);                  ro = true; } break;
            case 0x96: { data_point = (void *)&fg->learning_status;                               *length = sizeof(fg->learning_status);                                     } break;
            case 0x97: { data_point = (void *)&fg->capacity_update_count;                         *length = sizeof(fg->capacity_update_count);                    ro = true; } break;
            case 0x98: { data_point = (void *)FuelGauge_GetStatus();                            *length = sizeof(uint8_t);                                     ro = true; is_fuel_gauge = false; } break;
            case 0x9A: { data_point = (void *)FuelGauge_GetBalanceTargetMah();                  *length = SYS_CELL_COUNT_MAX * sizeof(uint16_t);               ro = true; is_fuel_gauge = false; } break;
            case 0x9B: { data_point = (void *)&fg_state_chunk;                                   *length = sizeof(fg_state_chunk);                              is_fuel_gauge = false; } break;
            case 0x9C:
            {
                uint32_t offset = (uint32_t)fg_state_chunk * COMM_FG_STATE_CHUNK_SIZE;
                uint32_t rest   = sizeof(FuelGauge_State_t) - offset;

                if(!write && fg_state_chunk == 0)
                {
                    FuelGauge_PeekState(&fg_state_image);
                }
                data_point = (void *)((uint8_t *)&fg_state_image + offset);
                if(rest > COMM_FG_STATE_CHUNK_SIZE)
                {
                    *length = COMM_FG_STATE_CHUNK_SIZE;
                }
                else
                {
                    *length = (uint8_t)rest;
                }
                is_fuel_gauge = false;
            }
            break;
            case 0x9F: { data_point = (void *)&model_cell;                                       *length = sizeof(model_cell);                                  is_fuel_gauge = false; } break;
            case 0xA0: { data_point = (void *)fg->soc_grid;                                       *length = sizeof(fg->soc_grid);                                } break;
            case 0xA1: { data_point = (void *)fg->cell[model_cell].ocv_dis;                                        *length = sizeof(fg->cell[0].ocv_dis);                                 } break;
            case 0xA2: { data_point = (void *)fg->cell[model_cell].ocv_chg;                                        *length = sizeof(fg->cell[0].ocv_chg);                                 } break;
            case 0xA3: { data_point = (void *)fg->cell[model_cell].r0_dis;                                         *length = sizeof(fg->cell[0].r0_dis);                                  } break;
            case 0xA4: { data_point = (void *)fg->cell[model_cell].r1_dis;                                         *length = sizeof(fg->cell[0].r1_dis);                                  } break;
            case 0xA5: { data_point = (void *)fg->cell[model_cell].tau1_dis;                                       *length = sizeof(fg->cell[0].tau1_dis);                                } break;
            case 0xA6: { data_point = (void *)fg->cell[model_cell].r2_dis;                                         *length = sizeof(fg->cell[0].r2_dis);                                  } break;
            case 0xA7: { data_point = (void *)fg->cell[model_cell].tau2_dis;                                       *length = sizeof(fg->cell[0].tau2_dis);                                } break;
            case 0xA8: { data_point = (void *)fg->cell[model_cell].r0_chg;                                         *length = sizeof(fg->cell[0].r0_chg);                                  } break;
            case 0xA9: { data_point = (void *)fg->cell[model_cell].r1_chg;                                         *length = sizeof(fg->cell[0].r1_chg);                                  } break;
            case 0xAA: { data_point = (void *)fg->cell[model_cell].tau1_chg;                                       *length = sizeof(fg->cell[0].tau1_chg);                                } break;
            case 0xAB: { data_point = (void *)fg->cell[model_cell].r2_chg;                                         *length = sizeof(fg->cell[0].r2_chg);                                  } break;
            case 0xAC: { data_point = (void *)fg->cell[model_cell].tau2_chg;                                       *length = sizeof(fg->cell[0].tau2_chg);                                } break;
            case 0xAD: { data_point = (void *)fg->q_temp_c;                                       *length = sizeof(fg->q_temp_c);                                } break;
            case 0xAE: { data_point = (void *)fg->q_temp_factor;                                  *length = sizeof(fg->q_temp_factor);                           } break;
            case 0xAF: { data_point = (void *)&fg->q_nom_ah;                                      *length = sizeof(fg->q_nom_ah);                                } break;
            case 0xB0: { data_point = (void *)&fg->coulombic_efficiency;                          *length = sizeof(fg->coulombic_efficiency);                    } break;
            case 0xB1: { data_point = (void *)&fg->model_temp_dis_c;                              *length = sizeof(fg->model_temp_dis_c);                        } break;
            case 0xB2: { data_point = (void *)&fg->model_temp_chg_c;                              *length = sizeof(fg->model_temp_chg_c);                        } break;
            case 0xB6: { data_point = (void *)&fg->ea_r0;                                         *length = sizeof(fg->ea_r0);                                   } break;
            case 0xB7: { data_point = (void *)&fg->ea_r1;                                         *length = sizeof(fg->ea_r1);                                   } break;
            case 0xB8: { data_point = (void *)&fg->ea_tau1;                                       *length = sizeof(fg->ea_tau1);                                 } break;
            case 0xB9: { data_point = (void *)&fg->ea_r2;                                         *length = sizeof(fg->ea_r2);                                   } break;
            case 0xBA: { data_point = (void *)&fg->ea_tau2;                                       *length = sizeof(fg->ea_tau2);                                 } break;
            case 0xBE: { data_point = (void *)&fg->kf_r_v;                                        *length = sizeof(fg->kf_r_v);                                  } break;
            case 0xD0: { data_point = (void *)fg->cell_soc_f;                                     *length = sizeof(fg->cell_soc_f);                               ro = true; } break;
            case 0xD1: { data_point = (void *)fg->cell_vrc1;                                      *length = sizeof(fg->cell_vrc1);                                ro = true; } break;
            case 0xD2: { data_point = (void *)fg->cell_vrc2;                                      *length = sizeof(fg->cell_vrc2);                                ro = true; } break;
            case 0xD3: { data_point = (void *)fg->cell_p;                                         *length = sizeof(fg->cell_p);                                   ro = true; } break;
            case 0xD4: { data_point = (void *)&fg->cell[model_cell].capacity_ah;                                  *length = sizeof(fg->cell[0].capacity_ah);                                       } break;
            case 0xD7: { data_point = (void *)&fg->cycle_count;                                   *length = sizeof(fg->cycle_count);                              ro = true; } break;
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
        if(frame_length != *length)
        {
            return CE_WRONG_CMD;
        }

        if(is_config && address == 0x30 && (raw_data[0] < SYS_CELL_COUNT_MIN || raw_data[0] > SYS_CELL_COUNT_MAX))
        {
            return CE_WRONG_CMD;
        }

        if(address == 0x9F && raw_data[0] >= SYS_CELL_COUNT_MAX)
        {
            return CE_WRONG_CMD;
        }

        if(address == 0x9B && raw_data[0] >= COMM_FG_STATE_CHUNKS)
        {
            return CE_WRONG_CMD;
        }

        if(is_config && ((address >= 0x31 && address <= 0x36) || address == 0x66 || address == 0x6B) && raw_data[0] > 1)
        {
            return CE_WRONG_CMD;
        }

        if(is_fuel_gauge)
        {
            if(EEPROM_SaveRunning())
            {
                return CE_STATE;
            }
            if(!Ctrl_WriteFuelGauge(data_point, raw_data, *length))
            {
                return CE_WRONG_CMD;
            }
        }
        else if(is_config)
        {
            if(!Ctrl_WriteConfig(data_point, raw_data, *length))
            {
                return CE_WRONG_CMD;
            }
        }
        else
        {
            memcpy(data_point, raw_data, *length);
        }
    }
    else
    {
        memcpy(raw_data, data_point, *length);
    }

    return CE_OK;
}
// Waits for the previous UART send to finish. A send still running after COMM_TX_TIMEOUT_MS is
// stuck and is aborted, so the main loop does not block.
static void Comm_UartWaitTxIdle(void)
{
    uint32_t start = HAL_GetTick();

    while(uart_cmd.tx_wait)
    {
        if((HAL_GetTick() - start) > COMM_TX_TIMEOUT_MS)
        {
            HAL_UART_AbortTransmit(&huart1);
            uart_cmd.tx_wait = false;
            Error_Set(OPENBMS_UART_TX_TIMEOUT);
        }
    }
}
static void Comm_ProcessCommand(CommPort_t port)
{
    uint8_t             *rx_buf;
    uint8_t             *tx_buf;
    uint8_t             cmd;
    uint8_t             length;
    uint8_t             address;
    uint8_t             crc_calc, crc_rec;
    CommErrorType_t     res;

    if(port == COMM_PORT_I2C)
    {
        rx_buf = i2c_cmd.rx_buffer;
        tx_buf = i2c_cmd.tx_buffer;
    }
    else
    {
        rx_buf = uart_cmd.rx_buffer;
        tx_buf = uart_cmd.tx_buffer;

        Comm_UartWaitTxIdle();
    }

    cmd = rx_buf[0];
    if( cmd != CC_WRITE && 
        cmd != CC_READ && 
        cmd != CC_BOOTLOADER &&
        cmd != CC_EEPROM_CONFIG &&
        cmd != CC_EEPROM_FUELGAUGE &&
        cmd != CC_EEPROM_ERASE &&
        cmd != CC_EEPROM_FG_STATE &&
        cmd != CC_HPPC_START &&
        cmd != CC_NORMAL_RUN &&
        cmd != CC_SET_FET &&
        cmd != CC_SET_PRE_FET &&
        cmd != CC_RESET_PROTECTION &&
        cmd != CC_CLEAR_ERRORS &&
        cmd != CC_CLEAR_LEARNING &&
        cmd != CC_CLEAR_FAULT &&
        cmd != CC_SHUTDOWN &&
        cmd != CC_SOC_RESET &&
        cmd != CC_ADS_STANDBY &&
        cmd != CC_MEAS_OFF &&
        cmd != CC_REBOOT &&
        cmd != CC_PING)
    {
        Comm_SendError(port, CE_WRONG_CMD);
        return;
    }

    if(cmd == CC_BOOTLOADER)
    {
        if(Ctrl_GetSystem()->ctrl_state == CTRL_STATE_SHUTDOWN)
        {
            Comm_SendError(port, CE_STATE);
            return;
        }

        Comm_SendResponse(port, (uint8_t*)"1\n",2);
        Ctrl_RequestReset(true);
        return;
    }

    address = rx_buf[1];
    length  = rx_buf[2];

    crc_calc = Comm_Checksum(rx_buf, length + 3);
    crc_rec  = rx_buf[length + 3];

    if(crc_calc != crc_rec)
    {
        Comm_SendError(port, CE_BAD_CRC);
        return;
    }

    Ctrl_SetLastCommunication();

    if(cmd == CC_WRITE)
    {
        res = Comm_ReadWriteRegister(address, &rx_buf[3], &length, true);
    }
    else if(cmd == CC_READ)
    {
        res = Comm_ReadWriteRegister(address, &tx_buf[3], &length, false);
    }
    else if(cmd == CC_REBOOT)
    {
        if(length != 0)
        {
            res = CE_WRONG_CMD;
        }
        else if(Ctrl_GetSystem()->ctrl_state == CTRL_STATE_SHUTDOWN)
        {
            res = CE_STATE;
        }
        else
        {
            res = CE_OK;
        }
    }
    else
    {
        res = Comm_ControlsSet(cmd, &rx_buf[3], length);
    }

    Comm_SendReply(port, res, cmd, address, length);

    if(cmd == CC_REBOOT && res == CE_OK)
    {
        Ctrl_RequestReset(false);
    }
}
static uint8_t Comm_Checksum(uint8_t *data, uint16_t length)
{
    uint8_t sum = 0;
    for(uint16_t i = 0; i < length; i++)
    {
        sum += (uint8_t)data[i];
    }
    return sum;
}
static void Comm_SendResponse(CommPort_t port, uint8_t *data, uint16_t length)
{
    if(port == COMM_PORT_I2C)
    {
        if(data != i2c_cmd.tx_buffer)
        {
            memcpy(i2c_cmd.tx_buffer, data, length);
        }

        __DMB();
        i2c_cmd.tx_length = length;
        return;
    }

    uart_cmd.tx_wait = true;

    if(HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data, length) != HAL_OK)
    {
        uart_cmd.tx_wait = false;
    }
}
static void Comm_SendError(CommPort_t port, CommErrorType_t err)
{
    static uint8_t buff[5];

    buff[0] = CC_ERROR;
    buff[1] = 0;
    buff[2] = 1;
    buff[3] = err;
    buff[4] = Comm_Checksum(buff, 4);

    Comm_SendResponse(port, buff, sizeof(buff));
}
static void Comm_SendReply(CommPort_t port, CommErrorType_t res, uint8_t cmd, uint8_t address, uint8_t length)
{
    uint8_t *tx_buf;

    if(port == COMM_PORT_I2C)
    {
        tx_buf = i2c_cmd.tx_buffer;
    }
    else
    {
        tx_buf = uart_cmd.tx_buffer;
    }

    if(res != CE_OK)
    {
        Comm_SendError(port, res);
        return;
    }

    if(cmd == CC_READ)
    {
        tx_buf[0] = cmd;
        tx_buf[1] = address;
        tx_buf[2] = length;
        tx_buf[length + 3] = Comm_Checksum(tx_buf, length + 3);
        Comm_SendResponse(port, tx_buf, length + 4);
    }
    else
    {
        tx_buf[0] = CC_ACK;
        tx_buf[1] = 0;
        tx_buf[2] = 0;
        tx_buf[3] = Comm_Checksum(tx_buf, 3);
        Comm_SendResponse(port, tx_buf, 4);
    }
}
void Comm_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(debug_uart, uart_cmd.rx_buffer, COMM_RX_BUFFER_SIZE);

    HAL_I2C_EnableListen_IT(host_i2c);

    can_cmd.broadcast_on = COMM_CAN_BROADCAST_DEFAULT;
    if(!Comm_CanStart())
    {
        Error_Set(OPENBMS_CAN_ERROR);
    }
}
void Comm_Run(void)
{
  if(uart_cmd.frame_ready)
  {
    Comm_ProcessCommand(COMM_PORT_UART);
    uart_cmd.frame_ready = false;
  }

  if(i2c_cmd.frame_ready)
  {
    Comm_ProcessCommand(COMM_PORT_I2C);
    i2c_cmd.frame_ready = false;
  }

  Comm_I2cCheck();

  if(can_cmd.command_ready)
  {
    Comm_CanProcessCommand();
    can_cmd.command_ready = false;
  }

  Comm_CanCheck();

  if(uart_cmd.rx_count != 0 && HAL_GetTick() - uart_cmd.rx_start_ms > COMM_UART_FRAME_TIMEOUT_MS)
  {
    HAL_UART_AbortReceive(debug_uart);
    uart_cmd.rx_count = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(debug_uart, uart_cmd.rx_buffer, COMM_RX_BUFFER_SIZE);
  }

  if(debug_uart->RxState == HAL_UART_STATE_READY)
  {
    uart_cmd.rx_count = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(debug_uart, uart_cmd.rx_buffer, COMM_RX_BUFFER_SIZE);
  }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t count;

    if(huart->Instance == USART1)
    {
        if(HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
        {
            return;
        }

        count = uart_cmd.rx_count + size;
        if(uart_cmd.rx_buffer[0] != CC_BOOTLOADER && count < COMM_RX_BUFFER_SIZE &&
           (count < 3U || count < (uint16_t)uart_cmd.rx_buffer[2] + 4U))
        {
            if(uart_cmd.rx_count == 0)
            {
                uart_cmd.rx_start_ms = HAL_GetTick();
            }
            uart_cmd.rx_count = count;
            HAL_UARTEx_ReceiveToIdle_DMA(debug_uart, &uart_cmd.rx_buffer[count], COMM_RX_BUFFER_SIZE - count);
            return;
        }

        uart_cmd.rx_count    = 0;
        uart_cmd.frame_ready = true;
        HAL_UARTEx_ReceiveToIdle_DMA(debug_uart, uart_cmd.rx_buffer, COMM_RX_BUFFER_SIZE);
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart_cmd.tx_wait = false;
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    Error_Set(OPENBMS_UART_ERROR);

    if(huart->gState == HAL_UART_STATE_READY)
    {
      uart_cmd.tx_wait = false;
    }

    if(huart->RxState == HAL_UART_STATE_READY)
    {
      uart_cmd.rx_count = 0;
      HAL_UARTEx_ReceiveToIdle_DMA(debug_uart, uart_cmd.rx_buffer, COMM_RX_BUFFER_SIZE);
    }
  }
}
static void Comm_I2cRestart(void)
{
    HAL_I2C_DeInit(host_i2c);
    HAL_I2C_Init(host_i2c);

    i2c_cmd.rx_count        = 0;
    i2c_cmd.transfer_active = false;
    i2c_cmd.restart         = false;

    HAL_I2C_EnableListen_IT(host_i2c);
}
static void Comm_I2cCheck(void)
{
    if(i2c_cmd.restart)
    {
        Comm_I2cRestart();
        return;
    }

    if(i2c_cmd.transfer_active)
    {
        if((HAL_GetTick() - i2c_cmd.transfer_start_ms) > COMM_I2C_TRANSFER_TIMEOUT_MS)
        {
            Error_Set(OPENBMS_I2C_ERROR);
            Comm_I2cRestart();
        }
        return;
    }

    if(HAL_I2C_GetState(host_i2c) == HAL_I2C_STATE_READY)
    {
        if(HAL_I2C_EnableListen_IT(host_i2c) != HAL_OK)
        {
            Comm_I2cRestart();
        }
    }
}
static void Comm_I2cReceiveNext(void)
{
    uint8_t *dest;

    if(i2c_cmd.frame_ready || i2c_cmd.rx_count >= COMM_RX_BUFFER_SIZE)
    {
        dest = &i2c_cmd.rx_discard;
    }
    else
    {
        dest = &i2c_cmd.rx_buffer[i2c_cmd.rx_count];
    }

    if(HAL_I2C_Slave_Seq_Receive_IT(host_i2c, dest, 1, I2C_NEXT_FRAME) != HAL_OK)
    {
        i2c_cmd.restart = true;
    }
}
static void Comm_I2cTransmitNext(void)
{
    uint8_t *src;

    if(i2c_cmd.tx_length == 0)
    {
        i2c_cmd.tx_fill = COMM_I2C_NOT_READY;
        src = &i2c_cmd.tx_fill;
    }
    else if(i2c_cmd.tx_index >= i2c_cmd.tx_length)
    {
        i2c_cmd.tx_fill = COMM_I2C_FILL;
        src = &i2c_cmd.tx_fill;
    }
    else
    {
        src = &i2c_cmd.tx_buffer[i2c_cmd.tx_index];
    }

    i2c_cmd.tx_index++;

    if(HAL_I2C_Slave_Seq_Transmit_IT(host_i2c, src, 1, I2C_NEXT_FRAME) != HAL_OK)
    {
        i2c_cmd.restart = true;
    }
}
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    UNUSED(AddrMatchCode);

    if(hi2c->Instance == I2C2)
    {
        i2c_cmd.direction         = TransferDirection;
        i2c_cmd.transfer_active   = true;
        i2c_cmd.transfer_start_ms = HAL_GetTick();

        if(TransferDirection == I2C_DIRECTION_TRANSMIT)
        {
            i2c_cmd.rx_count = 0;
            if(!i2c_cmd.frame_ready)
            {
                i2c_cmd.tx_length = 0;
            }
            Comm_I2cReceiveNext();
        }
        else
        {
            i2c_cmd.tx_index = 0;
            Comm_I2cTransmitNext();
        }
    }
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C2)
    {
        if(i2c_cmd.rx_count < UINT16_MAX)
        {
            i2c_cmd.rx_count++;
        }
        Comm_I2cReceiveNext();
    }
}
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C2)
    {
        Comm_I2cTransmitNext();
    }
}
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    uint16_t count;

    if(hi2c->Instance == I2C2)
    {
        count = i2c_cmd.rx_count;
        i2c_cmd.transfer_active = false;

        if(i2c_cmd.direction == I2C_DIRECTION_TRANSMIT && !i2c_cmd.frame_ready)
        {
            if(count >= 4 && count <= COMM_RX_BUFFER_SIZE && count == (uint16_t)i2c_cmd.rx_buffer[2] + 4)
            {
                i2c_cmd.frame_ready = true;
            }
        }
        i2c_cmd.rx_count = 0;

        if(HAL_I2C_EnableListen_IT(host_i2c) != HAL_OK)
        {
            i2c_cmd.restart = true;
        }
    }
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C2)
    {
        if(HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF)
        {
            return;
        }

        Error_Set(OPENBMS_I2C_ERROR);
        i2c_cmd.restart = true;
    }
    else if(hi2c->Instance == I2C1)
    {
        Periph_EepromI2cError();
    }
}
static bool Comm_CanStart(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = (uint32_t)(COMM_CAN_ID_COMMAND << COMM_CAN_FILTER_STD_SHIFT);
    filter.FilterIdLow          = 0;
    filter.FilterMaskIdHigh     = (uint32_t)(COMM_CAN_STD_ID_MASK << COMM_CAN_FILTER_STD_SHIFT);
    filter.FilterMaskIdLow      = CAN_ID_EXT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;

    if(HAL_CAN_ConfigFilter(host_can, &filter) != HAL_OK)
    {
        return false;
    }

    if(HAL_CAN_Start(host_can) != HAL_OK)
    {
        return false;
    }

    if(HAL_CAN_ActivateNotification(host_can, CAN_IT_RX_FIFO0_MSG_PENDING |
                                              CAN_IT_TX_MAILBOX_EMPTY |
                                              CAN_IT_BUSOFF |
                                              CAN_IT_ERROR) != HAL_OK)
    {
        return false;
    }

    return true;
}
static void Comm_CanRestart(void)
{
    HAL_CAN_Stop(host_can);
    HAL_CAN_DeInit(host_can);
    HAL_CAN_Init(host_can);

    can_cmd.tx_head = 0;
    can_cmd.tx_tail = 0;
    can_cmd.restart = false;

    if(!Comm_CanStart())
    {
        can_cmd.restart = true;
    }
}
static void Comm_CanCheck(void)
{
    uint32_t now = HAL_GetTick();

    if(can_cmd.restart || HAL_CAN_GetState(host_can) != HAL_CAN_STATE_LISTENING)
    {
        if((now - can_cmd.restart_last_ms) >= COMM_CAN_RESTART_MS)
        {
            can_cmd.restart_last_ms = now;
            Error_Set(OPENBMS_CAN_ERROR);
            Comm_CanRestart();
        }
        return;
    }

    if(can_cmd.send_once)
    {
        can_cmd.send_once         = false;
        can_cmd.broadcast_last_ms = now;
        Comm_CanBroadcast();
    }
    else if(can_cmd.broadcast_on && (now - can_cmd.broadcast_last_ms) >= COMM_CAN_BROADCAST_PERIOD_MS)
    {
        can_cmd.broadcast_last_ms = now;
        Comm_CanBroadcast();
    }
}
static bool Comm_CanQueue(uint32_t id, const uint8_t *data, uint8_t length)
{
    uint8_t next = (uint8_t)((can_cmd.tx_head + 1U) % COMM_CAN_TX_QUEUE_SIZE);
    bool    ok   = false;

    __disable_irq();
    if(next != can_cmd.tx_tail)
    {
        can_cmd.tx_queue[can_cmd.tx_head].id     = id;
        can_cmd.tx_queue[can_cmd.tx_head].length = length;
        memcpy(can_cmd.tx_queue[can_cmd.tx_head].data, data, length);
        can_cmd.tx_head = next;
        ok = true;
    }
    Comm_CanSendNext();
    __enable_irq();

    return ok;
}
static void Comm_CanSendNext(void)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t            mailbox;
    CAN_Message_t       *msg;

    while(can_cmd.tx_tail != can_cmd.tx_head && HAL_CAN_GetTxMailboxesFreeLevel(host_can) > 0)
    {
        msg = &can_cmd.tx_queue[can_cmd.tx_tail];

        header.StdId              = msg->id;
        header.IDE                = CAN_ID_STD;
        header.RTR                = CAN_RTR_DATA;
        header.DLC                = msg->length;
        header.TransmitGlobalTime = DISABLE;

        if(HAL_CAN_AddTxMessage(host_can, &header, msg->data, &mailbox) != HAL_OK)
        {
            break;
        }

        can_cmd.tx_tail = (uint8_t)((can_cmd.tx_tail + 1U) % COMM_CAN_TX_QUEUE_SIZE);
    }
}
static void Comm_CanBroadcast(void)
{
    const System_Data_t     *sys = Ctrl_GetSystem();
    const Config_Data_t     *cfg = Ctrl_GetConfig();
    const FuelGauge_Data_t  *fg  = Ctrl_GetFuelGauge();
    uint8_t                 msg[COMM_CAN_DATA_SIZE];
    uint32_t                u32;
    int32_t                 i32;
    uint16_t                u16;
    int16_t                 i16;
    uint8_t                 n;
    float                   soc;

    __disable_irq();
    if(can_cmd.tx_tail != can_cmd.tx_head)
    {
        HAL_CAN_AbortTxRequest(host_can, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        can_cmd.tx_tail = can_cmd.tx_head;
    }
    __enable_irq();

    Periph_GetData(&pd);

    u32 = (uint32_t)(pd.pack_voltage + 0.5f);
    i32 = (int32_t)lroundf(pd.pack_current);
    memcpy(&msg[0], &u32, sizeof(u32));
    memcpy(&msg[4], &i32, sizeof(i32));
    Comm_CanQueue(COMM_CAN_ID_PACK, msg, sizeof(u32) + sizeof(i32));

    n = 0;
    for(uint8_t i = 0; i < COMM_CAN_CELLS_PER_MESSAGE; i++)
    {
        u16 = (uint16_t)(pd.cell_voltage[i] + 0.5f);
        memcpy(&msg[n], &u16, sizeof(u16));
        n += sizeof(u16);
    }
    Comm_CanQueue(COMM_CAN_ID_CELLS_LOW, msg, n);

    n = 0;
    for(uint8_t i = COMM_CAN_CELLS_PER_MESSAGE; i < SYS_CELL_COUNT_MAX; i++)
    {
        u16 = (uint16_t)(pd.cell_voltage[i] + 0.5f);
        memcpy(&msg[n], &u16, sizeof(u16));
        n += sizeof(u16);
    }
    Comm_CanQueue(COMM_CAN_ID_CELLS_HIGH, msg, n);

    soc = 1.0f;
    for(uint8_t i = 0; i < cfg->cell_count && i < SYS_CELL_COUNT_MAX; i++)
    {
        if(fg->cell_soc_shown_f[i] < soc)
        {
            soc = fg->cell_soc_shown_f[i];
        }
    }
    if(cfg->cell_count == 0 || soc < 0.0f)
    {
        soc = 0.0f;
    }
    u16 = (uint16_t)(soc * COMM_CAN_SOC_SCALE + 0.5f);
    i16 = (int16_t)lroundf(pd.temperature_package * COMM_CAN_TEMP_SCALE);
    memcpy(&msg[0], &u16, sizeof(u16));
    memcpy(&msg[2], &i16, sizeof(i16));
    msg[4] = sys->battery_status;
    msg[5] = sys->charger_present;
    Comm_CanQueue(COMM_CAN_ID_SOC_TEMP, msg, sizeof(u16) + sizeof(i16) + sizeof(sys->battery_status) + sizeof(sys->charger_present));

    u32 = (uint32_t)(sys->error_flags & COMM_CAN_ERROR_MASK);
    msg[0] = sys->ctrl_state;
    memcpy(&msg[1], &pd.fet_status, sizeof(pd.fet_status));
    memcpy(&msg[3], &sys->protection_trigger, sizeof(sys->protection_trigger));
    memcpy(&msg[5], &u32, COMM_CAN_ERROR_BYTES);
    Comm_CanQueue(COMM_CAN_ID_STATE, msg, COMM_CAN_DATA_SIZE);
}
static void Comm_CanProcessCommand(void)
{
    uint8_t         cmd    = can_cmd.rx_data[0];
    uint8_t         length = can_cmd.rx_length;
    uint8_t         *args  = &can_cmd.rx_data[1];
    uint8_t         reply[2];
    CommErrorType_t res;

    if(length == 0)
    {
        return;
    }
    length--;

    if(cmd == CC_CAN_BROADCAST)
    {
        if(length != 1 || (args[0] != 0x00 && args[0] != 0x01))
        {
            res = CE_WRONG_CMD;
        }
        else
        {
            can_cmd.broadcast_on = (bool) args[0];
            res = CE_OK;
        }
    }
    else if(cmd == CC_CAN_SEND_ONCE)
    {
        if(length != 0)
        {
            res = CE_WRONG_CMD;
        }
        else
        {
            can_cmd.send_once = true;
            res = CE_OK;
        }
    }
    else if(cmd == CC_REBOOT)
    {
        if(length != 0)
        {
            res = CE_WRONG_CMD;
        }
        else if(Ctrl_GetSystem()->ctrl_state == CTRL_STATE_SHUTDOWN)
        {
            res = CE_STATE;
        }
        else
        {
            res = CE_OK;
        }
    }
    else if(cmd == CC_SET_FET || cmd == CC_RESET_PROTECTION || cmd == CC_CLEAR_ERRORS || cmd == CC_CLEAR_FAULT || cmd == CC_NORMAL_RUN || cmd == CC_SHUTDOWN)
    {
        res = Comm_ControlsSet(cmd, args, length);
    }
    else
    {
        res = CE_WRONG_CMD;
    }

    if(res == CE_OK)
    {
        Ctrl_SetLastCommunication();
    }

    reply[0] = cmd;
    reply[1] = (uint8_t)res;
    Comm_CanQueue(COMM_CAN_ID_REPLY, reply, sizeof(reply));

    if(cmd == CC_REBOOT && res == CE_OK)
    {
        Ctrl_RequestReset(false);
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t             data[COMM_CAN_DATA_SIZE];

    if(hcan->Instance == CAN1)
    {
        if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            return;
        }

        if(can_cmd.command_ready || header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.StdId != COMM_CAN_ID_COMMAND)
        {
            return;
        }

        memcpy(can_cmd.rx_data, data, sizeof(data));
        can_cmd.rx_length     = (uint8_t)header.DLC;
        can_cmd.command_ready = true;
    }
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1)
    {
        Comm_CanSendNext();
    }
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1)
    {
        Comm_CanSendNext();
    }
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1)
    {
        Comm_CanSendNext();
    }
}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1)
    {
        Comm_CanSendNext();
    }
}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1)
    {
        Comm_CanSendNext();
    }
}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1)
    {
        Comm_CanSendNext();
    }
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1)
    {
        if((HAL_CAN_GetError(hcan) & HAL_CAN_ERROR_BOF) != 0U)
        {
            Error_Set(OPENBMS_CAN_ERROR);
        }
        HAL_CAN_ResetError(hcan);
    }
}
