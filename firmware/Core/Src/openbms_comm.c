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

#define COMM_APP_FLAG_FLASH_PAGE  19U

#ifdef OPENBMS_SIMULATION
// CC_SIM_SAMPLE payload: 1..6 samples (255 / 38 B), each float cell_mv[7],
// float current_ma (+ charge, - discharge), float temperature_c, uint16_t dt_ms
#define COMM_SIM_SAMPLE_LENGTH    (SYS_CELL_COUNT_MAX * sizeof(float) + sizeof(float) + sizeof(float) + sizeof(uint16_t))
#define COMM_SIM_SAMPLE_FLOATS    (SYS_CELL_COUNT_MAX + 2U)
// Reply payload: uint16_t fet_status, uint8_t ctrl_state, uint16_t hppc_status,
// uint16_t hppc_extract_status, uint16_t protection_trigger, uint64_t error_flags (the state after the
// last sample run), uint8_t samples_run
#define COMM_SIM_STATE_LENGTH     (17U)
#define COMM_SIM_REPLY_LENGTH     (COMM_SIM_STATE_LENGTH + 1U)
#endif

static UART_HandleTypeDef   *debug_uart             = &huart1;
static UART_Command_t       uart_cmd                = {0};
static I2C_HandleTypeDef    *host_i2c               = &hi2c2;
static I2C_Command_t        i2c_cmd                 = {0};
static CAN_HandleTypeDef    *host_can               = &hcan1;
static CAN_Command_t        can_cmd                 = {0};
static Peripheral_Data_t    pd                      = {0};
static uint8_t              model_cell              = 0;    // cell the fuel gauge model registers (0xA1 to 0xAC, 0xD4) act on

static CommErrorType_t      Comm_ControlsSet(uint8_t cmd, uint8_t *data, uint8_t length);
static CommErrorType_t      Comm_ReadWriteRegister(uint8_t address, uint8_t *raw_data, uint8_t *length, bool write); 

static void                 Comm_RemoveAppFlag(void);
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
#ifdef OPENBMS_SIMULATION
static CommErrorType_t      Comm_SimSample(const uint8_t *data, uint8_t length, uint8_t *reply, uint8_t *reply_length);
static void                 Comm_SimState(uint8_t *state);
#endif


static CommErrorType_t Comm_ControlsSet(uint8_t cmd, uint8_t *data, uint8_t length)
{
    CommErrorType_t res = CE_OK;

    const System_Data_t *sys = Ctrl_GetSystem();

    switch(cmd)
    {
        case CC_PING:
        {
            // No argument, allowed in every state. The ACK is the answer.
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
        }
        break;

        case CC_EEPROM_CONFIG:
        {
            // No argument. Allowed in every state: it is how a device that is
            // waiting for its configuration gets out of that state.
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else if(!Ctrl_SaveConfig())
            {
                res = CE_EEPROM;
            }
        }
        break;

        case CC_HPPC_START:
        {
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else if(sys->ctrl_state != CTRL_STATE_NORMAL_RUN)
            {
                res = CE_STATE;
            }
            else
            {
                Ctrl_SetMode(CTRL_MODE_HPPC);
            }
        }
        break;

        case CC_NORMAL_RUN:
        {
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else if(sys->ctrl_state != CTRL_STATE_HPPC_RUN)
            {
                res = CE_STATE;
            }
            else
            {
                Ctrl_SetMode(CTRL_MODE_NORMAL);
            }
        }
        break;

        case CC_SET_FET:
        {
            // One argument: 0 - FET off, 1 - FET on
            if(length != 1 || (data[0] != 0x00 && data[0] != 0x01))
            {
                res = CE_WRONG_CMD;
            }
            else if(sys->ctrl_state != CTRL_STATE_NORMAL_RUN)
            {
                // Not configured yet, or the HPPC sequencer owns the FET
                res = CE_STATE;
            }
            else
            {
                // Ctrl_Run switches the FET, this only passes the host's request on
                Ctrl_SetFetRequest((bool) data[0]);
            }
        }
        break;

        case CC_SET_PRE_FET:
        {
            // One argument: 0 - pre-FET off, 1 - pre-FET on
            if(length != 1 || (data[0] != 0x00 && data[0] != 0x01))
            {
                res = CE_WRONG_CMD;
            }
            else if(sys->ctrl_state != CTRL_STATE_NORMAL_RUN)
            {
                // Not configured yet, or an HPPC test is running
                res = CE_STATE;
            }
            else
            {
                // Ctrl_Run switches the pre-FET, this only passes the host's request on
                Ctrl_SetPreFetRequest((bool) data[0]);
            }
        }
        break;

        case CC_RESET_PROTECTION:
        {
            // No argument, allowed in every state. A fault that is still there trips again
            // after its normal detection time.
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
            // No argument, allowed in every state. Clears every latched error flag;
            // an error that is still there is set again when it is next seen.
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else
            {
                Error_ClearAll();
            }
        }
        break;

        case CC_CLEAR_LEARNING:
        {
            // No argument, allowed in every state. Clears all LearningStatus bits (0x96) and capacity_update_count (0x97).
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else
            {
                FuelGauge_ClearLearning();
            }
        }
        break;

        case CC_CLEAR_FAULT:
        {
            // No argument, allowed in every state. The next protection trip is stored again.
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
    uint8_t  frame_length  = *length;   // payload length the host sent, *length becomes the register size below

    const System_Data_t    *sys;
    const Config_Data_t    *cfg;
    const Stats_Data_t     *st;
    const FuelGauge_Data_t *fg;

    if(address <= 0x0F)
    {
        // Peripheral live readings — 0x00 to 0x0F
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
            default: break;
        }
    }
    else if(address >= 0x10 && address <= 0x1F)
    {
        // System state — 0x10 to 0x1F
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
            default: break;
        }
    }
    else if(address >= 0x30 && address <= 0x6F)
    {
        // Config — 0x30 to 0x5F, device info — 0x60 to 0x65, balancing — 0x66 to 0x6A
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
            case 0x60: { data_point = (void *)cfg->firmware_version;                              *length = sizeof(cfg->firmware_version);                                  } break;
            case 0x61: { data_point = (void *)cfg->hardware_version;                              *length = sizeof(cfg->hardware_version);                                  } break;
            case 0x62: { data_point = (void *)cfg->manufacturer_name;                             *length = sizeof(cfg->manufacturer_name);                                 } break;
            case 0x63: { data_point = (void *)cfg->device_name;                                   *length = sizeof(cfg->device_name);                                       } break;
            case 0x64: { data_point = (void *)cfg->device_chemistry;                              *length = sizeof(cfg->device_chemistry);                                  } break;
            case 0x65: { data_point = (void *)cfg->manufacturer_data;                             *length = sizeof(cfg->manufacturer_data);                                 } break;
            case 0x66: { data_point = (void *)&cfg->balance_enable;                               *length = sizeof(cfg->balance_enable);                                    } break;
            case 0x67: { data_point = (void *)&cfg->balance_start_mv;                             *length = sizeof(cfg->balance_start_mv);                                  } break;
            case 0x68: { data_point = (void *)&cfg->balance_delta_mv;                             *length = sizeof(cfg->balance_delta_mv);                                  } break;
            case 0x69: { data_point = (void *)&cfg->balance_max_temp_c;                           *length = sizeof(cfg->balance_max_temp_c);                                } break;
            case 0x6A: { data_point = (void *)&cfg->balance_resistance_ohm;                       *length = sizeof(cfg->balance_resistance_ohm);                            } break;
            default: break;
        }
    }
    else if(address >= 0x70 && address <= 0x7F)
    {
        // Stats — 0x70 to 0x7F
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
        // Fuel gauge — outputs 0x90, model 0xA0, state 0xD0
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
            case 0xBB: { data_point = (void *)&fg->kf_q_soc;                                      *length = sizeof(fg->kf_q_soc);                                } break;
            case 0xBC: { data_point = (void *)&fg->kf_q_rc1;                                      *length = sizeof(fg->kf_q_rc1);                                } break;
            case 0xBD: { data_point = (void *)&fg->kf_q_rc2;                                      *length = sizeof(fg->kf_q_rc2);                                } break;
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
        // A write must carry exactly the register size, otherwise bytes past the payload would be copied
        if(frame_length != *length)
        {
            return CE_WRONG_CMD;
        }

        // The cell count has to stay inside what the hardware measures
        if(is_config && address == 0x30 && (raw_data[0] < SYS_CELL_COUNT_MIN || raw_data[0] > SYS_CELL_COUNT_MAX))
        {
            return CE_WRONG_CMD;
        }

        // The model registers act on one cell, picked with register 0x9F
        if(address == 0x9F && raw_data[0] >= SYS_CELL_COUNT_MAX)
        {
            return CE_WRONG_CMD;
        }

        // The on/off settings (0x31 to 0x36, 0x66) are a single byte, 0 or 1
        if(is_config && ((address >= 0x31 && address <= 0x36) || address == 0x66) && raw_data[0] > 1)
        {
            return CE_WRONG_CMD;
        }

        // The control module owns the config and the fuel gauge struct, data_point only
        // points into them, so it does the write. A NaN or inf float is refused, the old value stays.
        if(is_fuel_gauge)
        {
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
            // This module's own register (0x9F)
            memcpy(data_point, raw_data, *length);
        }
    }
    else
    {
        memcpy(raw_data, data_point, *length);
    }

    return CE_OK;
}
static void Comm_RemoveAppFlag(void)
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
    erase_init.Page        = COMM_APP_FLAG_FLASH_PAGE;
    erase_init.NbPages     = 1;

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);

    HAL_FLASH_Lock();

    if(status != HAL_OK)
    {
        //return ERROR_ERRASE_FAIL;
    }

    //return OK;
}
// Waits for the previous UART send to finish. The longest reply takes about 8 ms at 230400 baud,
// so a send still running after COMM_TX_TIMEOUT_MS is stuck: abort it instead of
// blocking the main loop for good.
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

    // Each port has its own buffers, so a reply still being sent on one port is never overwritten
    if(port == COMM_PORT_I2C)
    {
        rx_buf = i2c_cmd.rx_buffer;
        tx_buf = i2c_cmd.tx_buffer;
    }
    else
    {
        rx_buf = uart_cmd.rx_buffer;
        tx_buf = uart_cmd.tx_buffer;

        // The reply and error buffers are filled while the command runs, so the previous
        // reply must be fully sent before anything is written into them
        Comm_UartWaitTxIdle();
    }

    // -------------------------------------------------------
    // Parse command type
    // -------------------------------------------------------
    cmd = rx_buf[0];
    if( cmd != CC_WRITE && 
        cmd != CC_READ && 
        cmd != CC_BOOTLOADER &&
        cmd != CC_EEPROM_CONFIG &&
        cmd != CC_HPPC_START &&
        cmd != CC_NORMAL_RUN &&
        cmd != CC_SET_FET &&
        cmd != CC_SET_PRE_FET &&
        cmd != CC_RESET_PROTECTION &&
        cmd != CC_CLEAR_ERRORS &&
        cmd != CC_SIM_SAMPLE &&
        cmd != CC_CLEAR_LEARNING &&
        cmd != CC_CLEAR_FAULT &&
        cmd != CC_REBOOT &&
        cmd != CC_PING)
    {
        Comm_SendError(port, CE_WRONG_CMD);
        return;
    }

    // -------------------------------------------------------
    // Perform Bootloader Running or Command Execution
    // -------------------------------------------------------
    if(cmd == CC_BOOTLOADER)
    {
        Comm_RemoveAppFlag();
        Comm_SendResponse(port, (uint8_t*)"1\n",2);
        HAL_Delay(COMM_REBOOT_DELAY_MS);
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
    crc_calc = Comm_Checksum(rx_buf, length + 3);
    crc_rec  = rx_buf[length + 3];

    if(crc_calc != crc_rec)
    {
        Comm_SendError(port, CE_BAD_CRC);
        return;
    }

    // A good frame came from the host
    Ctrl_SetLastCommunication();

    // -------------------------------------------------------
    // Execute
    // -------------------------------------------------------
    if(cmd == CC_WRITE)
    {
        res = Comm_ReadWriteRegister(address, &rx_buf[3], &length, true);
    }
    else if(cmd == CC_READ)
    {
        // The peripheral data is copied with interrupts masked inside Periph_GetData
        res = Comm_ReadWriteRegister(address, &tx_buf[3], &length, false);
    }
    else if(cmd == CC_REBOOT)
    {
        // No argument. Allowed in every state.
        if(length != 0)
        {
            res = CE_WRONG_CMD;
        }
        else
        {
            res = CE_OK;
        }
    }
#ifdef OPENBMS_SIMULATION
    else if(cmd == CC_SIM_SAMPLE)
    {
        res = Comm_SimSample(&rx_buf[3], length, &tx_buf[3], &length);
    }
#endif
    else
    {
        res = Comm_ControlsSet(cmd, &rx_buf[3], length);
    }

    Comm_SendReply(port, res, cmd, address, length);

    // Reset only after the ACK has been handed to the UART (or the host had time to read it on I2C)
    if(cmd == CC_REBOOT && res == CE_OK)
    {
        HAL_Delay(COMM_REBOOT_DELAY_MS);
        NVIC_SystemReset();
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
        // The host reads the reply when it wants: just leave it in the I2C TX buffer
        if(data != i2c_cmd.tx_buffer)
        {
            memcpy(i2c_cmd.tx_buffer, data, length);
        }

        // The buffer must be complete before the interrupt can see a non-zero length
        __DMB();
        i2c_cmd.tx_length = length;
        return;
    }

    // Comm_ProcessCommand already waited for the previous send (Comm_UartWaitTxIdle)
    // Raise the flag and send new data
    uart_cmd.tx_wait = true;

    // If the transfer could not be started there will be no TX complete
    // interrupt to clear the flag, and the next send would wait for it forever
    if(HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data, length) != HAL_OK)
    {
        uart_cmd.tx_wait = false;
    }
}
static void Comm_SendError(CommPort_t port, CommErrorType_t err)
{
    // static: the DMA is still reading this buffer after the function has returned
    static uint8_t buff[5];

    buff[0] = CC_ERROR;
    buff[1] = 0;    // Address
    buff[2] = 1;    // Lenght
    buff[3] = err; // Error type
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

    if(cmd == CC_READ || cmd == CC_SIM_SAMPLE)
    {
        // Reply with data: the payload is already in tx_buf[3...]
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
#ifdef OPENBMS_SIMULATION
// Runs the samples in order, one pass of the control loop each. The batch stops at the first sample
// that changes the reply state (FET, ctrl state, HPPC, protection, errors): the host made the later
// samples for the old state and sends them again.
static CommErrorType_t Comm_SimSample(const uint8_t *data, uint8_t length, uint8_t *reply, uint8_t *reply_length)
{
    float    cell_mv[SYS_CELL_COUNT_MAX];
    float    current_ma;
    float    temperature_c;
    uint16_t dt_ms;
    uint8_t  before[COMM_SIM_STATE_LENGTH];
    uint8_t  count;
    uint8_t  run = 0;

    if(length == 0 || length % COMM_SIM_SAMPLE_LENGTH != 0)
    {
        return CE_WRONG_CMD;
    }
    count = length / COMM_SIM_SAMPLE_LENGTH;

    // Only a configured device runs the control loop
    const System_Data_t *sys = Ctrl_GetSystem();
    if(sys->ctrl_state != CTRL_STATE_NORMAL_RUN && sys->ctrl_state != CTRL_STATE_HPPC_RUN)
    {
        return CE_STATE;
    }

    // A NaN would stay in the fuel gauge and the extraction for good: check the whole batch first
    for(uint8_t s = 0; s < count; s++)
    {
        for(uint8_t k = 0; k < COMM_SIM_SAMPLE_FLOATS; k++)
        {
            float value;

            memcpy(&value, &data[s * COMM_SIM_SAMPLE_LENGTH + k * sizeof(float)], sizeof(value));
            if(!isfinite(value))
            {
                return CE_WRONG_CMD;
            }
        }
    }

    Comm_SimState(before);

    while(run < count)
    {
        const uint8_t *sample = &data[run * COMM_SIM_SAMPLE_LENGTH];

        memcpy(cell_mv, &sample[0], sizeof(cell_mv));
        memcpy(&current_ma, &sample[sizeof(cell_mv)], sizeof(current_ma));
        memcpy(&temperature_c, &sample[sizeof(cell_mv) + sizeof(current_ma)], sizeof(temperature_c));
        memcpy(&dt_ms, &sample[sizeof(cell_mv) + sizeof(current_ma) + sizeof(temperature_c)], sizeof(dt_ms));

        // One sample, one pass of the control loop
        Ctrl_SimSample(cell_mv, current_ma, temperature_c, dt_ms);
        run++;

        // The reply tells the host what the firmware did with it
        Comm_SimState(reply);
        if(memcmp(reply, before, COMM_SIM_STATE_LENGTH) != 0)
        {
            break;
        }
    }

    reply[COMM_SIM_STATE_LENGTH] = run;
    *reply_length = COMM_SIM_REPLY_LENGTH;
    return CE_OK;
}

static void Comm_SimState(uint8_t *state)
{
    const System_Data_t *sys = Ctrl_GetSystem();
    uint8_t n = 0;

    Periph_GetData(&pd);

    memcpy(&state[n], &pd.fet_status, sizeof(pd.fet_status));                     n += sizeof(pd.fet_status);
    memcpy(&state[n], &sys->ctrl_state, sizeof(sys->ctrl_state));                   n += sizeof(sys->ctrl_state);
    memcpy(&state[n], &sys->hppc_status, sizeof(sys->hppc_status));                 n += sizeof(sys->hppc_status);
    memcpy(&state[n], &sys->hppc_extract_status, sizeof(sys->hppc_extract_status)); n += sizeof(sys->hppc_extract_status);
    memcpy(&state[n], &sys->protection_trigger, sizeof(sys->protection_trigger));   n += sizeof(sys->protection_trigger);
    memcpy(&state[n], &sys->error_flags, sizeof(sys->error_flags));
}
#endif
void Comm_Init(void)
{
    // Start data receiving on UART
    HAL_UARTEx_ReceiveToIdle_DMA(debug_uart, uart_cmd.rx_buffer, COMM_RX_BUFFER_SIZE);

    // Start listening for the host on I2C
    HAL_I2C_EnableListen_IT(host_i2c);

    // Start CAN: command messages in, status broadcast out
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

  // A frame that stopped half way (lost bytes, a host that gave up) is dropped and reception starts over
  if(uart_cmd.rx_count != 0 && HAL_GetTick() - uart_cmd.rx_start_ms > COMM_UART_FRAME_TIMEOUT_MS)
  {
    HAL_UART_AbortReceive(debug_uart);
    uart_cmd.rx_count = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(debug_uart, uart_cmd.rx_buffer, COMM_RX_BUFFER_SIZE);
  }

  // Safety net: reception must always be running. If an error stopped it and the
  // re-arm in the error callback failed, start it again here.
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
        // Half transfer: the DMA is still receiving, wait for the line to go idle
        if(HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
        {
            return;
        }

        // A long frame can come in pieces (the USB-UART bridge pauses between USB packets):
        // keep receiving behind the piece until the length byte says the frame is complete.
        // The flasher's bootloader request ("B\n") has no length byte and is complete as it is.
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
    // transmission finished — buffer is now safe to reuse/refill
    uart_cmd.tx_wait = false;
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    // The HAL has already cleared the error flags and stopped the transfer that failed.
    // Report it and restart what was stopped, so the link keeps working.
    Error_Set(OPENBMS_UART_ERROR);

    // A failed send gives no TX complete interrupt, so release the waiting sender here
    if(huart->gState == HAL_UART_STATE_READY)
    {
      uart_cmd.tx_wait = false;
    }

    // Reception was aborted: drop the broken frame and listen again
    if(huart->RxState == HAL_UART_STATE_READY)
    {
      uart_cmd.rx_count = 0;
      HAL_UARTEx_ReceiveToIdle_DMA(debug_uart, uart_cmd.rx_buffer, COMM_RX_BUFFER_SIZE);
    }
  }
}
static void Comm_I2cRestart(void)
{
    // Full restart of the peripheral: clears a stuck state machine and releases the bus lines.
    // The handle keeps its Init settings, so HAL_I2C_Init sets it up exactly as main.c did.
    HAL_I2C_DeInit(host_i2c);
    HAL_I2C_Init(host_i2c);

    i2c_cmd.rx_count        = 0;
    i2c_cmd.transfer_active = false;
    i2c_cmd.restart         = false;

    HAL_I2C_EnableListen_IT(host_i2c);
}
static void Comm_I2cCheck(void)
{
    // A bus error stopped the transfer: restart the I2C from the main loop, not from the interrupt
    if(i2c_cmd.restart)
    {
        Comm_I2cRestart();
        return;
    }

    // A transfer that never ends (host gone in the middle, line held low) blocks the I2C for good
    if(i2c_cmd.transfer_active)
    {
        if((HAL_GetTick() - i2c_cmd.transfer_start_ms) > COMM_I2C_TRANSFER_TIMEOUT_MS)
        {
            Error_Set(OPENBMS_I2C_ERROR);
            Comm_I2cRestart();
        }
        return;
    }

    // Safety net: listening must always be running. If an error stopped it, start it again here.
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

    // Bytes that do not fit, or that arrive while the last frame is still being processed,
    // go to a dummy byte: the frame is dropped at the STOP
    if(i2c_cmd.frame_ready || i2c_cmd.rx_count >= COMM_RX_BUFFER_SIZE)
    {
        dest = &i2c_cmd.rx_discard;
    }
    else
    {
        dest = &i2c_cmd.rx_buffer[i2c_cmd.rx_count];
    }

    // One byte at a time: the frame length is only known from its third byte
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
        // Reply not ready yet: the host sees COMM_I2C_NOT_READY and reads again later
        i2c_cmd.tx_fill = COMM_I2C_NOT_READY;
        src = &i2c_cmd.tx_fill;
    }
    else if(i2c_cmd.tx_index >= i2c_cmd.tx_length)
    {
        // The host reads more than the reply
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
            // Host writes a new frame: the old reply is no longer valid.
            // While a frame is still being processed its reply is kept and the new bytes are dropped.
            i2c_cmd.rx_count = 0;
            if(!i2c_cmd.frame_ready)
            {
                i2c_cmd.tx_length = 0;
            }
            Comm_I2cReceiveNext();
        }
        else
        {
            // Host reads: every read starts at the first reply byte
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
        // STOP: the transfer is over
        count = i2c_cmd.rx_count;
        i2c_cmd.transfer_active = false;

        // A write is a frame only when its length matches the length byte: command, address,
        // length, payload, checksum. Anything else is dropped.
        if(i2c_cmd.direction == I2C_DIRECTION_TRANSMIT && !i2c_cmd.frame_ready)
        {
            if(count >= 4 && count <= COMM_RX_BUFFER_SIZE && count == (uint16_t)i2c_cmd.rx_buffer[2] + 4)
            {
                i2c_cmd.frame_ready = true;
            }
        }
        i2c_cmd.rx_count = 0;

        // Listen mode ends at every STOP: start it again for the next transfer
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
        // A NACK is the normal end of a transfer: the host NACKs the last byte it reads, and a
        // byte still waiting when the host sends STOP is reported the same way. The listen
        // complete callback follows at the STOP.
        if(HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF)
        {
            return;
        }

        // Real bus error (bus error, arbitration lost, overrun): report it and restart the I2C
        Error_Set(OPENBMS_I2C_ERROR);
        i2c_cmd.restart = true;
    }
}
static bool Comm_CanStart(void)
{
    CAN_FilterTypeDef filter = {0};

    // Only the command ID gets into FIFO 0
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = (uint32_t)(COMM_CAN_ID_COMMAND << COMM_CAN_FILTER_STD_SHIFT);
    filter.FilterIdLow          = 0;
    filter.FilterMaskIdHigh     = (uint32_t)(COMM_CAN_STD_ID_MASK << COMM_CAN_FILTER_STD_SHIFT);
    filter.FilterMaskIdLow      = CAN_ID_EXT;   // IDE bit must match: standard IDs only
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

    // Bus-off is recovered by the hardware (AutoBusOff in main.c), the interrupt only reports it
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
    // Full restart of the peripheral. The handle keeps its Init settings, so HAL_CAN_Init
    // sets it up exactly as main.c did. Queued messages are stale by now: drop them.
    HAL_CAN_Stop(host_can);
    HAL_CAN_DeInit(host_can);
    HAL_CAN_Init(host_can);

    can_cmd.tx_head = 0;
    can_cmd.tx_tail = 0;
    can_cmd.restart = false;

    if(!Comm_CanStart())
    {
        // Tried again at the next Comm_Run
        can_cmd.restart = true;
    }
}
static void Comm_CanCheck(void)
{
    uint32_t now = HAL_GetTick();

    // An error stopped the CAN, or it is not running for another reason: restart it from the main loop.
    // A start can wait for the bus to go idle (up to 10 ms in the HAL), so with a dead bus or no
    // transceiver power it is retried only every COMM_CAN_RESTART_MS, not on every pass.
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

    // The TX interrupt also moves the queue: keep it out while the slot is filled
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

    // Called with the CAN interrupts kept out (main loop) or from the CAN TX interrupt itself
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

    // Messages still queued from the last broadcast never got onto the bus (nobody acknowledges,
    // or the bus is too busy): they are stale now. Drop them and abort the mailboxes that hold
    // old ones, so the new values go out first.
    __disable_irq();
    if(can_cmd.tx_tail != can_cmd.tx_head)
    {
        HAL_CAN_AbortTxRequest(host_can, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        can_cmd.tx_tail = can_cmd.tx_head;
    }
    __enable_irq();

    Periph_GetData(&pd);

    // 0x6B0: pack voltage, pack current
    u32 = (uint32_t)(pd.pack_voltage + 0.5f);
    i32 = (int32_t)lroundf(pd.pack_current);
    memcpy(&msg[0], &u32, sizeof(u32));
    memcpy(&msg[4], &i32, sizeof(i32));
    Comm_CanQueue(COMM_CAN_ID_PACK, msg, sizeof(u32) + sizeof(i32));

    // 0x6B1: cells 1..4, 0x6B2: cells 5..7
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

    // 0x6B3: SOC of the weakest cell (as relative_soc, but in 0.1 %), temperature
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
    Comm_CanQueue(COMM_CAN_ID_SOC_TEMP, msg, sizeof(u16) + sizeof(i16));

    // 0x6B4: state, error flags low 24 bits
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
    length--;   // arguments after the command byte

    if(cmd == CC_CAN_BROADCAST)
    {
        // One argument: 0 - off, 1 - on. RAM only, back to COMM_CAN_BROADCAST_DEFAULT after a reboot.
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
        else
        {
            res = CE_OK;
        }
    }
    else if(cmd == CC_SET_FET || cmd == CC_RESET_PROTECTION || cmd == CC_CLEAR_ERRORS || cmd == CC_CLEAR_FAULT || cmd == CC_NORMAL_RUN)
    {
        res = Comm_ControlsSet(cmd, args, length);
    }
    else
    {
        res = CE_WRONG_CMD;
    }

    // A good command came from the host
    if(res == CE_OK)
    {
        Ctrl_SetLastCommunication();
    }

    reply[0] = cmd;
    reply[1] = (uint8_t)res;
    Comm_CanQueue(COMM_CAN_ID_REPLY, reply, sizeof(reply));

    // Reset only after the reply had time to leave
    if(cmd == CC_REBOOT && res == CE_OK)
    {
        HAL_Delay(COMM_REBOOT_DELAY_MS);
        NVIC_SystemReset();
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t             data[COMM_CAN_DATA_SIZE];

    if(hcan->Instance == CAN1)
    {
        // Always read: the message must leave the FIFO even when it is dropped
        if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            return;
        }

        // One command at a time: a command that arrives before the last one is processed is dropped
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
        // Bus-off is left by the hardware on its own (AutoBusOff). Report it, and restart the
        // CAN if the error left it stopped (checked in Comm_CanCheck).
        if((HAL_CAN_GetError(hcan) & HAL_CAN_ERROR_BOF) != 0U)
        {
            Error_Set(OPENBMS_CAN_ERROR);
        }
        HAL_CAN_ResetError(hcan);
    }
}
