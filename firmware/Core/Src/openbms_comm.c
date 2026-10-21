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

#define APP_FLAG_FLASH_PAGE  19U

#ifdef OPENBMS_SIMULATION
// CC_SIM_SAMPLE payload: float cell_mv[7], float current_ma (+ charge, - discharge), uint16_t dt_ms
#define SIM_SAMPLE_LENGTH    (BD_CELL_COUNT_MAX * sizeof(float) + sizeof(float) + sizeof(uint16_t))
// Reply payload: uint16_t fet_status, uint8_t ctrl_state, uint16_t hppc_status,
// uint16_t hppc_extract_status, uint8_t protection_trigger, uint64_t error_flags
#define SIM_REPLY_LENGTH     (16U)
#endif

static UART_HandleTypeDef   *debug_uart             = &huart1;
static UART_Command_t       uart_cmd                = {0};
static Peripheral_Data_t    pd                      = {0};
static Calibration_Data_t   cal                     = {0};
static Config_Data_t        cfg                     = {0};
static System_Data_t        sys                     = {0};
static Stats_Data_t         st                      = {0};
static FuelGauge_Data_t     fg                      = {0};
static uint8_t              model_cell              = 0;    // cell the fuel gauge model registers (0xA1 to 0xAC, 0xD4) act on

static CommErrorType_t      Controls_Set(uint8_t cmd, uint8_t *data, uint8_t length);
static CommErrorType_t      Data_ReadWriteRegister(uint8_t address, uint8_t *raw_data, uint8_t *length, bool write); 

static void                 RemoveAppFlag(void);
static void                 ProcessCommand(void);
static uint8_t              Checksum(uint8_t *data, uint16_t length);
static void                 SendResponse(uint8_t *data, uint8_t length);
static void                 SendError(CommErrorType_t err);
static void                 SendReply(CommErrorType_t res, uint8_t cmd, uint8_t address, uint8_t length);
#ifdef OPENBMS_SIMULATION
static CommErrorType_t      Sim_Sample(const uint8_t *data, uint8_t length, uint8_t *reply, uint8_t *reply_length);
#endif


static CommErrorType_t Controls_Set(uint8_t cmd, uint8_t *data, uint8_t length)
{
    CommErrorType_t res = CE_OK;

    // Load data
    Ctrl_GetSystem(&sys);

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
            else if(!Ctrl_ConfigValid())
            {
                // Not set up yet, for example the cell count was never written
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
            else if(sys.ctrl_state != BD_CTRL_STATE_NORMAL_RUN)
            {
                res = CE_STATE;
            }
            else
            {
                Ctrl_SetMode(BD_MAIN_CTR_MODE_HPPC);
            }
        }
        break;

        case CC_NORMAL_RUN:
        {
            if(length != 0)
            {
                res = CE_WRONG_CMD;
            }
            else if(sys.ctrl_state != BD_CTRL_STATE_HPPC_RUN)
            {
                res = CE_STATE;
            }
            else
            {
                Ctrl_SetMode(BD_MAIN_CTR_MODE_NORMAL);
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
            else if(sys.ctrl_state != BD_CTRL_STATE_NORMAL_RUN)
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
            else if(sys.ctrl_state != BD_CTRL_STATE_NORMAL_RUN)
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

        default:
        {
            res = CE_WRONG_CMD;
        }
        break;
    }

    return res;
}
static CommErrorType_t Data_ReadWriteRegister(uint8_t address, uint8_t *raw_data, uint8_t *length, bool write)
{
    void    *data_point    = NULL;
    bool     ro            = false;
    bool     is_fuel_gauge = false;
    bool     is_config     = false;
    uint8_t  frame_length  = *length;   // payload length the host sent, *length becomes the register size below

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
        Ctrl_GetSystem(&sys);
        switch (address)
        {
            case 0x10: { data_point = (void *)&sys.main_control;                                 *length = sizeof(sys.main_control);                            ro = true; } break;
            case 0x11: { data_point = (void *)&sys.last_communication_timestamp;                 *length = sizeof(sys.last_communication_timestamp);            ro = true; } break;
            case 0x12: { data_point = (void *)&sys.protection_trigger;                           *length = sizeof(sys.protection_trigger);                      ro = true; } break;
            case 0x13: { data_point = (void *)&sys.ctrl_state;                                   *length = sizeof(sys.ctrl_state);                              ro = true; } break;
            case 0x15: { data_point = (void *)&sys.error_flags;                                  *length = sizeof(sys.error_flags);                             ro = true; } break;
            case 0x17: { data_point = (void *)&sys.session_uptime_ms;                            *length = sizeof(sys.session_uptime_ms);                       ro = true; } break;
            case 0x18: { data_point = (void *)&sys.hppc_direction;                               *length = sizeof(sys.hppc_direction);                          ro = true; } break;
            case 0x19: { data_point = (void *)&sys.hppc_status;                                  *length = sizeof(sys.hppc_status);                             ro = true; } break;
            case 0x1A: { data_point = (void *)&sys.hppc_extract_status;                          *length = sizeof(sys.hppc_extract_status);                     ro = true; } break;
            default: break;
        }
    }
    else if(address >= 0x20 && address <= 0x2F)
    {
        // Calibration — 0x20 to 0x2F, the offsets measured at boot (the gains etc. are config registers)
        Periph_GetCalibration(&cal);
        switch (address)
        {
            case 0x20: { data_point = (void *)&cal.current_sensor_offset;                        *length = sizeof(cal.current_sensor_offset);                  ro = true; } break;
            case 0x22: { data_point = (void *)cal.voltage_offset;                                *length = sizeof(cal.voltage_offset);                         ro = true; } break;
            default: break;
        }
    }
    else if(address >= 0x30 && address <= 0x6F)
    {
        // Config — 0x30 to 0x5F, device info — 0x60 to 0x6F
        Ctrl_GetConfig(&cfg);
        is_config = true;
        switch (address)
        {
            case 0x30: { data_point = (void *)&cfg.cell_count;                                   *length = sizeof(cfg.cell_count);                             } break;
            case 0x31: { data_point = (void *)&cfg.use_i2c;                                      *length = sizeof(cfg.use_i2c);                                } break;
            case 0x32: { data_point = (void *)&cfg.use_can;                                      *length = sizeof(cfg.use_can);                                } break;
            case 0x33: { data_point = (void *)&cfg.use_uart;                                     *length = sizeof(cfg.use_uart);                               } break;
            case 0x34: { data_point = (void *)&cfg.volt_protection;                              *length = sizeof(cfg.volt_protection);                        } break;
            case 0x35: { data_point = (void *)&cfg.current_protection;                           *length = sizeof(cfg.current_protection);                     } break;
            case 0x36: { data_point = (void *)&cfg.temp_protection;                              *length = sizeof(cfg.temp_protection);                        } break;
            case 0x37: { data_point = (void *)&cfg.cell_capacity;                                *length = sizeof(cfg.cell_capacity);                          } break;
            case 0x38: { data_point = (void *)&cfg.voltage_cell_max;                             *length = sizeof(cfg.voltage_cell_max);                       } break;
            case 0x39: { data_point = (void *)&cfg.voltage_cell_min;                             *length = sizeof(cfg.voltage_cell_min);                       } break;
            case 0x3A: { data_point = (void *)&cfg.charging_term_current;                        *length = sizeof(cfg.charging_term_current);                  } break;
            case 0x3B: { data_point = (void *)&cfg.uvp_slow_threshold_mv;                        *length = sizeof(cfg.uvp_slow_threshold_mv);                  } break;
            case 0x3C: { data_point = (void *)&cfg.uvp_slow_time_ms;                             *length = sizeof(cfg.uvp_slow_time_ms);                       } break;
            case 0x3D: { data_point = (void *)&cfg.uvp_fast_threshold_mv;                        *length = sizeof(cfg.uvp_fast_threshold_mv);                  } break;
            case 0x3E: { data_point = (void *)&cfg.uvp_fast_time_ms;                             *length = sizeof(cfg.uvp_fast_time_ms);                       } break;
            case 0x3F: { data_point = (void *)&cfg.ovp_slow_threshold_mv;                        *length = sizeof(cfg.ovp_slow_threshold_mv);                  } break;
            case 0x40: { data_point = (void *)&cfg.ovp_slow_time_ms;                             *length = sizeof(cfg.ovp_slow_time_ms);                       } break;
            case 0x41: { data_point = (void *)&cfg.ovp_fast_threshold_mv;                        *length = sizeof(cfg.ovp_fast_threshold_mv);                  } break;
            case 0x42: { data_point = (void *)&cfg.ovp_fast_time_ms;                             *length = sizeof(cfg.ovp_fast_time_ms);                       } break;
            case 0x43: { data_point = (void *)&cfg.ocp_charge_threshold_ma;                      *length = sizeof(cfg.ocp_charge_threshold_ma);                } break;
            case 0x44: { data_point = (void *)&cfg.ocp_charge_time_ms;                           *length = sizeof(cfg.ocp_charge_time_ms);                     } break;
            case 0x45: { data_point = (void *)&cfg.ocp_discharge_slow_threshold_ma;              *length = sizeof(cfg.ocp_discharge_slow_threshold_ma);        } break;
            case 0x46: { data_point = (void *)&cfg.ocp_discharge_slow_time_ms;                   *length = sizeof(cfg.ocp_discharge_slow_time_ms);             } break;
            case 0x47: { data_point = (void *)&cfg.ocp_discharge_fast_threshold_ma;              *length = sizeof(cfg.ocp_discharge_fast_threshold_ma);        } break;
            case 0x48: { data_point = (void *)&cfg.ocp_discharge_fast_time_ms;                   *length = sizeof(cfg.ocp_discharge_fast_time_ms);             } break;
            case 0x49: { data_point = (void *)&cfg.otp_threshold_c;                              *length = sizeof(cfg.otp_threshold_c);                        } break;
            case 0x4A: { data_point = (void *)&cfg.otp_time_ms;                                  *length = sizeof(cfg.otp_time_ms);                            } break;
            case 0x4B: { data_point = (void *)&cfg.charge_stop_cell_mv;                          *length = sizeof(cfg.charge_stop_cell_mv);                    } break;
            case 0x4C: { data_point = (void *)&cfg.charge_monitor_cell_mv;                       *length = sizeof(cfg.charge_monitor_cell_mv);                 } break;
            case 0x4D: { data_point = (void *)&cfg.utp_threshold_c;                              *length = sizeof(cfg.utp_threshold_c);                        } break;
            case 0x4E: { data_point = (void *)&cfg.utp_time_ms;                                  *length = sizeof(cfg.utp_time_ms);                            } break;
            case 0x4F: { data_point = (void *)&cfg.otp_charge_threshold_c;                       *length = sizeof(cfg.otp_charge_threshold_c);                 } break;
            case 0x50: { data_point = (void *)&cfg.ocv_v_lo_mv;                                  *length = sizeof(cfg.ocv_v_lo_mv);                            } break;
            case 0x51: { data_point = (void *)&cfg.ocv_v_hi_mv;                                  *length = sizeof(cfg.ocv_v_hi_mv);                            } break;
            case 0x52: { data_point = (void *)&cfg.current_sensor_gain;                          *length = sizeof(cfg.current_sensor_gain);                    } break;
            case 0x53: { data_point = (void *)cfg.voltage_gain;                                  *length = sizeof(cfg.voltage_gain);                           } break;
            case 0x54: { data_point = (void *)&cfg.ntc_beta;                                     *length = sizeof(cfg.ntc_beta);                               } break;
            case 0x55: { data_point = (void *)&cfg.ntc_r_nominal;                                *length = sizeof(cfg.ntc_r_nominal);                          } break;
            case 0x56: { data_point = (void *)&cfg.ntc_r_fixed;                                  *length = sizeof(cfg.ntc_r_fixed);                            } break;
            case 0x57: { data_point = (void *)&cfg.ntc_t_nominal;                                *length = sizeof(cfg.ntc_t_nominal);                          } break;
            case 0x58: { data_point = (void *)&cfg.temperature_offset;                           *length = sizeof(cfg.temperature_offset);                     } break;
            case 0x59: { data_point = (void *)&cfg.shunt_resistance_mohm;                        *length = sizeof(cfg.shunt_resistance_mohm);                  } break;
            case 0x5A: { data_point = (void *)&cfg.current_bias_ma;                              *length = sizeof(cfg.current_bias_ma);                        } break;
            case 0x60: { data_point = (void *)cfg.firmware_version;                              *length = sizeof(cfg.firmware_version);                                  } break;
            case 0x61: { data_point = (void *)cfg.hardware_version;                              *length = sizeof(cfg.hardware_version);                                  } break;
            case 0x62: { data_point = (void *)cfg.manufacturer_name;                             *length = sizeof(cfg.manufacturer_name);                                 } break;
            case 0x63: { data_point = (void *)cfg.device_name;                                   *length = sizeof(cfg.device_name);                                       } break;
            case 0x64: { data_point = (void *)cfg.device_chemistry;                              *length = sizeof(cfg.device_chemistry);                                  } break;
            case 0x65: { data_point = (void *)cfg.manufacturer_data;                             *length = sizeof(cfg.manufacturer_data);                                 } break;
            default: break;
        }
    }
    else if(address >= 0x70 && address <= 0x7F)
    {
        // Stats — 0x70 to 0x7F
        Ctrl_GetStats(&st);
        switch (address)
        {
            case 0x70: { data_point = (void *)st.fault_snapshot_voltage;                         *length = sizeof(st.fault_snapshot_voltage);                   ro = true; } break;
            case 0x71: { data_point = (void *)&st.fault_snapshot_current;                        *length = sizeof(st.fault_snapshot_current);                   ro = true; } break;
            case 0x72: { data_point = (void *)&st.fault_snapshot_temperature;                    *length = sizeof(st.fault_snapshot_temperature);               ro = true; } break;
            case 0x73: { data_point = (void *)&st.fault_snapshot_soc;                            *length = sizeof(st.fault_snapshot_soc);                       ro = true; } break;
            case 0x74: { data_point = (void *)st.fault_code;                                     *length = sizeof(st.fault_code);                               ro = true; } break;
            case 0x75: { data_point = (void *)st.fault_timestamp;                                *length = sizeof(st.fault_timestamp);                          ro = true; } break;
            case 0x76: { data_point = (void *)st.cell_balancing_energy;                          *length = sizeof(st.cell_balancing_energy);                    ro = true; } break;
            case 0x77: { data_point = (void *)st.cell_balancing_time;                            *length = sizeof(st.cell_balancing_time);                      ro = true; } break;
            case 0x78: { data_point = (void *)st.cell_deepest_discharge;                         *length = sizeof(st.cell_deepest_discharge);                   ro = true; } break;
            case 0x79: { data_point = (void *)st.cell_max_temperature;                           *length = sizeof(st.cell_max_temperature);                     ro = true; } break;
            default: break;
        }
    }
    else if(address >= 0x90 && address <= 0xDF)
    {
        // Fuel gauge — outputs 0x90, model 0xA0, state 0xD0
        FuelGauge_GetData(&fg);
        is_fuel_gauge = true;
        switch (address)
        {
            case 0x90: { data_point = (void *)&fg.relative_soc;                                  *length = sizeof(fg.relative_soc);                             ro = true; } break;
            case 0x91: { data_point = (void *)&fg.absolute_soc;                                  *length = sizeof(fg.absolute_soc);                             ro = true; } break;
            case 0x92: { data_point = (void *)fg.cell_soc;                                       *length = sizeof(fg.cell_soc);                                 ro = true; } break;
            case 0x93: { data_point = (void *)fg.cell_soh;                                       *length = sizeof(fg.cell_soh);                                 ro = true; } break;
            case 0x94: { data_point = (void *)fg.cell_remaining_capacity;                        *length = sizeof(fg.cell_remaining_capacity);                  ro = true; } break;
            case 0x95: { data_point = (void *)fg.cell_self_discharge;                            *length = sizeof(fg.cell_self_discharge);                      ro = true; } break;
            case 0x96: { data_point = (void *)&fg.learning_status;                               *length = sizeof(fg.learning_status);                          ro = true; } break;
            case 0x9F: { data_point = (void *)&model_cell;                                       *length = sizeof(model_cell);                                  is_fuel_gauge = false; } break;
            case 0xA0: { data_point = (void *)fg.soc_grid;                                       *length = sizeof(fg.soc_grid);                                } break;
            case 0xA1: { data_point = (void *)fg.cell[model_cell].ocv_dis;                                        *length = sizeof(fg.cell[0].ocv_dis);                                 } break;
            case 0xA2: { data_point = (void *)fg.cell[model_cell].ocv_chg;                                        *length = sizeof(fg.cell[0].ocv_chg);                                 } break;
            case 0xA3: { data_point = (void *)fg.cell[model_cell].r0_dis;                                         *length = sizeof(fg.cell[0].r0_dis);                                  } break;
            case 0xA4: { data_point = (void *)fg.cell[model_cell].r1_dis;                                         *length = sizeof(fg.cell[0].r1_dis);                                  } break;
            case 0xA5: { data_point = (void *)fg.cell[model_cell].tau1_dis;                                       *length = sizeof(fg.cell[0].tau1_dis);                                } break;
            case 0xA6: { data_point = (void *)fg.cell[model_cell].r2_dis;                                         *length = sizeof(fg.cell[0].r2_dis);                                  } break;
            case 0xA7: { data_point = (void *)fg.cell[model_cell].tau2_dis;                                       *length = sizeof(fg.cell[0].tau2_dis);                                } break;
            case 0xA8: { data_point = (void *)fg.cell[model_cell].r0_chg;                                         *length = sizeof(fg.cell[0].r0_chg);                                  } break;
            case 0xA9: { data_point = (void *)fg.cell[model_cell].r1_chg;                                         *length = sizeof(fg.cell[0].r1_chg);                                  } break;
            case 0xAA: { data_point = (void *)fg.cell[model_cell].tau1_chg;                                       *length = sizeof(fg.cell[0].tau1_chg);                                } break;
            case 0xAB: { data_point = (void *)fg.cell[model_cell].r2_chg;                                         *length = sizeof(fg.cell[0].r2_chg);                                  } break;
            case 0xAC: { data_point = (void *)fg.cell[model_cell].tau2_chg;                                       *length = sizeof(fg.cell[0].tau2_chg);                                } break;
            case 0xAD: { data_point = (void *)fg.q_nom_temp_c;                                   *length = sizeof(fg.q_nom_temp_c);                            } break;
            case 0xAE: { data_point = (void *)fg.q_nom_temp_ah;                                  *length = sizeof(fg.q_nom_temp_ah);                           } break;
            case 0xAF: { data_point = (void *)&fg.q_nom_ah;                                      *length = sizeof(fg.q_nom_ah);                                } break;
            case 0xB0: { data_point = (void *)&fg.coulombic_efficiency;                          *length = sizeof(fg.coulombic_efficiency);                    } break;
            case 0xB6: { data_point = (void *)&fg.ea_r0;                                         *length = sizeof(fg.ea_r0);                                   } break;
            case 0xB7: { data_point = (void *)&fg.ea_r1;                                         *length = sizeof(fg.ea_r1);                                   } break;
            case 0xB8: { data_point = (void *)&fg.ea_tau1;                                       *length = sizeof(fg.ea_tau1);                                 } break;
            case 0xB9: { data_point = (void *)&fg.ea_r2;                                         *length = sizeof(fg.ea_r2);                                   } break;
            case 0xBA: { data_point = (void *)&fg.ea_tau2;                                       *length = sizeof(fg.ea_tau2);                                 } break;
            case 0xBB: { data_point = (void *)&fg.kf_q_soc;                                      *length = sizeof(fg.kf_q_soc);                                } break;
            case 0xBC: { data_point = (void *)&fg.kf_q_rc1;                                      *length = sizeof(fg.kf_q_rc1);                                } break;
            case 0xBD: { data_point = (void *)&fg.kf_q_rc2;                                      *length = sizeof(fg.kf_q_rc2);                                } break;
            case 0xBE: { data_point = (void *)&fg.kf_r_v;                                        *length = sizeof(fg.kf_r_v);                                  } break;
            case 0xD0: { data_point = (void *)fg.cell_soc_f;                                     *length = sizeof(fg.cell_soc_f);                               ro = true; } break;
            case 0xD1: { data_point = (void *)fg.cell_vrc1;                                      *length = sizeof(fg.cell_vrc1);                                ro = true; } break;
            case 0xD2: { data_point = (void *)fg.cell_vrc2;                                      *length = sizeof(fg.cell_vrc2);                                ro = true; } break;
            case 0xD3: { data_point = (void *)fg.cell_p;                                         *length = sizeof(fg.cell_p);                                   ro = true; } break;
            case 0xD4: { data_point = (void *)&fg.cell[model_cell].capacity_ah;                                  *length = sizeof(fg.cell[0].capacity_ah);                                       } break;
            case 0xD6: { data_point = (void *)fg.cell_qmax;                                      *length = sizeof(fg.cell_qmax);                                ro = true; } break;
            case 0xD7: { data_point = (void *)&fg.cycle_count;                                   *length = sizeof(fg.cycle_count);                              ro = true; } break;
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
        if(is_config && address == 0x30 && (raw_data[0] < BD_CELL_COUNT_MIN || raw_data[0] > BD_CELL_COUNT_MAX))
        {
            return CE_WRONG_CMD;
        }

        // The model registers act on one cell, picked with register 0x9F
        if(address == 0x9F && raw_data[0] >= BD_CELL_COUNT_MAX)
        {
            return CE_WRONG_CMD;
        }

        // The on/off settings (0x31 to 0x36) are a single byte, 0 or 1
        if(is_config && address >= 0x31 && address <= 0x36 && raw_data[0] > 1)
        {
            return CE_WRONG_CMD;
        }

        memcpy(data_point, raw_data, *length);

        // The fuel gauge owns its registers, so a write has to be pushed back
        // rather than left in this module's copy. Safe to write the whole
        // struct: it was refreshed immediately above, and the estimator runs
        // from Ctrl_Run() in the same superloop, so it cannot have advanced
        // in between.
        if(is_fuel_gauge)
        {
            // A NaN or inf float is refused, the fuel gauge keeps its old values
            if(!FuelGauge_SetData(&fg))
            {
                return CE_WRONG_CMD;
            }
        }

        // Same for the config: hand the changed settings to the control module
        if(is_config)
        {
            // A NaN or inf calibration float is refused, the control module keeps its old config
            if(!Ctrl_SetConfig(&cfg))
            {
                return CE_WRONG_CMD;
            }
        }
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
    erase_init.Page        = APP_FLAG_FLASH_PAGE;
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
    CommErrorType_t     res;

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
        cmd != CC_REBOOT &&
        cmd != CC_PING)
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

    // A good frame came from the host
    Ctrl_SetLastCommunication();

    // -------------------------------------------------------
    // Execute
    // -------------------------------------------------------
    if(cmd == CC_WRITE)
    {
        res = Data_ReadWriteRegister(address, &rx_buf[3], &length, true);
    }
    else if(cmd == CC_READ)
    {
        // The peripheral data is copied with interrupts masked inside Periph_GetData / Periph_GetCalibration
        res = Data_ReadWriteRegister(address, &tx_buf[3], &length, false);
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
        res = Sim_Sample(&rx_buf[3], length, &tx_buf[3], &length);
    }
#endif
    else
    {
        res = Controls_Set(cmd, &rx_buf[3], length);
    }

    SendReply(res, cmd, address, length);

    // Reset only after the ACK has been handed to the UART
    if(cmd == CC_REBOOT && res == CE_OK)
    {
        HAL_Delay(100);
        NVIC_SystemReset();
    }
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
    uint32_t start = HAL_GetTick();

    // Wait for previous send to finish. The longest reply takes about 8 ms at 230400 baud,
    // so a send still running after UART_TX_TIMEOUT_MS is stuck: abort it instead of
    // blocking the main loop for good.
    while(uart_cmd.tx_wait)
    {
        if((HAL_GetTick() - start) > UART_TX_TIMEOUT_MS)
        {
            HAL_UART_AbortTransmit(&huart1);
            uart_cmd.tx_wait = false;
            Error_Set(OPENBMS_UART_TX_TIMEOUT);
        }
    }

    // Raise the flag and send new data
    uart_cmd.tx_wait = true;

    // If the transfer could not be started there will be no TX complete
    // interrupt to clear the flag, and the next send would wait for it forever
    if(HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data, length) != HAL_OK)
    {
        uart_cmd.tx_wait = false;
    }
}
static void SendError(CommErrorType_t err)
{
    // static: the DMA is still reading this buffer after the function has returned
    static uint8_t buff[5];

    buff[0] = CC_ERROR;
    buff[1] = 0;    // Address
    buff[2] = 1;    // Lenght
    buff[3] = err; // Error type
    buff[4] = Checksum(buff, 4);

    SendResponse(buff, sizeof(buff));
}
static void SendReply(CommErrorType_t res, uint8_t cmd, uint8_t address, uint8_t length)
{
    uint8_t *tx_buf = uart_cmd.tx_buffer;

    if(res != CE_OK)
    {
        SendError(res);
        return;
    }

    if(cmd == CC_READ || cmd == CC_SIM_SAMPLE)
    {
        // Reply with data: the payload is already in tx_buf[3...]
        tx_buf[0] = cmd;
        tx_buf[1] = address;
        tx_buf[2] = length;
        tx_buf[length + 3] = Checksum(tx_buf, length + 3);
        SendResponse(tx_buf, length + 4);
    }
    else
    {
        tx_buf[0] = CC_ACK;
        tx_buf[1] = 0;
        tx_buf[2] = 0;
        tx_buf[3] = Checksum(tx_buf, 3);
        SendResponse(tx_buf, 4);
    }
}
#ifdef OPENBMS_SIMULATION
static CommErrorType_t Sim_Sample(const uint8_t *data, uint8_t length, uint8_t *reply, uint8_t *reply_length)
{
    float    cell_mv[BD_CELL_COUNT_MAX];
    float    current_ma;
    uint16_t dt_ms;
    uint8_t  n = 0;

    if(length != SIM_SAMPLE_LENGTH)
    {
        return CE_WRONG_CMD;
    }

    // Only a configured device runs the control loop
    Ctrl_GetSystem(&sys);
    if(sys.ctrl_state != BD_CTRL_STATE_NORMAL_RUN && sys.ctrl_state != BD_CTRL_STATE_HPPC_RUN)
    {
        return CE_STATE;
    }

    memcpy(cell_mv, &data[0], sizeof(cell_mv));
    memcpy(&current_ma, &data[sizeof(cell_mv)], sizeof(current_ma));
    memcpy(&dt_ms, &data[sizeof(cell_mv) + sizeof(current_ma)], sizeof(dt_ms));

    // A NaN would stay in the fuel gauge and the extraction for good
    if(!isfinite(current_ma))
    {
        return CE_WRONG_CMD;
    }

    for(uint8_t i = 0; i < BD_CELL_COUNT_MAX; i++)
    {
        if(!isfinite(cell_mv[i]))
        {
            return CE_WRONG_CMD;
        }
    }

    // One sample, one pass of the control loop
    Ctrl_SimSample(cell_mv, current_ma, dt_ms);

    // The reply tells the host what the firmware did with it
    Ctrl_GetSystem(&sys);
    Periph_GetData(&pd);

    memcpy(&reply[n], &pd.fet_status, sizeof(pd.fet_status));                     n += sizeof(pd.fet_status);
    memcpy(&reply[n], &sys.ctrl_state, sizeof(sys.ctrl_state));                   n += sizeof(sys.ctrl_state);
    memcpy(&reply[n], &sys.hppc_status, sizeof(sys.hppc_status));                 n += sizeof(sys.hppc_status);
    memcpy(&reply[n], &sys.hppc_extract_status, sizeof(sys.hppc_extract_status)); n += sizeof(sys.hppc_extract_status);
    memcpy(&reply[n], &sys.protection_trigger, sizeof(sys.protection_trigger));   n += sizeof(sys.protection_trigger);
    memcpy(&reply[n], &sys.error_flags, sizeof(sys.error_flags));                 n += sizeof(sys.error_flags);

    *reply_length = n;
    return CE_OK;
}
#endif
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
  }

  // Safety net: reception must always be running. If an error stopped it and the
  // re-arm in the error callback failed, start it again here.
  if(debug_uart->RxState == HAL_UART_STATE_READY)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(debug_uart, uart_cmd.rx_buffer, UART_RX_BUFFER_SIZE);
  }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    UNUSED(size);

    if(huart->Instance == USART1)
    {
        uart_cmd.frame_ready     = true;
        HAL_UARTEx_ReceiveToIdle_DMA(debug_uart, uart_cmd.rx_buffer, UART_RX_BUFFER_SIZE);
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
      HAL_UARTEx_ReceiveToIdle_DMA(debug_uart, uart_cmd.rx_buffer, UART_RX_BUFFER_SIZE);
    }
  }
}