/**
  ******************************************************************************
  * @file           : openbms_ctrl.c
  * @brief          : OpenBMS Control Module
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

#include "openbms_ctrl.h"
#include "openbms_periph.h"
#include "openbms_error.h"
#include "openbms_fuelgauge.h"
#include "openbms_hppc.h"
#include "openbms_hppc_extract.h"
#include "openbms_balance.h"
#include "openbms_eeprom.h"
#include <stddef.h>
#include <string.h>
#include <math.h>

#ifdef OPENBMS_TEST
static uint32_t ctrl_test_loop_time_us[2] = {0, 0};   // [0] last pass, [1] longest pass since boot
#endif

#define CTRL_PROT_CHARGE_MIN_MA          (50.0f)

#define CTRL_BATT_CURRENT_MA             (50.0f)         // charge / discharge current above this counts as charging / discharging
#define CTRL_BATT_FULL_MS                (10000UL)       // full condition held this long sets FULL
#define CTRL_BATT_EMPTY_MS               (2000UL)        // a cell at or below voltage_cell_min this long sets EMPTY (load peaks stay shorter)
#define CTRL_BATT_CLEAR_MS               (2000UL)        // current in the other direction this long clears the flag

#define CTRL_UVP_SHUTDOWN_MS             (300000UL)

static Config_Data_t    config_data             = {0};
static System_Data_t    system_data             = {0};
static Stats_Data_t     stats_data              = {0};
static FuelGauge_Data_t fuelgauge_data          = {0};
static bool             protection_reset        = false;
static bool             shutdown_request        = false;  // CC_SHUTDOWN, Ctrl_Run enters SHUTDOWN
static bool             reset_request           = false;  // CC_REBOOT / CC_BOOTLOADER, Ctrl_Run enters RESET
static bool             errors_cleared          = false;  // CC_CLEAR_ERRORS, Ctrl_Run leaves FAULT for STARTUP
static bool             hppc_model_saved        = false;  // the model of this HPPC test has been handed to the EEPROM
static bool             reset_bootloader        = false;
static uint32_t         reset_tick              = 0;    // HAL_GetTick when RESET was entered
static uint32_t         startup_tick            = 0;    // HAL_GetTick when the cell measuring went on at boot

static void Ctrl_FaultSnapshot(void);
static void Ctrl_BatteryStatus(void);
static void Ctrl_ChargerStatus(void);
static void Ctrl_UvpShutdown(void);
static bool Ctrl_CellsPresent(void);

#define CTRL_RESET_DELAY_MS     (100U)

#define CTRL_CELL_PRESENT_MV    (1000.0f)
#define CTRL_STARTUP_TIMEOUT_MS (1000U)

// True once any of the values has been past the threshold for longer than timeout_ms.
// over = true: past means above the threshold, over = false: below it. A timeout of 0 disables the check.
static bool Ctrl_ProtectionTriggered(const float *value, uint8_t value_count, float threshold, bool over,
                                 uint32_t timeout_ms, uint32_t *timer, bool *triggered)
{
  bool threshold_exceeded = false;

  if(timeout_ms > 0)
  {
    for(uint8_t i = 0; i < value_count; i++)
    {
      if(over)
      {
        if(value[i] > threshold) threshold_exceeded = true;
      }
      else
      {
        if(value[i] < threshold) threshold_exceeded = true;
      }

      if(threshold_exceeded)
      {
        break;
      }
    }

    if(threshold_exceeded)
    {
      if(!(*triggered))
      {
        *timer = HAL_GetTick();
        *triggered = true;
      }
      else 
      {
        if((HAL_GetTick() - *timer) > timeout_ms)
        {
          return true;
        }
      }
    }
    else 
    {
      *triggered = false;
    }
  }

  return false;
}
static void Ctrl_ProtectionCheck(void)
{
  static uint32_t ovp_slow_timer        = 0;
  static uint32_t ovp_fast_timer        = 0;
  static uint32_t uvp_slow_timer        = 0;
  static uint32_t uvp_fast_timer        = 0;
  static uint32_t ocp_chg_timer         = 0;
  static uint32_t ocp_dchg_slow_timer   = 0;
  static uint32_t ocp_dchg_fast_timer   = 0;
  static uint32_t otp_timer             = 0;
  static uint32_t utp_timer             = 0;
  static uint32_t otp_chg_timer         = 0;
  static uint32_t utp_chg_timer         = 0;

  static bool ovp_slow_triggered        = false;
  static bool ovp_fast_triggered        = false;
  static bool uvp_slow_triggered        = false;
  static bool uvp_fast_triggered        = false;
  static bool ocp_chg_triggered         = false;
  static bool ocp_dchg_slow_triggered   = false;
  static bool ocp_dchg_fast_triggered   = false;
  static bool otp_triggered             = false;
  static bool utp_triggered             = false;
  static bool otp_chg_triggered         = false;
  static bool utp_chg_triggered         = false;

  Peripheral_Data_t pd;
  Periph_GetData(&pd);

  if(protection_reset)
  {
    ovp_slow_triggered        = false;
    ovp_fast_triggered        = false;
    uvp_slow_triggered        = false;
    uvp_fast_triggered        = false;
    ocp_chg_triggered         = false;
    ocp_dchg_slow_triggered   = false;
    ocp_dchg_fast_triggered   = false;
    otp_triggered             = false;
    utp_triggered             = false;
    otp_chg_triggered         = false;
    utp_chg_triggered         = false;
    protection_reset          = false;
  }

  uint8_t cell_count = config_data.cell_count;
  if(cell_count > SYS_CELL_COUNT_MAX)
  {
    cell_count = SYS_CELL_COUNT_MAX;
  }

  if(config_data.volt_protection)
  {
    if(Ctrl_ProtectionTriggered(pd.cell_voltage_live, cell_count, config_data.ovp_slow_threshold_mv, true,
                                config_data.ovp_slow_time_ms, &ovp_slow_timer,
                                &ovp_slow_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_OVP_SLOW;
    }

    if(Ctrl_ProtectionTriggered(pd.cell_voltage_live, cell_count, config_data.ovp_fast_threshold_mv, true,
                                config_data.ovp_fast_time_ms, &ovp_fast_timer,
                                &ovp_fast_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_OVP_FAST;
    }

    if(Ctrl_ProtectionTriggered(pd.cell_voltage_live, cell_count, config_data.uvp_slow_threshold_mv, false,
                                config_data.uvp_slow_time_ms, &uvp_slow_timer,
                                &uvp_slow_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_UVP_SLOW;
    }

    if(Ctrl_ProtectionTriggered(pd.cell_voltage_live, cell_count, config_data.uvp_fast_threshold_mv, false,
                                config_data.uvp_fast_time_ms, &uvp_fast_timer,
                                &uvp_fast_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_UVP_FAST;
    }
                              
  }
  else
  {
    ovp_slow_triggered        = false;
    ovp_fast_triggered        = false;
    uvp_slow_triggered        = false;
    uvp_fast_triggered        = false;
  }

  if(config_data.current_protection)
  {
    float c = pd.pack_current;
    float d = -pd.pack_current;

    if(Ctrl_ProtectionTriggered(&c, 1, config_data.ocp_charge_threshold_ma, true,
                                config_data.ocp_charge_time_ms, &ocp_chg_timer,
                                &ocp_chg_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_OCP_CHARGE;
    }

    if(Ctrl_ProtectionTriggered(&d, 1, config_data.ocp_discharge_slow_threshold_ma, true,
                                config_data.ocp_discharge_slow_time_ms, &ocp_dchg_slow_timer,
                                &ocp_dchg_slow_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_OCP_DISCHARGE_SLOW;
    }

    if(Ctrl_ProtectionTriggered(&d, 1, config_data.ocp_discharge_fast_threshold_ma, true,
                                config_data.ocp_discharge_fast_time_ms, &ocp_dchg_fast_timer,
                                &ocp_dchg_fast_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_OCP_DISCHARGE_FAST;
    }
  }
  else
  {
    ocp_chg_triggered         = false;
    ocp_dchg_slow_triggered   = false;
    ocp_dchg_fast_triggered   = false;
  }

  if(config_data.temp_protection && !Error_IsSet(OPENBMS_NTC_FAULT))
  {
    float t = pd.temperature_package;

    if(Ctrl_ProtectionTriggered(&t, 1, config_data.otp_threshold_c, true,
                                config_data.otp_time_ms, &otp_timer,
                                &otp_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_OTP;
    }

    if(Ctrl_ProtectionTriggered(&t, 1, config_data.utp_threshold_c, false,
                                config_data.utp_time_ms, &utp_timer,
                                &utp_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_UTP;
    }

    if(pd.pack_current > CTRL_PROT_CHARGE_MIN_MA)
    {
      if(Ctrl_ProtectionTriggered(&t, 1, config_data.otp_charge_threshold_c, true,
                                  config_data.otp_time_ms, &otp_chg_timer,
                                  &otp_chg_triggered))
      {
        system_data.protection_trigger |= CTRL_PROT_OTP_CHARGE;
      }

      if(Ctrl_ProtectionTriggered(&t, 1, config_data.utp_charge_threshold_c, false,
                                  config_data.utp_time_ms, &utp_chg_timer,
                                  &utp_chg_triggered))
      {
        system_data.protection_trigger |= CTRL_PROT_UTP_CHARGE;
      }
    }
    else
    {
      otp_chg_triggered       = false;
      utp_chg_triggered       = false;
    }
  }
  else
  {
    otp_triggered             = false;
    utp_triggered             = false;
    otp_chg_triggered         = false;
    utp_chg_triggered         = false;
  }
}
static bool Ctrl_CellsPresent(void)
{
  Peripheral_Data_t pd;
  uint8_t           cell_count = config_data.cell_count;

  if(!EEPROM_ConfigLoaded())
  {
    return true;
  }
  if(cell_count > SYS_CELL_COUNT_MAX)
  {
    cell_count = SYS_CELL_COUNT_MAX;
  }

  Periph_GetData(&pd);
  for(uint8_t i = 0; i < cell_count; i++)
  {
    if(pd.cell_voltage_live[i] < CTRL_CELL_PRESENT_MV)
    {
      return false;
    }
  }
  return true;
}
void Ctrl_Init(void)
{
  Periph_Init();
  FuelGauge_Init(&fuelgauge_data, &config_data);
  Balance_Init(&config_data);

  EEPROM_Init(&config_data, &fuelgauge_data);
  system_data.ctrl_state = CTRL_STATE_STARTUP;
  if(EEPROM_ConfigLoaded())
  {
    Periph_SetCalibration(&config_data);
  }

  Periph_Wake();
  startup_tick = HAL_GetTick();

#ifdef OPENBMS_TEST
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT       = 0;
  DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}
void Ctrl_Run(void)
{
  uint8_t         hppc_direction;
  uint16_t        hppc_status;
  uint16_t        hppc_extract_status;
  bool            run_fet;
  bool            run_pre_fet;
  bool            run_balance;
  bool            hppc_fet = false;
#ifdef OPENBMS_TEST
  uint32_t        loop_start_cycles = DWT->CYCCNT;
#endif

  system_data.error_flags = Error_GetFlags();

  system_data.session_uptime_ms = HAL_GetTick();


  switch(system_data.ctrl_state)
  {
    case CTRL_STATE_STARTUP:
    {
      if(reset_request)
      {
        reset_tick             = HAL_GetTick();
        system_data.ctrl_state = CTRL_STATE_RESET;
      }
      else if((Error_GetFlags() & ERROR_FAULT_FLAGS) != 0U)
      {
        system_data.ctrl_state = CTRL_STATE_FAULT;
      }
      else if(Periph_CellMeasureReady())
      {
        if(Ctrl_CellsPresent())
        {
          system_data.ctrl_state = CTRL_STATE_NORMAL_RUN;
        }
        else
        {
          Error_Set(OPENBMS_CELL_MISSING);
          system_data.ctrl_state = CTRL_STATE_FAULT;
        }
      }
      else if((HAL_GetTick() - startup_tick) >= CTRL_STARTUP_TIMEOUT_MS)
      {
        Error_Set(OPENBMS_ADS131M08_DATA_LOST);
        system_data.ctrl_state = CTRL_STATE_FAULT;
      }
    }
    break;

    case CTRL_STATE_FAULT:
    {
      system_data.fet_request     = false;
      system_data.pre_fet_request = false;
      if(reset_request)
      {
        reset_tick             = HAL_GetTick();
        system_data.ctrl_state = CTRL_STATE_RESET;
      }
      else if(shutdown_request)
      {
        system_data.ctrl_state = CTRL_STATE_SHUTDOWN;
      }
      else if(errors_cleared)
      {
        system_data.main_control = CTRL_MODE_NORMAL;
        startup_tick             = HAL_GetTick();
        system_data.ctrl_state   = CTRL_STATE_STARTUP;
      }
      errors_cleared = false;
    }
    break;

    case CTRL_STATE_NORMAL_RUN:
    {
      if(reset_request)
      {
        reset_tick             = HAL_GetTick();
        system_data.ctrl_state = CTRL_STATE_RESET;
      }
      else if(shutdown_request)
      {
        system_data.ctrl_state = CTRL_STATE_SHUTDOWN;
      }
      else if((Error_GetFlags() & ERROR_FAULT_FLAGS) != 0U)
      {
        system_data.fet_request     = false;
        system_data.pre_fet_request = false;
        system_data.ctrl_state      = CTRL_STATE_FAULT;
      }
      else
      {
#ifdef OPENBMS_TEST
        if(EEPROM_FuelGaugeLoaded() && !Periph_MeasTestOffOn())
#else
        if(EEPROM_FuelGaugeLoaded())
#endif
        {
          FuelGauge_Run();
        }

        if(system_data.main_control == CTRL_MODE_HPPC)
        {
          system_data.fet_request     = false;
          system_data.pre_fet_request = false;
          EEPROM_StopFuelGauge();
          system_data.ctrl_state      = CTRL_STATE_HPPC_RUN;
        }
      }
    }
    break;

    case CTRL_STATE_HPPC_RUN:
    {
      if(reset_request)
      {
        reset_tick             = HAL_GetTick();
        system_data.ctrl_state = CTRL_STATE_RESET;
        break;
      }
      if(shutdown_request)
      {
        if(system_data.main_control == CTRL_MODE_HPPC)
        {
          Hppc_Abort(HPPC_STOP_SHUTDOWN, &hppc_fet);
          hppc_fet = false;
        }
        Hppc_GetStatus(&hppc_direction, &hppc_status, &hppc_extract_status);
        system_data.hppc_direction      = hppc_direction;
        system_data.hppc_status         = hppc_status;
        system_data.hppc_extract_status = hppc_extract_status;
        system_data.ctrl_state          = CTRL_STATE_SHUTDOWN;
        break;
      }
      if((Error_GetFlags() & ERROR_FAULT_FLAGS) != 0U)
      {
        if(system_data.main_control == CTRL_MODE_HPPC)
        {
          Hppc_Abort(HPPC_STOP_ERRORS, &hppc_fet);
          hppc_fet = false;
        }
        Hppc_GetStatus(&hppc_direction, &hppc_status, &hppc_extract_status);
        system_data.hppc_direction      = hppc_direction;
        system_data.hppc_status         = hppc_status;
        system_data.hppc_extract_status = hppc_extract_status;
        system_data.ctrl_state          = CTRL_STATE_FAULT;
        break;
      }

      if(system_data.main_control == CTRL_MODE_NORMAL)
      {
        Hppc_Stop(&hppc_fet);
      }
      else if(system_data.protection_trigger != 0)
      {
        Hppc_Abort(HPPC_STOP_PROTECTION, &hppc_fet);
      }
      else
      {
        Hppc_Run(&hppc_fet);
      }

      Hppc_GetStatus(&hppc_direction, &hppc_status, &hppc_extract_status);
      system_data.hppc_direction      = hppc_direction;
      system_data.hppc_status         = hppc_status;

      if(!hppc_model_saved && (hppc_extract_status & HX_STATUS_DONE) != 0 && (hppc_extract_status & HX_STATUS_BUSY) == 0)
      {
        hppc_model_saved = EEPROM_SaveFuelGaugeModel();
      }
      if(hppc_model_saved)
      {
        if(EEPROM_SaveRunning())
        {
          hppc_extract_status |= HX_STATUS_BUSY;
        }
        else if(((EEPROM_GetStatus() & EEPROM_STATUS_SAVE_MASK) >> EEPROM_STATUS_SAVE_SHIFT) == EEPROM_SAVE_DONE)
        {
          hppc_extract_status |= HX_STATUS_SAVED;
        }
      }
      system_data.hppc_extract_status = hppc_extract_status;
    }
    break;

    case CTRL_STATE_SHUTDOWN:
    {
      system_data.fet_request     = false;
      system_data.pre_fet_request = false;
      Periph_Off();

      if(EEPROM_FinalSave())
      {
        Periph_PowerOff();
      }
    }
    break;

    case CTRL_STATE_RESET:
    {
      system_data.fet_request     = false;
      system_data.pre_fet_request = false;
      Periph_Off();

      if(EEPROM_FinalSave() && (HAL_GetTick() - reset_tick) >= CTRL_RESET_DELAY_MS)
      {
        if(reset_bootloader)
        {
          Periph_RemoveAppFlag();
        }
        NVIC_SystemReset();
      }
    }
    break;

    default: break;
  }

  EEPROM_Run();
  system_data.eeprom_status = EEPROM_GetStatus();

  if(EEPROM_ConfigLoaded() && Periph_CellMeasureReady())
  {
    Ctrl_ProtectionCheck();
    Ctrl_FaultSnapshot();
    Ctrl_BatteryStatus();
    Ctrl_UvpShutdown();
  }
  Ctrl_ChargerStatus();

  if(system_data.protection_trigger != 0)
  {
    system_data.fet_request     = false;
    system_data.pre_fet_request = false;
  }

  run_fet = EEPROM_ConfigLoaded() && (system_data.protection_trigger == 0) && (hppc_fet | system_data.fet_request);

  run_pre_fet = EEPROM_ConfigLoaded() && (system_data.protection_trigger == 0) && system_data.pre_fet_request;

  Periph_SetFET(run_fet);
  Periph_SetPreFET(run_pre_fet);

  run_balance = EEPROM_ConfigLoaded() && (system_data.ctrl_state == CTRL_STATE_NORMAL_RUN) && (system_data.protection_trigger == 0);
  if(EEPROM_FuelGaugeLoaded())
  {
    Periph_SetBalance(Balance_Run(run_balance, FuelGauge_GetBalanceTarget()));
  }
  else
  {
    Periph_SetBalance(Balance_Run(run_balance, 0));
  }

  Periph_Run();

#ifdef OPENBMS_TEST
  ctrl_test_loop_time_us[0] = (DWT->CYCCNT - loop_start_cycles) / (SystemCoreClock / 1000000U);
  if(ctrl_test_loop_time_us[0] > ctrl_test_loop_time_us[1])
  {
    ctrl_test_loop_time_us[1] = ctrl_test_loop_time_us[0];
  }
#endif
}
#ifdef OPENBMS_TEST
const uint32_t *Ctrl_TestLoopTime(void)
{
  return ctrl_test_loop_time_us;
}
#endif
const Config_Data_t *Ctrl_GetConfig(void)
{
  return &config_data;
}
const System_Data_t *Ctrl_GetSystem(void)
{
  return &system_data;
}
const Stats_Data_t *Ctrl_GetStats(void)
{
  return &stats_data;
}
const FuelGauge_Data_t *Ctrl_GetFuelGauge(void)
{
  return &fuelgauge_data;
}
// Writes one field of a struct owned by this module, length is the size of that field.
// Floats between block_start and block_end must be real numbers.
static bool Ctrl_WriteField(void *base, uint32_t size, uint32_t block_start, uint32_t block_end,
                            const void *field, const uint8_t *data, uint8_t length)
{
  uint32_t offset = (uint32_t)((const uint8_t *)field - (const uint8_t *)base);
  float    value;

  if(offset + length > size)
  {
    return false;
  }

  if(offset >= block_start && offset < block_end)
  {
    for(uint32_t i = 0; i + sizeof(float) <= length; i += sizeof(float))
    {
      memcpy(&value, data + i, sizeof(float));
      if(!isfinite(value))
      {
        return false;
      }
    }
  }

  memcpy((uint8_t *)base + offset, data, length);
  return true;
}
bool Ctrl_WriteConfig(const void *field, const uint8_t *data, uint8_t length)
{
  return Ctrl_WriteField(&config_data, sizeof(Config_Data_t),
                         offsetof(Config_Data_t, current_sensor_gain), offsetof(Config_Data_t, device_chemistry),
                         field, data, length);
}
bool Ctrl_WriteFuelGauge(const void *field, const uint8_t *data, uint8_t length)
{
  if(field == &fuelgauge_data.kf_r_v)
  {
    float value;

    if(length != sizeof(float))
    {
      return false;
    }
    memcpy(&value, data, sizeof(float));
    if(!(value > 0.0f))
    {
      return false;
    }
  }

  return Ctrl_WriteField(&fuelgauge_data, sizeof(FuelGauge_Data_t),
                         offsetof(FuelGauge_Data_t, soc_grid), offsetof(FuelGauge_Data_t, cycle_count),
                         field, data, length);
}
void Ctrl_SetMode(uint16_t mode)
{
  system_data.main_control = mode;
}
bool Ctrl_StartHppc(uint8_t direction)
{
  if(system_data.ctrl_state == CTRL_STATE_HPPC_RUN)
  {
    if(system_data.main_control == CTRL_MODE_HPPC ||
       (HppcX_GetStatus() & HX_STATUS_BUSY) != 0 ||
       ((HppcX_GetStatus() & HX_STATUS_DONE) != 0 && !hppc_model_saved))
    {
      return false;
    }
    hppc_model_saved = false;
  }

  FuelGauge_SetDefaultsIfEmpty();

  Hppc_SetDirection(direction);
  system_data.main_control = CTRL_MODE_HPPC;
  return true;
}
// True once cond has been true for hold_ms without a break; *start is the tick it became true
static bool Ctrl_HeldFor(bool cond, uint32_t hold_ms, uint32_t *start, bool *active)
{
  if(!cond)
  {
    *active = false;
    return false;
  }
  if(!(*active))
  {
    *start  = HAL_GetTick();
    *active = true;
  }
  if((HAL_GetTick() - *start) >= hold_ms)
  {
    return true;
  }
  return false;
}
static void Ctrl_BatteryStatus(void)
{
  static uint32_t full_start         = 0;
  static uint32_t empty_start        = 0;
  static uint32_t full_clear_start   = 0;
  static uint32_t empty_clear_start  = 0;
  static bool     full_active        = false;
  static bool     empty_active       = false;
  static bool     full_clear_active  = false;
  static bool     empty_clear_active = false;
  Peripheral_Data_t pd;
  float           top_mv;
  float           bottom_mv;
  bool            full;
  uint8_t         cell_count = config_data.cell_count;

  if(system_data.ctrl_state != CTRL_STATE_NORMAL_RUN && system_data.ctrl_state != CTRL_STATE_HPPC_RUN)
  {
    return;
  }
  if(cell_count > SYS_CELL_COUNT_MAX)
  {
    cell_count = SYS_CELL_COUNT_MAX;
  }
  if(cell_count == 0)
  {
    return;
  }

  Periph_GetData(&pd);
  top_mv    = pd.cell_voltage[0];
  bottom_mv = pd.cell_voltage[0];
  for(uint8_t i = 1; i < cell_count; i++)
  {
    if(pd.cell_voltage[i] > top_mv)
    {
      top_mv = pd.cell_voltage[i];
    }
    if(pd.cell_voltage[i] < bottom_mv)
    {
      bottom_mv = pd.cell_voltage[i];
    }
  }

  full = (pd.pack_current > CTRL_BATT_CURRENT_MA) &&
         (top_mv >= (float)config_data.charge_monitor_cell_mv) &&
         (pd.pack_current <= (float)config_data.charging_term_current);
  if(Ctrl_HeldFor(full, CTRL_BATT_FULL_MS, &full_start, &full_active))
  {
    system_data.battery_status |= CTRL_BATT_FULL;
  }
  if(Ctrl_HeldFor(pd.pack_current < -CTRL_BATT_CURRENT_MA, CTRL_BATT_CLEAR_MS, &full_clear_start, &full_clear_active))
  {
    system_data.battery_status &= (uint8_t)~CTRL_BATT_FULL;
  }

  if(Ctrl_HeldFor(bottom_mv <= (float)config_data.voltage_cell_min, CTRL_BATT_EMPTY_MS, &empty_start, &empty_active))
  {
    system_data.battery_status |= CTRL_BATT_EMPTY;
  }
  if(Ctrl_HeldFor(pd.pack_current > CTRL_BATT_CURRENT_MA, CTRL_BATT_CLEAR_MS, &empty_clear_start, &empty_clear_active))
  {
    system_data.battery_status &= (uint8_t)~CTRL_BATT_EMPTY;
  }
}
static void Ctrl_ChargerStatus(void)
{
  Peripheral_Data_t pd;

  Periph_GetData(&pd);
  if(pd.sys_vcc_mv > CTRL_CHARGER_PRESENT_MV)
  {
    system_data.charger_present = 1U;
  }
  else
  {
    system_data.charger_present = 0U;
  }
}
static void Ctrl_UvpShutdown(void)
{
  static uint32_t   low_start  = 0;
  static bool       low_active = false;
  Peripheral_Data_t pd;
  float             bottom_mv;
  bool              empty;
  uint8_t           cell_count = config_data.cell_count;

  if(cell_count > SYS_CELL_COUNT_MAX)
  {
    cell_count = SYS_CELL_COUNT_MAX;
  }
  if(cell_count == 0)
  {
    return;
  }

  Periph_GetData(&pd);
  bottom_mv = pd.cell_voltage[0];
  for(uint8_t i = 1; i < cell_count; i++)
  {
    if(pd.cell_voltage[i] < bottom_mv)
    {
      bottom_mv = pd.cell_voltage[i];
    }
  }

  empty = ((system_data.protection_trigger & (CTRL_PROT_UVP_SLOW | CTRL_PROT_UVP_FAST)) != 0U) &&
          (system_data.charger_present == 0U) &&
          (bottom_mv <= (float)config_data.voltage_cell_min);
  if(Ctrl_HeldFor(empty, CTRL_UVP_SHUTDOWN_MS, &low_start, &low_active))
  {
    shutdown_request = true;
  }
}
static void Ctrl_FaultSnapshot(void)
{
  if(system_data.protection_trigger == 0 || stats_data.fault_code != 0)
  {
    return;
  }

  Peripheral_Data_t pd;
  Periph_GetData(&pd);

  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    float mv = pd.cell_voltage[i] + 0.5f;

    if(mv < 0.0f)               mv = 0.0f;
    if(mv > (float)UINT16_MAX)  mv = (float)UINT16_MAX;
    stats_data.fault_snapshot_voltage[i] = (uint16_t)mv;
  }

  float ma = roundf(pd.pack_current);
  if(ma < (float)INT16_MIN) ma = (float)INT16_MIN;
  if(ma > (float)INT16_MAX) ma = (float)INT16_MAX;
  stats_data.fault_snapshot_current = (int16_t)ma;

  float t = roundf(pd.temperature_package);
  if(t < (float)INT8_MIN) t = (float)INT8_MIN;
  if(t > (float)INT8_MAX) t = (float)INT8_MAX;
  stats_data.fault_snapshot_temperature = (int8_t)t;

  stats_data.fault_snapshot_soc = fuelgauge_data.relative_soc;
  stats_data.fault_code         = system_data.protection_trigger;
  stats_data.fault_timestamp    = system_data.session_uptime_ms;
}
void Ctrl_ClearFault(void)
{
  memset(stats_data.fault_snapshot_voltage, 0, sizeof(stats_data.fault_snapshot_voltage));
  stats_data.fault_snapshot_current     = 0;
  stats_data.fault_snapshot_temperature = 0;
  stats_data.fault_snapshot_soc         = 0;
  stats_data.fault_code                 = 0;
  stats_data.fault_timestamp            = 0;
}
void Ctrl_ClearErrors(void)
{
  if(Error_IsSet(OPENBMS_HW_DRV_FAULT))
  {
    Periph_DriverReset();
  }
  Error_ClearAll();
  if(system_data.ctrl_state == CTRL_STATE_FAULT)
  {
    errors_cleared = true;
  }
}
void Ctrl_Shutdown(void)
{
  shutdown_request = true;
}
void Ctrl_RequestReset(bool bootloader)
{
  reset_request = true;
  if(bootloader)
  {
    reset_bootloader = true;
  }
}
void Ctrl_ResetProtection(void)
{
  system_data.protection_trigger = 0;
  protection_reset               = true;
}
void Ctrl_SetFetRequest(bool state)
{
  system_data.fet_request = state;
}
void Ctrl_SetPreFetRequest(bool state)
{
  system_data.pre_fet_request = state;
}
void Ctrl_SetLastCommunication(void)
{
  system_data.last_communication_timestamp = HAL_GetTick();
}
