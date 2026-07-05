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
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_pwr_ex.h"

#define BATTERY_CURRENT_IDLE_THRESHOLD_MA   20

typedef enum
{
  LS_START,
  LS_CHECK_STATE,

} LearningState_t;

static LearningState_t  learning_state      = LS_START;
static Control_Data_t   control_data        = {0};

static uint32_t         learning_timer      = 0;
static bool             protection_trigger  = false;
static uint8_t          cell_number         = CELL_NUMBER_DEFAULT;

static void Timer_Set(uint32_t *timer)
{
  *timer = HAL_GetTick();
}
static bool Timer_Expired(uint32_t *timer, uint32_t timeout_ms)
{
  uint32_t current_time = HAL_GetTick();
  if((current_time - *timer) >= timeout_ms)
  {
    return true;
  }
  else 
  {
    return false;
  }
}
static void Time_Update(void)
{
  static uint32_t tick_prev = 0;
  static uint32_t time_ms   = 0;

  uint32_t tick_now  = HAL_GetTick();
  uint32_t tick_diff = tick_now - tick_prev;   // correct even on rollover
  time_ms           += tick_diff;
  tick_prev          = tick_now;

  control_data.uptime_counter = time_ms;
}
static bool Protection_OverTriggered(float *value, uint8_t value_count, float threshold, uint32_t timeout_ms, uint32_t *timer, bool *triggered)
{
  uint32_t elapsed;
  bool     threshold_exceeded = false;

  if(timeout_ms > 0)
  {
    if(value_count == 1)
    {
      if(*value > threshold) threshold_exceeded = true;
    }
    else 
    {
      for(uint8_t i = 0; i < value_count; i++)
      {
        if(value[i] > threshold) 
        {
          threshold_exceeded = true;
          break;
        }
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
        elapsed = HAL_GetTick() - *timer;
        if(elapsed > timeout_ms)
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
static bool Protection_UnderTriggered(float *value, uint8_t value_count, float threshold, uint32_t timeout_ms, uint32_t *timer, bool *triggered)
{
  uint32_t elapsed;
  bool     threshold_exceeded = false;

  if(timeout_ms > 0)
  {
    if(value_count == 1)
    {
      if(*value < threshold) threshold_exceeded = true;
    }
    else 
    {
      for(uint8_t i = 0; i < value_count; i++)
      {
        if(value[i] < threshold) 
        {
          threshold_exceeded = true;
          break;
        }
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
        elapsed = HAL_GetTick() - *timer;
        if(elapsed > timeout_ms)
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
static void Protection_Check(void)
{
  // Check for overvoltage, undervoltage, overcurrent, and overtemperature conditions
  // If any protection condition is met, take appropriate action (e.g., disable FETs)

  static uint32_t ovp_slow_timer        = 0;
  static uint32_t ovp_fast_timer        = 0;
  static uint32_t uvp_slow_timer        = 0;
  static uint32_t uvp_fast_timer        = 0;
  static uint32_t ocp_chg_timer         = 0;
  static uint32_t ocp_dchg_slow_timer   = 0;
  static uint32_t ocp_dchg_fast_timer   = 0;
  static uint32_t otp_timer             = 0;

  static bool ovp_slow_triggered        = false;
  static bool ovp_fast_triggered        = false;
  static bool uvp_slow_triggered        = false;
  static bool uvp_fast_triggered        = false;
  static bool ocp_chg_triggered         = false;
  static bool ocp_dchg_slow_triggered   = false;
  static bool ocp_dchg_fast_triggered   = false;
  static bool otp_triggered             = false;

  Peripheral_Data_t pd;
  Periph_GetData(&pd);

  // Voltage protection enabled
  if(control_data.configuration & BD_CONFIG_VOLT_PROT)
  {
    protection_trigger |= Protection_OverTriggered(pd.cell_voltage_filtered, cell_number, control_data.ovp_slow_threshold_mv, 
                              control_data.ovp_slow_time_ms, &ovp_slow_timer, 
                              &ovp_slow_triggered);

    protection_trigger |= Protection_OverTriggered(pd.cell_voltage_filtered, cell_number, control_data.ovp_fast_threshold_mv, 
                              control_data.ovp_fast_time_ms, &ovp_fast_timer, 
                              &ovp_fast_triggered);

    protection_trigger |= Protection_UnderTriggered(pd.cell_voltage_filtered, cell_number, control_data.uvp_slow_threshold_mv, 
                              control_data.uvp_slow_time_ms, &uvp_slow_timer, 
                              &uvp_slow_triggered);

    protection_trigger |= Protection_UnderTriggered(pd.cell_voltage_filtered, cell_number, control_data.uvp_fast_threshold_mv, 
                              control_data.uvp_fast_time_ms, &uvp_fast_timer, 
                              &uvp_fast_triggered);
                              
  }

  // Current protection enabled
  if(control_data.configuration & BD_CONFIG_CURR_PROT)
  {
    float c = pd.pack_current_filtered;

    protection_trigger |= Protection_OverTriggered(&c, 1, control_data.ocp_charge_threshold_ma, 
                              control_data.ocp_charge_time_ms, &ocp_chg_timer, 
                              &ocp_chg_triggered);

    protection_trigger |= Protection_OverTriggered(&c, 1, control_data.ocp_discharge_slow_threshold_ma, 
                              control_data.ocp_discharge_slow_time_ms, &ocp_dchg_slow_timer, 
                              &ocp_dchg_slow_triggered);

    protection_trigger |= Protection_OverTriggered(&c, 1, control_data.ocp_discharge_fast_threshold_ma, 
                              control_data.ocp_discharge_fast_time_ms, &ocp_dchg_fast_timer, 
                              &ocp_dchg_fast_triggered);
  }

  // Temperature protection enabled
  if(control_data.configuration & BD_CONFIG_TEMP_PROT)
  {
    float t = pd.temperature_package;

    protection_trigger |= Protection_OverTriggered(&t, 1, control_data.otp_threshold_c, 
                              control_data.otp_time_ms, &otp_timer, 
                              &otp_triggered);    
  }
}
static void Run_Learning(void)
{
  switch(learning_state)
  {
    case LS_START:
    {
      // Disable FETs first
      Periph_SetFET(false);
      Periph_SetFET(false);

      Timer_Set(&learning_timer);
      learning_state = LS_CHECK_STATE;
    }
    break;

    case LS_CHECK_STATE:
    {
      if(Timer_Expired(&learning_timer, 1000))
      {
        
      }
    }
    break;
  }
}
static void Ctrl_SetDefaults(void)
{
  // Configure BMS
  control_data.configuration           = 7 & BD_CONFIG_CELL_COUNT_MASK;   // 7-cell battery pack
  control_data.configuration          |= BD_CONFIG_VOLT_PROT;             // Enable voltage protection
  control_data.configuration          |= BD_CONFIG_CURR_PROT;             // Enable current protection
  control_data.configuration          |= BD_CONFIG_TEMP_PROT;             // Enable temp protection

  // Configure mode
  control_data.main_control            = (BD_MAIN_CTR_MODE_NORMAL << BD_MAIN_CTR_MODE_SHIFT) & BD_MAIN_CTR_MODE_MASK;

  // Configure cell data
  control_data.cell_capacity           = 2500;     // 2.5Ah
  control_data.voltage_cell_max        = 4200;     // 4.2V
  control_data.voltage_cell_min        = 2500;     // 2.5V
  control_data.charging_term_current   = 100;      // 0.1A

  // Set protection thresholds
  control_data.ovp_slow_threshold_mv   = 4220;    // 4.22V
  control_data.ovp_slow_time_ms        = 20000;   // 20s

  control_data.ovp_fast_threshold_mv   = 4300;    // 4.30V
  control_data.ovp_fast_time_ms        = 500;     // 0.5s

  control_data.uvp_slow_threshold_mv   = 2700;    // 2.7V
  control_data.uvp_slow_time_ms        = 20000;   // 20s

  control_data.uvp_fast_threshold_mv   = 2500;    // 2.5V
  control_data.uvp_fast_time_ms        = 500;     // 0.5s

  control_data.ocp_charge_threshold_ma          = 4000;  // 4A
  control_data.ocp_charge_time_ms               = 10000; // 10s

  control_data.ocp_discharge_slow_threshold_ma  = 15000; // 15A
  control_data.ocp_discharge_slow_time_ms       = 10000; // 10s

  control_data.ocp_discharge_fast_threshold_ma  = 18000; // 18A
  control_data.ocp_discharge_fast_time_ms       = 1000;  // 1s

  control_data.otp_threshold_c       = 60;      // 60 degrees C
  control_data.otp_time_ms           = 60000;   // 1 minute

}
void Ctrl_Init(void)
{
  Ctrl_SetDefaults();
  Periph_Init();
  Periph_SetFET(true);
}
void Ctrl_Run(void)
{
  uint16_t mode = control_data.main_control & BD_MAIN_CTR_MODE_MASK;

  if(mode & BD_MAIN_CTR_MODE_LEARNING)
  {
    Run_Learning();
  }

  Periph_Run();
  Time_Update();
}
void Ctrl_GetData(Control_Data_t *data)
{
  memcpy(data, &control_data, sizeof(Control_Data_t));
}
void Ctrl_SetMode(uint16_t mode)
{
  control_data.main_control = (control_data.main_control & ~BD_MAIN_CTR_MODE_MASK) | (mode & BD_MAIN_CTR_MODE_MASK);
}
void Periph_50msTimer(void)
{
  Protection_Check();
}