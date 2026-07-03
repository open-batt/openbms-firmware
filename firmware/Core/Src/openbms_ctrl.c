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

#define BATTERY_CURRENT_IDLE_THRESHOLD_MA   20

typedef enum
{
  LS_START,
  LS_CHECK_STATE,

} LearningState_t;

static LearningState_t  learning_state    = LS_START;
static Control_Data_t   control_data      = {0};

static uint32_t         learning_timer    = 0;

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
static uint32_t Time_Update(void)
{
  static uint32_t tick_prev = 0;
  static uint32_t time_ms   = 0;

  uint32_t tick_now  = HAL_GetTick();
  uint32_t tick_diff = tick_now - tick_prev;   // correct even on rollover
  time_ms           += tick_diff;
  tick_prev          = tick_now;

  return time_ms;
}
static void Protection_Check(void)
{
  // Check for overvoltage, undervoltage, overcurrent, and overtemperature conditions
  // If any protection condition is met, take appropriate action (e.g., disable FETs)
  
  static uint32_t ovp_slow_timer = 0;
  static uint32_t ovp_fast_timer = 0;
  static uint32_t uvp_slow_timer = 0;
  static uint32_t uvp_fast_timer = 0;

  static bool ovp_slow_triggered = false;
  static bool ovp_fast_triggered = false;
  static bool uvp_slow_triggered = false;
  static bool uvp_fast_triggered = false;

  static uint32_t ocp_charge_timer = 0;
  static uint32_t temp_prot_timer = 0;

  Peripheral_Data_t pd;
  Periph_GetData(&pd);

  if(pd.pack_voltage_filtered > control_data.ovp_fast_threshold_mv)
  {
    // Overvoltage protection triggered
    Periph_SetFET(false);
  }
  else if (pd.pack_voltage < control_data.uvp_fast_threshold_mv)
  {
    // Undervoltage protection triggered
    Periph_SetFET(false);
  }
  else if (pd.pack_current > control_data.ocp_charge_threshold_ma)
  {
    // Overcurrent protection triggered
    Periph_SetFET(false);
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
void Ctrl_Init(void)
{
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
  

  control_data.uptime_counter = Time_Update();
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