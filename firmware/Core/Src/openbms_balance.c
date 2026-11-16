/**
  ******************************************************************************
  * @file           : openbms_balance.c
  * @brief          : OpenBMS passive cell balancing
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

#include "openbms_balance.h"
#include "openbms_periph.h"
#include "openbms_error.h"
#include <math.h>

#define BAL_CHARGE_MIN_MA       (50.0f)     // the pack counts as charging above this current, at rest within +-this
#define BAL_ON_MS               (5000U)     // bleeding time between two measuring pauses
#define BAL_PAUSE_MS            (500U)      // every balancer off: settle, then fresh cell readings

#if BAL_PAUSE_MS <= PER_BAL_SETTLE_MS
#error "BAL_PAUSE_MS must be longer than PER_BAL_SETTLE_MS, or the pause gives no fresh reading"
#endif

static const Config_Data_t *bal_config      = 0;
static uint8_t              bal_mask        = 0;        // cells chosen to bleed (kept through a pause)
static bool                 bal_pause       = false;    // measuring pause running
static uint32_t             bal_phase_tick  = 0;        // start of the current bleed phase or pause
static bool                 bal_soc_mode    = false;    // the current bleed phase is SOC balancing (else voltage balancing)

void Balance_Init(const Config_Data_t *config)
{
  bal_config = config;
  bal_mask     = 0;
  bal_pause    = false;
  bal_soc_mode = false;
}

uint8_t Balance_Run(bool allowed, const float *target_ah)
{
  Peripheral_Data_t pd;
  uint8_t           cell_count;
  float             min_mv;
  float             max_mv;
  float             start_mv;
  float             delta_mv;
  float             stop_mv;
  bool              current_ok;
  bool              soc_mode = false;
  uint8_t           soc_mask = 0;

  uint32_t          now = HAL_GetTick();

  if(bal_config == 0 || !allowed || !bal_config->balance_enable)
  {
    bal_mask  = 0;
    bal_pause = false;
    return bal_mask;
  }

  Periph_GetData(&pd);

  cell_count = bal_config->cell_count;
  if(cell_count > SYS_CELL_COUNT_MAX)
  {
    cell_count = SYS_CELL_COUNT_MAX;
  }

  if(pd.pack_current > BAL_CHARGE_MIN_MA)
  {
    current_ok = true;
  }
  else if(bal_config->balance_rest_enable && pd.pack_current >= -BAL_CHARGE_MIN_MA)
  {
    current_ok = true;
  }
  else
  {
    current_ok = false;
  }

  if(cell_count < SYS_CELL_COUNT_MIN ||
     !current_ok ||
     Error_IsSet(OPENBMS_NTC_FAULT) ||
     Error_IsSet(OPENBMS_ADS131M08_DATA_LOST) ||
     !isfinite(pd.temperature_package) ||
     pd.temperature_package >= (float)bal_config->balance_max_temp_c)
  {
    bal_mask  = 0;
    bal_pause = false;
    return bal_mask;
  }

  if(target_ah != 0)
  {
    for(uint8_t i = 0; i < cell_count; i++)
    {
      if(target_ah[i] > 0.0f)
      {
        soc_mask |= (uint8_t)(1U << i);
      }
    }
    if(soc_mask != 0)
    {
      soc_mode = true;
    }
  }

  if(soc_mode != bal_soc_mode)
  {
    bal_mask     = 0;
    bal_pause    = false;
    bal_soc_mode = soc_mode;
  }

  if(soc_mode)
  {
    bal_mask &= soc_mask;
  }

  if(bal_mask != 0)
  {
    if(bal_pause)
    {
      if((now - bal_phase_tick) < BAL_PAUSE_MS)
      {
        return 0;
      }
      bal_pause = false;
    }
    else
    {
      if((now - bal_phase_tick) < BAL_ON_MS)
      {
        return bal_mask;
      }
      bal_pause      = true;
      bal_phase_tick = now;
      return 0;
    }
  }

  if(soc_mode)
  {
    bal_mask       = soc_mask;
    bal_phase_tick = now;
    return bal_mask;
  }

  min_mv = pd.cell_voltage[0];
  max_mv = pd.cell_voltage[0];
  for(uint8_t i = 1; i < cell_count; i++)
  {
    if(pd.cell_voltage[i] < min_mv)
    {
      min_mv = pd.cell_voltage[i];
    }
    if(pd.cell_voltage[i] > max_mv)
    {
      max_mv = pd.cell_voltage[i];
    }
  }

  delta_mv = (float)bal_config->balance_delta_mv;
  stop_mv  = (float)bal_config->balance_stop_mv;
  start_mv = (float)bal_config->balance_start_mv;
  if(bal_mask != 0)
  {
    start_mv -= delta_mv / 2.0f;
  }
  if(max_mv < start_mv)
  {
    bal_mask  = 0;
    bal_pause = false;
    return bal_mask;
  }

  for(uint8_t i = 0; i < cell_count; i++)
  {
    float above_mv = pd.cell_voltage[i] - min_mv;

    if((bal_mask & (1U << i)) != 0)
    {
      if(above_mv <= stop_mv)
      {
        bal_mask &= (uint8_t)~(1U << i);
      }
    }
    else
    {
      if(above_mv > delta_mv)
      {
        bal_mask |= (uint8_t)(1U << i);
      }
    }
  }

  bal_phase_tick = now;

  return bal_mask;
}
