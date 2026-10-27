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

// -----------------------------------------------------------------------------
// Passive top balancing. Each cell has its own FET and a resistor across its two
// taps (50 Ohm on RevB, up to 86 mA). While the pack is charging and the highest
// cell is above balance_start_mv, every cell more than balance_delta_mv above the
// lowest cell bleeds, until it is within balance_delta_mv / 2 of it.
//
// The module never touches a pin: Balance_Run returns the cells to bleed and
// Ctrl_Run passes them to Periph_SetBalance, like the FET.
// -----------------------------------------------------------------------------

#define BAL_CHARGE_MIN_MA       (50.0f)     // the pack counts as charging above this current

static const Config_Data_t *bal_config = 0;
static uint8_t              bal_mask   = 0;

void Balance_Init(const Config_Data_t *config)
{
  bal_config = config;
  bal_mask   = 0;
}

uint8_t Balance_Run(bool allowed)
{
  Peripheral_Data_t pd;
  uint8_t           cell_count;
  float             min_mv;
  float             max_mv;
  float             start_mv;
  float             delta_mv;

  if(bal_config == 0 || !allowed || !bal_config->balance_enable)
  {
    bal_mask = 0;
    return bal_mask;
  }

  Periph_GetData(&pd);

  cell_count = bal_config->cell_count;
  if(cell_count > SYS_CELL_COUNT_MAX)
  {
    cell_count = SYS_CELL_COUNT_MAX;
  }

  // Only while charging, with good measurements and below the temperature limit
  if(cell_count < SYS_CELL_COUNT_MIN ||
     pd.pack_current <= BAL_CHARGE_MIN_MA ||
     Error_IsSet(OPENBMS_NTC_FAULT) ||
     Error_IsSet(OPENBMS_ADS131M08_DATA_LOST) ||
     !isfinite(pd.temperature_package) ||
     pd.temperature_package >= (float)bal_config->balance_max_temp_c)
  {
    bal_mask = 0;
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

  // Top balancing only. Once running, it goes on down to half a delta below the start
  // voltage, so a bleeding top cell does not switch everything off and on again.
  delta_mv = (float)bal_config->balance_delta_mv;
  start_mv = (float)bal_config->balance_start_mv;
  if(bal_mask != 0)
  {
    start_mv -= delta_mv / 2.0f;
  }
  if(max_mv < start_mv)
  {
    bal_mask = 0;
    return bal_mask;
  }

  // A cell starts bleeding above delta and stops within delta / 2 of the lowest cell
  for(uint8_t i = 0; i < cell_count; i++)
  {
    float above_mv = pd.cell_voltage[i] - min_mv;

    if((bal_mask & (1U << i)) != 0)
    {
      if(above_mv <= delta_mv / 2.0f)
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

  return bal_mask;
}
