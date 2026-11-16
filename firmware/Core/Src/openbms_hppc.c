/**
  ******************************************************************************
  * @file           : openbms_hppc.c
  * @brief          : OpenBMS HPPC Sequencer
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

#include "openbms_hppc.h"
#include "openbms_hppc_extract.h"
#include "openbms_ctrl.h"
#include "openbms_periph.h"
#include "openbms_error.h"
#include <math.h>
#include <string.h>

static const uint8_t hppc_points[HPPC_POINT_COUNT] =
{
  100, 98, 95, 90, 85, 80, 70, 60, 50, 40,
   30, 25, 20, 15, 12, 10,  8,  6,  4,  2
};

typedef struct
{
  uint8_t  state;          // HPPC_STATE_*
  uint8_t  direction;      // HPPC_DIR_*, detected by the probe
  uint8_t  requested;      // HPPC_DIR_*, the direction the host asked for
  bool     fet;            // main FET state the sequencer wants, applied by Ctrl_Run
  uint16_t status;         // low byte HPPC_STATE_*, high byte HPPC_STOP_*
  uint8_t  checkpoint;     // index of the current checkpoint in walking order
  uint32_t state_tick;     // time the current state was entered in ms
  double   coulomb_mah;    // charge moved in the current ramp (double: one 10 ms step is tiny against the sum)
  float    target_mah;     // charge the current ramp has to move
  bool     open_ramp;      // last ramp: no target, runs until a stop condition
  uint32_t current_bad_ms; // how long the ramp current has been out of range
  uint32_t term_ms;        // how long the charge current has been at or below charging_term_current
  uint32_t temp_bad_ms;    // how long the pack temperature has been outside the window
  uint32_t last_tick;      // time of the previous pass, the charge is counted with the real time step
  uint32_t settle_window_ms;                    // time summed into the current settle window
  uint32_t settle_samples;                      // samples summed into the current settle window
  float    settle_sum_mv[SYS_CELL_COUNT_MAX];   // cell voltages summed over the current settle window
  float    settle_prev_mv[SYS_CELL_COUNT_MAX];  // average of the previous settle window
  bool     settle_have_prev;                    // a previous window exists to compare with
  bool     settle_relaxed;                      // the last two windows differed by less than HPPC_SETTLE_DVDT_MAX_V_PER_S
} Hppc_t;

static Hppc_t           hppc                = {0};
static Config_Data_t    cfg                 = {0};   // settings of the running test

static uint8_t Hppc_Point(uint8_t n)
{
  if(hppc.direction == HPPC_DIR_CHARGE)
  {
    return hppc_points[HPPC_POINT_COUNT - 1 - n];
  }
  else
  {
    return hppc_points[n];
  }
}
static void Hppc_SetState(uint8_t state)
{
  hppc.state      = state;
  hppc.state_tick = HAL_GetTick();
  hppc.status     = state;
}
static void Hppc_Finish(uint8_t state, uint8_t reason)
{
  hppc.fet = false;

  hppc.state  = state;
  hppc.status = (uint16_t)(((uint16_t)reason << 8) | state);
  Ctrl_SetMode(CTRL_MODE_NORMAL);

  if(state == HPPC_STATE_DONE)
  {
    HppcX_Done();
  }
  else
  {
    HppcX_Abort();
  }
}
static bool Hppc_TemperatureOk(const Peripheral_Data_t *pd)
{
  if(Error_IsSet(OPENBMS_NTC_FAULT))
  {
    return false;
  }
  if(!isfinite(pd->temperature_package))
  {
    return false;
  }
  if(pd->temperature_package < HPPC_TEMP_MIN_C || pd->temperature_package > HPPC_TEMP_MAX_C)
  {
    return false;
  }
  return true;
}
static bool Hppc_IsRunning(void)
{
  return (hppc.state == HPPC_STATE_SETTLE ||
          hppc.state == HPPC_STATE_DETECT ||
          hppc.state == HPPC_STATE_REST   ||
          hppc.state == HPPC_STATE_RAMP);
}
void Hppc_Stop(bool *fet)
{
  hppc.fet = false;

  if(Hppc_IsRunning())
  {
    HppcX_Abort();
    hppc.state  = HPPC_STATE_ABORTED;
    hppc.status = (uint16_t)(((uint16_t)HPPC_STOP_HOST << 8) | HPPC_STATE_ABORTED);
  }

  HppcX_Run();
  if((HppcX_GetStatus() & HX_STATUS_BUSY) != 0)
  {
    *fet = hppc.fet;
    return;
  }

  hppc.checkpoint  = 0;
  hppc.coulomb_mah = 0.0f;
  hppc.target_mah  = 0.0f;
  hppc.open_ramp   = false;

  *fet = hppc.fet;
}
static void Hppc_StartSettle(void)
{
  hppc.fet              = false;
  hppc.settle_window_ms = 0;
  hppc.settle_samples   = 0;
  hppc.settle_have_prev = false;
  hppc.settle_relaxed   = false;
  memset(hppc.settle_sum_mv, 0, sizeof(hppc.settle_sum_mv));
  memset(hppc.settle_prev_mv, 0, sizeof(hppc.settle_prev_mv));
  Hppc_SetState(HPPC_STATE_SETTLE);
}
static void Hppc_SettleStep(const Peripheral_Data_t *pd, uint32_t dt_ms)
{
  float window_s;
  float avg_mv;
  float dvdt;
  float dvdt_max;

  for(uint8_t i = 0; i < cfg.cell_count; i++)
  {
    hppc.settle_sum_mv[i] += pd->cell_voltage[i];
  }
  hppc.settle_samples++;
  hppc.settle_window_ms += dt_ms;

  if(hppc.settle_window_ms < HPPC_SETTLE_WINDOW_MS)
  {
    return;
  }

  window_s = (float)hppc.settle_window_ms / 1000.0f;
  dvdt_max = 0.0f;
  for(uint8_t i = 0; i < cfg.cell_count; i++)
  {
    avg_mv = hppc.settle_sum_mv[i] / (float)hppc.settle_samples;
    if(hppc.settle_have_prev)
    {
      dvdt = fabsf(avg_mv - hppc.settle_prev_mv[i]) / 1000.0f / window_s;
      if(dvdt > dvdt_max)
      {
        dvdt_max = dvdt;
      }
    }
    hppc.settle_prev_mv[i] = avg_mv;
    hppc.settle_sum_mv[i]  = 0.0f;
  }
  if(hppc.settle_have_prev)
  {
    hppc.settle_relaxed = (dvdt_max < HPPC_SETTLE_DVDT_MAX_V_PER_S);
  }
  hppc.settle_have_prev = true;
  hppc.settle_samples   = 0;
  hppc.settle_window_ms = 0;
}
static void Hppc_StartRest(void)
{
  hppc.fet = false;
  HppcX_RestStarted();
  Hppc_SetState(HPPC_STATE_REST);
}
static void Hppc_StartRamp(void)
{
  hppc.coulomb_mah    = 0.0f;
  hppc.current_bad_ms = 0;
  hppc.term_ms        = 0;

  hppc.fet = true;
  HppcX_RampStarted();
  Hppc_SetState(HPPC_STATE_RAMP);
}
static void Hppc_Step(void)
{
  Peripheral_Data_t pd;
  uint32_t          now = HAL_GetTick();
  uint32_t          dt_ms;
  uint8_t           cell_count;
  uint8_t           reason;
  bool              charging;
  bool              above_monitor;
  bool              current_fault;
  float             current_ma;
  float             dir_current_ma;
  float             step_pct;

  bool running = Hppc_IsRunning();

  if(!running)
  {
    hppc.direction   = HPPC_DIR_UNKNOWN;
    hppc.status      = HPPC_STATE_IDLE;
    hppc.checkpoint  = 0;
    hppc.coulomb_mah = 0.0f;
    hppc.target_mah  = 0.0f;
    hppc.open_ramp   = false;
    hppc.temp_bad_ms = 0;

    HppcX_Reset();

    memcpy(&cfg, Ctrl_GetConfig(), sizeof(cfg));

    if(cfg.cell_capacity == 0 || cfg.cell_count < SYS_CELL_COUNT_MIN || cfg.cell_count > SYS_CELL_COUNT_MAX || cfg.ocv_v_hi_mv <= cfg.ocv_v_lo_mv)
    {
      Hppc_Finish(HPPC_STATE_ABORTED, HPPC_STOP_NO_CONFIG);
      return;
    }

    Periph_GetData(&pd);
    if(!Hppc_TemperatureOk(&pd))
    {
      Hppc_Finish(HPPC_STATE_ABORTED, HPPC_STOP_TEMPERATURE);
      return;
    }

    if((Error_GetFlags() & ERROR_FAULT_FLAGS) != 0U)
    {
      Hppc_Finish(HPPC_STATE_ABORTED, HPPC_STOP_ERRORS);
      return;
    }

    for(uint8_t i = 0; i < cfg.cell_count; i++)
    {
      if(hppc.requested == HPPC_DIR_DISCHARGE && pd.cell_voltage[i] < HPPC_DIS_START_MIN_MV)
      {
        Hppc_Finish(HPPC_STATE_ABORTED, HPPC_STOP_NOT_FULL);
        return;
      }
      if(hppc.requested == HPPC_DIR_CHARGE && pd.cell_voltage[i] > HPPC_CHG_START_MAX_MV)
      {
        Hppc_Finish(HPPC_STATE_ABORTED, HPPC_STOP_NOT_EMPTY);
        return;
      }
    }

    hppc.last_tick = now;
    Hppc_StartSettle();
    return;
  }

  dt_ms          = now - hppc.last_tick;
  hppc.last_tick = now;
  if(dt_ms > HPPC_MAX_DT_MS)
  {
    dt_ms = HPPC_MAX_DT_MS;
  }

  Periph_GetData(&pd);
  current_ma = pd.pack_current;

  if(Hppc_TemperatureOk(&pd))
  {
    hppc.temp_bad_ms = 0;
  }
  else
  {
    hppc.temp_bad_ms += dt_ms;
  }
  if(Error_IsSet(OPENBMS_NTC_FAULT) || hppc.temp_bad_ms >= HPPC_TEMP_OUT_MS)
  {
    Hppc_Finish(HPPC_STATE_ABORTED, HPPC_STOP_TEMPERATURE);
    return;
  }

  switch(hppc.state)
  {
    case HPPC_STATE_SETTLE:
    {
      Hppc_SettleStep(&pd, dt_ms);

      if((hppc.settle_relaxed && (now - hppc.state_tick) >= HPPC_SETTLE_MIN_MS) ||
         (now - hppc.state_tick) >= HPPC_SETTLE_MAX_MS)
      {
        hppc.fet = true;
        Hppc_SetState(HPPC_STATE_DETECT);
      }
    }
    break;

    case HPPC_STATE_DETECT:
    {
      if((now - hppc.state_tick) >= HPPC_DETECT_MS)
      {
        if(fabsf(current_ma) < HPPC_DETECT_MIN_MA)
        {
          Hppc_Finish(HPPC_STATE_ABORTED, HPPC_STOP_NO_CURRENT);
        }
        else
        {
          if(current_ma > 0.0f)
          {
            hppc.direction = HPPC_DIR_CHARGE;
          }
          else
          {
            hppc.direction = HPPC_DIR_DISCHARGE;
          }

          if(hppc.direction != hppc.requested)
          {
            Hppc_Finish(HPPC_STATE_ABORTED, HPPC_STOP_WRONG_DIRECTION);
            return;
          }

          HppcX_Begin(hppc.direction, cfg.cell_count, cfg.cell_capacity, cfg.ocv_v_lo_mv, cfg.ocv_v_hi_mv, HPPC_REST_MS);
          Hppc_StartRest();
        }
      }
    }
    break;

    case HPPC_STATE_REST:
    {
      HppcX_Tick(pd.cell_voltage, current_ma, pd.temperature_package, dt_ms);

      if((now - hppc.state_tick) >= HPPC_REST_MS)
      {
        if((hppc.checkpoint + 1) < HPPC_POINT_COUNT)
        {
          hppc.checkpoint++;
          step_pct = (float)Hppc_Point(hppc.checkpoint - 1) - (float)Hppc_Point(hppc.checkpoint);
          if(step_pct < 0.0f)
          {
            step_pct = -step_pct;
          }
          hppc.target_mah = (float)cfg.cell_capacity * step_pct / 100.0f;
          hppc.open_ramp  = false;
        }
        else
        {
          hppc.target_mah = 0.0f;
          hppc.open_ramp  = true;
        }

        Hppc_StartRamp();
      }
    }
    break;

    case HPPC_STATE_RAMP:
    {
      HppcX_Tick(pd.cell_voltage, current_ma, pd.temperature_package, dt_ms);

      charging = (hppc.direction == HPPC_DIR_CHARGE);

      if(charging)
      {
        dir_current_ma = current_ma;
      }
      else
      {
        dir_current_ma = -current_ma;
      }

      hppc.coulomb_mah += (double)(dir_current_ma * (float)dt_ms / 3600000.0f);

      cell_count = cfg.cell_count;
      if(cell_count > SYS_CELL_COUNT_MAX)
      {
        cell_count = SYS_CELL_COUNT_MAX;
      }

      above_monitor = false;
      reason        = HPPC_STOP_NONE;

      for(uint8_t i = 0; i < cell_count; i++)
      {
        float cell_mv = pd.cell_voltage[i];

        if(charging)
        {
          if(cell_mv >= (float)cfg.voltage_cell_max)
          {
            reason = HPPC_STOP_CELL_MAX;
            break;
          }
          if(cell_mv >= (float)cfg.charge_monitor_cell_mv)
          {
            above_monitor = true;
          }
        }
        else
        {
          if(cell_mv <= (float)cfg.voltage_cell_min)
          {
            reason = HPPC_STOP_CELL_MIN;
            break;
          }
        }
      }

      if(charging && above_monitor && fabsf(current_ma) <= (float)cfg.charging_term_current)
      {
        hppc.term_ms += dt_ms;
      }
      else
      {
        hppc.term_ms = 0;
      }
      if(reason == HPPC_STOP_NONE && hppc.term_ms >= HPPC_CURRENT_FAULT_MS)
      {
        reason = HPPC_STOP_TERM_CURRENT;
      }

      current_fault  = false;
      if(dir_current_ma > (float)cfg.cell_capacity * HPPC_CURRENT_MAX_C)
      {
        current_fault = true;
      }
      if(dir_current_ma < (float)cfg.cell_capacity * HPPC_CURRENT_MIN_C && !(charging && above_monitor))
      {
        current_fault = true;
      }

      if(current_fault)
      {
        hppc.current_bad_ms += dt_ms;
      }
      else
      {
        hppc.current_bad_ms = 0;
      }

      if(reason != HPPC_STOP_NONE)
      {
        Hppc_Finish(HPPC_STATE_DONE, reason);
      }
      else if(hppc.current_bad_ms >= HPPC_CURRENT_FAULT_MS)
      {
        Hppc_Finish(HPPC_STATE_ABORTED, HPPC_STOP_CURRENT_RANGE);
      }
      else if(!hppc.open_ramp && hppc.coulomb_mah >= (double)hppc.target_mah)
      {
        Hppc_StartRest();
      }
    }
    break;

    default: break;
  }
}

void Hppc_SetDirection(uint8_t direction)
{
  hppc.requested = direction;
}

void Hppc_Run(bool *fet)
{
  Hppc_Step();

  HppcX_Run();

  *fet = hppc.fet;
}

void Hppc_Abort(uint8_t reason, bool *fet)
{
  Hppc_Finish(HPPC_STATE_ABORTED, reason);

  *fet = hppc.fet;
}

void Hppc_GetStatus(uint8_t *direction, uint16_t *status, uint16_t *extract)
{
  *direction = hppc.direction;
  *status    = hppc.status;
  *extract   = HppcX_GetStatus();
}
