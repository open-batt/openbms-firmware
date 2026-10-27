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

// The HPPC test settings (rest time, current range, ...) are in config.h

// SOC checkpoints in %, discharge order. A charge run walks the list backwards.
static const uint8_t hppc_points[HPPC_POINT_COUNT] =
{
  100, 98, 95, 90, 85, 80, 70, 60, 50, 40,
   30, 25, 20, 15, 12, 10,  8,  6,  4,  2
};

typedef struct
{
  uint8_t  state;          // HPPC_STATE_*
  uint8_t  direction;      // HPPC_DIR_*
  bool     fet;            // main FET state the sequencer wants, applied by Ctrl_Run
  uint16_t status;         // low byte HPPC_STATE_*, high byte HPPC_STOP_*
  uint8_t  checkpoint;     // index of the current checkpoint in walking order
  uint32_t state_tick;     // time the current state was entered in ms
  float    coulomb_mah;    // charge moved in the current ramp
  float    target_mah;     // charge the current ramp has to move
  bool     open_ramp;      // last ramp: no target, runs until a stop condition
  uint32_t current_bad_ms; // how long the ramp current has been out of range
  uint32_t temp_bad_ms;    // how long the pack temperature has been outside the window
  uint32_t last_tick;      // time of the previous pass, the charge is counted with the real time step
} Hppc_t;

static Hppc_t           hppc                = {0};
static Config_Data_t    cfg                 = {0};   // settings of the running test

// -----------------------------------------------------------------------------
// HPPC sequencer. Does what do_hppc() in battery_cycler.py does, on the board:
//
//   DETECT  FET on for a moment, the sign of the current gives the direction
//   REST    FET off, wait HPPC_REST_MS                          (first at 100 %)
//   RAMP    FET on, count charge until the next checkpoint is reached
//   REST ... RAMP ... REST, one pair per checkpoint, then one last open ramp
//
// The test ends when a stop condition is met in a ramp (cell voltage limit, or
// for charging the termination current), when the probe sees no current, or
// when the host switches back to normal run. Every end switches the FET off,
// writes hppc_status and returns main_control to normal. The device then stays in
// HPPC run with the FET off until a reboot; no new test can be started before that.
//
// The sequencer never touches the FET itself. It keeps the state it wants and
// Ctrl_Run takes it from the fet argument of Hppc_Run() / Hppc_Stop() and switches the FET.
// The parameter extraction (openbms_hppc_extract.c) runs from Hppc_Run() and Hppc_Stop(), so it only
// works while the device is in HPPC run. The device stays there until a reboot, so that work always finishes.
// -----------------------------------------------------------------------------
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
  hppc.state_tick = SYS_GET_TICK();
  hppc.status     = state;
}
static void Hppc_Finish(uint8_t state, uint8_t reason)
{
  hppc.fet = false;

  hppc.state  = state;
  hppc.status = (uint16_t)(((uint16_t)reason << 8) | state);
  Ctrl_SetMode(CTRL_MODE_NORMAL);

  // A test that ends by its own stop condition gets its tables built, any other end drops the collected data
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
  // A broken NTC gives no temperature at all, the test cannot tell where it is
  if(Error_IsSet(OPENBMS_NTC_FAULT))
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
  return (hppc.state == HPPC_STATE_DETECT ||
          hppc.state == HPPC_STATE_REST   ||
          hppc.state == HPPC_STATE_RAMP);
}
void Hppc_Stop(bool *fet)
{
  // Called on every pass once the test has ended. The FET goes off whatever the test was doing.
  hppc.fet = false;

  // A test that was still running is reported as aborted by the host. One that
  // already finished keeps its own status.
  if(Hppc_IsRunning())
  {
    HppcX_Abort();
    hppc.state  = HPPC_STATE_ABORTED;
    hppc.status = (uint16_t)(((uint16_t)HPPC_STOP_HOST << 8) | HPPC_STATE_ABORTED);
  }

  // A test that ended by itself may still have fits and tables to finish
  HppcX_Run();
  if((HppcX_GetStatus() & HX_STATUS_BUSY) != 0)
  {
    *fet = hppc.fet;
    return;
  }

  // Reset the sequencer so the next start begins from scratch
  hppc.checkpoint  = 0;
  hppc.coulomb_mah = 0.0f;
  hppc.target_mah  = 0.0f;
  hppc.open_ramp   = false;

  *fet = hppc.fet;
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

  hppc.fet = true;
  HppcX_RampStarted();
  Hppc_SetState(HPPC_STATE_RAMP);
}
static void Hppc_Step(void)
{
  Peripheral_Data_t pd;
  uint32_t          now = SYS_GET_TICK();
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

  // Not running: a new test starts here
  if(!running)
  {
    hppc.direction   = HPPC_DIR_UNKNOWN;
    hppc.status      = HPPC_STATE_IDLE;
    hppc.checkpoint  = 0;
    hppc.coulomb_mah = 0.0f;
    hppc.target_mah  = 0.0f;
    hppc.open_ramp   = false;
    hppc.temp_bad_ms = 0;

    // Clear what the last test left in the extraction, so its status is not shown for this one
    HppcX_Reset();

    // The settings are copied once, so they cannot change in the middle of a test
    memcpy(&cfg, Ctrl_GetConfig(), sizeof(cfg));

    // Without a valid OCV window the parameter extraction cannot run, so the test is not started
    if(cfg.cell_capacity == 0 || cfg.cell_count < SYS_CELL_COUNT_MIN || cfg.cell_count > SYS_CELL_COUNT_MAX || cfg.ocv_v_hi_mv <= cfg.ocv_v_lo_mv)
    {
      Hppc_Finish(HPPC_STATE_ABORTED, HPPC_STOP_NO_CONFIG);
      return;
    }

    // Tables measured far from room temperature are not wanted, the test is not started
    Periph_GetData(&pd);
    if(!Hppc_TemperatureOk(&pd))
    {
      Hppc_Finish(HPPC_STATE_ABORTED, HPPC_STOP_TEMPERATURE);
      return;
    }

    // Known FET state before anything else, then probe the direction
    hppc.fet       = true;
    hppc.last_tick = now;
    Hppc_SetState(HPPC_STATE_DETECT);
    return;
  }

  // Running: Ctrl_Run calls this once per main loop tick. The real time since the last pass is
  // used, so a pass that took longer than one tick does not lose charge.
  dt_ms          = now - hppc.last_tick;
  hppc.last_tick = now;
  if(dt_ms > HPPC_MAX_DT_MS)
  {
    dt_ms = HPPC_MAX_DT_MS;
  }

  Periph_GetData(&pd);
  current_ma = pd.pack_current;     // mA, positive while charging

  // Temperature window: a broken NTC stops the test at once, a temperature outside the window
  // only after HPPC_TEMP_OUT_MS, so a short excursion does not throw the test away
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

          // The direction is known: the parameter extraction starts, then the first rest at the first checkpoint
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
          // Ramp to the next checkpoint: the SOC step in % of the design capacity
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
          // Last checkpoint rested: ramp on until a stop condition ends the test
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

      // Current along the test direction: positive when it moves the SOC the way the test goes.
      // A current that changed sign counts against the ramp and is caught by the range check below.
      if(charging)
      {
        dir_current_ma = current_ma;
      }
      else
      {
        dir_current_ma = -current_ma;
      }

      hppc.coulomb_mah += dir_current_ma * (float)dt_ms / 3600000.0f;

      // Stop conditions, checked on every step of every ramp
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

      // Termination current: only looked at once a cell is above the monitor voltage
      if(reason == HPPC_STOP_NONE && charging && above_monitor &&
         fabsf(current_ma) <= (float)cfg.charging_term_current)
      {
        reason = HPPC_STOP_TERM_CURRENT;
      }

      // The ramp current has to stay within HPPC_CURRENT_MIN_C to HPPC_CURRENT_MAX_C of the capacity,
      // otherwise the charger or load is gone or wrong and the ramp may never end. The minimum is not
      // checked once a charge is above the monitor voltage, there the current falls to the termination current.
      // A current in the wrong direction is below the minimum.
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
      else if(!hppc.open_ramp && hppc.coulomb_mah >= hppc.target_mah)
      {
        Hppc_StartRest();
      }
    }
    break;

    default: break;
  }
}

void Hppc_Run(bool *fet)
{
  Hppc_Step();

  // Fits and tables of the parameter extraction, a few evaluations per pass
  HppcX_Run();

  *fet = hppc.fet;
}

void Hppc_Abort(uint8_t reason, bool *fet)
{
  // Same ending as a test that stops by itself, the stop reason is given by the caller
  Hppc_Finish(HPPC_STATE_ABORTED, reason);

  *fet = hppc.fet;
}

void Hppc_GetStatus(uint8_t *direction, uint16_t *status, uint16_t *extract)
{
  *direction = hppc.direction;
  *status    = hppc.status;
  *extract   = HppcX_GetStatus();
}
