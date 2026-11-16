/**
  ******************************************************************************
  * @file           : openbms_fuelgauge.c
  * @brief          : OpenBMS Fuel Gauge Module - 2RC Thevenin ECM + Extended
  *                   Kalman Filter SOC estimation
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
  *
  * One Extended Kalman Filter per cell, state x = [SOC, V_RC1, V_RC2], on a 2-RC Thevenin
  * model with the HPPC tables in FuelGauge_Data_t.
  *
  ******************************************************************************
  */

#include "openbms_fuelgauge.h"
#include "openbms_periph.h"
#include "openbms_error.h"

#include <math.h>
#include <string.h>
#include <stddef.h>

#define FG_TABLE_POINTS                 SYS_SOC_GRID_POINTS

#define FG_DEFAULT_SOC_GRID                 {2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f, 15.0f, 20.0f, 25.0f, 30.0f, \
                                             40.0f, 50.0f, 60.0f, 70.0f, 80.0f, 85.0f, 90.0f, 95.0f, 98.0f, 100.0f}   // SOC points in %, rising
#define FG_DEFAULT_Q_TEMP_C                 {-20.0f, -10.0f, 0.0f, 25.0f, 45.0f}        // capacity factor temperatures in degC, rising
#define FG_DEFAULT_Q_TEMP_FACTOR            {0.65f, 0.8f, 0.9f, 1.0f, 1.0f}             // usable capacity factor at those temperatures
#define FG_DEFAULT_COULOMBIC_EFFICIENCY     0.998f
#define FG_DEFAULT_EA_R0                    12500.0f        // activation energies in J/mol (0 = no temperature scaling)
#define FG_DEFAULT_EA_R1                    15500.0f
#define FG_DEFAULT_EA_TAU1                  7000.0f
#define FG_DEFAULT_EA_R2                    17000.0f
#define FG_DEFAULT_EA_TAU2                  1000.0f
#define FG_DEFAULT_KF_R_V                   2.5e-5f         // filter measurement noise in V^2

_Static_assert(_Alignof(FuelGauge_Data_t) % 4 == 0,
               "FuelGauge_Data_t must be 4-byte aligned for float access");
_Static_assert(offsetof(FuelGauge_Data_t, soc_grid) % 4 == 0,
               "soc_grid misaligned - adjust the reserved padding in openbms_data.h");
_Static_assert(offsetof(FuelGauge_Data_t, cell) % 4 == 0,
               "cell misaligned - a float member above it changed size");
_Static_assert(sizeof(FuelGauge_Cell_t) % 4 == 0,
               "FuelGauge_Cell_t must be a multiple of 4 bytes so every cell stays aligned");
_Static_assert((offsetof(FuelGauge_Data_t, cycle_count) - offsetof(FuelGauge_Data_t, soc_grid)) % sizeof(float) == 0,
               "the float block from soc_grid to cycle_count must hold only floats");

#define FG_GATE_SIGMA                   (4.0f)
#define FG_GATE_SIGMA_SQ                (FG_GATE_SIGMA * FG_GATE_SIGMA)

#define FG_MAX_SOC_RATE_PCT_PER_S       (1.0f)

#define FG_CURRENT_DEADZONE_A           (0.05f)
#define FG_SECONDS_PER_HOUR             (3600.0f)

#define FG_SOC_MIN                      (0.0f)
#define FG_SOC_MAX                      (1.0f)

#define FG_STATE_SOC_STEP               (0.01f)         // 1 %
#define FG_STATE_DIR_DISCHARGE          (0U)            // FuelGauge_State_t.last_charge: last current was a discharge
#define FG_STATE_DIR_CHARGE             (1U)            // ... a charge
#define FG_STATE_DIR_UNKNOWN            (2U)            // ... no current since a seed without a known direction (both OCV tables)
#define FG_REMAINING_MAX_MAH            ((float)UINT16_MAX)    // cell_remaining_capacity is a 16-bit register

#define FG_P0_SOC                       (0.0025f)
#define FG_P0_VRC                       (0.0004f)

#define FG_MIN_CAPACITY_AH              (0.05f)
#define FG_MIN_TAU_S                    (0.001f)
#define FG_MIN_INNOVATION_COV           (1e-12f)

#define FG_MAX_DT_S                     (0.5f)

#define FG_SEED_DELAY_MS                (500U)

#define FG_GAS_CONSTANT                 (8.314f)        // J/(mol K)
#define FG_KELVIN_OFFSET                (273.15f)
#define FG_TEMP_SCALE_MIN               (0.1f)          // limits of the scaling factor: a wrong temperature or Ea
#define FG_TEMP_SCALE_MAX               (20.0f)         // cannot blow the model up

#define FG_FULL_HOLD_MS                 (10000U)
#define FG_FULL_CELL_WINDOW_MV          (20.0f)         // cells this close to the highest one are full too (about 2 % SOC at the top)

#define FG_REST_MIN_MS                  (300000UL)      // 5 min
#define FG_REST_MAX_MS                  (1800000UL)     // 30 min
#define FG_REST_WINDOW_MS               (60000UL)       // 1 min
#define FG_REST_DVDT_MAX_V_PER_S        (4e-6f)         // 4 uV/s

#define FG_REST_WEIGHT_MAX              (0.8f)
#define FG_REST_SLOPE_FULL_MV_PER_PCT   (5.0f)
#define FG_REST_SLOPE_MIN_MV_PER_PCT    (1.0f)

#define FG_P_ANCHOR_SOC                 (1e-4f)

#define FG_CAP_MIN_DSOC                 (0.4f)          // 40 % SOC
#define FG_CAP_SLOPE_MIN_MV_PER_PCT     (3.0f)          // 1 mV of OCV error is at most 0.33 % SOC
#define FG_CAP_MAX_AGE_MS               (172800000UL)   // 48 h
#define FG_CAP_TEMP_MIN_C               (10.0f)
#define FG_CAP_TEMP_MAX_C               (40.0f)
#define FG_CAP_MAX_DEVIATION            (0.1f)          // 10 %

#define FG_BAL_SLOPE_MIN_MV_PER_PCT     (5.0f)          // 1 mV of OCV error is at most 0.2 % SOC (NMC: below ~15 % and above ~55 %)
#define FG_BAL_TARGET_MIN               (0.015f)        // 1.5 % of the cell capacity
#define FG_BAL_TARGET_MAX               (0.1f)          // 10 % of the cell capacity
#define FG_MAH_PER_AH                   (1000.0f)
#define FG_CAP_GAIN                     (0.2f)
#define FG_CAP_COUNT_MAX                (0xFFFFU)       // capacity_update_count stops here
#define FG_SOH_MAX_PCT                  (255.0f)        // cell_soh is 8 bits

#define FG_CYCLE_COUNT_MAX              (0xFFFFU)       // cycle_count stops here
#define FG_STATE_CYCLE_AH_MAX           (2.0f)          // saved partial cycle: at most this many q_nom_ah (a capacity update may lower q_nom_ah below it)

#define FG_SHOWN_SNAP                   (1e-4f)

#define FG_KF_CURRENT_NOISE_A           (0.02f)         // current sensor offset error, A
#define FG_KF_CURRENT_GAIN_ERROR        (0.01f)         // current sensor gain error (1 %)
#define FG_KF_MODEL_ERROR               (0.2f)          // relative error of the R0, R1, R2, tau1, tau2 tables (20 %)
#define FG_KF_R_SCALE_VAR               (0.1f)          // variance of ln(resistance scale): about x0.73 .. x1.37

typedef struct
{
  float r0;
  float r1;
  float tau1;
  float r2;
  float tau2;

} FuelGauge_TempScale_t;

typedef struct
{
  const float *ocv;
  const float *r0;
  const float *r1;
  const float *r2;
  const float *tau1;
  const float *tau2;
  const FuelGauge_TempScale_t *scale;     // temperature scaling of this branch (its own model temperature)

} FuelGauge_Branch_t;

static FuelGauge_Data_t  *fg_data             = NULL;
static const Config_Data_t *fg_config         = NULL;
static uint32_t           fg_last_tick        = 0;
static uint32_t           fg_init_tick        = 0;
static float              fg_prev_current_a   = 0.0f;
static float              fg_prev_bleed_a[SYS_CELL_COUNT_MAX]; // bleed + divider drain of each cell in the last step, A
static bool               fg_branch_charge    = true;
static bool               fg_branch_known     = false;  // the last current direction is known (saved one, or current has flowed)
static bool               fg_seeded           = false;
static bool               fg_soc_out_valid    = false;  // cell_soc holds a value: from now on it only moves the way the current goes

static uint32_t           fg_full_ms          = 0;      // time the full-charge condition holds
static bool               fg_full_done        = false;  // set once per charge, cleared by a discharge
static bool               fg_full_cell[SYS_CELL_COUNT_MAX]; // shows 100 % since the last full charge, cleared by a discharge

static uint32_t           fg_rest_ms          = 0;      // time at rest
static bool               fg_rest_done        = false;  // set once per rest, cleared by current
static uint32_t           fg_rest_window_ms   = 0;
static uint32_t           fg_rest_samples     = 0;
static float              fg_rest_v_sum[SYS_CELL_COUNT_MAX];
static float              fg_rest_v_prev[SYS_CELL_COUNT_MAX];
static float              fg_rest_i_sum[SYS_CELL_COUNT_MAX];     // cell current in A (drain included) summed over the window
static bool               fg_rest_have_prev   = false;
static bool               fg_rest_relaxed     = false;

static double             fg_cap_q_ah[SYS_CELL_COUNT_MAX];
static uint32_t           fg_cap_ms           = 0;
static bool               fg_cap_valid[SYS_CELL_COUNT_MAX];
static float              fg_cap_soc[SYS_CELL_COUNT_MAX];
static double             fg_cap_q_start[SYS_CELL_COUNT_MAX];
static uint32_t           fg_cap_ms_start[SYS_CELL_COUNT_MAX];
static bool               fg_cap_updated      = false;  // a cell's capacity changed at this rest
static bool               fg_cap_point        = false;  // a capacity learning point was stored at this rest
static bool               fg_cap_rest_done    = false;  // the capacity learning point of this rest was looked at
static uint8_t            fg_status           = 0;      // 0x98, FG_STATUS_* bits

static float              fg_temp_c           = 0.0f;   // last NTC temperature
static bool               fg_temp_ok          = false;  // fg_temp_c is a real temperature
static float              fg_bal_target_ah[SYS_CELL_COUNT_MAX];   // SOC balancing: Ah each cell still has to bleed
static float              fg_p_rscale[SYS_CELL_COUNT_MAX][3];     // covariance of the states with the resistance scale: P03, P13, P23
static uint16_t           fg_bal_target_mah[SYS_CELL_COUNT_MAX];  // the same in mAh (0x9A)
static bool               fg_bal_rest_done    = false;  // the balancing target was taken at this rest

static double             fg_cycle_ah         = 0.0;    // discharged charge toward the next cycle, Ah
static float              fg_soc_carry[SYS_CELL_COUNT_MAX];       // coulomb count: the part of the SOC steps the float SOC could not hold yet

static float              fg_state_shown[SYS_CELL_COUNT_MAX];
static uint16_t           fg_state_cycles     = 0;
static bool               fg_state_request    = false;
static bool               fg_state_soc_valid  = false;
static float              fg_state_soc[SYS_CELL_COUNT_MAX];
static float              fg_state_soc_shown[SYS_CELL_COUNT_MAX];
static bool               fg_state_charge     = true;   // last current direction given at boot for the seed
static bool               fg_state_dir_known  = false;  // ... and whether the saved state knew it

static FuelGauge_TempScale_t fg_temp_scale_dis = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
static FuelGauge_TempScale_t fg_temp_scale_chg = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
static float              fg_temp_capacity    = 1.0f;   // q_temp_factor at the present temperature

static void  FuelGauge_Seed(const Peripheral_Data_t *pd, float current_a);
static void  FuelGauge_UpdateTemperature(float temp_c);
static void  FuelGauge_UpdateOutputs(void);
static void  FuelGauge_UpdateShown(const float *soc_before, float current_a);
static void  FuelGauge_CheckFull(const Peripheral_Data_t *pd, float current_a, uint32_t elapsed_ms);
static void  FuelGauge_CheckRest(const Peripheral_Data_t *pd, float current_a, bool at_rest, uint32_t elapsed_ms);
static void  FuelGauge_UpdateStatus(bool at_rest);
static void  FuelGauge_CapacityPoint(const Peripheral_Data_t *pd);
static void  FuelGauge_BalanceTargetSet(void);
static void  FuelGauge_BalanceTargetClear(void);
static void  FuelGauge_RestReset(void);
static void  FuelGauge_FullReset(void);
static void  FuelGauge_Anchor(uint8_t idx, float soc, float weight);
static void  FuelGauge_CapacityReset(void);
static void  FuelGauge_CapacityCount(const Peripheral_Data_t *pd, float current_a, uint32_t elapsed_ms);
static void  FuelGauge_CapacityLearn(uint8_t idx, float soc_ocv);
static void  FuelGauge_CapacityApply(void);
static void  FuelGauge_CycleCount(float current_a, uint32_t elapsed_ms);
static bool  FuelGauge_AllFinite(const float *values, uint32_t count);
static float FuelGauge_BleedCurrent(const Peripheral_Data_t *pd, uint8_t cell);
static void  FuelGauge_UpdateCell(uint8_t idx, float dt_s, float i_prev_a, float i_now_a,
                                  float v_meas_v, const FuelGauge_Branch_t *predict,
                                  const FuelGauge_Branch_t *measure, bool at_rest);

static bool FuelGauge_AllFinite(const float *values, uint32_t count)
{
  for(uint32_t i = 0; i < count; i++)
  {
    if(!isfinite(values[i]))
    {
      return false;
    }
  }
  return true;
}
static uint8_t FuelGauge_Segment(float soc_pct, float *frac)
{
  const float *grid = fg_data->soc_grid;
  uint8_t      i;
  float        span;

  if(soc_pct <= grid[0])
  {
    *frac = 0.0f;
    return 1;
  }

  if(soc_pct >= grid[FG_TABLE_POINTS - 1])
  {
    *frac = 1.0f;
    return FG_TABLE_POINTS - 1;
  }

  for(i = 1; i < FG_TABLE_POINTS; i++)
  {
    if(soc_pct <= grid[i]) break;
  }

  if(i >= FG_TABLE_POINTS) i = FG_TABLE_POINTS - 1;

  span  = grid[i] - grid[i - 1];
  if(span > 0.0f)
  {
    *frac = (soc_pct - grid[i - 1]) / span;
  }
  else
  {
    *frac = 0.0f;
  }

  return i;
}
static float FuelGauge_Lookup(const float *table, float soc_frac)
{
  float   frac;
  uint8_t i = FuelGauge_Segment(soc_frac * 100.0f, &frac);

  return table[i - 1] + frac * (table[i] - table[i - 1]);
}
static float FuelGauge_LookupSlope(const float *table, float soc_frac)
{
  float   frac;
  uint8_t i = FuelGauge_Segment(soc_frac * 100.0f, &frac);
  float   span_frac;

  span_frac = (fg_data->soc_grid[i] - fg_data->soc_grid[i - 1]) / 100.0f;

  if(span_frac <= 0.0f) return 0.0f;

  return (table[i] - table[i - 1]) / span_frac;
}
static float FuelGauge_SmoothSlope(const float *table, float soc_frac)
{
  float   frac;
  uint8_t i = FuelGauge_Segment(soc_frac * 100.0f, &frac);
  float   slope_here = FuelGauge_LookupSlope(table, soc_frac);
  float   slope_next;
  float   span_frac;

  if(frac < 0.5f && i > 1U)
  {
    span_frac = (fg_data->soc_grid[i - 1] - fg_data->soc_grid[i - 2]) / 100.0f;
    if(span_frac <= 0.0f)
    {
      return slope_here;
    }
    slope_next = (table[i - 1] - table[i - 2]) / span_frac;
    return slope_next + (slope_here - slope_next) * (frac + 0.5f);
  }
  if(frac >= 0.5f && i < (FG_TABLE_POINTS - 1U))
  {
    span_frac = (fg_data->soc_grid[i + 1] - fg_data->soc_grid[i]) / 100.0f;
    if(span_frac <= 0.0f)
    {
      return slope_here;
    }
    slope_next = (table[i + 1] - table[i]) / span_frac;
    return slope_here + (slope_next - slope_here) * (frac - 0.5f);
  }
  return slope_here;
}
static float FuelGauge_OcvToSoc(const float *ocv, float voltage_v)
{
  const float *grid = fg_data->soc_grid;
  uint8_t      i;
  float        span;
  float        frac;

  if(voltage_v <  ocv[0])                     return FG_SOC_MIN;
  if(voltage_v == ocv[0])                     return grid[0] / 100.0f;
  if(voltage_v >= ocv[FG_TABLE_POINTS - 1])   return grid[FG_TABLE_POINTS - 1] / 100.0f;

  for(i = 1; i < FG_TABLE_POINTS; i++)
  {
    if(voltage_v <= ocv[i])
    {
      span = ocv[i] - ocv[i - 1];
      if(span > 0.0f)
      {
        frac = (voltage_v - ocv[i - 1]) / span;
      }
      else
      {
        frac = 0.0f;
      }

      return (grid[i - 1] + frac * (grid[i] - grid[i - 1])) / 100.0f;
    }
  }

  return grid[FG_TABLE_POINTS - 1] / 100.0f;
}
static float FuelGauge_RestSoc(uint8_t idx, float voltage_v, float *slope)
{
  const float *ocv_chg = fg_data->cell[idx].ocv_chg;
  const float *ocv_dis = fg_data->cell[idx].ocv_dis;
  float        soc;

  if(fg_branch_known)
  {
    const float *ocv;

    if(fg_branch_charge)
    {
      ocv = ocv_chg;
    }
    else
    {
      ocv = ocv_dis;
    }
    soc    = FuelGauge_OcvToSoc(ocv, voltage_v);
    *slope = FuelGauge_LookupSlope(ocv, soc);
    return soc;
  }

  soc    = 0.5f * (FuelGauge_OcvToSoc(ocv_chg, voltage_v) + FuelGauge_OcvToSoc(ocv_dis, voltage_v));
  *slope = 0.5f * (FuelGauge_LookupSlope(ocv_chg, soc) + FuelGauge_LookupSlope(ocv_dis, soc));
  return soc;
}
static bool FuelGauge_SelectBranch(float current_a, bool prev_charge)
{
  if(current_a >  FG_CURRENT_DEADZONE_A) return true;
  if(current_a < -FG_CURRENT_DEADZONE_A) return false;

  return prev_charge;
}
static void FuelGauge_GetBranch(bool charge, uint8_t cell, FuelGauge_Branch_t *branch)
{
  const FuelGauge_Cell_t *model = &fg_data->cell[cell];

  if(charge)
  {
    branch->ocv  = model->ocv_chg;
    branch->r0   = model->r0_chg;
    branch->r1   = model->r1_chg;
    branch->r2   = model->r2_chg;
    branch->tau1 = model->tau1_chg;
    branch->tau2 = model->tau2_chg;
    branch->scale = &fg_temp_scale_chg;
  }
  else
  {
    branch->ocv  = model->ocv_dis;
    branch->r0   = model->r0_dis;
    branch->r1   = model->r1_dis;
    branch->r2   = model->r2_dis;
    branch->tau1 = model->tau1_dis;
    branch->tau2 = model->tau2_dis;
    branch->scale = &fg_temp_scale_dis;
  }
}

// Current through the balance resistor of a cell, in A. It leaves the cell without passing
// the shunt, so it is taken off the pack current for that cell.
static float FuelGauge_BleedCurrent(const Peripheral_Data_t *pd, uint8_t cell)
{
  if((pd->fet_status & (1U << (PER_FET_BAL_SHIFT + cell))) == 0)
  {
    return 0.0f;
  }
  if(!(fg_config->balance_resistance_ohm > 0.0f))
  {
    return 0.0f;
  }
  return (pd->cell_voltage[cell] / 1000.0f) / fg_config->balance_resistance_ohm;
}
// Current the cell voltage dividers draw out of a cell, in A (cell k carries the taps
// k..SYS_CELL_COUNT_MAX, without passing the shunt). Only while cell measuring is on.
static float FuelGauge_DividerCurrent(const Peripheral_Data_t *pd, uint8_t cell)
{
  float tap_v = 0.0f;
  float drain_a = 0.0f;

  if(!Periph_CellMeasureOn())
  {
    return 0.0f;
  }

  for(uint8_t n = 0; n < SYS_CELL_COUNT_MAX; n++)
  {
    if(n < fg_config->cell_count)
    {
      tap_v += pd->cell_voltage[n] / 1000.0f;
    }
    if(n >= cell)
    {
      drain_a += tap_v / PER_CELL_DIVIDER_OHM;
    }
  }
  return drain_a;
}
// Everything that leaves a cell without passing the shunt: balancing bleed and divider drain, in A
static float FuelGauge_CellDrainCurrent(const Peripheral_Data_t *pd, uint8_t cell)
{
  return FuelGauge_BleedCurrent(pd, cell) + FuelGauge_DividerCurrent(pd, cell);
}
static void FuelGauge_UpdateCell(uint8_t idx, float dt_s, float i_prev_a, float i_now_a,
                                 float v_meas_v, const FuelGauge_Branch_t *predict,
                                 const FuelGauge_Branch_t *measure, bool at_rest)
{
  float soc  = fg_data->cell_soc_f[idx];
  float vrc1 = fg_data->cell_vrc1[idx];
  float vrc2 = fg_data->cell_vrc2[idx];

  float p[4][4];
  float pn[4][4];
  float t[4][4];

  p[0][0] = fg_data->cell_p[idx][0];
  p[0][1] = fg_data->cell_p[idx][1];
  p[0][2] = fg_data->cell_p[idx][2];
  p[1][1] = fg_data->cell_p[idx][3];
  p[1][2] = fg_data->cell_p[idx][4];
  p[2][2] = fg_data->cell_p[idx][5];
  p[0][3] = fg_p_rscale[idx][0];
  p[1][3] = fg_p_rscale[idx][1];
  p[2][3] = fg_p_rscale[idx][2];
  p[3][3] = FG_KF_R_SCALE_VAR;
  for(uint8_t r = 1; r < 4; r++)
  {
    for(uint8_t c = 0; c < r; c++)
    {
      p[r][c] = p[c][r];
    }
  }

  float capacity_ah = fg_data->cell[idx].capacity_ah;

  if(capacity_ah < FG_MIN_CAPACITY_AH) capacity_ah = FG_MIN_CAPACITY_AH;

  float tau1 = FuelGauge_Lookup(predict->tau1, soc) * predict->scale->tau1;
  float tau2 = FuelGauge_Lookup(predict->tau2, soc) * predict->scale->tau2;
  float r1   = FuelGauge_Lookup(predict->r1, soc) * predict->scale->r1;
  float r2   = FuelGauge_Lookup(predict->r2, soc) * predict->scale->r2;

  if(tau1 < FG_MIN_TAU_S) tau1 = FG_MIN_TAU_S;
  if(tau2 < FG_MIN_TAU_S) tau2 = FG_MIN_TAU_S;

  float a1 = expf(-dt_s / tau1);
  float a2 = expf(-dt_s / tau2);

  float b1 = r1 * (1.0f - a1) * i_prev_a;
  float b2 = r2 * (1.0f - a2) * i_prev_a;

  float sig_i_prev = FG_KF_CURRENT_NOISE_A + FG_KF_CURRENT_GAIN_ERROR * fabsf(i_prev_a);
  float q_soc      = sig_i_prev * dt_s / (FG_SECONDS_PER_HOUR * capacity_ah);
  float rc1_in     = r1 * (1.0f - a1);
  float rc2_in     = r2 * (1.0f - a2);
  float e1_r       = b1 * FG_KF_MODEL_ERROR;
  float e2_r       = b2 * FG_KF_MODEL_ERROR;
  float e1_tau     = (dt_s / tau1) * a1 * (vrc1 - r1 * i_prev_a) * FG_KF_MODEL_ERROR;
  float e2_tau     = (dt_s / tau2) * a2 * (vrc2 - r2 * i_prev_a) * FG_KF_MODEL_ERROR;

  float soc_step = (dt_s / (FG_SECONDS_PER_HOUR * capacity_ah)) * i_prev_a + fg_soc_carry[idx];
  float soc_cc   = soc + soc_step;

  fg_soc_carry[idx] = soc_step - (soc_cc - soc);
  if(!isfinite(fg_soc_carry[idx]))
  {
    fg_soc_carry[idx] = 0.0f;
  }
  soc  = soc_cc;
  vrc1 = a1 * vrc1 + b1;
  vrc2 = a2 * vrc2 + b2;

  if(soc < FG_SOC_MIN)
  {
    soc               = FG_SOC_MIN;
    fg_soc_carry[idx] = 0.0f;
  }
  if(soc > FG_SOC_MAX)
  {
    soc               = FG_SOC_MAX;
    fg_soc_carry[idx] = 0.0f;
  }

  float a[4][4] = { { 1.0f, 0.0f, 0.0f, 0.0f },
                    { 0.0f, a1,   0.0f, b1   },
                    { 0.0f, 0.0f, a2,   b2   },
                    { 0.0f, 0.0f, 0.0f, 1.0f } };

  for(uint8_t r = 0; r < 4; r++)
  {
    for(uint8_t c = 0; c < 4; c++)
    {
      float acc = 0.0f;
      for(uint8_t m = 0; m < 4; m++)
      {
        acc += a[r][m] * p[m][c];
      }
      t[r][c] = acc;
    }
  }
  for(uint8_t r = 0; r < 4; r++)
  {
    for(uint8_t c = 0; c < 4; c++)
    {
      float acc = 0.0f;
      for(uint8_t m = 0; m < 4; m++)
      {
        acc += t[r][m] * a[c][m];
      }
      pn[r][c] = acc;
    }
  }
  pn[0][0] += q_soc * q_soc;
  pn[1][1] += rc1_in * rc1_in * sig_i_prev * sig_i_prev + e1_r * e1_r + e1_tau * e1_tau;
  pn[2][2] += rc2_in * rc2_in * sig_i_prev * sig_i_prev + e2_r * e2_r + e2_tau * e2_tau;

  float r0     = FuelGauge_Lookup(measure->r0, soc) * measure->scale->r0;
  float v_pred = FuelGauge_Lookup(measure->ocv, soc) + i_now_a * r0 + vrc1 + vrc2;

  float sig_i_now = FG_KF_CURRENT_NOISE_A + FG_KF_CURRENT_GAIN_ERROR * fabsf(i_now_a);
  float e_r0      = i_now_a * r0 * FG_KF_MODEL_ERROR;
  float e_i       = r0 * sig_i_now;
  float r_meas    = fg_data->kf_r_v + e_r0 * e_r0 + e_i * e_i;

  float h[4] = { FuelGauge_LookupSlope(measure->ocv, soc), 1.0f, 1.0f, i_now_a * r0 };
  float ph[4];
  float s = r_meas;

  for(uint8_t r = 0; r < 4; r++)
  {
    float acc = 0.0f;
    for(uint8_t m = 0; m < 4; m++)
    {
      acc += pn[r][m] * h[m];
    }
    ph[r] = acc;
    s    += h[r] * acc;
  }

  if(s < FG_MIN_INNOVATION_COV) s = FG_MIN_INNOVATION_COV;

  float kg[4];
  for(uint8_t r = 0; r < 4; r++)
  {
    kg[r] = ph[r] / s;
  }
  kg[3] = 0.0f;

  if(at_rest)
  {
    kg[0] = 0.0f;
  }

  float y   = v_meas_v - v_pred;
  float nis = (y * y) / s;

  if(nis <= FG_GATE_SIGMA_SQ)
  {
    float scale        = 1.0f;
    float implied_dsoc = kg[0] * y;
    float max_dsoc     = FG_MAX_SOC_RATE_PCT_PER_S * dt_s / 100.0f;

    if(fabsf(implied_dsoc) > max_dsoc)
    {
      scale = max_dsoc / fabsf(implied_dsoc);
    }

    for(uint8_t r = 0; r < 4; r++)
    {
      kg[r] = scale * kg[r];
    }

    soc  = soc  + kg[0] * y;
    vrc1 = vrc1 + kg[1] * y;
    vrc2 = vrc2 + kg[2] * y;

    float mm[4][4];
    for(uint8_t r = 0; r < 4; r++)
    {
      for(uint8_t c = 0; c < 4; c++)
      {
        mm[r][c] = -kg[r] * h[c];
      }
      mm[r][r] += 1.0f;
    }
    for(uint8_t r = 0; r < 4; r++)
    {
      for(uint8_t c = 0; c < 4; c++)
      {
        float acc = 0.0f;
        for(uint8_t m = 0; m < 4; m++)
        {
          acc += mm[r][m] * pn[m][c];
        }
        t[r][c] = acc;
      }
    }
    for(uint8_t r = 0; r < 4; r++)
    {
      for(uint8_t c = 0; c < 4; c++)
      {
        float acc = 0.0f;
        for(uint8_t m = 0; m < 4; m++)
        {
          acc += t[r][m] * mm[c][m];
        }
        p[r][c] = acc + kg[r] * kg[c] * r_meas;
      }
    }
  }
  else
  {
    memcpy(p, pn, sizeof(p));
  }

  float state[12] = { soc, vrc1, vrc2, p[0][0], p[0][1], p[0][2], p[0][3],
                      p[1][1], p[1][2], p[1][3], p[2][2], p[2][3] };

  if(!FuelGauge_AllFinite(state, 12U))
  {
    if(FuelGauge_AllFinite(measure->ocv, FG_TABLE_POINTS))
    {
      soc = FuelGauge_OcvToSoc(measure->ocv, v_meas_v);
    }
    else
    {
      soc = 0.0f;
    }
    if(!isfinite(soc))
    {
      soc = 0.0f;
    }
    if(soc < FG_SOC_MIN) soc = FG_SOC_MIN;
    if(soc > FG_SOC_MAX) soc = FG_SOC_MAX;

    fg_data->cell_soc_f[idx] = soc;
    fg_soc_carry[idx]        = 0.0f;
    fg_data->cell_vrc1[idx]  = 0.0f;
    fg_data->cell_vrc2[idx]  = 0.0f;
    fg_data->cell_p[idx][0]  = FG_P0_SOC;
    fg_data->cell_p[idx][1]  = 0.0f;
    fg_data->cell_p[idx][2]  = 0.0f;
    fg_data->cell_p[idx][3]  = FG_P0_VRC;
    fg_data->cell_p[idx][4]  = 0.0f;
    fg_data->cell_p[idx][5]  = FG_P0_VRC;
    memset(fg_p_rscale[idx], 0, sizeof(fg_p_rscale[idx]));
    return;
  }

  if(soc < FG_SOC_MIN) soc = FG_SOC_MIN;
  if(soc > FG_SOC_MAX) soc = FG_SOC_MAX;

  fg_data->cell_soc_f[idx] = soc;
  fg_data->cell_vrc1[idx]  = vrc1;
  fg_data->cell_vrc2[idx]  = vrc2;

  fg_data->cell_p[idx][0] = p[0][0];
  fg_data->cell_p[idx][1] = p[0][1];
  fg_data->cell_p[idx][2] = p[0][2];
  fg_data->cell_p[idx][3] = p[1][1];
  fg_data->cell_p[idx][4] = p[1][2];
  fg_data->cell_p[idx][5] = p[2][2];
  fg_p_rscale[idx][0]     = p[0][3];
  fg_p_rscale[idx][1]     = p[1][3];
  fg_p_rscale[idx][2]     = p[2][3];
}

static void FuelGauge_Seed(const Peripheral_Data_t *pd, float current_a)
{
  bool last_charge = fg_branch_charge;
  bool known       = fg_branch_known;

  if(fg_state_soc_valid && fg_state_dir_known)
  {
    last_charge = fg_state_charge;
    known       = true;
  }
  if(current_a > FG_CURRENT_DEADZONE_A || current_a < -FG_CURRENT_DEADZONE_A)
  {
    known = true;
  }
  fg_branch_charge = FuelGauge_SelectBranch(current_a, last_charge);
  fg_branch_known  = known;

  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    float slope;

    if(fg_state_soc_valid)
    {
      fg_data->cell_soc_f[i] = fg_state_soc[i];
    }
    else
    {
      fg_data->cell_soc_f[i] = FuelGauge_RestSoc(i, pd->cell_voltage[i] / 1000.0f, &slope);
    }
    fg_data->cell_vrc1[i]  = 0.0f;
    fg_data->cell_vrc2[i]  = 0.0f;

    fg_data->cell_p[i][0] = FG_P0_SOC;
    fg_data->cell_p[i][1] = 0.0f;
    fg_data->cell_p[i][2] = 0.0f;
    fg_data->cell_p[i][3] = FG_P0_VRC;
    fg_data->cell_p[i][4] = 0.0f;
    fg_data->cell_p[i][5] = FG_P0_VRC;
    memset(fg_p_rscale[i], 0, sizeof(fg_p_rscale[i]));

    if(fg_state_soc_valid)
    {
      fg_data->cell_soc_shown_f[i] = fg_state_soc_shown[i];
    }
    else
    {
      fg_data->cell_soc_shown_f[i] = fg_data->cell_soc_f[i];
    }
    fg_state_shown[i] = fg_data->cell_soc_shown_f[i];
  }

  if(!fg_state_soc_valid)
  {
    fg_state_request = true;
  }
  fg_state_soc_valid = false;
  fg_state_cycles    = fg_data->cycle_count;

  fg_seeded        = true;
  fg_soc_out_valid = false;
  fg_full_ms       = 0;
  fg_full_done     = false;
  FuelGauge_FullReset();
  FuelGauge_RestReset();
  FuelGauge_CapacityReset();

  FuelGauge_UpdateOutputs();
}
// An OCV table the gauge can use: never falling, and the top above the bottom (an all-zero table from
// a cell whose fit failed in the HPPC test does not pass)
static bool FuelGauge_OcvRising(const float *ocv)
{
  for(uint8_t i = 1; i < FG_TABLE_POINTS; i++)
  {
    if(ocv[i] < ocv[i - 1])
    {
      return false;
    }
  }
  return (ocv[FG_TABLE_POINTS - 1] > ocv[0]);
}
static bool FuelGauge_GridRising(void)
{
  for(uint8_t i = 1; i < FG_TABLE_POINTS; i++)
  {
    if(fg_data->soc_grid[i] <= fg_data->soc_grid[i - 1])
    {
      return false;
    }
  }
  return true;
}
void FuelGauge_SetDefaultsIfEmpty(void)
{
  static const float grid[FG_TABLE_POINTS] = FG_DEFAULT_SOC_GRID;
  static const float q_temp_c[]            = FG_DEFAULT_Q_TEMP_C;
  static const float q_temp_factor[]       = FG_DEFAULT_Q_TEMP_FACTOR;

  _Static_assert(sizeof(q_temp_c) == sizeof(fg_data->q_temp_c), "FG_DEFAULT_Q_TEMP_C size");
  _Static_assert(sizeof(q_temp_factor) == sizeof(fg_data->q_temp_factor), "FG_DEFAULT_Q_TEMP_FACTOR size");

  if(FuelGauge_GridRising())
  {
    return;
  }

  memcpy(fg_data->soc_grid, grid, sizeof(fg_data->soc_grid));
  memcpy(fg_data->q_temp_c, q_temp_c, sizeof(fg_data->q_temp_c));
  memcpy(fg_data->q_temp_factor, q_temp_factor, sizeof(fg_data->q_temp_factor));
  fg_data->coulombic_efficiency = FG_DEFAULT_COULOMBIC_EFFICIENCY;
  fg_data->ea_r0                = FG_DEFAULT_EA_R0;
  fg_data->ea_r1                = FG_DEFAULT_EA_R1;
  fg_data->ea_tau1              = FG_DEFAULT_EA_TAU1;
  fg_data->ea_r2                = FG_DEFAULT_EA_R2;
  fg_data->ea_tau2              = FG_DEFAULT_EA_TAU2;
  fg_data->kf_r_v               = FG_DEFAULT_KF_R_V;
}
bool FuelGauge_IsReady(void)
{
  if((fg_data->learning_status & FG_LEARN_COMPLETE) == 0U)
  {
    return false;
  }
  if(fg_data->kf_r_v <= 0.0f || fg_data->coulombic_efficiency <= 0.0f)
  {
    return false;
  }
  if(!FuelGauge_GridRising())
  {
    return false;
  }
  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    if(fg_data->cell[i].capacity_ah <= 0.0f)
    {
      return false;
    }

    if(!FuelGauge_OcvRising(fg_data->cell[i].ocv_dis) || !FuelGauge_OcvRising(fg_data->cell[i].ocv_chg))
    {
      return false;
    }
  }

  if(fg_data->model_temp_dis_c + FG_KELVIN_OFFSET <= 0.0f || fg_data->model_temp_chg_c + FG_KELVIN_OFFSET <= 0.0f)
  {
    return false;
  }
  for(uint8_t i = 0; i < sizeof(fg_data->q_temp_c) / sizeof(fg_data->q_temp_c[0]); i++)
  {
    if(fg_data->q_temp_factor[i] <= 0.0f)
    {
      return false;
    }
    if(i > 0 && fg_data->q_temp_c[i] <= fg_data->q_temp_c[i - 1])
    {
      return false;
    }
  }
  return true;
}
static float FuelGauge_TempScale(float ea, float inv_t_diff)
{
  float scale = expf(ea / FG_GAS_CONSTANT * inv_t_diff);

  if(isnan(scale))               scale = 1.0f;
  if(scale < FG_TEMP_SCALE_MIN)  scale = FG_TEMP_SCALE_MIN;
  if(scale > FG_TEMP_SCALE_MAX)  scale = FG_TEMP_SCALE_MAX;
  return scale;
}
static float FuelGauge_TempCapacity(float temp_c)
{
  const float *t = fg_data->q_temp_c;
  const float *f = fg_data->q_temp_factor;
  uint8_t      n = sizeof(fg_data->q_temp_c) / sizeof(fg_data->q_temp_c[0]);

  if(temp_c <= t[0])
  {
    return f[0];
  }
  for(uint8_t i = 1; i < n; i++)
  {
    if(temp_c <= t[i])
    {
      return f[i - 1] + (temp_c - t[i - 1]) / (t[i] - t[i - 1]) * (f[i] - f[i - 1]);
    }
  }
  return f[n - 1];
}
static void FuelGauge_SetTempScale(FuelGauge_TempScale_t *scale, float temp_c, float model_temp_c)
{
  float inv_t_diff = 1.0f / (temp_c + FG_KELVIN_OFFSET) - 1.0f / (model_temp_c + FG_KELVIN_OFFSET);

  scale->r0   = FuelGauge_TempScale(fg_data->ea_r0,   inv_t_diff);
  scale->r1   = FuelGauge_TempScale(fg_data->ea_r1,   inv_t_diff);
  scale->tau1 = FuelGauge_TempScale(fg_data->ea_tau1, inv_t_diff);
  scale->r2   = FuelGauge_TempScale(fg_data->ea_r2,   inv_t_diff);
  scale->tau2 = FuelGauge_TempScale(fg_data->ea_tau2, inv_t_diff);
}
static void FuelGauge_ClearTempScale(FuelGauge_TempScale_t *scale)
{
  scale->r0   = 1.0f;
  scale->r1   = 1.0f;
  scale->tau1 = 1.0f;
  scale->r2   = 1.0f;
  scale->tau2 = 1.0f;
}
static void FuelGauge_UpdateTemperature(float temp_c)
{
  if(!isfinite(temp_c) || temp_c + FG_KELVIN_OFFSET <= 0.0f)
  {
    fg_temp_ok = false;
    FuelGauge_ClearTempScale(&fg_temp_scale_dis);
    FuelGauge_ClearTempScale(&fg_temp_scale_chg);
    fg_temp_capacity = 1.0f;
    return;
  }

  fg_temp_c  = temp_c;
  fg_temp_ok = true;

  FuelGauge_SetTempScale(&fg_temp_scale_dis, temp_c, fg_data->model_temp_dis_c);
  FuelGauge_SetTempScale(&fg_temp_scale_chg, temp_c, fg_data->model_temp_chg_c);
  fg_temp_capacity = FuelGauge_TempCapacity(temp_c);
}
static void FuelGauge_UpdateOutputs(void)
{
  uint8_t min_soc = 100;

  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    if(i >= fg_config->cell_count)
    {
      fg_data->cell_soc[i]                = 0;
      fg_data->cell_remaining_capacity[i] = 0;
      continue;
    }

    float   soc_pct = fg_data->cell_soc_shown_f[i] * 100.0f;
    uint8_t soc_int;

    if(soc_pct <   0.0f) soc_pct =   0.0f;
    if(soc_pct > 100.0f) soc_pct = 100.0f;

    soc_int = (uint8_t)(soc_pct + 0.5f);
    if(fg_soc_out_valid)
    {
      if(fg_prev_current_a > FG_CURRENT_DEADZONE_A)
      {
        if(soc_int < fg_data->cell_soc[i])
        {
          soc_int = fg_data->cell_soc[i];
        }
      }
      else
      {
        if(soc_int > fg_data->cell_soc[i])
        {
          soc_int = fg_data->cell_soc[i];
        }
      }
    }
    fg_data->cell_soc[i] = soc_int;

    if(fg_data->cell_soc[i] < min_soc) min_soc = fg_data->cell_soc[i];

    float remaining_mah = (soc_pct / 100.0f) * fg_data->cell[i].capacity_ah * fg_temp_capacity * 1000.0f + 0.5f;
    if(remaining_mah > FG_REMAINING_MAX_MAH) remaining_mah = FG_REMAINING_MAX_MAH;
    if(remaining_mah <     0.0f) remaining_mah =     0.0f;

    fg_data->cell_remaining_capacity[i] = (uint16_t)remaining_mah;
  }

  fg_data->relative_soc = min_soc;
  fg_soc_out_valid      = true;
}

static void FuelGauge_Anchor(uint8_t idx, float soc, float weight)
{
  float *p = fg_data->cell_p[idx];

  if(!isfinite(soc))
  {
    return;
  }
  if(soc < FG_SOC_MIN) soc = FG_SOC_MIN;
  if(soc > FG_SOC_MAX) soc = FG_SOC_MAX;

  fg_data->cell_soc_f[idx] = fg_data->cell_soc_f[idx] + weight * (soc - fg_data->cell_soc_f[idx]);

  p[0] = (1.0f - weight) * p[0] + weight * FG_P_ANCHOR_SOC;
  p[1] = 0.0f;
  p[2] = 0.0f;
  fg_p_rscale[idx][0] = 0.0f;
}
static void FuelGauge_UpdateShown(const float *soc_before, float current_a)
{
  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    float soc_old = soc_before[i];
    float soc_new = fg_data->cell_soc_f[i];
    float gap     = fg_data->cell_soc_shown_f[i] - soc_old;
    float ratio   = 1.0f;
    float shown;

    if(!isfinite(gap))
    {
      fg_data->cell_soc_shown_f[i] = soc_new;
      continue;
    }

    if(fg_full_cell[i])
    {
      continue;
    }

    if(fabsf(current_a) < FG_CURRENT_DEADZONE_A)
    {
      ratio = 1.0f;
    }
    else if(current_a < 0.0f)
    {
      if(soc_old > 0.0f)
      {
        ratio = soc_new / soc_old;
      }
    }
    else
    {
      if(soc_old < 1.0f)
      {
        ratio = (1.0f - soc_new) / (1.0f - soc_old);
      }
    }

    if(ratio > 1.0f) ratio = 1.0f;
    if(ratio < 0.0f) ratio = 0.0f;

    gap = gap * ratio;
    if(fabsf(gap) < FG_SHOWN_SNAP)
    {
      gap = 0.0f;
    }

    shown = soc_new + gap;
    if(shown < FG_SOC_MIN) shown = FG_SOC_MIN;
    if(shown > FG_SOC_MAX) shown = FG_SOC_MAX;

    fg_data->cell_soc_shown_f[i] = shown;
  }
}
static void FuelGauge_CheckFull(const Peripheral_Data_t *pd, float current_a, uint32_t elapsed_ms)
{
  float   top_mv = pd->cell_voltage[0];
  bool    full;

  if(current_a < -FG_CURRENT_DEADZONE_A)
  {
    fg_full_done = false;
    FuelGauge_FullReset();
  }

  for(uint8_t i = 1; i < fg_config->cell_count; i++)
  {
    if(pd->cell_voltage[i] > top_mv)
    {
      top_mv = pd->cell_voltage[i];
    }
  }

  full = (current_a > FG_CURRENT_DEADZONE_A) &&
         (top_mv >= (float)fg_config->charge_monitor_cell_mv) &&
         (current_a * 1000.0f <= (float)fg_config->charging_term_current);

  if(!full)
  {
    fg_full_ms = 0;
    return;
  }

  fg_full_ms += elapsed_ms;
  if(fg_full_done || fg_full_ms < FG_FULL_HOLD_MS)
  {
    return;
  }
  fg_full_done = true;

  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    if(top_mv - pd->cell_voltage[i] <= FG_FULL_CELL_WINDOW_MV)
    {
      FuelGauge_Anchor(i, FG_SOC_MAX, 1.0f);
      fg_data->cell_soc_shown_f[i] = FG_SOC_MAX;
      fg_full_cell[i]              = true;
    }
  }
}
static void FuelGauge_FullReset(void)
{
  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    fg_full_cell[i] = false;
  }
}
static void FuelGauge_RestReset(void)
{
  fg_rest_ms        = 0;
  fg_rest_done      = false;
  fg_rest_window_ms = 0;
  fg_rest_samples   = 0;
  fg_rest_have_prev = false;
  fg_rest_relaxed   = false;
  fg_cap_point      = false;
  fg_cap_updated    = false;
  fg_cap_rest_done  = false;
  fg_bal_rest_done  = false;

  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    fg_rest_v_sum[i] = 0.0f;
    fg_rest_i_sum[i] = 0.0f;
  }
}
static void FuelGauge_CheckRest(const Peripheral_Data_t *pd, float current_a, bool at_rest, uint32_t elapsed_ms)
{
  if(!at_rest)
  {
    FuelGauge_RestReset();
    return;
  }

  fg_rest_ms        += elapsed_ms;
  fg_rest_window_ms += elapsed_ms;
  fg_rest_samples++;

  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    fg_rest_v_sum[i] += pd->cell_voltage[i] / 1000.0f;
    fg_rest_i_sum[i] += current_a - FuelGauge_CellDrainCurrent(pd, i);
  }

  if(fg_rest_window_ms >= FG_REST_WINDOW_MS)
  {
    float window_s  = (float)fg_rest_window_ms / 1000.0f;
    float dvdt_max  = 0.0f;

    for(uint8_t i = 0; i < fg_config->cell_count; i++)
    {
      float avg = fg_rest_v_sum[i] / (float)fg_rest_samples;

      if(fg_rest_have_prev)
      {
        float        current_avg_a = fg_rest_i_sum[i] / (float)fg_rest_samples;
        float        drift_v_per_s = 0.0f;
        const float *ocv_chg       = fg_data->cell[i].ocv_chg;
        const float *ocv_dis       = fg_data->cell[i].ocv_dis;
        float        slope_v;
        float        dvdt;

        if(!fg_branch_known)
        {
          slope_v = 0.5f * (FuelGauge_SmoothSlope(ocv_chg, FuelGauge_OcvToSoc(ocv_chg, avg)) +
                            FuelGauge_SmoothSlope(ocv_dis, FuelGauge_OcvToSoc(ocv_dis, avg)));
        }
        else if(fg_branch_charge)
        {
          slope_v = FuelGauge_SmoothSlope(ocv_chg, FuelGauge_OcvToSoc(ocv_chg, avg));
        }
        else
        {
          slope_v = FuelGauge_SmoothSlope(ocv_dis, FuelGauge_OcvToSoc(ocv_dis, avg));
        }
        if(fg_data->cell[i].capacity_ah > 0.0f)
        {
          drift_v_per_s = current_avg_a / (fg_data->cell[i].capacity_ah * FG_SECONDS_PER_HOUR) * slope_v;
        }
        if(!isfinite(drift_v_per_s))
        {
          drift_v_per_s = 0.0f;
        }

        dvdt = fabsf((avg - fg_rest_v_prev[i]) / window_s - drift_v_per_s);
        if(dvdt > dvdt_max)
        {
          dvdt_max = dvdt;
        }
      }
      fg_rest_v_prev[i] = avg;
      fg_rest_v_sum[i]  = 0.0f;
      fg_rest_i_sum[i]  = 0.0f;
    }

    if(fg_rest_have_prev)
    {
      fg_rest_relaxed = (dvdt_max < FG_REST_DVDT_MAX_V_PER_S);
    }
    fg_rest_have_prev = true;
    fg_rest_window_ms = 0;
    fg_rest_samples   = 0;
  }

  if(fg_rest_relaxed && !fg_bal_rest_done && fg_rest_ms >= FG_REST_MIN_MS)
  {
    fg_bal_rest_done = true;
    FuelGauge_BalanceTargetSet();
  }

  if(fg_rest_relaxed && !fg_cap_rest_done && fg_rest_ms >= FG_REST_MIN_MS)
  {
    fg_cap_rest_done = true;
    FuelGauge_CapacityPoint(pd);
  }

  if(fg_rest_done || fg_rest_ms < FG_REST_MIN_MS)
  {
    return;
  }
  if(!fg_rest_relaxed && fg_rest_ms < FG_REST_MAX_MS)
  {
    return;
  }
  fg_rest_done = true;

  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    float soc_ocv;
    float slope;
    float slope_mv_per_pct;
    float weight;

    soc_ocv          = FuelGauge_RestSoc(i, fg_rest_v_prev[i], &slope);
    slope_mv_per_pct = slope * 1000.0f / 100.0f;

    if(slope_mv_per_pct < FG_REST_SLOPE_MIN_MV_PER_PCT)
    {
      continue;
    }

    weight = slope_mv_per_pct / FG_REST_SLOPE_FULL_MV_PER_PCT;
    if(weight > 1.0f)
    {
      weight = 1.0f;
    }
    weight = weight * FG_REST_WEIGHT_MAX;

    FuelGauge_Anchor(i, soc_ocv, weight);

    fg_data->cell_vrc1[i] = 0.0f;
    fg_data->cell_vrc2[i] = 0.0f;
  }

  fg_state_request = true;
}

static void FuelGauge_CapacityReset(void)
{
  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    fg_cap_valid[i] = false;
  }
}
static void FuelGauge_CapacityCount(const Peripheral_Data_t *pd, float current_a, uint32_t elapsed_ms)
{
  float cell_a;

  if(Error_IsSet(OPENBMS_ADS131M08_DATA_LOST))
  {
    FuelGauge_CapacityReset();
  }

  fg_cap_ms += elapsed_ms;

  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    cell_a = current_a;
    if(current_a > 0.0f)
    {
      cell_a = current_a * fg_data->coulombic_efficiency;
    }
    cell_a -= FuelGauge_CellDrainCurrent(pd, i);
    fg_cap_q_ah[i] += (double)(cell_a * ((float)elapsed_ms / 1000.0f) / 3600.0f);
  }
}
static void FuelGauge_CapacityPoint(const Peripheral_Data_t *pd)
{
  float temp_c = pd->temperature_package;

  if(!fg_branch_known || !isfinite(temp_c) || temp_c < FG_CAP_TEMP_MIN_C || temp_c > FG_CAP_TEMP_MAX_C)
  {
    return;
  }

  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    float slope;
    float soc_ocv          = FuelGauge_RestSoc(i, fg_rest_v_prev[i], &slope);
    float slope_mv_per_pct = slope * 1000.0f / 100.0f;

    if(isfinite(soc_ocv) && soc_ocv > fg_data->soc_grid[0] / 100.0f &&
       slope_mv_per_pct >= FG_CAP_SLOPE_MIN_MV_PER_PCT)
    {
      FuelGauge_CapacityLearn(i, soc_ocv);
    }
  }

  if(fg_cap_updated)
  {
    FuelGauge_CapacityApply();
    fg_state_request = true;
  }
}
static void FuelGauge_CapacityLearn(uint8_t idx, float soc_ocv)
{
  float dsoc;
  float measured_ah;
  float capacity_ah;

  if(fg_cap_valid[idx] && (fg_cap_ms - fg_cap_ms_start[idx]) <= FG_CAP_MAX_AGE_MS)
  {
    dsoc = soc_ocv - fg_cap_soc[idx];

    if(fabsf(dsoc) < FG_CAP_MIN_DSOC)
    {
      return;
    }

    measured_ah = (float)(fg_cap_q_ah[idx] - fg_cap_q_start[idx]) / dsoc;
    capacity_ah = fg_data->cell[idx].capacity_ah;

    if(isfinite(measured_ah) && capacity_ah > 0.0f &&
       fabsf(measured_ah - capacity_ah) <= FG_CAP_MAX_DEVIATION * capacity_ah)
    {
      fg_data->cell[idx].capacity_ah = capacity_ah + FG_CAP_GAIN * (measured_ah - capacity_ah);
      fg_cap_updated = true;
    }
  }

  fg_cap_point         = true;
  fg_cap_valid[idx]    = true;
  fg_cap_soc[idx]      = soc_ocv;
  fg_cap_q_start[idx]  = fg_cap_q_ah[idx];
  fg_cap_ms_start[idx] = fg_cap_ms;
}
static void FuelGauge_CapacityApply(void)
{
  float q_min_ah = fg_data->cell[0].capacity_ah;

  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    float soh_pct = 0.0f;

    if(fg_data->cell[i].capacity_ah < q_min_ah)
    {
      q_min_ah = fg_data->cell[i].capacity_ah;
    }

    if(fg_config->cell_capacity > 0U)
    {
      soh_pct = fg_data->cell[i].capacity_ah * 1000.0f / (float)fg_config->cell_capacity * 100.0f + 0.5f;
    }
    if(soh_pct > FG_SOH_MAX_PCT) soh_pct = FG_SOH_MAX_PCT;
    fg_data->cell_soh[i] = (uint8_t)soh_pct;
  }
  fg_data->q_nom_ah = q_min_ah;

  if(fg_data->capacity_update_count < FG_CAP_COUNT_MAX)
  {
    fg_data->capacity_update_count++;
  }
}

static void FuelGauge_CycleCount(float current_a, uint32_t elapsed_ms)
{
  if(current_a >= 0.0f || fg_data->q_nom_ah <= 0.0f)
  {
    return;
  }

  fg_cycle_ah += (double)(-current_a * ((float)elapsed_ms / 1000.0f) / 3600.0f);

  if(fg_cycle_ah >= (double)fg_data->q_nom_ah)
  {
    fg_cycle_ah -= (double)fg_data->q_nom_ah;

    if(fg_data->cycle_count < FG_CYCLE_COUNT_MAX)
    {
      fg_data->cycle_count++;
    }
  }
}

void FuelGauge_Init(FuelGauge_Data_t *data, const Config_Data_t *config)
{
  fg_data           = data;
  fg_config         = config;
  fg_last_tick      = HAL_GetTick();
  fg_init_tick      = fg_last_tick;
  fg_prev_current_a = 0.0f;
  memset(fg_prev_bleed_a, 0, sizeof(fg_prev_bleed_a));
  memset(fg_cap_q_ah, 0, sizeof(fg_cap_q_ah));
  fg_branch_charge  = true;
  fg_branch_known   = false;
  fg_state_dir_known = false;
  fg_seeded         = false;
  fg_soc_out_valid  = false;
  fg_full_ms        = 0;
  fg_full_done      = false;
  fg_state_request  = false;
  fg_state_soc_valid = false;
  FuelGauge_BalanceTargetClear();
  FuelGauge_FullReset();
  FuelGauge_RestReset();
  FuelGauge_CapacityReset();
}
void FuelGauge_Run(void)
{
  Peripheral_Data_t  pd;
  FuelGauge_Branch_t branch_predict;
  FuelGauge_Branch_t branch_measure;

  uint32_t now         = HAL_GetTick();
  uint32_t elapsed_ms  = now - fg_last_tick;

  fg_last_tick = now;

  Periph_GetData(&pd);
  FuelGauge_UpdateTemperature(pd.temperature_package);

  float i_now_a = pd.pack_current / 1000.0f;
  float dt_s    = (float)elapsed_ms / 1000.0f;

  if(!fg_seeded)
  {
    if(!Periph_CellMeasureReady())
    {
      fg_init_tick      = now;
      fg_prev_current_a = i_now_a;
      return;
    }

    if((now - fg_init_tick) < FG_SEED_DELAY_MS)
    {
      fg_prev_current_a = i_now_a;
      return;
    }

    FuelGauge_Seed(&pd, i_now_a);
    fg_prev_current_a = i_now_a;
    return;
  }

  if(dt_s > FG_MAX_DT_S)
  {
    FuelGauge_CapacityReset();
    fg_prev_current_a = i_now_a;
    return;
  }

  bool predict_charge = FuelGauge_SelectBranch(fg_prev_current_a, fg_branch_charge);
  bool measure_charge = FuelGauge_SelectBranch(i_now_a, predict_charge);

  bool balancing = ((pd.fet_status & (((1U << SYS_CELL_COUNT_MAX) - 1U) << PER_FET_BAL_SHIFT)) != 0);

  bool at_rest = (fabsf(fg_prev_current_a) < FG_CURRENT_DEADZONE_A) &&
                 (fabsf(i_now_a) < FG_CURRENT_DEADZONE_A) &&
                 !balancing;

  float soc_before[SYS_CELL_COUNT_MAX];
  memcpy(soc_before, fg_data->cell_soc_f, sizeof(soc_before));

  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    FuelGauge_GetBranch(predict_charge, i, &branch_predict);
    FuelGauge_GetBranch(measure_charge, i, &branch_measure);

    float bleed_a = FuelGauge_CellDrainCurrent(&pd, i);

    FuelGauge_UpdateCell(i, dt_s, fg_prev_current_a - fg_prev_bleed_a[i], i_now_a - bleed_a,
                         pd.cell_voltage[i] / 1000.0f,
                         &branch_predict, &branch_measure, at_rest);

    fg_prev_bleed_a[i] = bleed_a;

    fg_bal_target_ah[i] -= FuelGauge_BleedCurrent(&pd, i) * dt_s / FG_SECONDS_PER_HOUR;
    if(!(fg_bal_target_ah[i] > 0.0f))
    {
      fg_bal_target_ah[i] = 0.0f;
    }
  }

  fg_branch_charge  = measure_charge;
  if(i_now_a > FG_CURRENT_DEADZONE_A || i_now_a < -FG_CURRENT_DEADZONE_A)
  {
    fg_branch_known = true;
  }
  fg_prev_current_a = i_now_a;

  FuelGauge_UpdateShown(soc_before, i_now_a);
  FuelGauge_CapacityCount(&pd, i_now_a, elapsed_ms);
  FuelGauge_CycleCount(i_now_a, elapsed_ms);
  FuelGauge_CheckFull(&pd, i_now_a, elapsed_ms);
  FuelGauge_CheckRest(&pd, i_now_a, at_rest, elapsed_ms);
  FuelGauge_UpdateStatus(at_rest);

  FuelGauge_UpdateOutputs();
}
static void FuelGauge_UpdateStatus(bool at_rest)
{
  uint8_t status = 0;

  if(at_rest)
  {
    status |= FG_STATUS_AT_REST;
  }
  if(fg_rest_relaxed)
  {
    status |= FG_STATUS_RELAXED;
  }
  if(fg_rest_done)
  {
    status |= FG_STATUS_REST_CORRECTED;
  }
  if(fg_cap_point)
  {
    status |= FG_STATUS_CAP_POINT;
  }
  if(fg_cap_updated)
  {
    status |= FG_STATUS_CAP_UPDATED;
  }
  if(fg_full_ms > 0U && !fg_full_done)
  {
    status |= FG_STATUS_FULL_PENDING;
  }
  if(fg_full_done)
  {
    status |= FG_STATUS_FULL_SET;
  }
  fg_status = status;
}
const uint8_t *FuelGauge_GetStatus(void)
{
  return &fg_status;
}
const float *FuelGauge_GetBalanceTarget(void)
{
  return fg_bal_target_ah;
}
const uint16_t *FuelGauge_GetBalanceTargetMah(void)
{
  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    fg_bal_target_mah[i] = (uint16_t)(fg_bal_target_ah[i] * FG_MAH_PER_AH + 0.5f);
  }
  return fg_bal_target_mah;
}
static void FuelGauge_BalanceTargetClear(void)
{
  memset(fg_bal_target_ah, 0, sizeof(fg_bal_target_ah));
}
static void FuelGauge_BalanceTargetSet(void)
{
  float soc_ocv[SYS_CELL_COUNT_MAX];
  float room_ah[SYS_CELL_COUNT_MAX];
  float room_max_ah = 0.0f;

  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    float slope;
    float slope_mv_per_pct;

    soc_ocv[i]       = FuelGauge_RestSoc(i, fg_rest_v_prev[i], &slope);
    slope_mv_per_pct = slope * 1000.0f / 100.0f;
    if(!isfinite(soc_ocv[i]) || !(slope_mv_per_pct >= FG_BAL_SLOPE_MIN_MV_PER_PCT) ||
       !(fg_data->cell[i].capacity_ah > 0.0f))
    {
      return;
    }
  }

  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    float soc = soc_ocv[i];

    if(soc < 0.0f) soc = 0.0f;
    if(soc > 1.0f) soc = 1.0f;
    room_ah[i] = (1.0f - soc) * fg_data->cell[i].capacity_ah;
    if(room_ah[i] > room_max_ah)
    {
      room_max_ah = room_ah[i];
    }
  }

  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    float capacity_ah = fg_data->cell[i].capacity_ah;
    float target_ah   = room_max_ah - room_ah[i];

    if(target_ah < FG_BAL_TARGET_MIN * capacity_ah)
    {
      target_ah = 0.0f;
    }
    if(target_ah > FG_BAL_TARGET_MAX * capacity_ah)
    {
      target_ah = FG_BAL_TARGET_MAX * capacity_ah;
    }
    fg_bal_target_ah[i] = target_ah;
  }

  fg_state_request = true;
}
bool FuelGauge_SetData(const FuelGauge_Data_t *data)
{
  uint32_t count = (offsetof(FuelGauge_Data_t, cycle_count) - offsetof(FuelGauge_Data_t, soc_grid)) / sizeof(float);

  if(!FuelGauge_AllFinite(data->soc_grid, count))
  {
    return false;
  }

  if(data != fg_data)
  {
    memcpy(fg_data, data, sizeof(FuelGauge_Data_t));
  }
  return true;
}
const float *FuelGauge_GetSocGrid(void)
{
  return fg_data->soc_grid;
}
bool FuelGauge_SetCellTable(uint8_t cell, bool charge, const FuelGauge_Table_t *table)
{
  if(cell >= SYS_CELL_COUNT_MAX)
  {
    return false;
  }

  FuelGauge_Cell_t *c = &fg_data->cell[cell];

  if(!FuelGauge_AllFinite(table->ocv,  FG_TABLE_POINTS) || !FuelGauge_AllFinite(table->r0,   FG_TABLE_POINTS) ||
     !FuelGauge_AllFinite(table->r1,   FG_TABLE_POINTS) || !FuelGauge_AllFinite(table->tau1, FG_TABLE_POINTS) ||
     !FuelGauge_AllFinite(table->r2,   FG_TABLE_POINTS) || !FuelGauge_AllFinite(table->tau2, FG_TABLE_POINTS))
  {
    return false;
  }

  if(charge)
  {
    memcpy(c->ocv_chg,  table->ocv,  sizeof(c->ocv_chg));
    memcpy(c->r0_chg,   table->r0,   sizeof(c->r0_chg));
    memcpy(c->r1_chg,   table->r1,   sizeof(c->r1_chg));
    memcpy(c->tau1_chg, table->tau1, sizeof(c->tau1_chg));
    memcpy(c->r2_chg,   table->r2,   sizeof(c->r2_chg));
    memcpy(c->tau2_chg, table->tau2, sizeof(c->tau2_chg));
  }
  else
  {
    memcpy(c->ocv_dis,  table->ocv,  sizeof(c->ocv_dis));
    memcpy(c->r0_dis,   table->r0,   sizeof(c->r0_dis));
    memcpy(c->r1_dis,   table->r1,   sizeof(c->r1_dis));
    memcpy(c->tau1_dis, table->tau1, sizeof(c->tau1_dis));
    memcpy(c->r2_dis,   table->r2,   sizeof(c->r2_dis));
    memcpy(c->tau2_dis, table->tau2, sizeof(c->tau2_dis));
  }

  return true;
}
void FuelGauge_SetCellCapacity(uint8_t cell, float capacity_ah)
{
  if(cell >= SYS_CELL_COUNT_MAX || !isfinite(capacity_ah))
  {
    return;
  }
  fg_data->cell[cell].capacity_ah = capacity_ah;
}
void FuelGauge_SetModelTemperature(bool charge, float temp_c)
{
  if(!isfinite(temp_c))
  {
    return;
  }
  if(charge)
  {
    fg_data->model_temp_chg_c = temp_c;
  }
  else
  {
    fg_data->model_temp_dis_c = temp_c;
  }
}
void FuelGauge_SetNominalCapacity(float capacity_ah)
{
  if(!isfinite(capacity_ah))
  {
    return;
  }
  fg_data->q_nom_ah = capacity_ah;
}
void FuelGauge_SetLearned(bool charge)
{
  if(charge)
  {
    fg_data->learning_status |= FG_LEARN_CHARGE;
  }
  else
  {
    fg_data->learning_status |= FG_LEARN_DISCHARGE;
  }

  if((fg_data->learning_status & FG_LEARN_CHARGE) && (fg_data->learning_status & FG_LEARN_DISCHARGE))
  {
    fg_data->learning_status |= FG_LEARN_COMPLETE;
  }
}
void FuelGauge_ClearLearning(void)
{
  fg_data->learning_status = 0;
}
void FuelGauge_GetState(FuelGauge_State_t *state)
{
  FuelGauge_PeekState(state);
  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    fg_state_shown[i] = fg_data->cell_soc_shown_f[i];
  }
  fg_state_cycles  = fg_data->cycle_count;
  fg_state_request = false;
}
void FuelGauge_PeekState(FuelGauge_State_t *state)
{
  memset(state, 0, sizeof(FuelGauge_State_t));
  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    state->capacity_ah[i]      = fg_data->cell[i].capacity_ah;
    state->cell_soc_f[i]       = fg_data->cell_soc_f[i];
    state->cell_soc_shown_f[i] = fg_data->cell_soc_shown_f[i];
    state->cell_soh[i]         = fg_data->cell_soh[i];
  }
  state->q_nom_ah              = fg_data->q_nom_ah;
  state->cycle_count           = fg_data->cycle_count;
  state->capacity_update_count = fg_data->capacity_update_count;
  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    state->bal_target_ah[i] = fg_bal_target_ah[i];
  }
  state->cycle_ah = (float)fg_cycle_ah;
  if(!fg_branch_known)
  {
    state->last_charge = FG_STATE_DIR_UNKNOWN;
  }
  else if(fg_branch_charge)
  {
    state->last_charge = FG_STATE_DIR_CHARGE;
  }
  else
  {
    state->last_charge = FG_STATE_DIR_DISCHARGE;
  }
}
bool FuelGauge_StateValid(const FuelGauge_State_t *state)
{
  uint8_t cell_count = fg_config->cell_count;

  if(cell_count > SYS_CELL_COUNT_MAX)
  {
    cell_count = SYS_CELL_COUNT_MAX;
  }

  if(!isfinite(state->q_nom_ah) || state->q_nom_ah <= 0.0f)
  {
    return false;
  }
  for(uint8_t i = 0; i < cell_count; i++)
  {
    if(!isfinite(state->capacity_ah[i]) || state->capacity_ah[i] <= 0.0f ||
       !isfinite(state->cell_soc_f[i]) || state->cell_soc_f[i] < FG_SOC_MIN || state->cell_soc_f[i] > FG_SOC_MAX ||
       !isfinite(state->cell_soc_shown_f[i]) || state->cell_soc_shown_f[i] < FG_SOC_MIN || state->cell_soc_shown_f[i] > FG_SOC_MAX)
    {
      return false;
    }
  }

  if(state->last_charge > FG_STATE_DIR_UNKNOWN)
  {
    return false;
  }

  if(!isfinite(state->cycle_ah) || state->cycle_ah < 0.0f || state->cycle_ah > FG_STATE_CYCLE_AH_MAX * state->q_nom_ah)
  {
    return false;
  }

  for(uint8_t i = 0; i < cell_count; i++)
  {
    if(!isfinite(state->bal_target_ah[i]) || state->bal_target_ah[i] < 0.0f ||
       state->bal_target_ah[i] > FG_BAL_TARGET_MAX * state->capacity_ah[i])
    {
      return false;
    }
  }

  return true;
}
bool FuelGauge_SetState(const FuelGauge_State_t *state)
{
  uint8_t cell_count = fg_config->cell_count;

  if(cell_count > SYS_CELL_COUNT_MAX)
  {
    cell_count = SYS_CELL_COUNT_MAX;
  }

  if(!FuelGauge_StateValid(state))
  {
    return false;
  }

  for(uint8_t i = 0; i < cell_count; i++)
  {
    fg_data->cell[i].capacity_ah = state->capacity_ah[i];
    fg_data->cell_soh[i]         = state->cell_soh[i];
    fg_state_soc[i]              = state->cell_soc_f[i];
    fg_state_soc_shown[i]        = state->cell_soc_shown_f[i];
  }
  fg_data->q_nom_ah              = state->q_nom_ah;
  fg_data->cycle_count           = state->cycle_count;
  fg_data->capacity_update_count = state->capacity_update_count;
  fg_cycle_ah                    = (double)state->cycle_ah;
  for(uint8_t i = 0; i < cell_count; i++)
  {
    fg_bal_target_ah[i] = state->bal_target_ah[i];
  }
  fg_state_charge                = (state->last_charge == FG_STATE_DIR_CHARGE);
  fg_state_dir_known             = (state->last_charge != FG_STATE_DIR_UNKNOWN);
  fg_state_soc_valid             = true;
  return true;
}
bool FuelGauge_IsSeeded(void)
{
  return fg_seeded;
}
#ifdef OPENBMS_TEST
bool FuelGauge_ResetSocFromOcv(void)
{
  Peripheral_Data_t pd;

  if(!fg_seeded)
  {
    return false;
  }

  Periph_GetData(&pd);
  float current_a = pd.pack_current / 1000.0f;
  if(fabsf(current_a) >= FG_CURRENT_DEADZONE_A)
  {
    return false;
  }

  fg_state_soc_valid = false;
  FuelGauge_Seed(&pd, current_a);
  return true;
}
#endif
bool FuelGauge_StateChanged(void)
{
  if(!fg_seeded)
  {
    return false;
  }
  if(fg_state_request || fg_data->cycle_count != fg_state_cycles)
  {
    return true;
  }
  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    if(fabsf(fg_data->cell_soc_shown_f[i] - fg_state_shown[i]) >= FG_STATE_SOC_STEP)
    {
      return true;
    }
  }
  return false;
}

