/**
  ******************************************************************************
  * @file           : openbms_fuelgauge.c
  * @brief          : OpenBMS Fuel Gauge Module — 2RC Thevenin ECM + Extended
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
  * Real-time port of python_scripts/soc_estimator.py. One independent 3-state
  * Extended Kalman Filter per cell, state x = [SOC, V_RC1, V_RC2]:
  *
  *   predict   SOC advances by coulomb counting (I*dt/Q_nom); V_RC1/V_RC2
  *             decay toward their steady state at the rates set by tau1/tau2
  *             and R1/R2 at the current SOC.
  *   correct   the predicted terminal voltage V_OCV(SOC) + I*R0 + V_RC1 +
  *             V_RC2 is compared against the measured cell voltage, and the
  *             residual is fed back through the local OCV slope dV_OCV/dSOC.
  *
  * Model parameters come from the fuel gauge registers (FuelGauge_Data_t),
  * which hold the HPPC lookup tables. They are filled by the on-board HPPC
  * extraction (openbms_hppc_extract.c) or written by the host.
  * See battery-model.md for the theory and python-script.md for the pipeline.
  *
  * Differences from the Python reference, all deliberate:
  *   - OCV(SOC) is interpolated piecewise-linearly rather than with a PCHIP
  *     spline, so dOCV/dSOC is constant within a table segment instead of
  *     smooth. The 20-point grid is denser where the OCV curve is steepest,
  *     which is where that approximation costs the most.
  *   - Outside the table range the lookups clamp to the end value instead of
  *     extrapolating, which keeps a below-2%-SOC reading from running off a
  *     steep extrapolated curve.
  *   - Every cell has its own tables and capacity (FuelGauge_Cell_t), like the
  *     per-cell tables of the Python pipeline.
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

// The lookups below take float pointers into FuelGauge_Data_t and dereference
// them directly, which on Cortex-M4 compiles to VLDR — an instruction that
// raises a UsageFault on an unaligned address rather than reading slowly.
// FuelGauge_Data_t is packed, so its alignment is not automatic: openbms_data.h
// pads the byte block ahead of soc_grid and pins the struct to aligned(4).
// Every float member after soc_grid is an array whose size is a multiple of 4,
// so holding the first and last of them is enough to hold the whole block.
_Static_assert(_Alignof(FuelGauge_Data_t) % 4 == 0,
               "FuelGauge_Data_t must be 4-byte aligned for float access");
_Static_assert(offsetof(FuelGauge_Data_t, soc_grid) % 4 == 0,
               "soc_grid misaligned — adjust the reserved padding in openbms_data.h");
_Static_assert(offsetof(FuelGauge_Data_t, cell) % 4 == 0,
               "cell misaligned — a float member above it changed size");
_Static_assert(sizeof(FuelGauge_Cell_t) % 4 == 0,
               "FuelGauge_Cell_t must be a multiple of 4 bytes so every cell stays aligned");
// FuelGauge_SetData checks soc_grid up to cycle_count as one block of floats
_Static_assert((offsetof(FuelGauge_Data_t, cycle_count) - offsetof(FuelGauge_Data_t, soc_grid)) % sizeof(float) == 0,
               "the float block from soc_grid to cycle_count must hold only floats");

// -------------------------------------------------------------------------
// Filter tuning — see soc_estimator.py, whose values these mirror
// -------------------------------------------------------------------------

// Innovation gate. A residual larger than this many standard deviations of
// the filter's own uncertainty is treated as a glitch rather than
// information — 4 sigma is generous enough to leave normal noise alone.
#define FG_GATE_SIGMA                   (4.0f)
#define FG_GATE_SIGMA_SQ                (FG_GATE_SIGMA * FG_GATE_SIGMA)

// Cap on how fast a measurement update may move SOC. Right after a long rest
// the covariance can still be loose enough that a large first residual passes
// the sigma test while being physically absurd — coulomb counting alone moves
// SOC nowhere near this fast.
#define FG_MAX_SOC_RATE_PCT_PER_S       (1.0f)

// Below this current the pack counts as at rest: the charge/discharge branch
// holds rather than flip-flopping on noise, and the SOC row of the Kalman
// gain is blocked.
#define FG_CURRENT_DEADZONE_A           (0.05f)

#define FG_SOC_MIN                      (0.0f)
#define FG_SOC_MAX                      (1.0f)
#define FG_REMAINING_MAX_MAH            ((float)UINT16_MAX)    // cell_remaining_capacity is a 16-bit register

// Initial covariance. battery-model.md notes P0 is script-side only with no
// register of its own, so it lives here; the values match the p0_soc/p0_vrc1/
// p0_vrc2 columns of kalman_parameters.csv (0.05^2 and 0.02^2).
#define FG_P0_SOC                       (0.0025f)
#define FG_P0_VRC                       (0.0004f)

// Guards. A register write of zero capacity would otherwise divide by zero,
// and a zero tau would divide by zero in the RC decay.
#define FG_MIN_CAPACITY_AH              (0.05f)
#define FG_MIN_TAU_S                    (0.001f)
#define FG_MIN_INNOVATION_COV           (1e-12f)

// A gap longer than this means the estimator was not running — an HPPC test,
// waiting for the config, a stall, or the first pass after boot. Integrating the current we happen to
// see now across the whole gap would inject a large bogus charge, so the step
// is skipped and the timebase resynchronised instead.
#define FG_MAX_DT_S                     (0.5f)

// How long to wait after init before seeding SOC. The cell voltage IIR filters
// in openbms_periph.c start from zero and converge with a time constant of
// roughly 24 ms, so a reading taken too early sits well below the real cell
// voltage — seeding off that would put SOC far too low and leave the filter
// crawling back at FG_MAX_SOC_RATE_PCT_PER_S for tens of seconds. 500 ms is
// ~20 time constants, by which point the filters have fully settled.
#define FG_SEED_DELAY_MS                (500U)

// Temperature model: the tables hold R and tau at model_temp_chg_c / model_temp_dis_c, at another temperature they are
// scaled with Arrhenius, R(T) = R_table * exp(Ea / R_gas * (1/T - 1/T_model)). Ea 0 - no scaling.
#define FG_GAS_CONSTANT                 (8.314f)        // J/(mol K)
#define FG_KELVIN_OFFSET                (273.15f)
#define FG_TEMP_SCALE_MIN               (0.1f)          // limits of the scaling factor: a wrong temperature or Ea
#define FG_TEMP_SCALE_MAX               (20.0f)         // cannot blow the model up

// -------------------------------------------------------------------------
// Full-charge and rest corrections
// -------------------------------------------------------------------------

// Full charge: charging, the highest cell at or above charge_monitor_cell_mv and the current at
// or below charging_term_current for this long. The highest cell and every cell within
// FG_FULL_CELL_WINDOW_MV of it are then full: SOC 100 %, shown 100 % until the next discharge.
#define FG_FULL_HOLD_MS                 (10000U)
#define FG_FULL_CELL_WINDOW_MV          (20.0f)         // cells this close to the highest one are full too (about 2 % SOC at the top)

// Rest: the current stays inside FG_CURRENT_DEADZONE_A. The cell voltage is averaged over
// windows of FG_REST_WINDOW_MS, and the cells count as relaxed once two windows in a row differ
// by less than FG_REST_DVDT_MAX_V_PER_S, or after FG_REST_MAX_MS whatever the voltage does.
// No correction before FG_REST_MIN_MS.
#define FG_REST_MIN_MS                  (300000UL)      // 5 min
#define FG_REST_MAX_MS                  (1800000UL)     // 30 min
#define FG_REST_WINDOW_MS               (60000UL)       // 1 min
#define FG_REST_DVDT_MAX_V_PER_S        (4e-6f)         // 4 uV/s

// How far a rest moves SOC toward the OCV SOC: FG_REST_WEIGHT_MAX where the OCV curve is at
// least FG_REST_SLOPE_FULL_MV_PER_PCT steep, less where it is flatter (a voltage error there is
// a bigger SOC error), nothing below FG_REST_SLOPE_MIN_MV_PER_PCT.
#define FG_REST_WEIGHT_MAX              (0.8f)
#define FG_REST_SLOPE_FULL_MV_PER_PCT   (5.0f)
#define FG_REST_SLOPE_MIN_MV_PER_PCT    (1.0f)

// SOC variance after a full-charge or rest correction: about 1 % SOC
#define FG_P_ANCHOR_SOC                 (1e-4f)

// Capacity learning: two relaxed rests (dV/dt settled, not the FG_REST_MAX_MS timeout) where the
// OCV curve is at least FG_CAP_SLOPE_MIN_MV_PER_PCT steep give two OCV SOCs. The charge counted
// between them over their SOC difference is a capacity measurement, used when the SOC moved at
// least FG_CAP_MIN_DSOC, within FG_CAP_MAX_AGE_MS and FG_CAP_TEMP_MIN_C..FG_CAP_TEMP_MAX_C.
// A measurement more than FG_CAP_MAX_DEVIATION away from the capacity is rejected, otherwise the
// capacity moves FG_CAP_GAIN of the way toward it.
#define FG_CAP_MIN_DSOC                 (0.4f)          // 40 % SOC
#define FG_CAP_SLOPE_MIN_MV_PER_PCT     (3.0f)          // 1 mV of OCV error is at most 0.33 % SOC
#define FG_CAP_MAX_AGE_MS               (172800000UL)   // 48 h
#define FG_CAP_TEMP_MIN_C               (10.0f)
#define FG_CAP_TEMP_MAX_C               (40.0f)
#define FG_CAP_MAX_DEVIATION            (0.1f)          // 10 %
#define FG_CAP_GAIN                     (0.2f)
#define FG_CAP_COUNT_MAX                (0xFFFFU)       // capacity_update_count stops here
#define FG_SOH_MAX_PCT                  (255.0f)        // cell_soh is 8 bits

// Cycle count: discharged charge (every discharge current, also inside the deadzone) is added up,
// each q_nom_ah of it is one cycle. A smaller learned capacity counts cycles faster.
#define FG_CYCLE_COUNT_MAX              (0xFFFFU)       // cycle_count stops here

// The shown SOC (registers) is moved onto the true SOC once the gap is smaller than this
#define FG_SHOWN_SNAP                   (1e-4f)

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

// Temperature scaling of the table values, the same for every cell (one temperature sensor)

static FuelGauge_Data_t  *fg_data             = NULL;
static const Config_Data_t *fg_config         = NULL;
static uint32_t           fg_last_tick        = 0;
static uint32_t           fg_init_tick        = 0;
static float              fg_prev_current_a   = 0.0f;
static float              fg_prev_bleed_a[SYS_CELL_COUNT_MAX]; // balance current of each cell in the last step, A
static bool               fg_branch_charge    = true;
static bool               fg_seeded           = false;

static uint32_t           fg_full_ms          = 0;      // time the full-charge condition holds
static bool               fg_full_done        = false;  // set once per charge, cleared by a discharge
static bool               fg_full_cell[SYS_CELL_COUNT_MAX]; // shows 100 % since the last full charge, cleared by a discharge

static uint32_t           fg_rest_ms          = 0;      // time at rest
static bool               fg_rest_done        = false;  // set once per rest, cleared by current
static uint32_t           fg_rest_window_ms   = 0;
static uint32_t           fg_rest_samples     = 0;
static float              fg_rest_v_sum[SYS_CELL_COUNT_MAX];
static float              fg_rest_v_prev[SYS_CELL_COUNT_MAX];
static bool               fg_rest_have_prev   = false;
static bool               fg_rest_relaxed     = false;

// Capacity learning: charge counted since boot per cell (Ah, + charge, double so small steps are
// not lost; a balancing cell's bleed is taken off its own counter) and per cell the last learning
// point: its OCV SOC, the counter and the time at that point
static double             fg_cap_q_ah[SYS_CELL_COUNT_MAX];
static uint32_t           fg_cap_ms           = 0;
static bool               fg_cap_valid[SYS_CELL_COUNT_MAX];
static float              fg_cap_soc[SYS_CELL_COUNT_MAX];
static double             fg_cap_q_start[SYS_CELL_COUNT_MAX];
static uint32_t           fg_cap_ms_start[SYS_CELL_COUNT_MAX];
static bool               fg_cap_updated      = false;  // a cell's capacity changed at this rest

static double             fg_cycle_ah         = 0.0;    // discharged charge toward the next cycle, Ah

static FuelGauge_TempScale_t fg_temp_scale_dis = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
static FuelGauge_TempScale_t fg_temp_scale_chg = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
static float              fg_temp_capacity    = 1.0f;   // q_temp_factor at the present temperature

static void  FuelGauge_Seed(const Peripheral_Data_t *pd, float current_a);
static void  FuelGauge_UpdateTemperature(float temp_c);
static void  FuelGauge_UpdateOutputs(void);
static void  FuelGauge_UpdateShown(const float *soc_before, float current_a);
static void  FuelGauge_CheckFull(const Peripheral_Data_t *pd, float current_a, uint32_t elapsed_ms);
static void  FuelGauge_CheckRest(const Peripheral_Data_t *pd, bool at_rest, uint32_t elapsed_ms);
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

// -------------------------------------------------------------------------
// Table lookups
// -------------------------------------------------------------------------
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
  // Returns the upper index of the SOC grid segment containing soc_pct, and
  // the 0..1 position within it. Outside the table the value clamps to the
  // nearest end point.
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
  // Slope of the containing segment, in table units per SOC *fraction* —
  // this is the dOCV/dSOC that forms H[0] in the measurement model.
  float   frac;
  uint8_t i = FuelGauge_Segment(soc_frac * 100.0f, &frac);
  float   span_frac;

  (void)frac;

  span_frac = (fg_data->soc_grid[i] - fg_data->soc_grid[i - 1]) / 100.0f;

  if(span_frac <= 0.0f) return 0.0f;

  return (table[i] - table[i - 1]) / span_frac;
}
static float FuelGauge_OcvToSoc(const float *ocv, float voltage_v)
{
  // Invert the OCV curve to seed SOC from a resting voltage. The table is
  // monotonically rising, so a forward scan is enough.
  const float *grid = fg_data->soc_grid;
  uint8_t      i;
  float        span;
  float        frac;

  if(voltage_v <= ocv[0])                     return grid[0] / 100.0f;
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

// -------------------------------------------------------------------------
// Extended Kalman Filter — one cell, one step
// -------------------------------------------------------------------------
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
static void FuelGauge_UpdateCell(uint8_t idx, float dt_s, float i_prev_a, float i_now_a,
                                 float v_meas_v, const FuelGauge_Branch_t *predict,
                                 const FuelGauge_Branch_t *measure, bool at_rest)
{
  float soc  = fg_data->cell_soc_f[idx];
  float vrc1 = fg_data->cell_vrc1[idx];
  float vrc2 = fg_data->cell_vrc2[idx];

  // Covariance is stored as the upper triangle [P00, P01, P02, P11, P12, P22].
  float p00 = fg_data->cell_p[idx][0];
  float p01 = fg_data->cell_p[idx][1];
  float p02 = fg_data->cell_p[idx][2];
  float p11 = fg_data->cell_p[idx][3];
  float p12 = fg_data->cell_p[idx][4];
  float p22 = fg_data->cell_p[idx][5];

  float capacity_ah = fg_data->cell[idx].capacity_ah;

  if(capacity_ah < FG_MIN_CAPACITY_AH) capacity_ah = FG_MIN_CAPACITY_AH;

  // ---------------- predict ----------------
  // Parameters are looked up at the previous SOC, before coulomb counting
  // advances it — matching the reference implementation.
  float tau1 = FuelGauge_Lookup(predict->tau1, soc) * predict->scale->tau1;
  float tau2 = FuelGauge_Lookup(predict->tau2, soc) * predict->scale->tau2;
  float r1   = FuelGauge_Lookup(predict->r1, soc) * predict->scale->r1;
  float r2   = FuelGauge_Lookup(predict->r2, soc) * predict->scale->r2;

  if(tau1 < FG_MIN_TAU_S) tau1 = FG_MIN_TAU_S;
  if(tau2 < FG_MIN_TAU_S) tau2 = FG_MIN_TAU_S;

  float a1 = expf(-dt_s / tau1);
  float a2 = expf(-dt_s / tau2);

  soc  = soc + (dt_s / (3600.0f * capacity_ah)) * i_prev_a;
  vrc1 = a1 * vrc1 + r1 * (1.0f - a1) * i_prev_a;
  vrc2 = a2 * vrc2 + r2 * (1.0f - a2) * i_prev_a;

  if(soc < FG_SOC_MIN) soc = FG_SOC_MIN;
  if(soc > FG_SOC_MAX) soc = FG_SOC_MAX;

  // P = A*P*A' + Q, with A = diag(1, a1, a2) and Q diagonal, so the whole
  // product collapses to a scaling of each element.
  p00 = p00 + fg_data->kf_q_soc;
  p01 = a1 * p01;
  p02 = a2 * p02;
  p11 = a1 * a1 * p11 + fg_data->kf_q_rc1;
  p12 = a1 * a2 * p12;
  p22 = a2 * a2 * p22 + fg_data->kf_q_rc2;

  // ---------------- measurement ----------------
  float r0     = FuelGauge_Lookup(measure->r0, soc) * measure->scale->r0;
  float v_pred = FuelGauge_Lookup(measure->ocv, soc) + i_now_a * r0 + vrc1 + vrc2;
  float h0     = FuelGauge_LookupSlope(measure->ocv, soc);

  // S = H*P*H' + R, with H = [dOCV/dSOC, 1, 1]
  float s = h0 * h0 * p00 + 2.0f * h0 * p01 + 2.0f * h0 * p02
            + p11 + 2.0f * p12 + p22 + fg_data->kf_r_v;

  if(s < FG_MIN_INNOVATION_COV) s = FG_MIN_INNOVATION_COV;

  // K = P*H' / S
  float k0 = (h0 * p00 + p01 + p02) / s;
  float k1 = (h0 * p01 + p11 + p12) / s;
  float k2 = (h0 * p02 + p12 + p22) / s;

  float y = v_meas_v - v_pred;

  // SOC observability: with no current flowing, coulomb counting says SOC
  // cannot be changing, so a voltage residual can only mean the relaxation
  // estimate is a little off — never that real charge moved. Zeroing the SOC
  // row of the gain lets V_RC1/V_RC2 absorb the residual while keeping it out
  // of SOC. Without this, a relaxation mismatch during a long rest reads as a
  // slow steady SOC drift for as long as the rest lasts.
  float k0_gain;
  if(at_rest)
  {
    k0_gain = 0.0f;
  }
  else
  {
    k0_gain = k0;
  }

  float nis = (y * y) / s;

  // Above FG_GATE_SIGMA_SQ the sample is a statistical outlier and is ignored
  // entirely this step. No mean update and no covariance shrink; P still evolves via Q next step.
  if(nis <= FG_GATE_SIGMA_SQ)
  {
    float scale        = 1.0f;
    float implied_dsoc = k0_gain * y;
    float max_dsoc     = FG_MAX_SOC_RATE_PCT_PER_S * dt_s / 100.0f;

    if(fabsf(implied_dsoc) > max_dsoc)
    {
      // Not a statistical outlier, but the implied SOC jump is far faster
      // than coulomb counting could produce. Throttle the mean correction
      // only — the covariance still updates in full, because skipping that
      // would leave P un-shrunk, K growing, and this cap tripping on nearly
      // every later sample.
      scale = max_dsoc / fabsf(implied_dsoc);
    }

    soc  = soc  + scale * k0_gain * y;
    vrc1 = vrc1 + scale * k1 * y;
    vrc2 = vrc2 + scale * k2 * y;

    // Joseph-form covariance update:
    //
    //     P = (I - K*H) * P * (I - K*H)' + K * R * K'
    //
    // The plain (I - K*H)*P form is only symmetric when K is the optimal
    // gain, and zeroing the SOC row at rest makes it non-optimal by
    // construction — replaying a real log shows the reference implementation's
    // P drifting asymmetric by ~1.3e-3 through rest periods. Only the upper
    // triangle is stored here, so that asymmetry cannot be represented and
    // some symmetric approximation is forced regardless. Joseph form is the
    // one to pick: symmetric and positive semi-definite for ANY gain, it
    // tracks the reference more closely than either discarding or averaging
    // the lower triangle, and it stays well conditioned even if a host writes
    // an odd value into one of the noise registers.
    float mm00 = 1.0f - k0_gain * h0, mm01 = -k0_gain,   mm02 = -k0_gain;
    float mm10 = -k1 * h0,            mm11 = 1.0f - k1,  mm12 = -k1;
    float mm20 = -k2 * h0,            mm21 = -k2,        mm22 = 1.0f - k2;

    // T = (I - K*H) * P
    float n00 = mm00 * p00 + mm01 * p01 + mm02 * p02;
    float n01 = mm00 * p01 + mm01 * p11 + mm02 * p12;
    float n02 = mm00 * p02 + mm01 * p12 + mm02 * p22;
    float n10 = mm10 * p00 + mm11 * p01 + mm12 * p02;
    float n11 = mm10 * p01 + mm11 * p11 + mm12 * p12;
    float n12 = mm10 * p02 + mm11 * p12 + mm12 * p22;
    float n20 = mm20 * p00 + mm21 * p01 + mm22 * p02;
    float n21 = mm20 * p01 + mm21 * p11 + mm22 * p12;
    float n22 = mm20 * p02 + mm21 * p12 + mm22 * p22;

    // P = T * (I - K*H)' + K*R*K' — the product is symmetric by construction,
    // so only the upper triangle needs computing.
    float r_v = fg_data->kf_r_v;

    p00 = n00 * mm00 + n01 * mm01 + n02 * mm02 + k0_gain * k0_gain * r_v;
    p01 = n00 * mm10 + n01 * mm11 + n02 * mm12 + k0_gain * k1 * r_v;
    p02 = n00 * mm20 + n01 * mm21 + n02 * mm22 + k0_gain * k2 * r_v;
    p11 = n10 * mm10 + n11 * mm11 + n12 * mm12 + k1 * k1 * r_v;
    p12 = n10 * mm20 + n11 * mm21 + n12 * mm22 + k1 * k2 * r_v;
    p22 = n20 * mm20 + n21 * mm21 + n22 * mm22 + k2 * k2 * r_v;
  }

  // A NaN or inf passes every clamp and would stay in the state for good, so the
  // cell starts again from its OCV table. With a broken OCV table the cell reads 0.
  float state[9] = { soc, vrc1, vrc2, p00, p01, p02, p11, p12, p22 };

  if(!FuelGauge_AllFinite(state, 9U))
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

    vrc1 = 0.0f;
    vrc2 = 0.0f;
    p00  = FG_P0_SOC;
    p01  = 0.0f;
    p02  = 0.0f;
    p11  = FG_P0_VRC;
    p12  = 0.0f;
    p22  = FG_P0_VRC;
  }

  if(soc < FG_SOC_MIN) soc = FG_SOC_MIN;
  if(soc > FG_SOC_MAX) soc = FG_SOC_MAX;

  fg_data->cell_soc_f[idx] = soc;
  fg_data->cell_vrc1[idx]  = vrc1;
  fg_data->cell_vrc2[idx]  = vrc2;

  fg_data->cell_p[idx][0] = p00;
  fg_data->cell_p[idx][1] = p01;
  fg_data->cell_p[idx][2] = p02;
  fg_data->cell_p[idx][3] = p11;
  fg_data->cell_p[idx][4] = p12;
  fg_data->cell_p[idx][5] = p22;

}

// -------------------------------------------------------------------------
// State seeding and register outputs
// -------------------------------------------------------------------------
static void FuelGauge_Seed(const Peripheral_Data_t *pd, float current_a)
{
  // SOC is seeded by inverting the present cell voltage through the OCV
  // table, which assumes a relaxed cell. If the pack is reset under load the
  // IR drop corrupts the seed — that is exactly the error the filter is meant
  // to correct out over the following seconds.
  bool         charge = FuelGauge_SelectBranch(current_a, true);

  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    const float *ocv;
    if(charge)
    {
      ocv = fg_data->cell[i].ocv_chg;
    }
    else
    {
      ocv = fg_data->cell[i].ocv_dis;
    }

    fg_data->cell_soc_f[i] = FuelGauge_OcvToSoc(ocv, pd->cell_voltage[i] / 1000.0f);
    fg_data->cell_vrc1[i]  = 0.0f;
    fg_data->cell_vrc2[i]  = 0.0f;

    fg_data->cell_p[i][0] = FG_P0_SOC;
    fg_data->cell_p[i][1] = 0.0f;
    fg_data->cell_p[i][2] = 0.0f;
    fg_data->cell_p[i][3] = FG_P0_VRC;
    fg_data->cell_p[i][4] = 0.0f;
    fg_data->cell_p[i][5] = FG_P0_VRC;

    fg_data->cell_soc_shown_f[i] = fg_data->cell_soc_f[i];
  }

  fg_branch_charge = charge;
  fg_seeded        = true;
  fg_full_ms       = 0;
  fg_full_done     = false;
  FuelGauge_FullReset();
  FuelGauge_RestReset();
  FuelGauge_CapacityReset();

  FuelGauge_UpdateOutputs();
}
bool FuelGauge_IsReady(void)
{
  // The model is loaded by the host: tables learned in both directions, a rising SOC grid,
  // and the filter values the update divides by or scales with
  if((fg_data->learning_status & FG_LEARN_COMPLETE) == 0U)
  {
    return false;
  }
  if(fg_data->kf_r_v <= 0.0f || fg_data->coulombic_efficiency <= 0.0f)
  {
    return false;
  }
  if(fg_data->kf_q_soc <= 0.0f || fg_data->kf_q_rc1 <= 0.0f || fg_data->kf_q_rc2 <= 0.0f)
  {
    return false;
  }
  for(uint8_t i = 1; i < FG_TABLE_POINTS; i++)
  {
    if(fg_data->soc_grid[i] <= fg_data->soc_grid[i - 1])
    {
      return false;
    }
  }
  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    if(fg_data->cell[i].capacity_ah <= 0.0f)
    {
      return false;
    }
  }

  // Temperature model: real model temperatures, rising setpoints, positive capacity factors
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
  // Linear between the setpoints, held beyond the first and the last
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
  // Without a temperature the tables are used as they are
  if(!isfinite(temp_c) || temp_c + FG_KELVIN_OFFSET <= 0.0f)
  {
    FuelGauge_ClearTempScale(&fg_temp_scale_dis);
    FuelGauge_ClearTempScale(&fg_temp_scale_chg);
    fg_temp_capacity = 1.0f;
    return;
  }

  // Each direction scales from the temperature its own HPPC test ran at
  FuelGauge_SetTempScale(&fg_temp_scale_dis, temp_c, fg_data->model_temp_dis_c);
  FuelGauge_SetTempScale(&fg_temp_scale_chg, temp_c, fg_data->model_temp_chg_c);
  fg_temp_capacity = FuelGauge_TempCapacity(temp_c);
}
static void FuelGauge_UpdateOutputs(void)
{
  uint8_t min_soc = 100;

  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    // Cells not in use read 0 and are left out of the pack SOC
    if(i >= fg_config->cell_count)
    {
      fg_data->cell_soc[i]                = 0;
      fg_data->cell_remaining_capacity[i] = 0;
      continue;
    }

    float soc_pct = fg_data->cell_soc_shown_f[i] * 100.0f;

    if(soc_pct <   0.0f) soc_pct =   0.0f;
    if(soc_pct > 100.0f) soc_pct = 100.0f;

    fg_data->cell_soc[i] = (uint8_t)(soc_pct + 0.5f);

    if(fg_data->cell_soc[i] < min_soc) min_soc = fg_data->cell_soc[i];

    // The register is 16 bits, a larger value would not fit the cast
    // What can be taken out at the present temperature: a cold cell gives less
    float remaining_mah = (soc_pct / 100.0f) * fg_data->cell[i].capacity_ah * fg_temp_capacity * 1000.0f + 0.5f;
    if(remaining_mah > FG_REMAINING_MAX_MAH) remaining_mah = FG_REMAINING_MAX_MAH;
    if(remaining_mah <     0.0f) remaining_mah =     0.0f;

    fg_data->cell_remaining_capacity[i] = (uint16_t)remaining_mah;
  }

  // The weakest cell governs the pack — the conservative convention, since
  // that cell hits its cut-off first.
  fg_data->relative_soc = min_soc;
}

// -------------------------------------------------------------------------
// Full-charge and rest corrections, shown SOC
// -------------------------------------------------------------------------
static void FuelGauge_Anchor(uint8_t idx, float soc, float weight)
{
  // Moves the true SOC of one cell toward a trusted value. The RC voltages are not touched,
  // the SOC uncertainty shrinks by the same weight and its links to the RC states are dropped.
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
}
static void FuelGauge_UpdateShown(const float *soc_before, float current_a)
{
  // The shown SOC is the true SOC plus a gap. At rest the shown SOC holds, so a rest correction
  // of the true SOC opens the gap. While current flows the gap shrinks in proportion to the
  // true SOC, so it is spread over the rest of the charge or discharge:
  //   discharging: gap/true stays constant, both reach 0 together
  //   charging:    gap/(1 - true) stays constant, both reach 1 together
  // The gap only shrinks (ratio at most 1), so filter noise cannot grow it. After a full charge
  // the full cells show 100 % until the next discharge starts.
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

    // At rest, and on a full cell until the next discharge, the shown SOC holds
    if(fabsf(current_a) < FG_CURRENT_DEADZONE_A || fg_full_cell[i])
    {
      continue;
    }

    if(current_a < 0.0f)
    {
      // At 0 the ratio is undefined: the gap holds
      if(soc_old > 0.0f)
      {
        ratio = soc_new / soc_old;
      }
    }
    else
    {
      // At 1 (just after a full charge) the ratio is undefined: the gap holds
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

  // A discharge ends the charge: the next one may set 100 % again, the shown SOC moves again
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

  // The highest cell and the cells close to it are full. A cell left lower by imbalance keeps its
  // own SOC and must not show 100 %. The filter may move a full cell's SOC down again afterwards,
  // its shown SOC stays at 100 % and the gap is spread over the next discharge.
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

  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    fg_rest_v_sum[i] = 0.0f;
  }
}
static void FuelGauge_CheckRest(const Peripheral_Data_t *pd, bool at_rest, uint32_t elapsed_ms)
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
  }

  // End of a window: the cells are relaxed when no cell's average moved more than the limit
  if(fg_rest_window_ms >= FG_REST_WINDOW_MS)
  {
    float window_s  = (float)fg_rest_window_ms / 1000.0f;
    float dvdt_max  = 0.0f;

    for(uint8_t i = 0; i < fg_config->cell_count; i++)
    {
      float avg = fg_rest_v_sum[i] / (float)fg_rest_samples;

      if(fg_rest_have_prev)
      {
        float dvdt = fabsf(avg - fg_rest_v_prev[i]) / window_s;
        if(dvdt > dvdt_max) dvdt_max = dvdt;
      }
      fg_rest_v_prev[i] = avg;
      fg_rest_v_sum[i]  = 0.0f;
    }

    if(fg_rest_have_prev)
    {
      fg_rest_relaxed = (dvdt_max < FG_REST_DVDT_MAX_V_PER_S);
    }
    fg_rest_have_prev = true;
    fg_rest_window_ms = 0;
    fg_rest_samples   = 0;
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
  fg_cap_updated = false;

  // One correction per rest, on the OCV branch of the last current direction. The shown SOC is
  // left alone: the gap is spread over the next charge or discharge.
  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    const float *ocv;
    float        soc_ocv;
    float        slope_mv_per_pct;
    float        weight;

    if(fg_branch_charge)
    {
      ocv = fg_data->cell[i].ocv_chg;
    }
    else
    {
      ocv = fg_data->cell[i].ocv_dis;
    }

    soc_ocv          = FuelGauge_OcvToSoc(ocv, pd->cell_voltage[i] / 1000.0f);
    slope_mv_per_pct = FuelGauge_LookupSlope(ocv, soc_ocv) * 1000.0f / 100.0f;   // V per SOC fraction -> mV per %

    // Only a settled rest on a steep part of the OCV curve is a capacity learning point
    if(fg_rest_relaxed && slope_mv_per_pct >= FG_CAP_SLOPE_MIN_MV_PER_PCT && isfinite(soc_ocv))
    {
      FuelGauge_CapacityLearn(i, soc_ocv);
    }

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

    // Relaxed: the RC voltages have decayed
    fg_data->cell_vrc1[i] = 0.0f;
    fg_data->cell_vrc2[i] = 0.0f;
  }

  if(fg_cap_updated)
  {
    FuelGauge_CapacityApply();
  }
}

// -------------------------------------------------------------------------
// Capacity learning
// -------------------------------------------------------------------------
static void FuelGauge_CapacityReset(void)
{
  // Drops every learning point: the next settled rest starts a new span
  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    fg_cap_valid[i] = false;
  }
}
static void FuelGauge_CapacityCount(const Peripheral_Data_t *pd, float current_a, uint32_t elapsed_ms)
{
  float temp_c = pd->temperature_package;
  float cell_a;

  // Outside the temperature window, without a temperature or with stale measurements the
  // capacity is not measured: the span starts again
  if(!isfinite(temp_c) || temp_c < FG_CAP_TEMP_MIN_C || temp_c > FG_CAP_TEMP_MAX_C ||
     Error_IsSet(OPENBMS_ADS131M08_DATA_LOST))
  {
    FuelGauge_CapacityReset();
  }

  fg_cap_ms += elapsed_ms;

  // Every current is counted, also inside FG_CURRENT_DEADZONE_A: a small standby load over a
  // long span adds up. The deadzone only decides what is rest. The charging efficiency applies
  // to the charge current only; a balancing cell's bleed leaves the cell completely.
  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    cell_a = current_a;
    if(current_a > 0.0f)
    {
      cell_a = current_a * fg_data->coulombic_efficiency;
    }
    cell_a -= FuelGauge_BleedCurrent(pd, i);
    fg_cap_q_ah[i] += (double)(cell_a * ((float)elapsed_ms / 1000.0f) / 3600.0f);
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

    // Too small a span: keep the older point, the span may still grow
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

  // This rest is the start of the next span
  fg_cap_valid[idx]    = true;
  fg_cap_soc[idx]      = soc_ocv;
  fg_cap_q_start[idx]  = fg_cap_q_ah[idx];
  fg_cap_ms_start[idx] = fg_cap_ms;
}
static void FuelGauge_CapacityApply(void)
{
  // After a capacity update: the pack capacity is the weakest cell's, SOH against the factory capacity
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

// -------------------------------------------------------------------------
// Cycle count
// -------------------------------------------------------------------------
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

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------
void FuelGauge_Init(FuelGauge_Data_t *data, const Config_Data_t *config)
{
  fg_data           = data;
  fg_config         = config;
  // The model itself lives in Ctrl (fuelgauge_data) and comes from the host (later from the EEPROM)
  fg_last_tick      = SYS_GET_TICK();
  fg_init_tick      = fg_last_tick;
  fg_prev_current_a = 0.0f;
  memset(fg_prev_bleed_a, 0, sizeof(fg_prev_bleed_a));
  memset(fg_cap_q_ah, 0, sizeof(fg_cap_q_ah));
  fg_branch_charge  = true;
  fg_seeded         = false;
  fg_full_ms        = 0;
  fg_full_done      = false;
  FuelGauge_FullReset();
  FuelGauge_RestReset();
  FuelGauge_CapacityReset();
}
void FuelGauge_Run(void)
{
  Peripheral_Data_t  pd;
  FuelGauge_Branch_t branch_predict;
  FuelGauge_Branch_t branch_measure;

  uint32_t now         = SYS_GET_TICK();
  uint32_t elapsed_ms  = now - fg_last_tick;

  fg_last_tick = now;

  Periph_GetData(&pd);
  FuelGauge_UpdateTemperature(pd.temperature_package);

  // pack_current is in mA and positive while charging, matching the
  // sign convention of the HPPC tables.
  float i_now_a = pd.pack_current / 1000.0f;
  float dt_s    = (float)elapsed_ms / 1000.0f;

  if(!fg_seeded)
  {
    // Hold off until the measurement filters have settled — see
    // FG_SEED_DELAY_MS. SOC reads 0 until then.
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
    // Charge went uncounted: the capacity learning points are no longer usable
    FuelGauge_CapacityReset();
    fg_prev_current_a = i_now_a;
    return;
  }

  // The branch follows the pack current, so it is shared by every cell: the
  // prediction uses the current that flowed across the interval just past,
  // the correction the current measured now.
  bool predict_charge = FuelGauge_SelectBranch(fg_prev_current_a, fg_branch_charge);
  bool measure_charge = FuelGauge_SelectBranch(i_now_a, predict_charge);

  bool at_rest = (fabsf(fg_prev_current_a) < FG_CURRENT_DEADZONE_A) &&
                 (fabsf(i_now_a) < FG_CURRENT_DEADZONE_A);

  float soc_before[SYS_CELL_COUNT_MAX];
  memcpy(soc_before, fg_data->cell_soc_f, sizeof(soc_before));

  for(uint8_t i = 0; i < fg_config->cell_count; i++)
  {
    // Each cell is predicted and corrected with its own tables
    FuelGauge_GetBranch(predict_charge, i, &branch_predict);
    FuelGauge_GetBranch(measure_charge, i, &branch_measure);

    // A balancing cell also loses its bleed current, which the shunt does not see
    float bleed_a = FuelGauge_BleedCurrent(&pd, i);

    FuelGauge_UpdateCell(i, dt_s, fg_prev_current_a - fg_prev_bleed_a[i], i_now_a - bleed_a,
                         pd.cell_voltage[i] / 1000.0f,
                         &branch_predict, &branch_measure, at_rest);

    fg_prev_bleed_a[i] = bleed_a;
  }

  fg_branch_charge  = measure_charge;
  fg_prev_current_a = i_now_a;

  // The shown SOC follows what the filter did this step, then the corrections may move the true SOC
  FuelGauge_UpdateShown(soc_before, i_now_a);
  FuelGauge_CapacityCount(&pd, i_now_a, elapsed_ms);
  FuelGauge_CycleCount(i_now_a, elapsed_ms);
  FuelGauge_CheckFull(&pd, i_now_a, elapsed_ms);
  FuelGauge_CheckRest(&pd, at_rest, elapsed_ms);

  FuelGauge_UpdateOutputs();
}
bool FuelGauge_SetData(const FuelGauge_Data_t *data)
{
  // Every float of the model and the filter state (soc_grid up to cycle_count) has to be a
  // real number, one NaN or inf would end up in the SOC
  uint32_t count = (offsetof(FuelGauge_Data_t, cycle_count) - offsetof(FuelGauge_Data_t, soc_grid)) / sizeof(float);

  if(!FuelGauge_AllFinite(data->soc_grid, count))
  {
    return false;
  }

  memcpy(fg_data, data, sizeof(FuelGauge_Data_t));
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

  // A table with a NaN or inf is not taken
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
  // Called by the HPPC extraction once the tables of a finished test are written
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
  fg_data->learning_status       = 0;
  fg_data->capacity_update_count = 0;
}

