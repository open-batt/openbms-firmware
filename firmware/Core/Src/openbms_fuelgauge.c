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
  * which hold the HPPC-derived lookup tables the Python pipeline produces.
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
  *   - One shared parameter table serves all cells, scaled per cell by
  *     cell_q_nom_ah[] and cell_r0_scale[]; the Python pipeline keeps a full
  *     table per cell. Measured cell-to-cell R0 spread is only ~0.4%.
  *
  ******************************************************************************
  */

#include "openbms_fuelgauge.h"
#include "openbms_periph.h"

#include <math.h>
#include <string.h>
#include <stddef.h>

#define FG_CELL_COUNT                   (CELL_NUMBER_DEFAULT)
#define FG_TABLE_POINTS                 (20)

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
_Static_assert(offsetof(FuelGauge_Data_t, cell_r0_scale) % 4 == 0,
               "cell_r0_scale misaligned — a float member above it changed size");

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

// A gap longer than this means the estimator was not running — config mode,
// a stall, or the first pass after boot. Integrating the current we happen to
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

typedef struct
{
  const float *ocv;
  const float *r0;
  const float *r1;
  const float *r2;
  const float *tau1;
  const float *tau2;

} FuelGauge_Branch_t;

static FuelGauge_Data_t   fuelgauge_data      = {0};
static uint32_t           fg_last_tick        = 0;
static uint32_t           fg_init_tick        = 0;
static float              fg_prev_current_a   = 0.0f;
static bool               fg_branch_charge    = true;
static bool               fg_seeded           = false;

static void  FuelGauge_SetDefaults(void);
static void  FuelGauge_Seed(const Peripheral_Data_t *pd, float current_a);
static void  FuelGauge_UpdateOutputs(void);
static bool  FuelGauge_UpdateCell(uint8_t idx, float dt_s, float i_prev_a, float i_now_a,
                                  float v_meas_v, const FuelGauge_Branch_t *predict,
                                  const FuelGauge_Branch_t *measure, bool at_rest);

// -------------------------------------------------------------------------
// Table lookups
// -------------------------------------------------------------------------
static uint8_t FuelGauge_Segment(float soc_pct, float *frac)
{
  // Returns the upper index of the SOC grid segment containing soc_pct, and
  // the 0..1 position within it. Outside the table the value clamps to the
  // nearest end point.
  const float *grid = fuelgauge_data.soc_grid;
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
  *frac = (span > 0.0f) ? ((soc_pct - grid[i - 1]) / span) : 0.0f;

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

  span_frac = (fuelgauge_data.soc_grid[i] - fuelgauge_data.soc_grid[i - 1]) / 100.0f;

  if(span_frac <= 0.0f) return 0.0f;

  return (table[i] - table[i - 1]) / span_frac;
}
static float FuelGauge_OcvToSoc(const float *ocv, float voltage_v)
{
  // Invert the OCV curve to seed SOC from a resting voltage. The table is
  // monotonically rising, so a forward scan is enough.
  const float *grid = fuelgauge_data.soc_grid;
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
      frac = (span > 0.0f) ? ((voltage_v - ocv[i - 1]) / span) : 0.0f;

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
static void FuelGauge_GetBranch(bool charge, FuelGauge_Branch_t *branch)
{
  if(charge)
  {
    branch->ocv  = fuelgauge_data.ocv_chg;
    branch->r0   = fuelgauge_data.r0_chg;
    branch->r1   = fuelgauge_data.r1_chg;
    branch->r2   = fuelgauge_data.r2_chg;
    branch->tau1 = fuelgauge_data.tau1_chg;
    branch->tau2 = fuelgauge_data.tau2_chg;
  }
  else
  {
    branch->ocv  = fuelgauge_data.ocv_dis;
    branch->r0   = fuelgauge_data.r0_dis;
    branch->r1   = fuelgauge_data.r1_dis;
    branch->r2   = fuelgauge_data.r2_dis;
    branch->tau1 = fuelgauge_data.tau1_dis;
    branch->tau2 = fuelgauge_data.tau2_dis;
  }
}

// -------------------------------------------------------------------------
// Extended Kalman Filter — one cell, one step
// -------------------------------------------------------------------------
static bool FuelGauge_UpdateCell(uint8_t idx, float dt_s, float i_prev_a, float i_now_a,
                                 float v_meas_v, const FuelGauge_Branch_t *predict,
                                 const FuelGauge_Branch_t *measure, bool at_rest)
{
  float soc  = fuelgauge_data.cell_soc_f[idx];
  float vrc1 = fuelgauge_data.cell_vrc1[idx];
  float vrc2 = fuelgauge_data.cell_vrc2[idx];

  // Covariance is stored as the upper triangle [P00, P01, P02, P11, P12, P22].
  float p00 = fuelgauge_data.cell_p[idx][0];
  float p01 = fuelgauge_data.cell_p[idx][1];
  float p02 = fuelgauge_data.cell_p[idx][2];
  float p11 = fuelgauge_data.cell_p[idx][3];
  float p12 = fuelgauge_data.cell_p[idx][4];
  float p22 = fuelgauge_data.cell_p[idx][5];

  float capacity_ah = fuelgauge_data.cell_q_nom_ah[idx];
  bool  gated       = false;

  if(capacity_ah < FG_MIN_CAPACITY_AH) capacity_ah = FG_MIN_CAPACITY_AH;

  // ---------------- predict ----------------
  // Parameters are looked up at the previous SOC, before coulomb counting
  // advances it — matching the reference implementation.
  float tau1 = FuelGauge_Lookup(predict->tau1, soc);
  float tau2 = FuelGauge_Lookup(predict->tau2, soc);
  float r1   = FuelGauge_Lookup(predict->r1, soc);
  float r2   = FuelGauge_Lookup(predict->r2, soc);

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
  p00 = p00 + fuelgauge_data.kf_q_soc;
  p01 = a1 * p01;
  p02 = a2 * p02;
  p11 = a1 * a1 * p11 + fuelgauge_data.kf_q_rc1;
  p12 = a1 * a2 * p12;
  p22 = a2 * a2 * p22 + fuelgauge_data.kf_q_rc2;

  // ---------------- measurement ----------------
  float r0     = FuelGauge_Lookup(measure->r0, soc) * fuelgauge_data.cell_r0_scale[idx];
  float v_pred = FuelGauge_Lookup(measure->ocv, soc) + i_now_a * r0 + vrc1 + vrc2;
  float h0     = FuelGauge_LookupSlope(measure->ocv, soc);

  // S = H*P*H' + R, with H = [dOCV/dSOC, 1, 1]
  float s = h0 * h0 * p00 + 2.0f * h0 * p01 + 2.0f * h0 * p02
            + p11 + 2.0f * p12 + p22 + fuelgauge_data.kf_r_v;

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
  float k0_gain = at_rest ? 0.0f : k0;

  float nis = (y * y) / s;

  if(nis > FG_GATE_SIGMA_SQ)
  {
    // Statistical outlier — ignore the measurement entirely this step. No
    // mean update and no covariance shrink; P still evolves via Q next step.
    gated = true;
  }
  else
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
      gated = true;
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
    float r_v = fuelgauge_data.kf_r_v;

    p00 = n00 * mm00 + n01 * mm01 + n02 * mm02 + k0_gain * k0_gain * r_v;
    p01 = n00 * mm10 + n01 * mm11 + n02 * mm12 + k0_gain * k1 * r_v;
    p02 = n00 * mm20 + n01 * mm21 + n02 * mm22 + k0_gain * k2 * r_v;
    p11 = n10 * mm10 + n11 * mm11 + n12 * mm12 + k1 * k1 * r_v;
    p12 = n10 * mm20 + n11 * mm21 + n12 * mm22 + k1 * k2 * r_v;
    p22 = n20 * mm20 + n21 * mm21 + n22 * mm22 + k2 * k2 * r_v;
  }

  if(soc < FG_SOC_MIN) soc = FG_SOC_MIN;
  if(soc > FG_SOC_MAX) soc = FG_SOC_MAX;

  fuelgauge_data.cell_soc_f[idx] = soc;
  fuelgauge_data.cell_vrc1[idx]  = vrc1;
  fuelgauge_data.cell_vrc2[idx]  = vrc2;

  fuelgauge_data.cell_p[idx][0] = p00;
  fuelgauge_data.cell_p[idx][1] = p01;
  fuelgauge_data.cell_p[idx][2] = p02;
  fuelgauge_data.cell_p[idx][3] = p11;
  fuelgauge_data.cell_p[idx][4] = p12;
  fuelgauge_data.cell_p[idx][5] = p22;

  return gated;
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
  const float *ocv    = charge ? fuelgauge_data.ocv_chg : fuelgauge_data.ocv_dis;

  for(uint8_t i = 0; i < FG_CELL_COUNT; i++)
  {
    fuelgauge_data.cell_soc_f[i] = FuelGauge_OcvToSoc(ocv, pd->cell_voltage_filtered[i] / 1000.0f);
    fuelgauge_data.cell_vrc1[i]  = 0.0f;
    fuelgauge_data.cell_vrc2[i]  = 0.0f;

    fuelgauge_data.cell_p[i][0] = FG_P0_SOC;
    fuelgauge_data.cell_p[i][1] = 0.0f;
    fuelgauge_data.cell_p[i][2] = 0.0f;
    fuelgauge_data.cell_p[i][3] = FG_P0_VRC;
    fuelgauge_data.cell_p[i][4] = 0.0f;
    fuelgauge_data.cell_p[i][5] = FG_P0_VRC;
  }

  fg_branch_charge = charge;
  fg_seeded        = true;

  fuelgauge_data.learning_status |= FG_STATUS_SEEDED;

  FuelGauge_UpdateOutputs();
}
static void FuelGauge_UpdateOutputs(void)
{
  uint8_t min_soc = 100;

  for(uint8_t i = 0; i < FG_CELL_COUNT; i++)
  {
    float soc_pct = fuelgauge_data.cell_soc_f[i] * 100.0f;

    if(soc_pct <   0.0f) soc_pct =   0.0f;
    if(soc_pct > 100.0f) soc_pct = 100.0f;

    fuelgauge_data.cell_soc[i] = (uint8_t)(soc_pct + 0.5f);

    if(fuelgauge_data.cell_soc[i] < min_soc) min_soc = fuelgauge_data.cell_soc[i];

    fuelgauge_data.cell_remaining_capacity[i] =
        (uint16_t)((soc_pct / 100.0f) * fuelgauge_data.cell_q_nom_ah[i] * 1000.0f + 0.5f);
  }

  // The weakest cell governs the pack — the conservative convention, since
  // that cell hits its cut-off first.
  fuelgauge_data.relative_soc = min_soc;

  // Nothing separates the two until SOH learning exists to produce a learned
  // capacity that differs from the design capacity.
  fuelgauge_data.absolute_soc = min_soc;
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------
void FuelGauge_Init(void)
{
  FuelGauge_SetDefaults();

  fg_last_tick      = HAL_GetTick();
  fg_init_tick      = fg_last_tick;
  fg_prev_current_a = 0.0f;
  fg_branch_charge  = true;
  fg_seeded         = false;
}
void FuelGauge_Run(void)
{
  Peripheral_Data_t  pd;
  FuelGauge_Branch_t branch_predict;
  FuelGauge_Branch_t branch_measure;

  uint32_t now         = HAL_GetTick();
  uint32_t elapsed_ms  = now - fg_last_tick;

  if(elapsed_ms < FG_UPDATE_PERIOD_MS) return;

  fg_last_tick = now;

  Periph_GetData(&pd);

  // pack_current_filtered is in mA and positive while charging, matching the
  // sign convention of the HPPC tables.
  float i_now_a = pd.pack_current_filtered / 1000.0f;
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
  bool gated   = false;

  FuelGauge_GetBranch(predict_charge, &branch_predict);
  FuelGauge_GetBranch(measure_charge, &branch_measure);

  for(uint8_t i = 0; i < FG_CELL_COUNT; i++)
  {
    gated |= FuelGauge_UpdateCell(i, dt_s, fg_prev_current_a, i_now_a,
                                  pd.cell_voltage_filtered[i] / 1000.0f,
                                  &branch_predict, &branch_measure, at_rest);
  }

  fg_branch_charge  = measure_charge;
  fg_prev_current_a = i_now_a;

  if(gated)   fuelgauge_data.learning_status |=  FG_STATUS_GATED;
  else        fuelgauge_data.learning_status &= ~FG_STATUS_GATED;

  if(at_rest) fuelgauge_data.learning_status |=  FG_STATUS_AT_REST;
  else        fuelgauge_data.learning_status &= ~FG_STATUS_AT_REST;

  FuelGauge_UpdateOutputs();
}
void FuelGauge_GetData(FuelGauge_Data_t *data)
{
  memcpy(data, &fuelgauge_data, sizeof(FuelGauge_Data_t));
}
void FuelGauge_SetData(const FuelGauge_Data_t *data)
{
  memcpy(&fuelgauge_data, data, sizeof(FuelGauge_Data_t));
}

// -------------------------------------------------------------------------
// Defaults
//
// Generated from the HPPC pipeline output in python_scripts/results/:
// hppc_soc_table_all_cells_{charge,discharge}.csv and kalman_parameters.csv.
// The 7 per-cell tables are averaged into the single shared table the
// register map holds; per-cell deviation is carried by cell_q_nom_ah[] and
// cell_r0_scale[]. A host can overwrite any of this over UART at runtime.
// -------------------------------------------------------------------------
static void FuelGauge_SetDefaults(void)
{
  // -------------------------------------------------------------------------
  // 0x86 — SOC grid (%)
  //
  // Non-uniform on purpose: denser at both ends, where the OCV curve is
  // steepest and piecewise-linear interpolation costs the most accuracy.
  // -------------------------------------------------------------------------
  static const float soc_grid_vals[20] = {
      2.0f,  4.0f,  6.0f,  8.0f,  10.0f,
     12.0f, 15.0f, 20.0f, 25.0f,  30.0f,
     40.0f, 50.0f, 60.0f, 70.0f,  80.0f,
     85.0f, 90.0f, 95.0f, 98.0f, 100.0f
  };
  memcpy(fuelgauge_data.soc_grid, soc_grid_vals, sizeof(soc_grid_vals));

  // -------------------------------------------------------------------------
  // 0x87 — OCV discharge (V)
  // -------------------------------------------------------------------------
  static const float ocv_dis_vals[20] = {
      3.4575f, 3.4686f, 3.4770f, 3.4870f, 3.5041f,
      3.5262f, 3.5556f, 3.5885f, 3.6091f, 3.6246f,
      3.6551f, 3.6970f, 3.7682f, 3.8705f, 3.9674f,
      4.0203f, 4.0761f, 4.1349f, 4.1740f, 4.2068f
  };
  memcpy(fuelgauge_data.ocv_dis, ocv_dis_vals, sizeof(ocv_dis_vals));

  // -------------------------------------------------------------------------
  // 0x88 — OCV charge (V)
  // -------------------------------------------------------------------------
  static const float ocv_chg_vals[20] = {
      3.3798f, 3.4567f, 3.4731f, 3.4814f, 3.4909f,
      3.5061f, 3.5354f, 3.5827f, 3.6096f, 3.6250f,
      3.6528f, 3.6926f, 3.7686f, 3.8496f, 3.9456f,
      3.9977f, 4.0529f, 4.1107f, 4.1497f, 4.2000f
  };
  memcpy(fuelgauge_data.ocv_chg, ocv_chg_vals, sizeof(ocv_chg_vals));

  // -------------------------------------------------------------------------
  // 0x89 — R0 discharge (Ω)
  // -------------------------------------------------------------------------
  static const float r0_dis_vals[20] = {
      0.023994f, 0.023192f, 0.022521f, 0.021976f, 0.021442f,
      0.021031f, 0.020660f, 0.020315f, 0.020012f, 0.019739f,
      0.019547f, 0.019609f, 0.019894f, 0.020147f, 0.020227f,
      0.020339f, 0.020501f, 0.020602f, 0.020576f, 0.020576f
  };
  memcpy(fuelgauge_data.r0_dis, r0_dis_vals, sizeof(r0_dis_vals));

  // -------------------------------------------------------------------------
  // 0x8A — R1 discharge (Ω)
  // -------------------------------------------------------------------------
  static const float r1_dis_vals[20] = {
      0.017715f, 0.015254f, 0.012777f, 0.010933f, 0.010027f,
      0.009823f, 0.009857f, 0.009926f, 0.009915f, 0.009904f,
      0.009879f, 0.009788f, 0.009818f, 0.010382f, 0.011271f,
      0.011418f, 0.010614f, 0.009461f, 0.008702f, 0.008702f
  };
  memcpy(fuelgauge_data.r1_dis, r1_dis_vals, sizeof(r1_dis_vals));

  // -------------------------------------------------------------------------
  // 0x8B — tau1 discharge (s)
  // -------------------------------------------------------------------------
  static const float tau1_dis_vals[20] = {
      13.162f, 12.901f, 12.676f, 12.867f, 13.646f,
      14.528f, 15.448f, 16.115f, 16.375f, 16.575f,
      16.426f, 15.289f, 13.780f, 13.560f, 14.673f,
      15.008f, 14.148f, 13.091f, 12.476f, 12.476f
  };
  memcpy(fuelgauge_data.tau1_dis, tau1_dis_vals, sizeof(tau1_dis_vals));

  // -------------------------------------------------------------------------
  // 0x8C — R2 discharge (Ω)
  // -------------------------------------------------------------------------
  static const float r2_dis_vals[20] = {
      0.009273f, 0.007326f, 0.005910f, 0.005646f, 0.006740f,
      0.008318f, 0.010173f, 0.011675f, 0.011731f, 0.010602f,
      0.009018f, 0.008587f, 0.009361f, 0.009283f, 0.007041f,
      0.004563f, 0.003100f, 0.002485f, 0.002314f, 0.002314f
  };
  memcpy(fuelgauge_data.r2_dis, r2_dis_vals, sizeof(r2_dis_vals));

  // -------------------------------------------------------------------------
  // 0x8D — tau2 discharge (s)
  // -------------------------------------------------------------------------
  static const float tau2_dis_vals[20] = {
      105.446f, 106.610f, 107.807f, 109.516f, 113.634f,
      119.202f, 125.790f, 129.958f, 128.596f, 127.294f,
      132.877f, 142.892f, 144.148f, 131.343f, 115.459f,
      107.425f, 103.355f,  99.141f,  95.629f,  95.629f
  };
  memcpy(fuelgauge_data.tau2_dis, tau2_dis_vals, sizeof(tau2_dis_vals));

  // -------------------------------------------------------------------------
  // 0x8E — R0 charge (Ω)
  // -------------------------------------------------------------------------
  static const float r0_chg_vals[20] = {
      0.036460f, 0.032986f, 0.030064f, 0.028226f, 0.026913f,
      0.026225f, 0.025413f, 0.024115f, 0.022956f, 0.022369f,
      0.021716f, 0.020868f, 0.020283f, 0.019682f, 0.019479f,
      0.019483f, 0.019511f, 0.019569f, 0.019594f, 0.019608f
  };
  memcpy(fuelgauge_data.r0_chg, r0_chg_vals, sizeof(r0_chg_vals));

  // -------------------------------------------------------------------------
  // 0x8F — R1 charge (Ω)
  // -------------------------------------------------------------------------
  static const float r1_chg_vals[20] = {
      0.012893f, 0.012238f, 0.011584f, 0.011235f, 0.010940f,
      0.010623f, 0.010260f, 0.009872f, 0.009821f, 0.009906f,
      0.009823f, 0.009598f, 0.009890f, 0.010816f, 0.012165f,
      0.012716f, 0.012680f, 0.011867f, 0.011514f, 0.011474f
  };
  memcpy(fuelgauge_data.r1_chg, r1_chg_vals, sizeof(r1_chg_vals));

  // -------------------------------------------------------------------------
  // 0x90 — tau1 charge (s)
  // -------------------------------------------------------------------------
  static const float tau1_chg_vals[20] = {
      13.370f, 13.393f, 13.799f, 14.615f, 15.337f,
      15.468f, 15.354f, 14.927f, 14.637f, 14.623f,
      14.637f, 14.407f, 14.428f, 15.051f, 16.097f,
      16.456f, 16.278f, 15.494f, 15.174f, 15.148f
  };
  memcpy(fuelgauge_data.tau1_chg, tau1_chg_vals, sizeof(tau1_chg_vals));

  // -------------------------------------------------------------------------
  // 0x91 — R2 charge (Ω)
  // -------------------------------------------------------------------------
  static const float r2_chg_vals[20] = {
      0.008274f, 0.007936f, 0.007691f, 0.007867f, 0.008243f,
      0.008246f, 0.007928f, 0.007153f, 0.006756f, 0.007134f,
      0.008360f, 0.008886f, 0.008193f, 0.007774f, 0.009071f,
      0.010550f, 0.011611f, 0.011670f, 0.011446f, 0.011411f
  };
  memcpy(fuelgauge_data.r2_chg, r2_chg_vals, sizeof(r2_chg_vals));

  // -------------------------------------------------------------------------
  // 0x92 — tau2 charge (s)
  // -------------------------------------------------------------------------
  static const float tau2_chg_vals[20] = {
      136.349f, 134.047f, 130.108f, 125.350f, 121.328f,
      119.935f, 119.468f, 120.058f, 120.612f, 120.491f,
      120.804f, 124.349f, 128.722f, 129.472f, 127.006f,
      126.493f, 128.016f, 132.348f, 133.844f, 134.035f
  };
  memcpy(fuelgauge_data.tau2_chg, tau2_chg_vals, sizeof(tau2_chg_vals));

  // -------------------------------------------------------------------------
  // 0x93 — Q_nom temperature setpoints
  // 0x94 — Q_nom capacity at each temperature
  //
  // Reserved: the HPPC pipeline runs at room temperature only, so there is no
  // fitted capacity-vs-temperature curve yet and the estimator does not
  // derate. Left flat at the nominal capacity rather than carrying invented
  // numbers that would look like measurements.
  // -------------------------------------------------------------------------
  fuelgauge_data.q_nom_temp_c[0]                = -20.0f;
  fuelgauge_data.q_nom_temp_c[1]                = -10.0f;
  fuelgauge_data.q_nom_temp_c[2]                =   0.0f;
  fuelgauge_data.q_nom_temp_c[3]                =  25.0f;
  fuelgauge_data.q_nom_temp_c[4]                =  45.0f;

  for(uint8_t i = 0; i < 5; i++)
  {
    fuelgauge_data.q_nom_temp_ah[i]             = 2.0169f;
  }

  // -------------------------------------------------------------------------
  // 0x95 — Nominal capacity at 25°C (mean of the 7 fitted cell capacities)
  // 0x96 — Coulombic efficiency
  //
  // Coulombic efficiency is stored for completeness but is not applied: the
  // reference estimator does not use it either.
  // -------------------------------------------------------------------------
  fuelgauge_data.q_nom_ah                       = 2.0169f;
  fuelgauge_data.coulombic_efficiency           = 0.998f;

  // -------------------------------------------------------------------------
  // 0x97-0x9B — Reference R/tau at 25°C
  //
  // Mid-SOC values from the discharge table, for reference only — the
  // estimator interpolates the full tables above rather than using these.
  // -------------------------------------------------------------------------
  fuelgauge_data.r0_ref                         =   0.019609f;
  fuelgauge_data.r1_ref                         =   0.009788f;
  fuelgauge_data.tau1_ref                       =  15.289f;
  fuelgauge_data.r2_ref                         =   0.008587f;
  fuelgauge_data.tau2_ref                       = 142.892f;

  // -------------------------------------------------------------------------
  // 0x9C-0xA0 — Arrhenius activation energies
  //
  // Reserved for temperature compensation, which is not implemented. Left at
  // zero so nothing reads them as fitted values.
  // -------------------------------------------------------------------------
  fuelgauge_data.ea_r0                          = 0.0f;
  fuelgauge_data.ea_r1                          = 0.0f;
  fuelgauge_data.ea_tau1                        = 0.0f;
  fuelgauge_data.ea_r2                          = 0.0f;
  fuelgauge_data.ea_tau2                        = 0.0f;

  // -------------------------------------------------------------------------
  // 0xA1 — Kalman process noise SOC
  // 0xA2 — Kalman process noise RC1
  // 0xA3 — Kalman process noise RC2
  // 0xA4 — Kalman measurement noise (0.005 V sensor sigma, squared)
  // -------------------------------------------------------------------------
  fuelgauge_data.kf_q_soc                       = 1e-6f;
  fuelgauge_data.kf_q_rc1                       = 1e-6f;
  fuelgauge_data.kf_q_rc2                       = 1e-6f;
  fuelgauge_data.kf_r_v                         = 2.5e-5f;

  // -------------------------------------------------------------------------
  // 0xA9 — Per-cell capacity (Ah), fitted from the HPPC sweep
  // 0xAA — Per-cell R0 scale against the shared table (1.0 = nominal)
  // -------------------------------------------------------------------------
  static const float cell_capacity_vals[7] = {
      2.0562f, 2.0254f, 2.0139f, 2.0139f, 2.0157f, 2.0065f, 1.9869f
  };
  static const float cell_r0_scale_vals[7] = {
      0.9944f, 1.0027f, 1.0010f, 0.9974f, 1.0005f, 0.9999f, 1.0040f
  };
  memcpy(fuelgauge_data.cell_q_nom_ah,  cell_capacity_vals, sizeof(cell_capacity_vals));
  memcpy(fuelgauge_data.cell_r0_scale,  cell_r0_scale_vals, sizeof(cell_r0_scale_vals));

  // -------------------------------------------------------------------------
  // 0xA5-0xA8 — Filter state
  //
  // Zeroed here; FuelGauge_Seed() fills it from the first voltage reading.
  // -------------------------------------------------------------------------
  for(uint8_t i = 0; i < FG_CELL_COUNT; i++)
  {
    fuelgauge_data.cell_soc_f[i]                = 0.0f;
    fuelgauge_data.cell_vrc1[i]                 = 0.0f;
    fuelgauge_data.cell_vrc2[i]                 = 0.0f;

    fuelgauge_data.cell_p[i][0]                 = FG_P0_SOC;
    fuelgauge_data.cell_p[i][1]                 = 0.0f;
    fuelgauge_data.cell_p[i][2]                 = 0.0f;
    fuelgauge_data.cell_p[i][3]                 = FG_P0_VRC;
    fuelgauge_data.cell_p[i][4]                 = 0.0f;
    fuelgauge_data.cell_p[i][5]                 = FG_P0_VRC;

    // SOH stays at 100% until capacity learning exists to move it.
    fuelgauge_data.cell_soh[i]                  = 100;
  }

  // -------------------------------------------------------------------------
  // 0xAB — CycleCount
  // 0xAC — LearningStatus
  // -------------------------------------------------------------------------
  fuelgauge_data.cycle_count                    = 0;
  fuelgauge_data.learning_status                = 0x0000;
}
