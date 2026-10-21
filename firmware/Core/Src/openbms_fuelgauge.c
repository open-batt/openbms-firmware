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

#include <math.h>
#include <string.h>
#include <stddef.h>

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
static uint8_t            fg_cell_count       = 0;      // cells in use, from cfg.cell_count

static void  FuelGauge_SetDefaults(void);
static void  FuelGauge_Seed(const Peripheral_Data_t *pd, float current_a);
static void  FuelGauge_UpdateOutputs(void);
static bool  FuelGauge_AllFinite(const float *values, uint32_t count);
static bool  FuelGauge_UpdateCell(uint8_t idx, float dt_s, float i_prev_a, float i_now_a,
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
static void FuelGauge_GetBranch(bool charge, uint8_t cell, FuelGauge_Branch_t *branch)
{
  const FuelGauge_Cell_t *model = &fuelgauge_data.cell[cell];

  if(charge)
  {
    branch->ocv  = model->ocv_chg;
    branch->r0   = model->r0_chg;
    branch->r1   = model->r1_chg;
    branch->r2   = model->r2_chg;
    branch->tau1 = model->tau1_chg;
    branch->tau2 = model->tau2_chg;
  }
  else
  {
    branch->ocv  = model->ocv_dis;
    branch->r0   = model->r0_dis;
    branch->r1   = model->r1_dis;
    branch->r2   = model->r2_dis;
    branch->tau1 = model->tau1_dis;
    branch->tau2 = model->tau2_dis;
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

  float capacity_ah = fuelgauge_data.cell[idx].capacity_ah;
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
  float r0     = FuelGauge_Lookup(measure->r0, soc);
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

    fuelgauge_data.learning_status |= FG_STATUS_NAN_RESET;
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

  for(uint8_t i = 0; i < fg_cell_count; i++)
  {
    const float *ocv = charge ? fuelgauge_data.cell[i].ocv_chg : fuelgauge_data.cell[i].ocv_dis;

    fuelgauge_data.cell_soc_f[i] = FuelGauge_OcvToSoc(ocv, pd->cell_voltage[i] / 1000.0f);
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

  for(uint8_t i = 0; i < BD_CELL_COUNT_MAX; i++)
  {
    // Cells not in use read 0 and are left out of the pack SOC
    if(i >= fg_cell_count)
    {
      fuelgauge_data.cell_soc[i]                = 0;
      fuelgauge_data.cell_remaining_capacity[i] = 0;
      continue;
    }

    float soc_pct = fuelgauge_data.cell_soc_f[i] * 100.0f;

    if(soc_pct <   0.0f) soc_pct =   0.0f;
    if(soc_pct > 100.0f) soc_pct = 100.0f;

    fuelgauge_data.cell_soc[i] = (uint8_t)(soc_pct + 0.5f);

    if(fuelgauge_data.cell_soc[i] < min_soc) min_soc = fuelgauge_data.cell_soc[i];

    // The register is 16 bits, a larger value would not fit the cast
    float remaining_mah = (soc_pct / 100.0f) * fuelgauge_data.cell[i].capacity_ah * 1000.0f + 0.5f;
    if(remaining_mah > FG_REMAINING_MAX_MAH) remaining_mah = FG_REMAINING_MAX_MAH;
    if(remaining_mah <     0.0f) remaining_mah =     0.0f;

    fuelgauge_data.cell_remaining_capacity[i] = (uint16_t)remaining_mah;
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

  fg_last_tick      = BD_GET_TICK();
  fg_init_tick      = fg_last_tick;
  fg_prev_current_a = 0.0f;
  fg_branch_charge  = true;
  fg_seeded         = false;
}
void FuelGauge_Run(uint8_t cell_count)
{
  Peripheral_Data_t  pd;
  FuelGauge_Branch_t branch_predict;
  FuelGauge_Branch_t branch_measure;

  uint32_t now         = BD_GET_TICK();
  uint32_t elapsed_ms  = now - fg_last_tick;

  fg_last_tick = now;

  // A new cell count (config written by the host) starts the filter again from the cell voltages
  if(cell_count != fg_cell_count)
  {
    fg_cell_count = cell_count;
    fg_seeded     = false;
  }

  Periph_GetData(&pd);

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

  for(uint8_t i = 0; i < fg_cell_count; i++)
  {
    // Each cell is predicted and corrected with its own tables
    FuelGauge_GetBranch(predict_charge, i, &branch_predict);
    FuelGauge_GetBranch(measure_charge, i, &branch_measure);

    gated |= FuelGauge_UpdateCell(i, dt_s, fg_prev_current_a, i_now_a,
                                  pd.cell_voltage[i] / 1000.0f,
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
bool FuelGauge_SetData(const FuelGauge_Data_t *data)
{
  // Every float of the model and the filter state (soc_grid up to cycle_count) has to be a
  // real number, one NaN or inf would end up in the SOC
  uint32_t count = (offsetof(FuelGauge_Data_t, cycle_count) - offsetof(FuelGauge_Data_t, soc_grid)) / sizeof(float);

  if(!FuelGauge_AllFinite(data->soc_grid, count))
  {
    return false;
  }

  memcpy(&fuelgauge_data, data, sizeof(FuelGauge_Data_t));
  return true;
}
void FuelGauge_GetSocGrid(float *grid)
{
  memcpy(grid, fuelgauge_data.soc_grid, sizeof(fuelgauge_data.soc_grid));
}
void FuelGauge_SetCellTable(uint8_t cell, bool charge, const FuelGauge_Table_t *table)
{
  if(cell >= BD_CELL_COUNT_MAX)
  {
    return;
  }

  FuelGauge_Cell_t *c = &fuelgauge_data.cell[cell];

  // A table with a NaN or inf is not taken
  if(!FuelGauge_AllFinite(table->ocv,  FG_TABLE_POINTS) || !FuelGauge_AllFinite(table->r0,   FG_TABLE_POINTS) ||
     !FuelGauge_AllFinite(table->r1,   FG_TABLE_POINTS) || !FuelGauge_AllFinite(table->tau1, FG_TABLE_POINTS) ||
     !FuelGauge_AllFinite(table->r2,   FG_TABLE_POINTS) || !FuelGauge_AllFinite(table->tau2, FG_TABLE_POINTS))
  {
    return;
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
}
void FuelGauge_SetCellCapacity(uint8_t cell, float capacity_ah)
{
  if(cell >= BD_CELL_COUNT_MAX || !isfinite(capacity_ah))
  {
    return;
  }
  fuelgauge_data.cell[cell].capacity_ah = capacity_ah;
}

// -------------------------------------------------------------------------
// Defaults
//
// Values set at boot. The per-cell tables are not set here, they come from the
// HPPC extraction or from the host. A host can overwrite any of this over UART
// at runtime.
// -------------------------------------------------------------------------
static void FuelGauge_SetDefaults(void)
{
  // -------------------------------------------------------------------------
  // 0xA0 — SOC grid (%)
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
  // Per-cell model (fuelgauge_data.cell[]: OCV, R0, R1, R2, tau1, tau2 for
  // discharge and charge, and the capacity): no defaults here, the tables are
  // zero until the HPPC extraction or the host writes them (0x9F selects the
  // cell, 0xA1-0xAC the tables, 0xD4 the capacity).
  // -------------------------------------------------------------------------

  // -------------------------------------------------------------------------
  // 0xAD — Q_nom temperature setpoints
  // 0xAE — Q_nom capacity at each temperature
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
  // 0xAF — Nominal capacity at 25°C
  // 0xB0 — Coulombic efficiency
  //
  // Coulombic efficiency is stored for completeness but is not applied: the
  // reference estimator does not use it either.
  // -------------------------------------------------------------------------
  fuelgauge_data.q_nom_ah                       = 2.0169f;
  fuelgauge_data.coulombic_efficiency           = 0.998f;

  // -------------------------------------------------------------------------
  // 0xB6-0xBA — Arrhenius activation energies
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
  // 0xBB — Kalman process noise SOC
  // 0xBC — Kalman process noise RC1
  // 0xBD — Kalman process noise RC2
  // 0xBE — Kalman measurement noise (0.005 V sensor sigma, squared)
  //
  // The process noise is added once per update. 1e-6 was tuned at 20 Hz, so at
  // 100 Hz (BD_LOOP_PERIOD_MS = 10 ms) it is 2e-7 to keep the same noise per second.
  // -------------------------------------------------------------------------
  fuelgauge_data.kf_q_soc                       = 2e-7f;
  fuelgauge_data.kf_q_rc1                       = 2e-7f;
  fuelgauge_data.kf_q_rc2                       = 2e-7f;
  fuelgauge_data.kf_r_v                         = 2.5e-5f;

  // -------------------------------------------------------------------------
  // 0xD0-0xD3 — Filter state
  //
  // Zeroed here; FuelGauge_Seed() fills it from the first voltage reading.
  // -------------------------------------------------------------------------
  for(uint8_t i = 0; i < BD_CELL_COUNT_MAX; i++)
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
  // 0xD7 — CycleCount
  // 0x96 — LearningStatus
  // -------------------------------------------------------------------------
  fuelgauge_data.cycle_count                    = 0;
  fuelgauge_data.learning_status                = 0x0000;
}
