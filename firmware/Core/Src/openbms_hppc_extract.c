/**
  ******************************************************************************
  * @file           : openbms_hppc_extract.c
  * @brief          : OpenBMS HPPC parameter extraction
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

#include "openbms_hppc_extract.h"
#include "openbms_hppc.h"
#include "openbms_fuelgauge.h"
#include <math.h>
#include <string.h>

// -----------------------------------------------------------------------------
// What hppc_pipeline.py does offline, done on the board while the test runs.
//
// Every ramp (pulse) followed by its rest is one cycle. For every cycle and
// every cell:
//   - OCV before the pulse and at the end of the rest (average of the last ticks)
//   - R0 from the voltage step a few ticks into the pulse (dV / dI)
//   - the charge moved (pulse and rest) for the capacity axis
//   - R1, R2, tau1, tau2 from a 2-RC fit of the rest relaxation
// When the test ends by its own stop condition the cycles are turned into the
// tables on the SOC grid: the OCV curve is extended to the OCV limits of the
// config (extend_ocv_table), R1, R2, tau1, tau2 are smoothed, and everything is
// resampled on the grid and written into the model of each cell. The unfinished
// last ramp is not a cycle, as in the pipeline.
//
// The fit and the table building run in small slices of the main loop, a few
// evaluations per tick, so the 10 ms loop is never held up. The fit of a rest
// has to be ready before the next rest starts, otherwise BD_HPPC_X_OVERRUN is set and
// that cycle is left out.
// -----------------------------------------------------------------------------

// Same constants as hppc_pipeline.py
#define HX_Q_LO_PAD_AH          0.010f          // Q_LO_PAD_AH
#define HX_TOP_TAPER            0.60f           // TOP_TAPER
#define HX_LARGE_GAP_V          0.15f           // LARGE_GAP_WARNING_V
#define HX_Q_MAX_FACTOR         1.5f            // a cell's capacity above 1.5 x cell_capacity is not real, the cell gets no tables (board only)
#define HX_MAX_FIT_RMSE_V       0.020f          // MAX_ACCEPTABLE_REST_FIT_RMSE_MV
#define HX_RATIO_LO             0.05f           // FIT_LB / FIT_UB of the two ratios
#define HX_RATIO_HI             0.95f
#define HX_TAU1_LO              0.5f            // FIT_LB / FIT_UB of tau1 and tau2 in s
#define HX_TAU1_HI              60.0f
#define HX_TAU2_LO              10.0f
#define HX_TAU2_HI              400.0f

// Board side choices
#ifndef HX_R0_TICKS
#define HX_R0_TICKS             2U              // ticks into the pulse where the R0 step is read (20-30 ms after the FET went on)
#endif
#define HX_START_AVG_TICKS      15U             // OCV before the pulse: average of the last ticks of the rest
#define HX_STOP_AVG_TICKS       20U             // OCV at the end of the rest
#define HX_RING                 20U             // ring of the newest voltages, at least the longer average
#define HX_MAX_CYCLES           HPPC_POINT_COUNT    // one per checkpoint
#define HX_MIN_CYCLES           4U              // fewer than this and no curve can be built
#define HX_REST_SAMPLES_MAX     450U            // rest samples at 1 Hz, up to 450 s
#define HX_SAMPLE_TICKS         (1000UL / BD_LOOP_PERIOD_MS)
#define HX_GRID_POINTS          20U
#define HX_FIT_GRID_N           14U             // coarse search grid of tau1, tau2
#define HX_FIT_EVALS_PER_TICK   4U              // objective evaluations per main loop pass, about 0.15 ms each
#define HX_TAU_RATIO_MIN        1.2f            // tau2 has to stay above tau1 by this factor

// Phases of the data collection
#define HX_MODE_NONE            0U
#define HX_MODE_REST            1U
#define HX_MODE_RAMP            2U

typedef struct
{
  float   y[HX_REST_SAMPLES_MAX];       // rest voltage minus its final value in V
  uint16_t m;                           // number of samples
  float   yy;                           // sum of y^2
  float   v0;                           // y[0]
  float   lo[2];                        // bounds of the two amplitudes (ratio * v0)
  float   hi[2];
  uint8_t phase;                        // 0 coarse grid, 1 refine
  uint8_t gi, gj;                       // grid position
  float   d1, d2;                       // grid spacing in ln(tau)
  float   u1, u2, sse;                  // best point so far
  float   s1, s2;                       // refine step
  uint8_t nb;                           // neighbour being tried
  float   nb_u1, nb_u2, nb_sse;         // best neighbour of this round
  bool    nb_found;
} HxFit_t;

static bool      hx_active;             // a test is running and being collected
static uint8_t   hx_mode;
static uint8_t   hx_direction;
static uint8_t   hx_cells;
static float     hx_v_lo, hx_v_hi;      // OCV limits in V
static float     hx_q_max_ah;           // largest capacity a cell may come out with
static uint16_t  hx_rest_samples;       // samples of a full rest, the size the buffer is used up to
static uint16_t  hx_rest_len;           // samples the last rest really delivered
static uint16_t  hx_status;

static uint8_t   hx_cycle;              // index of the ramp running or of the next one, equals finished cycles in between
static uint32_t  hx_n;                  // ticks in the current phase
static bool      hx_rest_store;         // the running rest follows a ramp and is recorded
static bool      hx_rest_skip;          // ... but the fit of the last rest was not ready
static bool      hx_have_prev;
static float     hx_prev_i;             // previous tick current for the trapezoid, mA
static float     hx_cc_ah;              // charge of the running cycle (ramp + rest)
static float     hx_i_sum;
static uint32_t  hx_i_cnt;
static float     hx_pre_v[BD_CELL_COUNT_MAX];   // last tick before the pulse, mV
static float     hx_pre_i;                      // mA
static float     hx_r0_v[BD_CELL_COUNT_MAX];    // R0 tick of the pulse, mV
static float     hx_r0_i;
static bool      hx_r0_taken;

static float     hx_ring[BD_CELL_COUNT_MAX][HX_RING];
static uint8_t   hx_ring_pos;
static uint8_t   hx_ring_fill;

// Per cycle results
static float     hx_cc[HX_MAX_CYCLES];                              // Ah, signed
static float     hx_avg_i[HX_MAX_CYCLES];                           // A, mean current of the pulse
static float     hx_ocv_start[BD_CELL_COUNT_MAX][HX_MAX_CYCLES];    // V
static float     hx_ocv_stop[BD_CELL_COUNT_MAX][HX_MAX_CYCLES];
static float     hx_r0[BD_CELL_COUNT_MAX][HX_MAX_CYCLES];           // Ω
static float     hx_r1[BD_CELL_COUNT_MAX][HX_MAX_CYCLES];
static float     hx_r2[BD_CELL_COUNT_MAX][HX_MAX_CYCLES];
static float     hx_tau1[BD_CELL_COUNT_MAX][HX_MAX_CYCLES];         // s
static float     hx_tau2[BD_CELL_COUNT_MAX][HX_MAX_CYCLES];

static uint16_t  hx_rest[BD_CELL_COUNT_MAX][HX_REST_SAMPLES_MAX];   // rest voltage at 1 Hz in 0.1 mV
static float     hx_rest_final[BD_CELL_COUNT_MAX];                  // V

// Background work
static HxFit_t  hx_fit;
static bool     hx_fit_busy;
static bool     hx_fit_started;     // the search of hx_fit_cell has begun
static uint8_t  hx_fit_cycle;
static uint8_t  hx_fit_cell;
static bool     hx_final_pending;
static uint8_t  hx_final_cell;
static uint8_t  hx_final_cycles;
static bool     hx_final_written;

// -----------------------------------------------------------------------------
// Small helpers
// -----------------------------------------------------------------------------
static float Hx_Nan(void)
{
  return nanf("");
}
static float Hx_Clamp(float x, float lo, float hi)
{
  if(x < lo)
  {
    return lo;
  }
  if(x > hi)
  {
    return hi;
  }
  return x;
}
static float Hx_RingAverage(uint8_t cell, uint8_t count)
{
  float   sum = 0.0f;
  uint8_t n   = count;

  if(n > hx_ring_fill)
  {
    n = hx_ring_fill;
  }
  if(n == 0)
  {
    return 0.0f;
  }

  for(uint8_t k = 0; k < n; k++)
  {
    uint8_t idx = (uint8_t)((hx_ring_pos + HX_RING - 1U - k) % HX_RING);
    sum += hx_ring[cell][idx];
  }

  return sum / (float)n;
}

// -----------------------------------------------------------------------------
// 2-RC fit of one rest: y(t) = a1 * exp(-t / tau1) + a2 * exp(-t / tau2), with
// y = v - v_final and a_i = v0 * ratio_i, as find_params.m / fit_cycle().
//
// For fixed taus the two amplitudes are a linear least squares problem, so only
// (tau1, tau2) are searched: a coarse grid in ln(tau), then a compass search.
// The samples are 1 s apart, so exp(-j / tau) comes from a running product
// instead of an exp() per sample. The first sample is the voltage right after
// the FET went off while the current is still settling. It sets v0 and with it
// the bounds of the amplitudes, but it is not fitted: in the 20 Hz logs it is
// one row in thousands, at 1 Hz it would weigh 20 times more.
// -----------------------------------------------------------------------------
static float HxFit_Eval(float tau1, float tau2, float *a1_out, float *a2_out)
{
  const HxFit_t *f = &hx_fit;

  float q1  = expf(-1.0f / tau1);
  float q2  = expf(-1.0f / tau2);
  float e1  = q1;
  float e2  = q2;
  float s11 = 0.0f;
  float s12 = 0.0f;
  float s22 = 0.0f;
  float t1  = 0.0f;
  float t2  = 0.0f;

  for(uint16_t j = 1; j < f->m; j++)
  {
    float y = f->y[j];

    s11 += e1 * e1;
    s12 += e1 * e2;
    s22 += e2 * e2;
    t1  += y * e1;
    t2  += y * e2;

    e1 *= q1;
    e2 *= q2;
  }

  float det = s11 * s22 - s12 * s12;
  if(!(det > 1e-7f * s11 * s22))
  {
    return INFINITY;
  }

  float best_sse = INFINITY;
  float best_a1  = 0.0f;
  float best_a2  = 0.0f;

  // Unconstrained solution first
  float a1 = (t1 * s22 - t2 * s12) / det;
  float a2 = (t2 * s11 - t1 * s12) / det;

  if(a1 >= f->lo[0] && a1 <= f->hi[0] && a2 >= f->lo[1] && a2 <= f->hi[1])
  {
    best_a1  = a1;
    best_a2  = a2;
    best_sse = f->yy - 2.0f * (a1 * t1 + a2 * t2) + a1 * a1 * s11 + 2.0f * a1 * a2 * s12 + a2 * a2 * s22;
  }
  else
  {
    // On a bound: one amplitude sits on a bound, the other one is solved and clamped
    for(uint8_t k = 0; k < 4; k++)
    {
      float c1;
      float c2;

      if(k < 2)
      {
        if(k == 0)
        {
          c1 = f->lo[0];
        }
        else
        {
          c1 = f->hi[0];
        }
        c2 = Hx_Clamp((t2 - s12 * c1) / s22, f->lo[1], f->hi[1]);
      }
      else
      {
        if(k == 2)
        {
          c2 = f->lo[1];
        }
        else
        {
          c2 = f->hi[1];
        }
        c1 = Hx_Clamp((t1 - s12 * c2) / s11, f->lo[0], f->hi[0]);
      }

      float sse = f->yy - 2.0f * (c1 * t1 + c2 * t2) + c1 * c1 * s11 + 2.0f * c1 * c2 * s12 + c2 * c2 * s22;
      if(sse < best_sse)
      {
        best_sse = sse;
        best_a1  = c1;
        best_a2  = c2;
      }
    }
  }

  *a1_out = best_a1;
  *a2_out = best_a2;
  return best_sse;
}

static bool HxFit_Valid(float u1, float u2)
{
  if(u1 < logf(HX_TAU1_LO) || u1 > logf(HX_TAU1_HI))
  {
    return false;
  }
  if(u2 < logf(HX_TAU2_LO) || u2 > logf(HX_TAU2_HI))
  {
    return false;
  }
  if(u2 < u1 + logf(HX_TAU_RATIO_MIN))
  {
    return false;
  }
  return true;
}

static bool HxFit_Start(uint8_t cell)
{
  HxFit_t *f = &hx_fit;

  float vfinal = hx_rest_final[cell];

  f->m  = hx_rest_len;
  if(f->m < 10U)
  {
    return false;
  }
  f->yy = 0.0f;
  for(uint16_t j = 0; j < f->m; j++)
  {
    f->y[j] = (float)hx_rest[cell][j] * 1e-4f - vfinal;
    if(j > 0)
    {
      f->yy += f->y[j] * f->y[j];
    }
  }

  f->v0 = f->y[0];
  if(fabsf(f->v0) < 1e-4f)
  {
    return false;
  }

  float b1 = HX_RATIO_LO * f->v0;
  float b2 = HX_RATIO_HI * f->v0;
  if(b1 < b2)
  {
    f->lo[0] = b1;
    f->hi[0] = b2;
  }
  else
  {
    f->lo[0] = b2;
    f->hi[0] = b1;
  }
  f->lo[1] = f->lo[0];
  f->hi[1] = f->hi[0];

  f->phase = 0;
  f->gi    = 0;
  f->gj    = 0;
  f->d1    = (logf(HX_TAU1_HI) - logf(HX_TAU1_LO)) / (float)(HX_FIT_GRID_N - 1U);
  f->d2    = (logf(HX_TAU2_HI) - logf(HX_TAU2_LO)) / (float)(HX_FIT_GRID_N - 1U);
  f->sse   = INFINITY;
  f->u1    = 0.0f;
  f->u2    = 0.0f;

  return true;
}

// One slice of the search. Returns true when the search is finished, the best
// point is then in hx_fit.
static bool HxFit_Step(uint8_t budget)
{
  static const int8_t dir1[8] = { 1, -1, 0,  0, 1,  1, -1, -1 };
  static const int8_t dir2[8] = { 0,  0, 1, -1, 1, -1,  1, -1 };

  HxFit_t *f = &hx_fit;

  while(budget > 0)
  {
    budget--;

    if(f->phase == 0)
    {
      float u1 = logf(HX_TAU1_LO) + (float)f->gi * f->d1;
      float u2 = logf(HX_TAU2_LO) + (float)f->gj * f->d2;

      if(HxFit_Valid(u1, u2))
      {
        float a1;
        float a2;
        float sse = HxFit_Eval(expf(u1), expf(u2), &a1, &a2);

        if(sse < f->sse)
        {
          f->sse = sse;
          f->u1  = u1;
          f->u2  = u2;
        }
      }

      f->gj++;
      if(f->gj >= HX_FIT_GRID_N)
      {
        f->gj = 0;
        f->gi++;
        if(f->gi >= HX_FIT_GRID_N)
        {
          if(f->sse == INFINITY)
          {
            return true;
          }
          f->phase    = 1;
          f->s1       = f->d1;
          f->s2       = f->d2;
          f->nb       = 0;
          f->nb_found = false;
          f->nb_sse   = INFINITY;
        }
      }
    }
    else
    {
      float u1 = f->u1 + (float)dir1[f->nb] * f->s1;
      float u2 = f->u2 + (float)dir2[f->nb] * f->s2;

      if(HxFit_Valid(u1, u2))
      {
        float a1;
        float a2;
        float sse = HxFit_Eval(expf(u1), expf(u2), &a1, &a2);

        if(sse < f->nb_sse)
        {
          f->nb_sse   = sse;
          f->nb_u1    = u1;
          f->nb_u2    = u2;
          f->nb_found = true;
        }
      }

      f->nb++;
      if(f->nb >= 8U)
      {
        if(f->nb_found && f->nb_sse < f->sse)
        {
          f->u1  = f->nb_u1;
          f->u2  = f->nb_u2;
          f->sse = f->nb_sse;
        }
        else
        {
          f->s1 *= 0.5f;
          f->s2 *= 0.5f;
        }

        f->nb       = 0;
        f->nb_found = false;
        f->nb_sse   = INFINITY;

        if(f->s1 < 1e-3f && f->s2 < 1e-3f)
        {
          return true;
        }
      }
    }
  }

  return false;
}

// The search of one cell is done: store r1, r2, tau1, tau2 of the cycle
static void HxFit_Store(uint8_t cell, uint8_t cycle, bool found)
{
  HxFit_t *f = &hx_fit;

  if(found && f->sse < INFINITY)
  {
    float a1;
    float a2;
    float tau1 = expf(f->u1);
    float tau2 = expf(f->u2);

    (void)HxFit_Eval(tau1, tau2, &a1, &a2);

    float mse = f->sse / (float)(f->m - 1U);
    float rmse = 0.0f;
    if(mse > 0.0f)
    {
      rmse = sqrtf(mse);
    }
    if(rmse > HX_MAX_FIT_RMSE_V)
    {
      hx_status |= BD_HPPC_X_FIT_WARN;
    }

    // R = amplitude / current, as v0 * ratio / avg_current
    hx_r1[cell][cycle]   = a1 / hx_avg_i[cycle];
    hx_r2[cell][cycle]   = a2 / hx_avg_i[cycle];
    hx_tau1[cell][cycle] = tau1;
    hx_tau2[cell][cycle] = tau2;
  }
  else
  {
    hx_r1[cell][cycle]   = Hx_Nan();
    hx_r2[cell][cycle]   = Hx_Nan();
    hx_tau1[cell][cycle] = Hx_Nan();
    hx_tau2[cell][cycle] = Hx_Nan();
    hx_status |= BD_HPPC_X_FIT_WARN;
  }
}

// Fit job: one cell after the other, a few evaluations per call
static void Hx_FitRun(void)
{
  if(!hx_fit_busy)
  {
    return;
  }

  if(!hx_fit_started)
  {
    if(!HxFit_Start(hx_fit_cell))
    {
      HxFit_Store(hx_fit_cell, hx_fit_cycle, false);
      hx_fit_cell++;
    }
    else
    {
      hx_fit_started = true;
    }
  }
  else
  {
    if(HxFit_Step(HX_FIT_EVALS_PER_TICK))
    {
      HxFit_Store(hx_fit_cell, hx_fit_cycle, true);
      hx_fit_started = false;
      hx_fit_cell++;
    }
  }

  if(hx_fit_cell >= hx_cells)
  {
    hx_fit_busy  = false;
    hx_fit_started = false;
  }
}

// -----------------------------------------------------------------------------
// Tables: extend_ocv_table() and the resampling of process_cell()
// -----------------------------------------------------------------------------

// Derivatives of the shape preserving cubic through (x, y), as scipy PchipInterpolator
static float Hx_PchipEdge(float h0, float h1, float m0, float m1)
{
  float d = ((2.0f * h0 + h1) * m0 - h0 * m1) / (h0 + h1);

  if((d > 0.0f) != (m0 > 0.0f) || d == 0.0f)
  {
    d = 0.0f;
  }
  else if(((m0 > 0.0f) != (m1 > 0.0f)) && fabsf(d) > 3.0f * fabsf(m0))
  {
    d = 3.0f * m0;
  }
  return d;
}
static void Hx_PchipSlopes(const float *x, const float *y, uint8_t n, float *d)
{
  // Static to keep them off the 1 KB stack, the function is only called from the main loop
  static float h[HX_MAX_CYCLES + 1U];
  static float m[HX_MAX_CYCLES + 1U];

  for(uint8_t k = 0; k + 1U < n; k++)
  {
    h[k] = x[k + 1U] - x[k];
    m[k] = (y[k + 1U] - y[k]) / h[k];
  }

  for(uint8_t k = 1; k + 1U < n; k++)
  {
    if(((m[k - 1U] > 0.0f) == (m[k] > 0.0f)) && m[k - 1U] != 0.0f && m[k] != 0.0f)
    {
      float w1 = 2.0f * h[k] + h[k - 1U];
      float w2 = h[k] + 2.0f * h[k - 1U];
      d[k] = (w1 + w2) / (w1 / m[k - 1U] + w2 / m[k]);
    }
    else
    {
      d[k] = 0.0f;
    }
  }

  d[0]      = Hx_PchipEdge(h[0], h[1], m[0], m[1]);
  d[n - 1U] = Hx_PchipEdge(h[n - 2U], h[n - 3U], m[n - 2U], m[n - 3U]);
}
static float Hx_PchipEval(const float *x, const float *y, const float *d, uint8_t n, float z)
{
  uint8_t i = 0;

  while((uint8_t)(i + 2U) < n && z > x[i + 1U])
  {
    i++;
  }

  float h = x[i + 1U] - x[i];
  float t = (z - x[i]) / h;
  float t2 = t * t;
  float t3 = t2 * t;

  return y[i]      * (2.0f * t3 - 3.0f * t2 + 1.0f)
       + h * d[i]  * (t3 - 2.0f * t2 + t)
       + y[i + 1U] * (-2.0f * t3 + 3.0f * t2)
       + h * d[i + 1U] * (t3 - t2);
}

// np.interp: x ascending, the ends are held
static float Hx_Interp(float xq, const float *x, const float *y, uint8_t n)
{
  if(xq <= x[0])
  {
    return y[0];
  }
  if(xq >= x[n - 1U])
  {
    return y[n - 1U];
  }

  uint8_t i = 0;
  while((uint8_t)(i + 2U) < n && xq > x[i + 1U])
  {
    i++;
  }
  return y[i] + (y[i + 1U] - y[i]) * (xq - x[i]) / (x[i + 1U] - x[i]);
}

// A failed value (NaN) takes its neighbours: linear inside, held at the ends.
// Returns false when fewer than two values are good.
static bool Hx_FillGaps(float *v, uint8_t n)
{
  uint8_t good = 0;

  for(uint8_t k = 0; k < n; k++)
  {
    if(!isnan(v[k]))
    {
      good++;
    }
  }
  if(good < 2U)
  {
    return false;
  }

  for(uint8_t k = 0; k < n; k++)
  {
    if(!isnan(v[k]))
    {
      continue;
    }

    int16_t lo = (int16_t)k - 1;
    int16_t hi = (int16_t)k + 1;
    while(lo >= 0 && isnan(v[lo]))
    {
      lo--;
    }
    while(hi < (int16_t)n && isnan(v[hi]))
    {
      hi++;
    }

    if(lo < 0)
    {
      v[k] = v[hi];
    }
    else if(hi >= (int16_t)n)
    {
      v[k] = v[lo];
    }
    else
    {
      v[k] = v[lo] + (v[hi] - v[lo]) * (float)(k - lo) / (float)(hi - lo);
    }
  }
  return true;
}

// Gaussian weighted moving average over 7 points (FWHM of 3 points), as gaussian_smooth()
static void Hx_Smooth(float *v, uint8_t n)
{
  // Static to keep them off the 1 KB stack, the function is only called from the main loop
  static float out[HX_MAX_CYCLES];
  static float w[7];
  const float sigma = 3.0f / (2.0f * sqrtf(2.0f * logf(2.0f)));

  // weights for the offsets -3 ... +3
  for(uint8_t j = 0; j < 7U; j++)
  {
    float dx = ((float)j - 3.0f) / sigma;
    w[j] = expf(-0.5f * dx * dx);
  }

  for(uint8_t k = 0; k < n; k++)
  {
    int16_t lo = (int16_t)k - 3;
    int16_t hi = (int16_t)k + 4;
    float   sw = 0.0f;
    float   sx = 0.0f;

    if(lo < 0)
    {
      lo = 0;
    }
    if(hi > (int16_t)n)
    {
      hi = (int16_t)n;
    }

    for(int16_t j = lo; j < hi; j++)
    {
      float wj = w[j - (int16_t)k + 3];
      sw += wj;
      sx += wj * v[j];
    }
    out[k] = sx / sw;
  }

  for(uint8_t k = 0; k < n; k++)
  {
    v[k] = out[k];
  }
}

// The tables of one cell. Returns false when the cycles cannot be turned into a curve.
static bool Hx_BuildCell(uint8_t cell, uint8_t n, FuelGauge_Table_t *tab, float *q_tot_out)
{
  // static: the stack of the board is only 1 KB
  static float ocv_raw[HX_MAX_CYCLES + 1U];
  static float cap_raw[HX_MAX_CYCLES + 1U];
  static float ocv[HX_MAX_CYCLES + 1U];
  static float cap[HX_MAX_CYCLES + 1U];
  static float z[HX_MAX_CYCLES + 1U];
  static float dz[HX_MAX_CYCLES + 1U];
  static float soc[HX_MAX_CYCLES];
  static float r0[HX_MAX_CYCLES];
  static float r1[HX_MAX_CYCLES];
  static float r2[HX_MAX_CYCLES];
  static float tau1[HX_MAX_CYCLES];
  static float tau2[HX_MAX_CYCLES];
  static float grid[HX_GRID_POINTS];

  uint8_t np = (uint8_t)(n + 1U);
  bool    charge = (hx_direction == BD_HPPC_DIR_CHARGE);

  // Measured OCV points and the charge between them, in test order
  cap_raw[0] = 0.0f;
  for(uint8_t k = 0; k < n; k++)
  {
    cap_raw[k + 1U] = cap_raw[k] + hx_cc[k];
    ocv_raw[k]      = hx_ocv_start[cell][k];
  }
  ocv_raw[n] = hx_ocv_stop[cell][n - 1U];

  // The curve is built bottom up. A discharge runs top down, so it is turned around
  // and the capacity starts at 0 at the lowest OCV point.
  for(uint8_t k = 0; k < np; k++)
  {
    if(charge)
    {
      ocv[k] = ocv_raw[k];
      cap[k] = cap_raw[k];
    }
    else
    {
      ocv[k] = ocv_raw[np - 1U - k];
      cap[k] = cap_raw[np - 1U - k] - cap_raw[np - 1U];
    }
  }

  for(uint8_t k = 0; k + 1U < np; k++)
  {
    if(!(ocv[k + 1U] > ocv[k]) || !(cap[k + 1U] > cap[k]))
    {
      return false;
    }
  }

  // extend_ocv_table()
  float v_lo = hx_v_lo;
  float v_hi = hx_v_hi;
  if(ocv[0] <= v_lo)
  {
    v_lo = ocv[0] - 0.01f;
  }
  if(ocv[np - 1U] >= v_hi)
  {
    v_hi = ocv[np - 1U] + 0.01f;
  }

  float dqdv_top = (cap[np - 1U] - cap[np - 2U]) / (ocv[np - 1U] - ocv[np - 2U]);
  float q_hi_pad = HX_TOP_TAPER * dqdv_top * (v_hi - ocv[np - 1U]);
  float q_tot    = HX_Q_LO_PAD_AH + cap[np - 1U] + q_hi_pad;

  // A flat top of the OCV curve can blow the capacity up, such a cell is not used
  if(!(q_tot <= hx_q_max_ah))
  {
    return false;
  }

  if((ocv[0] - v_lo) > HX_LARGE_GAP_V)
  {
    hx_status |= BD_HPPC_X_GAP_LO;
  }
  if((v_hi - ocv[np - 1U]) > HX_LARGE_GAP_V)
  {
    hx_status |= BD_HPPC_X_GAP_HI;
  }

  for(uint8_t k = 0; k < np; k++)
  {
    z[k] = (HX_Q_LO_PAD_AH + cap[k]) / q_tot;
  }
  Hx_PchipSlopes(z, ocv, np, dz);

  float za = z[0];
  float zb = z[np - 1U];
  float pl = dz[0] * za / (ocv[0] - v_lo);
  float ph = dz[np - 1U] * (1.0f - zb) / (v_hi - ocv[np - 1U]);

  // SOC of every finished cycle, ascending, with the cycle data in the same order
  for(uint8_t m = 0; m < n; m++)
  {
    uint8_t k;

    if(charge)
    {
      soc[m] = z[m + 1U] * 100.0f;
      k      = m;
    }
    else
    {
      soc[m] = z[m] * 100.0f;
      k      = (uint8_t)(n - 1U - m);
    }

    r0[m]   = hx_r0[cell][k];
    r1[m]   = hx_r1[cell][k];
    r2[m]   = hx_r2[cell][k];
    tau1[m] = hx_tau1[cell][k];
    tau2[m] = hx_tau2[cell][k];
  }

  if(!Hx_FillGaps(r0, n) || !Hx_FillGaps(r1, n) || !Hx_FillGaps(r2, n) ||
     !Hx_FillGaps(tau1, n) || !Hx_FillGaps(tau2, n))
  {
    return false;
  }
  Hx_Smooth(r1, n);
  Hx_Smooth(r2, n);
  Hx_Smooth(tau1, n);
  Hx_Smooth(tau2, n);

  // Resample on the SOC grid of the fuel gauge
  FuelGauge_GetSocGrid(grid);

  float v_prev = -INFINITY;
  for(uint8_t g = 0; g < HX_GRID_POINTS; g++)
  {
    float zg = Hx_Clamp(grid[g] / 100.0f, 0.0f, 1.0f);
    float v;

    if(zg < za)
    {
      v = v_lo + (ocv[0] - v_lo) * powf(zg / za, pl);
    }
    else if(zg > zb)
    {
      v = v_hi - (v_hi - ocv[np - 1U]) * powf((1.0f - zg) / (1.0f - zb), ph);
    }
    else
    {
      v = Hx_PchipEval(z, ocv, dz, np, zg);
    }
    if(v < v_prev)
    {
      v = v_prev;
    }
    v_prev = v;

    tab->ocv[g]  = v;
    tab->r0[g]   = Hx_Interp(grid[g], soc, r0, n);
    tab->r1[g]   = Hx_Interp(grid[g], soc, r1, n);
    tab->r2[g]   = Hx_Interp(grid[g], soc, r2, n);
    tab->tau1[g] = Hx_Interp(grid[g], soc, tau1, n);
    tab->tau2[g] = Hx_Interp(grid[g], soc, tau2, n);
  }

  *q_tot_out = q_tot;
  return true;
}

// One cell per call. When all cells are done the status tells the result.
static void Hx_FinalRun(void)
{
  static FuelGauge_Table_t tab;
  float                    q_tot = 0.0f;

  if(!hx_final_pending || hx_fit_busy)
  {
    return;
  }

  if(hx_final_cell < hx_cells)
  {
    if(Hx_BuildCell(hx_final_cell, hx_final_cycles, &tab, &q_tot))
    {
      FuelGauge_SetCellTable(hx_final_cell, hx_direction == BD_HPPC_DIR_CHARGE, &tab);

      // One capacity is enough, it comes from the discharge test
      if(hx_direction == BD_HPPC_DIR_DISCHARGE)
      {
        FuelGauge_SetCellCapacity(hx_final_cell, q_tot);
      }
      hx_final_written = true;
    }
    else
    {
      hx_status |= (uint16_t)(1U << hx_final_cell);
    }
    hx_final_cell++;
  }

  if(hx_final_cell >= hx_cells)
  {
    hx_final_pending = false;
    if(hx_final_written)
    {
      hx_status |= BD_HPPC_X_DONE;
    }
  }
}

// -----------------------------------------------------------------------------
// Events from the sequencer
// -----------------------------------------------------------------------------
void HppcX_Begin(uint8_t direction, uint8_t cell_count, uint32_t capacity_mah, uint16_t v_lo_mv, uint16_t v_hi_mv, uint32_t rest_ms)
{
  HppcX_Reset();

  if(cell_count == 0 || cell_count > BD_CELL_COUNT_MAX || v_hi_mv <= v_lo_mv ||
     (rest_ms / 1000UL) < 2UL || (rest_ms / 1000UL) > HX_REST_SAMPLES_MAX)
  {
    return;
  }

  hx_direction    = direction;
  hx_cells        = cell_count;
  hx_v_lo         = (float)v_lo_mv / 1000.0f;
  hx_v_hi         = (float)v_hi_mv / 1000.0f;
  hx_q_max_ah     = (float)capacity_mah / 1000.0f * HX_Q_MAX_FACTOR;
  hx_rest_samples = (uint16_t)(rest_ms / 1000UL);

  hx_cycle      = 0;
  hx_mode       = HX_MODE_NONE;
  hx_ring_pos   = 0;
  hx_ring_fill  = 0;
  hx_rest_store = false;
  hx_rest_skip  = false;
  hx_final_written = false;

  hx_active = true;
}
void HppcX_Reset(void)
{
  HppcX_Abort();
  hx_status = 0;
}
void HppcX_Abort(void)
{
  hx_active        = false;
  hx_mode          = HX_MODE_NONE;
  hx_fit_busy      = false;
  hx_fit_started   = false;
  hx_final_pending = false;
  hx_status       &= (uint16_t)~BD_HPPC_X_BUSY;
}
void HppcX_RestStarted(void)
{
  if(!hx_active)
  {
    return;
  }

  if(hx_mode == HX_MODE_RAMP)
  {
    // A ramp has ended: its pulse data is complete
    uint8_t k = hx_cycle;

    if(hx_i_cnt > 0)
    {
      hx_avg_i[k] = hx_i_sum / (float)hx_i_cnt / 1000.0f;
    }
    else
    {
      hx_avg_i[k] = 0.0f;
    }

    for(uint8_t c = 0; c < hx_cells; c++)
    {
      float di = hx_r0_i - hx_pre_i;

      if(hx_r0_taken && di != 0.0f)
      {
        hx_r0[c][k] = fabsf((hx_r0_v[c] - hx_pre_v[c]) / di);        // mV / mA = Ω
      }
      else
      {
        hx_r0[c][k] = Hx_Nan();
      }
    }

    // The fit of the previous rest has to be finished, the sample buffer is reused
    hx_rest_skip = hx_fit_busy;
    if(hx_rest_skip)
    {
      hx_status |= BD_HPPC_X_OVERRUN;
    }
    hx_rest_store = true;
  }
  else
  {
    hx_rest_store = false;      // the first rest, before any pulse
  }

  hx_mode      = HX_MODE_REST;
  hx_n         = 0;
  hx_have_prev = false;
}
void HppcX_RampStarted(void)
{
  if(!hx_active || hx_mode != HX_MODE_REST)
  {
    return;
  }

  if(hx_rest_store)
  {
    // A rest has ended: cycle hx_cycle is complete
    uint8_t k = hx_cycle;

    hx_cc[k] = hx_cc_ah;

    for(uint8_t c = 0; c < hx_cells; c++)
    {
      hx_ocv_stop[c][k]  = Hx_RingAverage(c, (uint8_t)HX_STOP_AVG_TICKS) / 1000.0f;
    }

    // A skipped rest has no samples, and the fit of the previous rest may still read
    // hx_rest_len and hx_rest_final, so they are only updated for a rest that is fitted
    if(hx_rest_skip)
    {
      for(uint8_t c = 0; c < hx_cells; c++)
      {
        hx_r1[c][k]   = Hx_Nan();
        hx_r2[c][k]   = Hx_Nan();
        hx_tau1[c][k] = Hx_Nan();
        hx_tau2[c][k] = Hx_Nan();
      }
    }
    else
    {
      hx_rest_len = (uint16_t)((hx_n + HX_SAMPLE_TICKS - 1UL) / HX_SAMPLE_TICKS);
      if(hx_rest_len > hx_rest_samples)
      {
        hx_rest_len = hx_rest_samples;
      }

      for(uint8_t c = 0; c < hx_cells; c++)
      {
        hx_rest_final[c] = hx_ocv_stop[c][k];
      }

      hx_fit_cycle = k;
      hx_fit_cell  = 0;
      hx_fit_busy  = true;
    }

    hx_cycle++;
  }

  // Checked after the increment, right where the index is used
  if(hx_cycle >= HX_MAX_CYCLES)
  {
    HppcX_Abort();      // more ramps than checkpoints, cannot be stored
    return;
  }

  // OCV before the coming pulse
  for(uint8_t c = 0; c < hx_cells; c++)
  {
    hx_ocv_start[c][hx_cycle] = Hx_RingAverage(c, (uint8_t)HX_START_AVG_TICKS) / 1000.0f;
  }

  hx_mode      = HX_MODE_RAMP;
  hx_n         = 0;
  hx_have_prev = false;
  hx_cc_ah     = 0.0f;
  hx_i_sum     = 0.0f;
  hx_i_cnt     = 0;
  hx_r0_taken  = false;
}
void HppcX_Tick(const float *cell_mv, float current_ma, uint32_t dt_ms)
{
  if(!hx_active || hx_mode == HX_MODE_NONE)
  {
    return;
  }

  const float dt_s = (float)dt_ms / 1000.0f;      // real time since the last tick, for the charge

  for(uint8_t c = 0; c < hx_cells; c++)
  {
    hx_ring[c][hx_ring_pos] = cell_mv[c];
  }
  hx_ring_pos = (uint8_t)((hx_ring_pos + 1U) % HX_RING);
  if(hx_ring_fill < HX_RING)
  {
    hx_ring_fill++;
  }

  // Charge moved, trapezoid inside a phase as np.trapz over the rows of a segment
  if(hx_have_prev)
  {
    hx_cc_ah += 0.5f * (hx_prev_i + current_ma) * dt_s / 3600000.0f;      // mA * s -> Ah
  }
  hx_prev_i    = current_ma;
  hx_have_prev = true;

  if(hx_mode == HX_MODE_REST)
  {
    // Last tick of the rest is the baseline of the R0 step of the next pulse
    for(uint8_t c = 0; c < hx_cells; c++)
    {
      hx_pre_v[c] = cell_mv[c];
    }
    hx_pre_i = current_ma;

    if(hx_rest_store && !hx_rest_skip && (hx_n % HX_SAMPLE_TICKS) == 0)
    {
      uint32_t j = hx_n / HX_SAMPLE_TICKS;

      if(j < hx_rest_samples)
      {
        for(uint8_t c = 0; c < hx_cells; c++)
        {
          hx_rest[c][j] = (uint16_t)(Hx_Clamp(cell_mv[c], 0.0f, 6553.0f) * 10.0f + 0.5f);
        }
      }
    }
  }
  else
  {
    hx_i_sum += current_ma;
    hx_i_cnt++;

    if(hx_n == HX_R0_TICKS)
    {
      for(uint8_t c = 0; c < hx_cells; c++)
      {
        hx_r0_v[c] = cell_mv[c];
      }
      hx_r0_i     = current_ma;
      hx_r0_taken = true;
    }
  }

  hx_n++;
}
void HppcX_Done(void)
{
  if(!hx_active)
  {
    return;
  }

  hx_active = false;
  hx_mode   = HX_MODE_NONE;

  if(hx_cycle < HX_MIN_CYCLES)
  {
    hx_status |= BD_HPPC_X_FEW_CYCLES;
    return;
  }

  hx_final_cycles  = hx_cycle;
  hx_final_cell    = 0;
  hx_final_pending = true;
}
void HppcX_Run(void)
{
  Hx_FitRun();
  Hx_FinalRun();

  if(hx_active || hx_fit_busy || hx_final_pending)
  {
    hx_status |= BD_HPPC_X_BUSY;
  }
  else
  {
    hx_status &= (uint16_t)~BD_HPPC_X_BUSY;
  }
}
uint16_t HppcX_GetStatus(void)
{
  return hx_status;
}
