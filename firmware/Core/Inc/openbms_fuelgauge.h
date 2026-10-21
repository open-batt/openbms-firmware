#ifndef OPENBMS_FUELGAUGE_H
#define OPENBMS_FUELGAUGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// -------------------------------------------------------------------------
// 0x96 — LearningStatus register bit definitions (read-only)
//
// Only the bits the estimator itself drives are defined here; the remaining
// bits stay reserved for the SOH/capacity learning stage.
// -------------------------------------------------------------------------
#define FG_STATUS_SEEDED                (1 << 0)    // Bit 0 — state seeded from OCV, filter running
#define FG_STATUS_GATED                 (1 << 1)    // Bit 1 — last update had its correction gated
#define FG_STATUS_AT_REST               (1 << 2)    // Bit 2 — |I| below deadzone, SOC correction blocked
#define FG_STATUS_NAN_RESET             (1 << 3)    // Bit 3 — a cell state went NaN or inf and was seeded again, stays set until reboot

// One direction of the model of one cell, on the SOC grid
typedef struct
{
  float ocv[20];                                    // OCV in V
  float r0[20];                                     // R0 in Ω
  float r1[20];                                     // R1 in Ω
  float tau1[20];                                   // tau1 in s
  float r2[20];                                     // R2 in Ω
  float tau2[20];                                   // tau2 in s
} FuelGauge_Table_t;

void FuelGauge_Init(void);
void FuelGauge_Run(uint8_t cell_count);

void FuelGauge_GetData(FuelGauge_Data_t *data);
bool FuelGauge_SetData(const FuelGauge_Data_t *data);                            // false: a model or state float is NaN or inf, nothing written

void FuelGauge_GetSocGrid(float *grid);                                             // the 20 SOC grid points in %
void FuelGauge_SetCellTable(uint8_t cell, bool charge, const FuelGauge_Table_t *table);
void FuelGauge_SetCellCapacity(uint8_t cell, float capacity_ah);

#ifdef __cplusplus
}
#endif

#endif // OPENBMS_FUELGAUGE_H
