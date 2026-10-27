#ifndef OPENBMS_FUELGAUGE_H
#define OPENBMS_FUELGAUGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// -------------------------------------------------------------------------
// 0x96 — LearningStatus register bit definitions (read-only, cleared with CC_CLEAR_LEARNING)
// -------------------------------------------------------------------------
#define FG_LEARN_CHARGE                 (1 << 0)    // Bit 0 — a finished HPPC charge test wrote the charge tables
#define FG_LEARN_DISCHARGE              (1 << 1)    // Bit 1 — a finished HPPC discharge test wrote the discharge tables
#define FG_LEARN_COMPLETE               (1 << 2)    // Bit 2 — both directions are done

// One direction of the model of one cell, on the SOC grid
typedef struct
{
  float ocv[SYS_SOC_GRID_POINTS];                                    // OCV in V
  float r0[SYS_SOC_GRID_POINTS];                                     // R0 in Ω
  float r1[SYS_SOC_GRID_POINTS];                                     // R1 in Ω
  float tau1[SYS_SOC_GRID_POINTS];                                   // tau1 in s
  float r2[SYS_SOC_GRID_POINTS];                                     // R2 in Ω
  float tau2[SYS_SOC_GRID_POINTS];                                   // tau2 in s
} FuelGauge_Table_t;

void FuelGauge_Init(FuelGauge_Data_t *data, const Config_Data_t *config);
void FuelGauge_Run(void);
bool FuelGauge_IsReady(void);                                                       // model loaded: learning complete, SOC grid, filter values, capacities

bool FuelGauge_SetData(const FuelGauge_Data_t *data);                            // false: a model or state float is NaN or inf, nothing written

const float *FuelGauge_GetSocGrid(void);                                            // the 20 SOC grid points in %
bool FuelGauge_SetCellTable(uint8_t cell, bool charge, const FuelGauge_Table_t *table);
void FuelGauge_SetCellCapacity(uint8_t cell, float capacity_ah);
void FuelGauge_SetModelTemperature(bool charge, float temp_c);                      // model_temp_chg_c or model_temp_dis_c, the average temperature of an HPPC test
void FuelGauge_SetNominalCapacity(float capacity_ah);                               // q_nom_ah, the pack capacity from a discharge test
void FuelGauge_SetLearned(bool charge);                                             // sets the LearningStatus bit of that direction
void FuelGauge_ClearLearning(void);                                                 // clears all LearningStatus bits and capacity_update_count

#ifdef __cplusplus
}
#endif

#endif // OPENBMS_FUELGAUGE_H
