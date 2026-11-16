#ifndef OPENBMS_FUELGAUGE_H
#define OPENBMS_FUELGAUGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define FG_LEARN_CHARGE                 (1 << 0)    // Bit 0 - a finished HPPC charge test wrote the charge tables
#define FG_LEARN_DISCHARGE              (1 << 1)    // Bit 1 - a finished HPPC discharge test wrote the discharge tables
#define FG_LEARN_COMPLETE               (1 << 2)    // Bit 2 - both directions are done

#define FG_STATUS_AT_REST               (1 << 0)    // Bit 0 - at rest: |I| below the deadzone and no cell balancing
#define FG_STATUS_RELAXED               (1 << 1)    // Bit 1 - this rest: every cell settled (dV/dt below the limit between two 1 min averages)
#define FG_STATUS_REST_CORRECTED        (1 << 2)    // Bit 2 - this rest: the OCV correction was done (once per rest)
#define FG_STATUS_CAP_POINT             (1 << 3)    // Bit 3 - this rest: a capacity learning point was stored
#define FG_STATUS_CAP_UPDATED           (1 << 4)    // Bit 4 - this rest: a cell's capacity was updated
#define FG_STATUS_FULL_PENDING          (1 << 5)    // Bit 5 - charging and the full-charge condition holds, the hold time is running
#define FG_STATUS_FULL_SET              (1 << 6)    // Bit 6 - the gauge set 100 % in this charge, cleared by a discharge

typedef struct
{
  float ocv[SYS_SOC_GRID_POINTS];                                    // OCV in V
  float r0[SYS_SOC_GRID_POINTS];                                     // R0 in Ohm
  float r1[SYS_SOC_GRID_POINTS];                                     // R1 in Ohm
  float tau1[SYS_SOC_GRID_POINTS];                                   // tau1 in s
  float r2[SYS_SOC_GRID_POINTS];                                     // R2 in Ohm
  float tau2[SYS_SOC_GRID_POINTS];                                   // tau2 in s
} FuelGauge_Table_t;

void FuelGauge_Init(FuelGauge_Data_t *data, const Config_Data_t *config);
void FuelGauge_Run(void);
bool FuelGauge_IsReady(void);                                                       // model loaded: learning complete, SOC grid, filter values, capacities, rising OCV tables of every used cell

bool FuelGauge_SetData(const FuelGauge_Data_t *data);                            // false: a model or state float is NaN or inf, nothing written

const float *FuelGauge_GetSocGrid(void);                                            // the 20 SOC grid points in %
bool FuelGauge_SetCellTable(uint8_t cell, bool charge, const FuelGauge_Table_t *table);
void FuelGauge_SetCellCapacity(uint8_t cell, float capacity_ah);
void FuelGauge_SetModelTemperature(bool charge, float temp_c);                      // model_temp_chg_c or model_temp_dis_c, the average temperature of an HPPC test
void FuelGauge_SetNominalCapacity(float capacity_ah);                               // q_nom_ah, the pack capacity from a discharge test
void FuelGauge_SetLearned(bool charge);                                             // sets the LearningStatus bit of that direction
void FuelGauge_ClearLearning(void);                                                 // clears all LearningStatus bits (CC_CLEAR_LEARNING stores the model after it)

void FuelGauge_GetState(FuelGauge_State_t *state);                                  // copy to save; FuelGauge_StateChanged counts from this copy
void FuelGauge_PeekState(FuelGauge_State_t *state);                                 // the same copy for the host (0x9C), the save triggers stay as they are
bool FuelGauge_StateValid(const FuelGauge_State_t *state);                          // every value of the used cells in range (what FuelGauge_SetState checks)
bool FuelGauge_SetState(const FuelGauge_State_t *state);                            // at boot, after the model; false if a value is out of range
void FuelGauge_SetDefaultsIfEmpty(void);                                           // empty model (SOC grid not rising): the FG_DEFAULT_* values HPPC does not measure
bool FuelGauge_IsSeeded(void);                                                      // the SOC has been seeded since boot, before that there is no state to save
#ifdef OPENBMS_TEST
bool FuelGauge_ResetSocFromOcv(void);                                               // test build: SOC seeded again from the cell voltages, state save requested; false: not seeded or not at rest
#endif
const float *FuelGauge_GetBalanceTarget(void);                                      // SOC balancing: Ah each cell still has to bleed (0 = none), for Balance_Run
const uint16_t *FuelGauge_GetBalanceTargetMah(void);                                // 0x9A the same in mAh
const uint8_t *FuelGauge_GetStatus(void);                                          // 0x98 fuelgauge_status, FG_STATUS_* bits
bool FuelGauge_StateChanged(void);                                                  // something worth saving changed since the last FuelGauge_GetState

#ifdef __cplusplus
}
#endif

#endif // OPENBMS_FUELGAUGE_H
