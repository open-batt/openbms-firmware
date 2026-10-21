#ifndef OPENBMS_CTRL_H
#define OPENBMS_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void Ctrl_Init(void);
void Ctrl_Run(void);

void Ctrl_GetConfig(Config_Data_t *data);
void Ctrl_GetSystem(System_Data_t *data);
void Ctrl_GetStats(Stats_Data_t *data);
bool Ctrl_SetConfig(const Config_Data_t *data);
bool Ctrl_ConfigValid(void);
bool Ctrl_SaveConfig(void);
void Ctrl_SetMode(uint16_t mode);
void Ctrl_SetFetRequest(bool state);
void Ctrl_SetPreFetRequest(bool state);
void Ctrl_SetLastCommunication(void);
void Ctrl_ResetProtection(void);

#ifdef OPENBMS_SIMULATION
void Ctrl_SimSample(const float *cell_mv, float current_ma, uint16_t dt_ms);   // one host sample: sets the measurements, runs Ctrl_Run once
#endif

#ifdef __cplusplus
}
#endif

#endif // OPENBMS_CTRL_H
