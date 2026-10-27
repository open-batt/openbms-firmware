#ifndef OPENBMS_CTRL_H
#define OPENBMS_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void Ctrl_Init(void);
void Ctrl_Run(void);

const Config_Data_t *Ctrl_GetConfig(void);
const System_Data_t *Ctrl_GetSystem(void);
const Stats_Data_t  *Ctrl_GetStats(void);
const FuelGauge_Data_t *Ctrl_GetFuelGauge(void);
bool Ctrl_WriteConfig(const void *field, const uint8_t *data, uint8_t length);    // false: outside the config or a NaN / inf calibration value, nothing written
bool Ctrl_WriteFuelGauge(const void *field, const uint8_t *data, uint8_t length); // false: outside the struct or a NaN / inf model value, nothing written
bool Ctrl_SaveConfig(void);
void Ctrl_SetMode(uint16_t mode);
void Ctrl_SetFetRequest(bool state);
void Ctrl_SetPreFetRequest(bool state);
void Ctrl_SetLastCommunication(void);
void Ctrl_ResetProtection(void);
void Ctrl_ClearFault(void);

#ifdef OPENBMS_SIMULATION
void Ctrl_SimSample(const float *cell_mv, float current_ma, float temperature_c, uint16_t dt_ms);   // one host sample: sets the measurements, runs Ctrl_Run once
#endif

#ifdef __cplusplus
}
#endif

#endif // OPENBMS_CTRL_H
