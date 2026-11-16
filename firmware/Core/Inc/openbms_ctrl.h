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
void Ctrl_SetMode(uint16_t mode);
bool Ctrl_StartHppc(uint8_t direction);                                           // starts or restarts a test, false while a test or its extraction runs
void Ctrl_SetFetRequest(bool state);
void Ctrl_SetPreFetRequest(bool state);
void Ctrl_SetLastCommunication(void);
void Ctrl_ResetProtection(void);
void Ctrl_ClearFault(void);
void Ctrl_ClearErrors(void);                                                      // CC_CLEAR_ERRORS: resets the TPS12000 on HW_DRV_FAULT, clears every error, FAULT goes back to STARTUP
void Ctrl_Shutdown(void);                                                         // CTRL_STATE_SHUTDOWN: everything off, fuel gauge state saved, power off
void Ctrl_RequestReset(bool bootloader);                                          // CTRL_STATE_RESET: everything off, fuel gauge state saved, then reset (into the bootloader when true)

#ifdef OPENBMS_TEST
const uint32_t *Ctrl_TestLoopTime(void);                                          // test build: [0] last, [1] longest Ctrl_Run pass since boot, in us
#endif


#ifdef __cplusplus
}
#endif

#endif // OPENBMS_CTRL_H
