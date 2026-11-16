/**
  ******************************************************************************
  * @file           : openbms_eeprom.h
  * @brief          : OpenBMS EEPROM storage: config, fuel gauge model and fuel gauge state
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

#ifndef OPENBMS_EEPROM_H
#define OPENBMS_EEPROM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define EEPROM_STATUS_CONFIG_LOADED      (1U << 0)       // a valid config was loaded at boot: protections, FET and balancing work
#define EEPROM_STATUS_MODEL_LOADED       (1U << 1)       // a fuel gauge model was loaded at boot and is in use
#define EEPROM_STATUS_STATE_LOADED       (1U << 2)       // the saved fuel gauge state belonged to that model and was applied
#define EEPROM_STATUS_SAVE_SHIFT         (4U)            // bits 4-5: the last config or model save, EEPROM_SAVE_*
#define EEPROM_STATUS_SAVE_MASK          (3U << EEPROM_STATUS_SAVE_SHIFT)
#define EEPROM_STATUS_BLOCK_SHIFT        (6U)            // bits 6-7: what that save writes, EEPROM_BLOCK_*
#define EEPROM_STATUS_BLOCK_MASK         (3U << EEPROM_STATUS_BLOCK_SHIFT)

#define EEPROM_SAVE_IDLE                 (0U)            // nothing saved yet in this boot
#define EEPROM_SAVE_RUNNING              (1U)            // waiting for a running write, or pages being written (one per loop pass)
#define EEPROM_SAVE_DONE                 (2U)            // written
#define EEPROM_SAVE_FAILED               (3U)            // a write failed (EEPROM error flag set)

#define EEPROM_BLOCK_NONE                (0U)
#define EEPROM_BLOCK_CONFIG              (1U)            // CC_EEPROM_CONFIG
#define EEPROM_BLOCK_MODEL               (2U)            // CC_EEPROM_FUELGAUGE, end of an HPPC test, or CC_EEPROM_FG_STATE store
#define EEPROM_BLOCK_ERASE               (3U)            // CC_EEPROM_ERASE (whole EEPROM) or CC_EEPROM_FG_STATE erase (state only)

void    EEPROM_Init(Config_Data_t *config, FuelGauge_Data_t *fuelgauge);   // at boot, before the main loop: loads the config, the model and the state
void    EEPROM_Run(void);                                                  // once per loop pass: one step of a page write (the I2C runs by interrupt), never blocks
bool    EEPROM_SaveConfig(void);                                           // stores the config on the next passes; false while a save is running
bool    EEPROM_SaveFuelGaugeModel(void);                                   // stores the model on the next passes and stops the gauge; false while a save is running
bool    EEPROM_Erase(void);                                                // sets the whole EEPROM to 0xFF on the next passes and stops the gauge; false while a save is running
bool    EEPROM_SaveFuelGaugeState(const FuelGauge_State_t *state);       // stores a host state for the loaded model, false while saving or without a model
bool    EEPROM_EraseFuelGaugeState(void);                                  // sets only the fuel gauge state to 0xFF and stops the gauge; false while a save is running
void    EEPROM_StopFuelGauge(void);                                        // HPPC start: the gauge stops, its state is not saved any more
bool    EEPROM_FinalSave(void);                                            // shutdown / reset, every pass: true once every save and one last state save are written
bool    EEPROM_ConfigLoaded(void);
bool    EEPROM_FuelGaugeLoaded(void);                                      // the gauge runs (model loaded at boot, still in use)
bool    EEPROM_SaveRunning(void);                                          // a config or model save is waiting or being written
uint8_t EEPROM_GetStatus(void);                                            // EEPROM_STATUS_* (register 0x1D)

#ifdef __cplusplus
}
#endif

#endif // OPENBMS_EEPROM_H
