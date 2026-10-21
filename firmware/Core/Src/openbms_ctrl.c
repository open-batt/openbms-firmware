/**
  ******************************************************************************
  * @file           : openbms_ctrl.c
  * @brief          : OpenBMS Control Module
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

#include "openbms_ctrl.h"
#include "openbms_periph.h"
#include "openbms_error.h"
#include "openbms_fuelgauge.h"
#include "openbms_hppc.h"
#include <stddef.h>
#include <string.h>
#include <math.h>

// What is stored in the EEPROM: a version byte, then the Config_Data_t bytes
// (the CRC-32 is added by Periph_EepromWriteChecked). The version is kept at 1.
#define CONFIG_EEPROM_VERSION           1U

typedef struct __attribute__((packed))
{
    uint8_t        version;
    Config_Data_t  config;
} Config_Stored_t;

static Config_Data_t    config_data         = {0};
static System_Data_t    system_data         = {0};
static Stats_Data_t     stats_data          = {0};
static Config_Stored_t  stored_image        = {0};   // static: too big for the 1 KB stack
static bool             protection_reset    = false; // set by Ctrl_ResetProtection, handled in Protection_Check
#ifdef OPENBMS_SIMULATION
static uint32_t         sim_tick_ms         = 0;     // simulated time: the sum of the CC_SIM_SAMPLE time steps
#endif

static bool Ctrl_LoadConfig(void);

// Ctrl_SetConfig checks current_sensor_gain up to firmware_version as one block of floats
_Static_assert((offsetof(Config_Data_t, firmware_version) - offsetof(Config_Data_t, current_sensor_gain)) % sizeof(float) == 0,
               "the calibration block from current_sensor_gain to firmware_version must hold only floats");

// True when every calibration float is a real number. The packed struct may leave
// them unaligned, so each one is copied out before it is checked.
static bool Ctrl_CalibrationFinite(const Config_Data_t *data)
{
  const uint8_t *block = (const uint8_t *)data + offsetof(Config_Data_t, current_sensor_gain);
  uint32_t       count = (offsetof(Config_Data_t, firmware_version) - offsetof(Config_Data_t, current_sensor_gain)) / sizeof(float);
  float          value;

  for(uint32_t i = 0; i < count; i++)
  {
    memcpy(&value, block + i * sizeof(float), sizeof(float));
    if(!isfinite(value))
    {
      return false;
    }
  }

  return true;
}
// True when the cell count is one the hardware supports
static bool Ctrl_CellCountValid(uint8_t cell_count)
{
  return (cell_count >= BD_CELL_COUNT_MIN) && (cell_count <= BD_CELL_COUNT_MAX);
}

// True once any of the values has been past the threshold for longer than timeout_ms.
// over = true: past means above the threshold, over = false: below it. A timeout of 0 disables the check.
static bool Protection_Triggered(const float *value, uint8_t value_count, float threshold, bool over,
                                 uint32_t timeout_ms, uint32_t *timer, bool *triggered)
{
  bool threshold_exceeded = false;

  if(timeout_ms > 0)
  {
    for(uint8_t i = 0; i < value_count; i++)
    {
      if(over)
      {
        if(value[i] > threshold) threshold_exceeded = true;
      }
      else
      {
        if(value[i] < threshold) threshold_exceeded = true;
      }

      if(threshold_exceeded)
      {
        break;
      }
    }

    if(threshold_exceeded)
    {
      if(!(*triggered))
      {
        *timer = BD_GET_TICK();
        *triggered = true;
      }
      else 
      {
        if((BD_GET_TICK() - *timer) > timeout_ms)
        {
          return true;
        }
      }
    }
    else 
    {
      *triggered = false;
    }
  }

  return false;
}
static void Protection_Check(void)
{
  // Check for overvoltage, undervoltage, overcurrent, and overtemperature conditions
  // If any protection condition is met, take appropriate action (e.g., disable FETs)

  static uint32_t ovp_slow_timer        = 0;
  static uint32_t ovp_fast_timer        = 0;
  static uint32_t uvp_slow_timer        = 0;
  static uint32_t uvp_fast_timer        = 0;
  static uint32_t ocp_chg_timer         = 0;
  static uint32_t ocp_dchg_slow_timer   = 0;
  static uint32_t ocp_dchg_fast_timer   = 0;
  static uint32_t otp_timer             = 0;

  static bool ovp_slow_triggered        = false;
  static bool ovp_fast_triggered        = false;
  static bool uvp_slow_triggered        = false;
  static bool uvp_fast_triggered        = false;
  static bool ocp_chg_triggered         = false;
  static bool ocp_dchg_slow_triggered   = false;
  static bool ocp_dchg_fast_triggered   = false;
  static bool otp_triggered             = false;

  Peripheral_Data_t pd;
  Periph_GetData(&pd);

  // Host reset: forget the running detections, so every protection starts its detection time again
  if(protection_reset)
  {
    ovp_slow_triggered        = false;
    ovp_fast_triggered        = false;
    uvp_slow_triggered        = false;
    uvp_fast_triggered        = false;
    ocp_chg_triggered         = false;
    ocp_dchg_slow_triggered   = false;
    ocp_dchg_fast_triggered   = false;
    otp_triggered             = false;
    protection_reset          = false;
  }

  // Number of cells to check, from the config (never more than the hardware measures)
  uint8_t cell_count = config_data.cell_count;
  if(cell_count > BD_CELL_COUNT_MAX)
  {
    cell_count = BD_CELL_COUNT_MAX;
  }

  // Voltage protection enabled
  if(config_data.volt_protection)
  {
    system_data.protection_trigger |= Protection_Triggered(pd.cell_voltage, cell_count, config_data.ovp_slow_threshold_mv, true, 
                              config_data.ovp_slow_time_ms, &ovp_slow_timer, 
                              &ovp_slow_triggered);

    system_data.protection_trigger |= Protection_Triggered(pd.cell_voltage, cell_count, config_data.ovp_fast_threshold_mv, true, 
                              config_data.ovp_fast_time_ms, &ovp_fast_timer, 
                              &ovp_fast_triggered);

    system_data.protection_trigger |= Protection_Triggered(pd.cell_voltage, cell_count, config_data.uvp_slow_threshold_mv, false, 
                              config_data.uvp_slow_time_ms, &uvp_slow_timer, 
                              &uvp_slow_triggered);

    system_data.protection_trigger |= Protection_Triggered(pd.cell_voltage, cell_count, config_data.uvp_fast_threshold_mv, false, 
                              config_data.uvp_fast_time_ms, &uvp_fast_timer, 
                              &uvp_fast_triggered);
                              
  }
  else
  {
    // Not checked: forget the detections, so a re-enabled protection starts its detection time again
    ovp_slow_triggered        = false;
    ovp_fast_triggered        = false;
    uvp_slow_triggered        = false;
    uvp_fast_triggered        = false;
  }

  // Current protection enabled
  if(config_data.current_protection)
  {
    float c = pd.pack_current;         // positive while charging
    float d = -pd.pack_current;        // positive while discharging

    system_data.protection_trigger |= Protection_Triggered(&c, 1, config_data.ocp_charge_threshold_ma, true, 
                              config_data.ocp_charge_time_ms, &ocp_chg_timer, 
                              &ocp_chg_triggered);

    system_data.protection_trigger |= Protection_Triggered(&d, 1, config_data.ocp_discharge_slow_threshold_ma, true, 
                              config_data.ocp_discharge_slow_time_ms, &ocp_dchg_slow_timer, 
                              &ocp_dchg_slow_triggered);

    system_data.protection_trigger |= Protection_Triggered(&d, 1, config_data.ocp_discharge_fast_threshold_ma, true, 
                              config_data.ocp_discharge_fast_time_ms, &ocp_dchg_fast_timer, 
                              &ocp_dchg_fast_triggered);
  }
  else
  {
    ocp_chg_triggered         = false;
    ocp_dchg_slow_triggered   = false;
    ocp_dchg_fast_triggered   = false;
  }

  // Temperature protection enabled. With a broken NTC there is no temperature to check,
  // OPENBMS_NTC_FAULT tells the host instead.
  if(config_data.temp_protection && !Error_IsSet(OPENBMS_NTC_FAULT))
  {
    float t = pd.temperature_package;

    system_data.protection_trigger |= Protection_Triggered(&t, 1, config_data.otp_threshold_c, true, 
                              config_data.otp_time_ms, &otp_timer, 
                              &otp_triggered);    
  }
  else
  {
    otp_triggered             = false;
  }
}
void Ctrl_Init(void)
{
  Periph_Init();
  FuelGauge_Init();

  if(Ctrl_LoadConfig())
  {
    system_data.ctrl_state = BD_CTRL_STATE_NORMAL_RUN;
  }
  else
  {
    system_data.ctrl_state = BD_CTRL_STATE_WAIT_CONFIG;
  }
}
void Ctrl_Run(void)
{
  uint8_t         hppc_direction;
  uint16_t        hppc_status;
  uint16_t        hppc_extract_status;
  bool            run_fet;
  bool            run_pre_fet;
  bool            hppc_fet = false;

  // Errors are latched by the module that sees them, the register shows a copy
  system_data.error_flags = Error_GetFlags();

  // Uptime since this boot (the simulated time in the simulation build)
  system_data.session_uptime_ms = BD_GET_TICK();

  switch(system_data.ctrl_state)
  {
    case BD_CTRL_STATE_WAIT_CONFIG:
    {
      // Nothing to do here: Ctrl_SaveConfig leaves this state once the host has saved a config
    }
    break;

    case BD_CTRL_STATE_NORMAL_RUN:
    {
      // Fuel gauge: one update per loop tick
      FuelGauge_Run(config_data.cell_count);

      // Normal operation. main_control is the host's request, ctrl_state follows it.
      if(system_data.main_control == BD_MAIN_CTR_MODE_HPPC)
      {
        // The HPPC sequencer owns the FET from now on, a host request must not keep it on
        system_data.fet_request = false;
        system_data.ctrl_state  = BD_CTRL_STATE_HPPC_RUN;
      }
    }
    break;

    case BD_CTRL_STATE_HPPC_RUN:
    {
      if(system_data.main_control == BD_MAIN_CTR_MODE_NORMAL)
      {
        // Back to normal run: stop the test (FET off, sequencer reset) before leaving.
        // A finished test may still be building its tables, HPPC run is left when that is done.
        if(Hppc_Stop(&hppc_fet))
        {
          system_data.ctrl_state = BD_CTRL_STATE_NORMAL_RUN;
        }
      }
      else if(system_data.protection_trigger)
      {
        // A protection tripped: end the test, main_control goes back to normal
        Hppc_Abort(BD_HPPC_STOP_PROTECTION, &hppc_fet);
      }
      else
      {
        Hppc_Run(&hppc_fet);
      }

      // Show the sequencer result in the system registers
      Hppc_GetStatus(&hppc_direction, &hppc_status, &hppc_extract_status);
      system_data.hppc_direction      = hppc_direction;      // packed members, so assign through locals
      system_data.hppc_status         = hppc_status;
      system_data.hppc_extract_status = hppc_extract_status;
    }
    break;

    default: break;
  }

  // Measurement watchdog: no good ADS sample for a while. Only reported, the FET is not switched by it.
  if(!Periph_MeasurementOk())
  {
    Error_Set(OPENBMS_ADS131M08_DATA_LOST);
  }

  // Protections are checked on every pass (10 ms); their timeouts use BD_GET_TICK
  Protection_Check();

  // Set FET state
  run_fet = (!system_data.protection_trigger) && (hppc_fet | system_data.fet_request);
  Periph_SetFET(run_fet);

  // Set pre-FET state, only the host switches it
  run_pre_fet = (!system_data.protection_trigger) && system_data.pre_fet_request;
  Periph_SetPreFET(run_pre_fet);

  // Update peripherals state
  Periph_Run();
  
}
void Ctrl_GetConfig(Config_Data_t *data)
{
  memcpy(data, &config_data, sizeof(Config_Data_t));
}
void Ctrl_GetSystem(System_Data_t *data)
{
  memcpy(data, &system_data, sizeof(System_Data_t));
}
void Ctrl_GetStats(Stats_Data_t *data)
{
  memcpy(data, &stats_data, sizeof(Stats_Data_t));
}
bool Ctrl_SetConfig(const Config_Data_t *data)
{
  // A NaN or inf calibration value would make the measurements NaN and the protections
  // could never trip, so such a config is refused and the old one is kept
  if(!Ctrl_CalibrationFinite(data))
  {
    return false;
  }

  // Called from the main loop, like Protection_Check, so nothing reads the config meanwhile
  memcpy(&config_data, data, sizeof(Config_Data_t));

  // The sensor calibration in the config is used by the peripheral code
  Periph_SetCalibration(&config_data);
  return true;
}
bool Ctrl_ConfigValid(void)
{
  // The registers are written one by one, so the whole config is only checked before it is saved
  return Ctrl_CellCountValid(config_data.cell_count);
}
bool Ctrl_SaveConfig(void)
{
  stored_image.version = CONFIG_EEPROM_VERSION;
  memcpy(&stored_image.config, &config_data, sizeof(Config_Data_t));

  if(!Periph_EepromWriteChecked(CONFIG_EEPROM_ADDRESS, (const uint8_t *)&stored_image, sizeof(Config_Stored_t)))
  {
    return false;
  }

  // The host has set the device up and the config is safely stored
  if(system_data.ctrl_state == BD_CTRL_STATE_WAIT_CONFIG)
  {
    system_data.ctrl_state = BD_CTRL_STATE_NORMAL_RUN;
  }

  return true;
}
static bool Ctrl_LoadConfig(void)
{
  if(!Periph_EepromReadChecked(CONFIG_EEPROM_ADDRESS, (uint8_t *)&stored_image, sizeof(Config_Stored_t)))
  {
    return false;
  }

  if(stored_image.version != CONFIG_EEPROM_VERSION)
  {
    return false;
  }

  // An image with a bad cell count or calibration is treated like no config
  if(!Ctrl_CellCountValid(stored_image.config.cell_count))
  {
    return false;
  }

  return Ctrl_SetConfig(&stored_image.config);
}
void Ctrl_SetMode(uint16_t mode)
{
  system_data.main_control = mode;
}
void Ctrl_ResetProtection(void)
{
  // Called from the main loop (Comm_Run), like Protection_Check, so no masking is needed
  system_data.protection_trigger = false;
  protection_reset               = true;
}
void Ctrl_SetFetRequest(bool state)
{
  system_data.fet_request = state;
}
void Ctrl_SetPreFetRequest(bool state)
{
  system_data.pre_fet_request = state;
}
void Ctrl_SetLastCommunication(void)
{
  // Called from the main loop (Comm_Run) for every frame with a good CRC
  system_data.last_communication_timestamp = BD_GET_TICK();
}
#ifdef OPENBMS_SIMULATION
uint32_t Ctrl_SimGetTick(void)
{
  return sim_tick_ms;
}
void Ctrl_SimSample(const float *cell_mv, float current_ma, uint16_t dt_ms)
{
  // One host sample is one pass of the control loop, dt_ms later than the previous one
  sim_tick_ms += dt_ms;
  Periph_SimSetData(cell_mv, current_ma);
  Ctrl_Run();
}
#endif
