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
#include "openbms_balance.h"
#include <stddef.h>
#include <string.h>
#include <math.h>

// Charge over- and under-temperature are checked only above this charging current (noise at rest stays below)
#define CTRL_PROT_CHARGE_MIN_MA          (50.0f)

// What is stored in the EEPROM: a version byte, then the Config_Data_t bytes
// (the CRC-32 is added by Periph_EepromWriteChecked). CTRL_CONFIG_EEPROM_VERSION is in config.h.

typedef struct __attribute__((packed))
{
    uint8_t        version;
    Config_Data_t  config;
} Config_Stored_t;

static Config_Data_t    config_data             = {0};
static System_Data_t    system_data             = {0};
static Stats_Data_t     stats_data              = {0};
static FuelGauge_Data_t fuelgauge_data          = {0};
static Config_Stored_t  stored_image            = {0};
static bool             protection_reset        = false;
static bool             fuelgauge_model_loaded  = false;
#ifdef OPENBMS_SIMULATION
static uint32_t         sim_tick_ms             = 0;
#endif

static bool Ctrl_LoadConfig(void);
static bool Ctrl_SetConfig(const Config_Data_t *data);
static bool Ctrl_LoadFuelGaugeModel(void);
static void Ctrl_FaultSnapshot(void);

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
// True once any of the values has been past the threshold for longer than timeout_ms.
// over = true: past means above the threshold, over = false: below it. A timeout of 0 disables the check.
static bool Ctrl_ProtectionTriggered(const float *value, uint8_t value_count, float threshold, bool over,
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
        *timer = SYS_GET_TICK();
        *triggered = true;
      }
      else 
      {
        if((SYS_GET_TICK() - *timer) > timeout_ms)
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
static void Ctrl_ProtectionCheck(void)
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
  static uint32_t utp_timer             = 0;
  static uint32_t otp_chg_timer         = 0;
  static uint32_t utp_chg_timer         = 0;

  static bool ovp_slow_triggered        = false;
  static bool ovp_fast_triggered        = false;
  static bool uvp_slow_triggered        = false;
  static bool uvp_fast_triggered        = false;
  static bool ocp_chg_triggered         = false;
  static bool ocp_dchg_slow_triggered   = false;
  static bool ocp_dchg_fast_triggered   = false;
  static bool otp_triggered             = false;
  static bool utp_triggered             = false;
  static bool otp_chg_triggered         = false;
  static bool utp_chg_triggered         = false;

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
    utp_triggered             = false;
    otp_chg_triggered         = false;
    utp_chg_triggered         = false;
    protection_reset          = false;
  }

  // Number of cells to check, from the config (never more than the hardware measures)
  uint8_t cell_count = config_data.cell_count;
  if(cell_count > SYS_CELL_COUNT_MAX)
  {
    cell_count = SYS_CELL_COUNT_MAX;
  }

  // Voltage protection enabled
  if(config_data.volt_protection)
  {
    if(Ctrl_ProtectionTriggered(pd.cell_voltage, cell_count, config_data.ovp_slow_threshold_mv, true,
                                config_data.ovp_slow_time_ms, &ovp_slow_timer,
                                &ovp_slow_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_OVP_SLOW;
    }

    if(Ctrl_ProtectionTriggered(pd.cell_voltage, cell_count, config_data.ovp_fast_threshold_mv, true,
                                config_data.ovp_fast_time_ms, &ovp_fast_timer,
                                &ovp_fast_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_OVP_FAST;
    }

    if(Ctrl_ProtectionTriggered(pd.cell_voltage, cell_count, config_data.uvp_slow_threshold_mv, false,
                                config_data.uvp_slow_time_ms, &uvp_slow_timer,
                                &uvp_slow_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_UVP_SLOW;
    }

    if(Ctrl_ProtectionTriggered(pd.cell_voltage, cell_count, config_data.uvp_fast_threshold_mv, false,
                                config_data.uvp_fast_time_ms, &uvp_fast_timer,
                                &uvp_fast_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_UVP_FAST;
    }
                              
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

    if(Ctrl_ProtectionTriggered(&c, 1, config_data.ocp_charge_threshold_ma, true,
                                config_data.ocp_charge_time_ms, &ocp_chg_timer,
                                &ocp_chg_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_OCP_CHARGE;
    }

    if(Ctrl_ProtectionTriggered(&d, 1, config_data.ocp_discharge_slow_threshold_ma, true,
                                config_data.ocp_discharge_slow_time_ms, &ocp_dchg_slow_timer,
                                &ocp_dchg_slow_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_OCP_DISCHARGE_SLOW;
    }

    if(Ctrl_ProtectionTriggered(&d, 1, config_data.ocp_discharge_fast_threshold_ma, true,
                                config_data.ocp_discharge_fast_time_ms, &ocp_dchg_fast_timer,
                                &ocp_dchg_fast_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_OCP_DISCHARGE_FAST;
    }
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

    if(Ctrl_ProtectionTriggered(&t, 1, config_data.otp_threshold_c, true,
                                config_data.otp_time_ms, &otp_timer,
                                &otp_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_OTP;
    }

    if(Ctrl_ProtectionTriggered(&t, 1, config_data.utp_threshold_c, false,
                                config_data.utp_time_ms, &utp_timer,
                                &utp_triggered))
    {
      system_data.protection_trigger |= CTRL_PROT_UTP;
    }

    // Charge over- and under-temperature: only while charging current flows, the detection
    // starts again when charging stops
    if(pd.pack_current > CTRL_PROT_CHARGE_MIN_MA)
    {
      if(Ctrl_ProtectionTriggered(&t, 1, config_data.otp_charge_threshold_c, true,
                                  config_data.otp_time_ms, &otp_chg_timer,
                                  &otp_chg_triggered))
      {
        system_data.protection_trigger |= CTRL_PROT_OTP_CHARGE;
      }

      if(Ctrl_ProtectionTriggered(&t, 1, config_data.utp_charge_threshold_c, false,
                                  config_data.utp_time_ms, &utp_chg_timer,
                                  &utp_chg_triggered))
      {
        system_data.protection_trigger |= CTRL_PROT_UTP_CHARGE;
      }
    }
    else
    {
      otp_chg_triggered       = false;
      utp_chg_triggered       = false;
    }
  }
  else
  {
    otp_triggered             = false;
    utp_triggered             = false;
    otp_chg_triggered         = false;
    utp_chg_triggered         = false;
  }
}
void Ctrl_Init(void)
{
  Periph_Init();

  // Load config data
  if(Ctrl_LoadConfig())
  {
    system_data.ctrl_state = CTRL_STATE_NORMAL_RUN;
  }
  else
  {
    system_data.ctrl_state = CTRL_STATE_WAIT_CONFIG;
  }

  // Initialize fuel gauge model
  FuelGauge_Init(&fuelgauge_data, &config_data);
  Balance_Init(&config_data);
  if(Ctrl_LoadFuelGaugeModel() && FuelGauge_IsReady())
  {
    fuelgauge_model_loaded = true;
  }
}
void Ctrl_Run(void)
{
  uint8_t         hppc_direction;
  uint16_t        hppc_status;
  uint16_t        hppc_extract_status;
  bool            run_fet;
  bool            run_pre_fet;
  bool            run_balance;
  bool            hppc_fet = false;

  // Errors are latched by the module that sees them, the register shows a copy
  system_data.error_flags = Error_GetFlags();

  // Uptime since this boot (the simulated time in the simulation build)
  system_data.session_uptime_ms = SYS_GET_TICK();

  switch(system_data.ctrl_state)
  {
    case CTRL_STATE_NORMAL_RUN:
    {
      // Fuel gauge: one update per loop tick, only once the host has loaded its model
      if(fuelgauge_model_loaded)
      {
        FuelGauge_Run();
      }

      // Normal operation. main_control is the host's request, ctrl_state follows it.
      if(system_data.main_control == CTRL_MODE_HPPC)
      {
        // The HPPC sequencer owns the FET from now on, a host request must not keep it or the pre-FET on
        system_data.fet_request     = false;
        system_data.pre_fet_request = false;
        system_data.ctrl_state      = CTRL_STATE_HPPC_RUN;
      }
    }
    break;

    case CTRL_STATE_HPPC_RUN:
    {
      if(system_data.main_control == CTRL_MODE_NORMAL)
      {
        // The test has ended (by itself, by a protection or by the host): FET off, a running test is
        // aborted, a finished one still builds its tables. The device stays in HPPC run until a
        // reboot, which also starts the fuel gauge fresh on the new tables.
        Hppc_Stop(&hppc_fet);
      }
      else if(system_data.protection_trigger != 0)
      {
        // A protection tripped: end the test, main_control goes back to normal
        Hppc_Abort(HPPC_STOP_PROTECTION, &hppc_fet);
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

    case CTRL_STATE_WAIT_CONFIG:
    {
      // Nothing to do here: Ctrl_SaveConfig leaves this state once the host has saved a config
    }
    break;

    default: break;
  }

  // Protections are checked on every pass (10 ms); their timeouts use SYS_GET_TICK
  Ctrl_ProtectionCheck();
  Ctrl_FaultSnapshot();

  // Set FET state
  run_fet = (system_data.protection_trigger == 0) && (hppc_fet | system_data.fet_request);
  Periph_SetFET(run_fet);

  // Set pre-FET state, only the host switches it
  run_pre_fet = (system_data.protection_trigger == 0) && system_data.pre_fet_request;
  Periph_SetPreFET(run_pre_fet);

  // Cell balancing: only in normal run and with no protection tripped
  run_balance = (system_data.ctrl_state == CTRL_STATE_NORMAL_RUN) && (system_data.protection_trigger == 0);
  Periph_SetBalance(Balance_Run(run_balance));

  // Update peripherals state
  Periph_Run();
  
}
const Config_Data_t *Ctrl_GetConfig(void)
{
  return &config_data;
}
const System_Data_t *Ctrl_GetSystem(void)
{
  return &system_data;
}
const Stats_Data_t *Ctrl_GetStats(void)
{
  return &stats_data;
}
const FuelGauge_Data_t *Ctrl_GetFuelGauge(void)
{
  return &fuelgauge_data;
}
// Writes one field of a struct owned by this module. field points into the struct, length is
// the size of that field. Floats between block_start and block_end must be real numbers.
// Called from the main loop, like Ctrl_Run, so nothing reads the struct meanwhile.
static bool Ctrl_WriteField(void *base, uint32_t size, uint32_t block_start, uint32_t block_end,
                            const void *field, const uint8_t *data, uint8_t length)
{
  uint32_t offset = (uint32_t)((const uint8_t *)field - (const uint8_t *)base);
  float    value;

  if(offset + length > size)
  {
    return false;
  }

  if(offset >= block_start && offset < block_end)
  {
    for(uint32_t i = 0; i + sizeof(float) <= length; i += sizeof(float))
    {
      memcpy(&value, data + i, sizeof(float));
      if(!isfinite(value))
      {
        return false;
      }
    }
  }

  memcpy((uint8_t *)base + offset, data, length);
  return true;
}
bool Ctrl_WriteConfig(const void *field, const uint8_t *data, uint8_t length)
{
  // A NaN or inf calibration value would make the measurements NaN and the protections
  // could never trip, so such a value is refused and the old one is kept
  return Ctrl_WriteField(&config_data, sizeof(Config_Data_t),
                         offsetof(Config_Data_t, current_sensor_gain), offsetof(Config_Data_t, firmware_version),
                         field, data, length);
}
bool Ctrl_WriteFuelGauge(const void *field, const uint8_t *data, uint8_t length)
{
  // The filter noise values (kf_q_soc, kf_q_rc1, kf_q_rc2, kf_r_v) have to be above 0: at 0 or
  // below the covariance can go negative or the gain divides by 0 while the gauge runs
  if(field == &fuelgauge_data.kf_q_soc || field == &fuelgauge_data.kf_q_rc1 ||
     field == &fuelgauge_data.kf_q_rc2 || field == &fuelgauge_data.kf_r_v)
  {
    float value;

    if(length != sizeof(float))
    {
      return false;
    }
    memcpy(&value, data, sizeof(float));
    if(!(value > 0.0f))
    {
      return false;
    }
  }

  // Every float of the model and the filter state (soc_grid up to cycle_count) has to be a
  // real number, one NaN or inf would end up in the SOC
  return Ctrl_WriteField(&fuelgauge_data, sizeof(FuelGauge_Data_t),
                         offsetof(FuelGauge_Data_t, soc_grid), offsetof(FuelGauge_Data_t, cycle_count),
                         field, data, length);
}
static bool Ctrl_SetConfig(const Config_Data_t *data)
{
  // A NaN or inf calibration value would make the measurements NaN and the protections
  // could never trip, so such a config is refused and the old one is kept
  if(!Ctrl_CalibrationFinite(data))
  {
    return false;
  }

  // Called from the main loop, like Ctrl_ProtectionCheck, so nothing reads the config meanwhile
  memcpy(&config_data, data, sizeof(Config_Data_t));

  return true;
}
bool Ctrl_SaveConfig(void)
{
  stored_image.version = CTRL_CONFIG_EEPROM_VERSION;
  memcpy(&stored_image.config, &config_data, sizeof(Config_Data_t));

  if(!Periph_EepromWriteChecked(CTRL_CONFIG_EEPROM_ADDRESS, (const uint8_t *)&stored_image, sizeof(Config_Stored_t)))
  {
    return false;
  }

  return true;
}
static bool Ctrl_LoadConfig(void)
{
  if(!Periph_EepromReadChecked(CTRL_CONFIG_EEPROM_ADDRESS, (uint8_t *)&stored_image, sizeof(Config_Stored_t)))
  {
    return false;
  }

  if(stored_image.version != CTRL_CONFIG_EEPROM_VERSION)
  {
    return false;
  }

  if(Ctrl_SetConfig(&stored_image.config))
  {
      // The sensor calibration in the config is used by the peripheral code
    Periph_SetCalibration(&config_data);
    return true;
  }
  else
  {
    return false;
  }
}
static bool Ctrl_LoadFuelGaugeModel(void)
{
  // Placeholder until the model is stored in the EEPROM: the model from
  // python_scripts/simulation/fuel_gauge_registers.json (7 cells, 2000 mAh, HPPC charge and
  // discharge tests). The SOC outputs and the filter state start at 0, the gauge seeds from OCV.
  static const FuelGauge_Data_t default_model =
  {
    .cell_soh             = {100, 100, 100, 100, 100, 100, 100},
    .soc_grid             = {2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f, 15.0f, 20.0f, 25.0f, 30.0f, 40.0f, 50.0f, 60.0f, 70.0f, 80.0f, 85.0f, 90.0f, 95.0f, 98.0f, 100.0f},
    .cell                 =
    {
        {   // cell 1
            .ocv_dis     = {3.405089f, 3.448374f, 3.478893f, 3.50115f, 3.518542f, 3.533046f, 3.550746f, 3.574396f, 3.5941f, 3.613455f, 3.655965f, 3.707405f, 3.771221f, 3.848885f, 3.941454f, 3.99354f, 4.050124f, 4.114208f, 4.159432f, 4.2f},
            .ocv_chg     = {3.279638f, 3.377224f, 3.430172f, 3.465936f, 3.492056f, 3.512006f, 3.535033f, 3.56339f, 3.585073f, 3.604816f, 3.646331f, 3.694691f, 3.75538f, 3.829549f, 3.918363f, 3.968113f, 4.022311f, 4.08482f, 4.135031f, 4.2f},
            .r0_dis      = {0.03631596f, 0.03546426f, 0.03462311f, 0.03356038f, 0.03258171f, 0.03171544f, 0.03148632f, 0.0311013f, 0.03073403f, 0.03089555f, 0.03163586f, 0.03144018f, 0.03238956f, 0.03252966f, 0.03271822f, 0.03259064f, 0.03292174f, 0.03331884f, 0.03341895f, 0.03341895f},
            .r1_dis      = {0.01726429f, 0.01654538f, 0.01575003f, 0.0149113f, 0.0142343f, 0.01368224f, 0.0131942f, 0.012429f, 0.01176431f, 0.01136449f, 0.01095889f, 0.0107174f, 0.01062148f, 0.01062244f, 0.01064f, 0.01062008f, 0.01058939f, 0.0106003f, 0.01062622f, 0.01062622f},
            .tau1_dis    = {11.40495f, 11.57309f, 11.68962f, 11.67724f, 11.63082f, 11.59737f, 11.59769f, 11.61714f, 11.73402f, 11.92027f, 12.33828f, 12.89086f, 13.58485f, 14.40161f, 15.257f, 15.81057f, 16.23545f, 16.66205f, 16.89499f, 16.89499f},
            .r2_dis      = {0.01670051f, 0.01606106f, 0.01591723f, 0.01653985f, 0.01724635f, 0.01779031f, 0.01790956f, 0.01797193f, 0.01802147f, 0.01805155f, 0.01790243f, 0.0174068f, 0.01652373f, 0.01518612f, 0.0134418f, 0.01201928f, 0.010617f, 0.008988257f, 0.008027744f, 0.008027744f},
            .tau2_dis    = {108.295f, 109.5499f, 111.2463f, 112.4749f, 113.1746f, 113.7068f, 114.2605f, 115.0851f, 116.1778f, 117.3854f, 119.6923f, 122.3718f, 125.2633f, 127.8655f, 129.1328f, 128.4802f, 127.0077f, 126.1294f, 126.1298f, 126.1298f},
            .r0_chg      = {0.04344456f, 0.04158172f, 0.03954249f, 0.0373247f, 0.03600048f, 0.0351756f, 0.03420138f, 0.03333517f, 0.03190219f, 0.03131242f, 0.0319492f, 0.03120853f, 0.03124565f, 0.03176611f, 0.03185454f, 0.03209965f, 0.03292443f, 0.03305211f, 0.03305211f, 0.03305211f},
            .r1_chg      = {0.0191197f, 0.01864649f, 0.01782547f, 0.01684237f, 0.01578091f, 0.01492871f, 0.01403912f, 0.01305441f, 0.01220381f, 0.01155209f, 0.01104515f, 0.01080113f, 0.01071602f, 0.01063031f, 0.01055039f, 0.01051517f, 0.01050428f, 0.01050318f, 0.01050318f, 0.01050318f},
            .tau1_chg    = {11.54476f, 11.64498f, 11.77484f, 11.85699f, 11.89041f, 11.90706f, 11.93469f, 12.00708f, 12.14739f, 12.32224f, 12.54582f, 12.8553f, 13.28608f, 13.80543f, 14.34799f, 14.79691f, 15.12349f, 15.16724f, 15.16724f, 15.16724f},
            .r2_chg      = {0.01772902f, 0.01714837f, 0.01622447f, 0.01543211f, 0.01524902f, 0.01566923f, 0.01644279f, 0.01721164f, 0.01743822f, 0.01750506f, 0.01771678f, 0.01777655f, 0.01749019f, 0.01686322f, 0.01583154f, 0.01468419f, 0.01374522f, 0.01361694f, 0.01361694f, 0.01361694f},
            .tau2_chg    = {110.4532f, 111.2937f, 112.2586f, 112.648f, 112.6979f, 112.9316f, 113.5191f, 114.5638f, 115.7014f, 116.8777f, 118.8092f, 121.7365f, 124.8474f, 126.9188f, 127.7935f, 128.1477f, 128.5512f, 128.6095f, 128.6095f, 128.6095f},
            .capacity_ah = 1.944197f,
        },
        {   // cell 2
            .ocv_dis     = {3.376199f, 3.429923f, 3.465599f, 3.49177f, 3.511287f, 3.526813f, 3.545943f, 3.570973f, 3.591403f, 3.610973f, 3.65311f, 3.704615f, 3.768324f, 3.846368f, 3.939422f, 3.991849f, 4.049049f, 4.113872f, 4.159289f, 4.2f},
            .ocv_chg     = {3.279274f, 3.377338f, 3.430633f, 3.466146f, 3.492105f, 3.512174f, 3.535387f, 3.563672f, 3.585074f, 3.60481f, 3.645896f, 3.694895f, 3.756022f, 3.830678f, 3.919767f, 3.96959f, 4.02414f, 4.086275f, 4.135936f, 4.2f},
            .r0_dis      = {0.0374821f, 0.03560312f, 0.03482899f, 0.03352644f, 0.03267131f, 0.03238323f, 0.03175414f, 0.03117559f, 0.03078717f, 0.03076861f, 0.03146409f, 0.03194864f, 0.03192575f, 0.03242234f, 0.03299731f, 0.0332434f, 0.03313874f, 0.03269667f, 0.03301187f, 0.03301187f},
            .r1_dis      = {0.0180697f, 0.01727508f, 0.01632244f, 0.01528418f, 0.01444002f, 0.01375406f, 0.01319806f, 0.01242309f, 0.01184852f, 0.01151257f, 0.01113524f, 0.01085745f, 0.01067473f, 0.01056046f, 0.01050519f, 0.01052808f, 0.01063504f, 0.01079508f, 0.01087806f, 0.01087806f},
            .tau1_dis    = {11.33596f, 11.50741f, 11.70893f, 11.83099f, 11.8394f, 11.80734f, 11.77579f, 11.75553f, 11.7913f, 11.88593f, 12.18723f, 12.68642f, 13.27255f, 13.90257f, 14.66116f, 15.33461f, 16.06436f, 16.95506f, 17.50022f, 17.50022f},
            .r2_dis      = {0.01730151f, 0.01660036f, 0.01638759f, 0.01693407f, 0.017596f, 0.01811527f, 0.01820693f, 0.01819438f, 0.01815775f, 0.01814002f, 0.01795432f, 0.0174507f, 0.01661718f, 0.01536862f, 0.01365864f, 0.01217633f, 0.01065678f, 0.00888712f, 0.007847704f, 0.007847704f},
            .tau2_dis    = {109.559f, 110.3564f, 111.6645f, 112.9489f, 113.529f, 113.749f, 113.978f, 114.8618f, 116.4265f, 117.8392f, 120.0214f, 122.2167f, 123.9665f, 125.0019f, 125.7133f, 126.4499f, 127.796f, 130.0175f, 131.4738f, 131.4738f},
            .r0_chg      = {0.04389835f, 0.04166088f, 0.03948767f, 0.03744865f, 0.03627867f, 0.03542754f, 0.03421432f, 0.0334125f, 0.0318275f, 0.03131158f, 0.03182974f, 0.03095022f, 0.03110645f, 0.03219616f, 0.03248264f, 0.03176784f, 0.03251314f, 0.03290376f, 0.03290376f, 0.03290376f},
            .r1_chg      = {0.01934123f, 0.0187378f, 0.0177474f, 0.01667902f, 0.01561291f, 0.0147547f, 0.01386036f, 0.01293644f, 0.01224764f, 0.01176925f, 0.01134462f, 0.01100774f, 0.01079061f, 0.01068409f, 0.01064356f, 0.01062773f, 0.01062347f, 0.01062335f, 0.01062335f, 0.01062335f},
            .tau1_chg    = {11.37771f, 11.39337f, 11.41688f, 11.45274f, 11.51304f, 11.5909f, 11.71076f, 11.8998f, 12.09438f, 12.30583f, 12.64568f, 13.02766f, 13.39881f, 13.84542f, 14.36491f, 14.798f, 15.16704f, 15.26328f, 15.26328f, 15.26328f},
            .r2_chg      = {0.01770905f, 0.01721753f, 0.01639283f, 0.01563281f, 0.01540959f, 0.01578057f, 0.01652697f, 0.0172368f, 0.01738336f, 0.01737503f, 0.01753329f, 0.01761525f, 0.01740306f, 0.01683278f, 0.01583305f, 0.01477709f, 0.01379009f, 0.01352474f, 0.01352474f, 0.01352474f},
            .tau2_chg    = {109.6066f, 109.3644f, 109.0533f, 109.0298f, 109.4522f, 110.1558f, 111.3189f, 113.3791f, 115.958f, 118.6066f, 121.4084f, 123.7745f, 125.4225f, 126.6567f, 127.5234f, 128.0429f, 128.5676f, 128.7183f, 128.7183f, 128.7183f},
            .capacity_ah = 1.944208f,
        },
        {   // cell 3
            .ocv_dis     = {3.438029f, 3.470858f, 3.494821f, 3.513425f, 3.528517f, 3.541258f, 3.557061f, 3.579083f, 3.598497f, 3.617803f, 3.659987f, 3.71182f, 3.775328f, 3.852735f, 3.944297f, 3.995891f, 4.052243f, 4.115796f, 4.160191f, 4.2f},
            .ocv_chg     = {3.279987f, 3.377266f, 3.430169f, 3.465493f, 3.491885f, 3.511799f, 3.534598f, 3.563104f, 3.58481f, 3.604495f, 3.645219f, 3.693925f, 3.753914f, 3.827491f, 3.915532f, 3.964442f, 4.018632f, 4.082767f, 4.134153f, 4.2f},
            .r0_dis      = {0.03560004f, 0.03500759f, 0.03383516f, 0.03316634f, 0.03234001f, 0.03127621f, 0.03123746f, 0.03147881f, 0.03156008f, 0.03152443f, 0.03151473f, 0.03138714f, 0.03239542f, 0.03214109f, 0.03249793f, 0.03308792f, 0.03321276f, 0.03312947f, 0.03305088f, 0.03305088f},
            .r1_dis      = {0.01673308f, 0.01606407f, 0.0152572f, 0.0144019f, 0.01374387f, 0.01322789f, 0.01280488f, 0.01217483f, 0.01163749f, 0.01129449f, 0.01090459f, 0.01063171f, 0.01045241f, 0.01037055f, 0.01042634f, 0.01055533f, 0.01068995f, 0.01075553f, 0.01074855f, 0.01074855f},
            .tau1_dis    = {11.6236f, 11.71983f, 11.73674f, 11.67184f, 11.61303f, 11.57768f, 11.60133f, 11.72686f, 11.99678f, 12.26956f, 12.71111f, 13.14078f, 13.58065f, 14.09381f, 14.76039f, 15.40828f, 16.06745f, 16.62211f, 16.8473f, 16.8473f},
            .r2_dis      = {0.01571599f, 0.01520879f, 0.01522483f, 0.01597862f, 0.01677073f, 0.01738137f, 0.01756887f, 0.01771581f, 0.01780617f, 0.0178499f, 0.01772037f, 0.01727625f, 0.01650146f, 0.01529072f, 0.0135957f, 0.01211529f, 0.01061877f, 0.008953308f, 0.008022513f, 0.008022513f},
            .tau2_dis    = {112.8103f, 113.032f, 112.653f, 112.1834f, 112.174f, 112.4038f, 112.8615f, 114.0078f, 115.9187f, 117.7371f, 120.5452f, 122.9454f, 124.4255f, 125.1268f, 126.0363f, 127.5065f, 128.9591f, 128.8342f, 127.8448f, 127.8448f},
            .r0_chg      = {0.04384991f, 0.04138633f, 0.03932757f, 0.03752116f, 0.03565302f, 0.03460986f, 0.03383084f, 0.03308933f, 0.0327046f, 0.03241767f, 0.03119413f, 0.0312401f, 0.03196102f, 0.03151636f, 0.03146067f, 0.03234415f, 0.03215539f, 0.03215539f, 0.03215539f, 0.03215539f},
            .r1_chg      = {0.01927413f, 0.01868359f, 0.01773199f, 0.01667807f, 0.01561411f, 0.01480352f, 0.01395804f, 0.01300125f, 0.01220921f, 0.01165065f, 0.011186f, 0.01087134f, 0.01065243f, 0.01050709f, 0.01045542f, 0.01048891f, 0.01054603f, 0.01054603f, 0.01054603f, 0.01054603f},
            .tau1_chg    = {11.24339f, 11.27842f, 11.37593f, 11.5112f, 11.61027f, 11.62794f, 11.61115f, 11.61443f, 11.74176f, 11.96194f, 12.31804f, 12.82373f, 13.43732f, 14.01087f, 14.5335f, 14.97561f, 15.27193f, 15.27193f, 15.27193f, 15.27193f},
            .r2_chg      = {0.01783467f, 0.01731247f, 0.01644758f, 0.01567142f, 0.01550736f, 0.01595437f, 0.01671711f, 0.01746581f, 0.01765238f, 0.01766649f, 0.01779736f, 0.01778637f, 0.01745983f, 0.01680945f, 0.01570057f, 0.01448513f, 0.01361165f, 0.01361165f, 0.01361165f, 0.01361165f},
            .tau2_chg    = {108.3229f, 108.4165f, 108.8718f, 109.5043f, 110.2533f, 111.1083f, 112.2292f, 113.65f, 115.1534f, 116.8587f, 119.1772f, 121.6218f, 123.9751f, 125.9651f, 127.4679f, 128.4266f, 128.8839f, 128.8839f, 128.8839f, 128.8839f},
            .capacity_ah = 1.944727f,
        },
        {   // cell 4
            .ocv_dis     = {3.311692f, 3.393909f, 3.441232f, 3.473891f, 3.497832f, 3.516486f, 3.538084f, 3.565206f, 3.586808f, 3.606886f, 3.648876f, 3.700297f, 3.763947f, 3.842249f, 3.936079f, 3.989296f, 4.047289f, 4.112724f, 4.158814f, 4.2f},
            .ocv_chg     = {3.278769f, 3.377687f, 3.430696f, 3.466467f, 3.492628f, 3.512613f, 3.535616f, 3.563653f, 3.585614f, 3.605214f, 3.646743f, 3.696265f, 3.757273f, 3.832484f, 3.922247f, 3.97207f, 4.026918f, 4.08885f, 4.137613f, 4.2f},
            .r0_dis      = {0.03842714f, 0.03671748f, 0.03538644f, 0.03405192f, 0.03291063f, 0.03197712f, 0.03189507f, 0.03199324f, 0.03155585f, 0.03134127f, 0.03183678f, 0.03163046f, 0.03196181f, 0.03237858f, 0.03245169f, 0.03294237f, 0.0330112f, 0.03308827f, 0.03309761f, 0.03309761f},
            .r1_dis      = {0.01918581f, 0.01829844f, 0.01712793f, 0.01583544f, 0.01484156f, 0.01407002f, 0.01345726f, 0.01257403f, 0.01187996f, 0.0114658f, 0.01101204f, 0.01068641f, 0.01047782f, 0.01039694f, 0.01044875f, 0.01053532f, 0.01061186f, 0.0106781f, 0.01071011f, 0.01071011f},
            .tau1_dis    = {11.19479f, 11.33523f, 11.44356f, 11.50214f, 11.54057f, 11.57933f, 11.66372f, 11.84946f, 12.10202f, 12.3371f, 12.74194f, 13.16484f, 13.57324f, 14.07164f, 14.72661f, 15.26375f, 15.78848f, 16.4236f, 16.82733f, 16.82733f},
            .r2_dis      = {0.01840742f, 0.01753334f, 0.01722465f, 0.01772957f, 0.01833078f, 0.01877059f, 0.01873606f, 0.01851792f, 0.01830817f, 0.01819526f, 0.01793007f, 0.01741396f, 0.0165987f, 0.01534184f, 0.01361418f, 0.01215512f, 0.010706f, 0.00903887f, 0.008051713f, 0.008051713f},
            .tau2_dis    = {110.5064f, 110.7989f, 110.8322f, 111.0208f, 111.4875f, 112.0438f, 112.8116f, 114.4404f, 116.5261f, 118.1833f, 120.4314f, 122.2424f, 123.3167f, 124.0915f, 125.4587f, 126.8158f, 127.7613f, 128.5551f, 129.169f, 129.169f},
            .r0_chg      = {0.04380661f, 0.0417476f, 0.03920824f, 0.03714977f, 0.03574788f, 0.03484178f, 0.03377358f, 0.03268731f, 0.03224333f, 0.03154926f, 0.03139345f, 0.0307838f, 0.03143539f, 0.03190932f, 0.0319449f, 0.03223106f, 0.03269321f, 0.0329084f, 0.0329084f, 0.0329084f},
            .r1_chg      = {0.01918656f, 0.01861028f, 0.01765642f, 0.01662753f, 0.01561998f, 0.01481585f, 0.01399096f, 0.01310285f, 0.01230742f, 0.01163308f, 0.01111535f, 0.01083139f, 0.01069947f, 0.01061276f, 0.01057318f, 0.01057599f, 0.01058964f, 0.01059545f, 0.01059545f, 0.01059545f},
            .tau1_chg    = {11.36496f, 11.40965f, 11.47251f, 11.53655f, 11.60958f, 11.67526f, 11.74176f, 11.7991f, 11.83727f, 11.90303f, 12.12168f, 12.58809f, 13.25946f, 13.97966f, 14.6098f, 14.95889f, 15.20176f, 15.26342f, 15.26342f, 15.26342f},
            .r2_chg      = {0.01765447f, 0.0171651f, 0.01633313f, 0.01555062f, 0.01526197f, 0.0155713f, 0.01631682f, 0.01707925f, 0.01733141f, 0.01744495f, 0.01769098f, 0.01773377f, 0.01742422f, 0.01682003f, 0.01586784f, 0.01497965f, 0.01397635f, 0.01353368f, 0.01353368f, 0.01353368f},
            .tau2_chg    = {110.0329f, 109.9619f, 109.7217f, 109.3898f, 109.4666f, 110.2268f, 111.7569f, 113.7913f, 115.354f, 116.5879f, 118.237f, 120.7051f, 123.602f, 126.2675f, 128.4909f, 129.6862f, 130.3941f, 130.492f, 130.492f, 130.492f},
            .capacity_ah = 1.943889f,
        },
        {   // cell 5
            .ocv_dis     = {3.417366f, 3.456199f, 3.484435f, 3.505621f, 3.52212f, 3.535817f, 3.552773f, 3.575986f, 3.595706f, 3.615021f, 3.657249f, 3.708784f, 3.772678f, 3.850128f, 3.942403f, 3.994385f, 4.050937f, 4.114846f, 4.159774f, 4.2f},
            .ocv_chg     = {3.279691f, 3.377348f, 3.430311f, 3.465889f, 3.491804f, 3.511904f, 3.535131f, 3.5632f, 3.584778f, 3.604647f, 3.645644f, 3.694096f, 3.755127f, 3.829218f, 3.917017f, 3.96699f, 4.021152f, 4.083919f, 4.134436f, 4.2f},
            .r0_dis      = {0.03607225f, 0.03500986f, 0.03377496f, 0.03311161f, 0.03273845f, 0.03235187f, 0.03219858f, 0.03178225f, 0.03168337f, 0.03180267f, 0.03176796f, 0.03166667f, 0.03257168f, 0.03273248f, 0.0325713f, 0.03277048f, 0.03283594f, 0.03306378f, 0.03314433f, 0.03314433f},
            .r1_dis      = {0.01700029f, 0.0162876f, 0.01549111f, 0.01461995f, 0.01390874f, 0.01333281f, 0.01286182f, 0.01217374f, 0.01160319f, 0.01124471f, 0.01085488f, 0.01064203f, 0.01058276f, 0.01059947f, 0.01063569f, 0.01064592f, 0.01065043f, 0.01066642f, 0.01067417f, 0.01067417f},
            .tau1_dis    = {11.31969f, 11.41695f, 11.48488f, 11.46344f, 11.40889f, 11.36397f, 11.40374f, 11.54914f, 11.76852f, 11.96459f, 12.3423f, 12.90453f, 13.5678f, 14.204f, 14.87232f, 15.46874f, 16.07839f, 16.74093f, 17.12156f, 17.12156f},
            .r2_dis      = {0.01644274f, 0.01582033f, 0.01572766f, 0.01641733f, 0.01717919f, 0.01776694f, 0.01790586f, 0.01797061f, 0.01801546f, 0.01805762f, 0.01793543f, 0.01743226f, 0.01653848f, 0.01522572f, 0.0134937f, 0.01203659f, 0.01057999f, 0.008911518f, 0.007947056f, 0.007947056f},
            .tau2_dis    = {107.3896f, 108.71f, 110.412f, 111.6661f, 112.2244f, 112.5364f, 113.0785f, 114.3986f, 116.1554f, 117.5623f, 119.7222f, 122.2677f, 124.7896f, 126.6678f, 127.927f, 128.4396f, 128.2336f, 127.2804f, 126.5573f, 126.5573f},
            .r0_chg      = {0.04374827f, 0.04170296f, 0.03906034f, 0.03759504f, 0.03618269f, 0.03519486f, 0.03419197f, 0.03268095f, 0.03230763f, 0.03164471f, 0.03127314f, 0.03187955f, 0.03124776f, 0.03137526f, 0.03219146f, 0.03241903f, 0.03248119f, 0.03248521f, 0.03248521f, 0.03248521f},
            .r1_chg      = {0.01915444f, 0.01858918f, 0.01767865f, 0.01670418f, 0.01573178f, 0.01495372f, 0.01411761f, 0.01317594f, 0.01240135f, 0.01183574f, 0.01132453f, 0.01090738f, 0.01062639f, 0.01051706f, 0.01052053f, 0.01052647f, 0.01050466f, 0.01050288f, 0.01050288f, 0.01050288f},
            .tau1_chg    = {11.29864f, 11.33574f, 11.41732f, 11.512f, 11.57863f, 11.62255f, 11.69802f, 11.88497f, 12.14208f, 12.36076f, 12.55037f, 12.76016f, 13.0695f, 13.56242f, 14.15517f, 14.57254f, 14.74356f, 14.75546f, 14.75546f, 14.75546f},
            .r2_chg      = {0.01785473f, 0.01731435f, 0.01641222f, 0.01558352f, 0.01535143f, 0.01575135f, 0.01650185f, 0.01723411f, 0.01741179f, 0.01744623f, 0.01765917f, 0.01777098f, 0.01755171f, 0.01694498f, 0.01587424f, 0.01467136f, 0.01377972f, 0.01371346f, 0.01371346f, 0.01371346f},
            .tau2_chg    = {108.2107f, 108.5405f, 109.2947f, 110.3372f, 111.3696f, 112.1773f, 113.2144f, 114.9764f, 117.0276f, 118.8182f, 120.6219f, 122.1022f, 123.3683f, 125.1032f, 126.582f, 126.6299f, 125.8539f, 125.7914f, 125.7914f, 125.7914f},
            .capacity_ah = 1.944484f,
        },
        {   // cell 6
            .ocv_dis     = {3.358454f, 3.419246f, 3.458138f, 3.486003f, 3.506837f, 3.523428f, 3.543302f, 3.569135f, 3.589804f, 3.60946f, 3.651966f, 3.703311f, 3.766949f, 3.84486f, 3.938313f, 3.991004f, 4.048418f, 4.11335f, 4.159088f, 4.2f},
            .ocv_chg     = {3.279211f, 3.37767f, 3.430707f, 3.466364f, 3.49217f, 3.511983f, 3.535146f, 3.56365f, 3.58529f, 3.605023f, 3.646288f, 3.695383f, 3.756772f, 3.831339f, 3.920793f, 3.970694f, 4.025329f, 4.087184f, 4.136463f, 4.2f},
            .r0_dis      = {0.03776354f, 0.03619558f, 0.03448082f, 0.03375519f, 0.03324008f, 0.03256867f, 0.03212014f, 0.03140724f, 0.03121838f, 0.0312422f, 0.03128942f, 0.03202252f, 0.03246083f, 0.03282805f, 0.03255127f, 0.03324456f, 0.03290283f, 0.03264852f, 0.03255212f, 0.03255212f},
            .r1_dis      = {0.0182976f, 0.01744962f, 0.01644435f, 0.01535633f, 0.01449358f, 0.01380549f, 0.01324112f, 0.01242799f, 0.01180393f, 0.01143672f, 0.01101172f, 0.0106563f, 0.01042434f, 0.01037886f, 0.01045985f, 0.01052031f, 0.01057055f, 0.01064514f, 0.01069032f, 0.01069032f},
            .tau1_dis    = {11.49047f, 11.46163f, 11.39397f, 11.3139f, 11.28456f, 11.29057f, 11.39013f, 11.66031f, 12.05171f, 12.36498f, 12.78906f, 13.18545f, 13.61243f, 14.14817f, 14.84044f, 15.41821f, 15.9873f, 16.70666f, 17.17537f, 17.17537f},
            .r2_dis      = {0.01761513f, 0.01691411f, 0.01673587f, 0.01732415f, 0.01798292f, 0.01847323f, 0.01850239f, 0.01837469f, 0.01821275f, 0.0181142f, 0.01786627f, 0.0173787f, 0.01658031f, 0.01532035f, 0.01358362f, 0.01211945f, 0.01065628f, 0.008960021f, 0.007960699f, 0.007960699f},
            .tau2_dis    = {109.5856f, 109.6269f, 109.5807f, 109.704f, 110.0737f, 110.5404f, 111.3739f, 113.3306f, 116.1084f, 118.358f, 121.1948f, 123.0387f, 124.0575f, 125.0229f, 126.2128f, 126.9583f, 127.5081f, 128.4634f, 129.2608f, 129.2608f},
            .r0_chg      = {0.04440975f, 0.04193032f, 0.03910537f, 0.03738427f, 0.03592719f, 0.03494427f, 0.03414905f, 0.03278126f, 0.03189444f, 0.0311579f, 0.03162539f, 0.03182115f, 0.03197353f, 0.0319652f, 0.03222276f, 0.03223415f, 0.03227489f, 0.03230411f, 0.03230411f, 0.03230411f},
            .r1_chg      = {0.01926877f, 0.01873491f, 0.01781726f, 0.0167756f, 0.01571346f, 0.0148583f, 0.01397069f, 0.01302472f, 0.01225086f, 0.01167225f, 0.01120641f, 0.01089679f, 0.01071967f, 0.01061478f, 0.01055225f, 0.01052279f, 0.01050324f, 0.01049791f, 0.01049791f, 0.01049791f},
            .tau1_chg    = {11.28256f, 11.30245f, 11.35667f, 11.45921f, 11.57586f, 11.64252f, 11.69427f, 11.83431f, 12.15453f, 12.53189f, 12.8524f, 13.12217f, 13.49953f, 14.01318f, 14.56894f, 14.97967f, 15.31827f, 15.41914f, 15.41914f, 15.41914f},
            .r2_chg      = {0.01776222f, 0.01724553f, 0.01638409f, 0.01559176f, 0.01533733f, 0.01569308f, 0.01645358f, 0.01719387f, 0.01735273f, 0.01734629f, 0.01753555f, 0.01763607f, 0.01740839f, 0.01682812f, 0.01583954f, 0.01484138f, 0.01386843f, 0.01355424f, 0.01355424f, 0.01355424f},
            .tau2_chg    = {108.7041f, 108.9553f, 109.6212f, 110.8393f, 112.3804f, 113.4802f, 114.3365f, 115.2437f, 116.8209f, 118.9235f, 121.1235f, 122.9396f, 124.6882f, 126.7577f, 128.6679f, 129.6003f, 130.0343f, 130.1057f, 130.1057f, 130.1057f},
            .capacity_ah = 1.944092f,
        },
        {   // cell 7
            .ocv_dis     = {3.446456f, 3.476866f, 3.499512f, 3.517215f, 3.531412f, 3.543401f, 3.558863f, 3.580617f, 3.599832f, 3.619108f, 3.661368f, 3.712804f, 3.776934f, 3.854023f, 3.945314f, 3.996711f, 4.052761f, 4.11587f, 4.160181f, 4.2f},
            .ocv_chg     = {3.279985f, 3.377182f, 3.429985f, 3.465675f, 3.491585f, 3.511586f, 3.534704f, 3.562972f, 3.584606f, 3.604274f, 3.645094f, 3.693503f, 3.753713f, 3.82712f, 3.914965f, 3.964025f, 4.018043f, 4.082235f, 4.133742f, 4.2f},
            .r0_dis      = {0.03550548f, 0.03418687f, 0.03360334f, 0.03315775f, 0.03265462f, 0.0322491f, 0.03196563f, 0.03165656f, 0.03171251f, 0.03175406f, 0.0314809f, 0.0314903f, 0.03241092f, 0.03188897f, 0.03215861f, 0.03267619f, 0.03328736f, 0.0324401f, 0.03243751f, 0.03243751f},
            .r1_dis      = {0.01630192f, 0.01569432f, 0.01498354f, 0.01420909f, 0.01356812f, 0.01303899f, 0.01258907f, 0.01195455f, 0.01149553f, 0.01123966f, 0.01095262f, 0.0107268f, 0.01060118f, 0.01057605f, 0.01060848f, 0.01065875f, 0.01071835f, 0.01074223f, 0.01072282f, 0.01072282f},
            .tau1_dis    = {11.63119f, 11.68465f, 11.69196f, 11.63354f, 11.59324f, 11.58492f, 11.69044f, 11.95015f, 12.26041f, 12.48406f, 12.82325f, 13.28172f, 13.86035f, 14.46422f, 15.06655f, 15.55037f, 16.03757f, 16.50362f, 16.71458f, 16.71458f},
            .r2_dis      = {0.0154735f, 0.01500261f, 0.01503998f, 0.01582217f, 0.01665229f, 0.01730038f, 0.01750594f, 0.01766329f, 0.01775806f, 0.01781492f, 0.0177134f, 0.017273f, 0.01645915f, 0.01519867f, 0.01348666f, 0.01202364f, 0.01055717f, 0.008918402f, 0.007999496f, 0.007999496f},
            .tau2_dis    = {109.4452f, 110.4958f, 111.4837f, 112.0289f, 112.1928f, 112.2658f, 112.6812f, 114.1054f, 116.4246f, 118.3178f, 120.896f, 123.462f, 125.9949f, 127.7967f, 128.6954f, 129.2534f, 129.4652f, 128.2324f, 126.7899f, 126.7899f},
            .r0_chg      = {0.04452962f, 0.04137575f, 0.03922087f, 0.03739823f, 0.03563959f, 0.03510115f, 0.03443376f, 0.03253968f, 0.03180807f, 0.03171109f, 0.03156024f, 0.03133011f, 0.03253329f, 0.0317313f, 0.03215662f, 0.03211291f, 0.03286588f, 0.03286588f, 0.03286588f, 0.03286588f},
            .r1_chg      = {0.01933616f, 0.01876398f, 0.01785294f, 0.01683661f, 0.01576663f, 0.01493605f, 0.01407024f, 0.01310404f, 0.01230561f, 0.01172717f, 0.01120795f, 0.01089036f, 0.01074889f, 0.01066565f, 0.01059601f, 0.01055874f, 0.01054341f, 0.01054341f, 0.01054341f, 0.01054341f},
            .tau1_chg    = {11.24057f, 11.30139f, 11.44016f, 11.60619f, 11.73212f, 11.77993f, 11.80151f, 11.83414f, 11.95392f, 12.16298f, 12.53249f, 12.97139f, 13.37652f, 13.78025f, 14.34399f, 14.8866f, 15.2263f, 15.2263f, 15.2263f, 15.2263f},
            .r2_chg      = {0.01786013f, 0.01728146f, 0.01634122f, 0.01552323f, 0.01538233f, 0.01587235f, 0.01666244f, 0.01742424f, 0.01761549f, 0.01763872f, 0.01777314f, 0.01775231f, 0.01741678f, 0.01675475f, 0.01558351f, 0.01437356f, 0.01356339f, 0.01356339f, 0.01356339f, 0.01356339f},
            .tau2_chg    = {109.3892f, 109.4302f, 109.9055f, 110.7223f, 111.5472f, 112.1848f, 112.9455f, 114.1027f, 115.5587f, 117.1622f, 119.523f, 122.1385f, 124.3745f, 126.0049f, 127.3888f, 128.3965f, 128.9437f, 128.9437f, 128.9437f, 128.9437f},
            .capacity_ah = 1.944679f,
        },
    },
    .q_temp_c             = {-20.0f, -10.0f, 0.0f, 25.0f, 45.0f},
    .q_temp_factor        = {0.65f, 0.8f, 0.9f, 1.0f, 1.0f},
    .q_nom_ah             = 2.0169f,
    .coulombic_efficiency = 0.998f,
    .ea_r0                = 25000.0f,
    .ea_r1                = 35000.0f,
    .ea_tau1              = 15000.0f,
    .ea_r2                = 35000.0f,
    .ea_tau2              = 15000.0f,
    .model_temp_dis_c     = 25.0f,
    .model_temp_chg_c     = 25.0f,
    .kf_q_soc             = 2e-07f,
    .kf_q_rc1             = 2e-07f,
    .kf_q_rc2             = 2e-07f,
    .kf_r_v               = 2.5e-05f,
    .cycle_count          = 0,
    .learning_status      = 7,
  };

  /*
  if(!Periph_EepromReadChecked(FUELGAUGE_EEPROM_ADDRESS, (uint8_t *)&data, sizeof(FuelGauge_Data_t)))
  {
    return false;
  }

  if(!FuelGauge_SetData(&data))
  {
    return false;
  }
  */

  return FuelGauge_SetData(&default_model);
}
void Ctrl_SetMode(uint16_t mode)
{
  system_data.main_control = mode;
}
static void Ctrl_FaultSnapshot(void)
{
  // Only the first trip is stored: it stays until the host clears it
  if(system_data.protection_trigger == 0 || stats_data.fault_code != 0)
  {
    return;
  }

  Peripheral_Data_t pd;
  Periph_GetData(&pd);

  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    float mv = pd.cell_voltage[i] + 0.5f;

    if(mv < 0.0f)               mv = 0.0f;
    if(mv > (float)UINT16_MAX)  mv = (float)UINT16_MAX;
    stats_data.fault_snapshot_voltage[i] = (uint16_t)mv;
  }

  float ma = roundf(pd.pack_current);
  if(ma < (float)INT16_MIN) ma = (float)INT16_MIN;
  if(ma > (float)INT16_MAX) ma = (float)INT16_MAX;
  stats_data.fault_snapshot_current = (int16_t)ma;

  float t = roundf(pd.temperature_package);
  if(t < (float)INT8_MIN) t = (float)INT8_MIN;
  if(t > (float)INT8_MAX) t = (float)INT8_MAX;
  stats_data.fault_snapshot_temperature = (int8_t)t;

  stats_data.fault_snapshot_soc = fuelgauge_data.relative_soc;
  stats_data.fault_code         = system_data.protection_trigger;
  stats_data.fault_timestamp    = system_data.session_uptime_ms;
}
void Ctrl_ClearFault(void)
{
  // Called from the main loop (Comm_Run), like Ctrl_FaultSnapshot, so no masking is needed
  memset(stats_data.fault_snapshot_voltage, 0, sizeof(stats_data.fault_snapshot_voltage));
  stats_data.fault_snapshot_current     = 0;
  stats_data.fault_snapshot_temperature = 0;
  stats_data.fault_snapshot_soc         = 0;
  stats_data.fault_code                 = 0;
  stats_data.fault_timestamp            = 0;
}
void Ctrl_ResetProtection(void)
{
  // Called from the main loop (Comm_Run), like Ctrl_ProtectionCheck, so no masking is needed
  system_data.protection_trigger = 0;
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
  system_data.last_communication_timestamp = SYS_GET_TICK();
}
#ifdef OPENBMS_SIMULATION
uint32_t Ctrl_SimGetTick(void)
{
  return sim_tick_ms;
}
void Ctrl_SimSample(const float *cell_mv, float current_ma, float temperature_c, uint16_t dt_ms)
{
  // One host sample is one pass of the control loop, dt_ms later than the previous one
  sim_tick_ms += dt_ms;
  Periph_SimSetData(cell_mv, current_ma, temperature_c);
  Ctrl_Run();
}
#endif
