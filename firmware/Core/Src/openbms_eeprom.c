/**
  ******************************************************************************
  * @file           : openbms_eeprom.c
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

#include "openbms_eeprom.h"
#include "openbms_periph.h"
#include "openbms_fuelgauge.h"
#include "openbms_error.h"
#include <stddef.h>
#include <string.h>
#include <math.h>

#define EEPROM_STREAM_BUSY               (0U)            // more steps, call again
#define EEPROM_STREAM_DONE               (1U)            // data and CRC written
#define EEPROM_STREAM_FAIL               (2U)            // failed (error flag set), nothing more is written

#define EEPROM_STEP_IDLE                 (0U)            // no stream running
#define EEPROM_STEP_PAGE                 (1U)            // start the next page
#define EEPROM_STEP_COMPARE              (2U)            // compare read running
#define EEPROM_STEP_WRITE                (3U)            // page write running
#define EEPROM_STEP_POLL                 (4U)            // EEPROM write cycle
#define EEPROM_STEP_VERIFY               (5U)            // read back running
#define EEPROM_WRITE_CYCLE_TIMEOUT_MS    (20U)           // the write cycle takes up to 5 ms

#define EEPROM_WRITE_NONE                (0U)
#define EEPROM_WRITE_CONFIG              (1U)
#define EEPROM_WRITE_MODEL               (2U)
#define EEPROM_WRITE_STATE               (3U)
#define EEPROM_WRITE_ERASE               (4U)
#define EEPROM_WRITE_HOST_STATE          (5U)            // the fuel gauge state from the host
#define EEPROM_WRITE_STATE_ERASE         (6U)            // only the fuel gauge state set to 0xFF

#define EEPROM_ERASE_VALUE               (0xFFU)         // an erased EEPROM byte (as delivered)

#define EEPROM_STATE_SLOTS               (2U)

#define EEPROM_FINAL_NONE                (0U)            // not asked for
#define EEPROM_FINAL_REQUESTED           (1U)            // waits for the config / model saves, then starts
#define EEPROM_FINAL_STARTED             (2U)            // started (or nothing to save), done once no write runs

_Static_assert(EEPROM_CONFIG_ADDRESS + sizeof(Config_Stored_t) + sizeof(uint32_t) <= EEPROM_FUELGAUGE_MODEL_ADDRESS,
               "the config runs into the fuel gauge model in the EEPROM");
_Static_assert(EEPROM_FUELGAUGE_MODEL_ADDRESS + sizeof(FuelGauge_Data_t) + sizeof(uint32_t) <= EEPROM_FUELGAUGE_STATE_ADDRESS,
               "the fuel gauge model runs into the fuel gauge state in the EEPROM");
_Static_assert(sizeof(FuelGauge_StateStored_t) + sizeof(uint32_t) <= EEPROM_FUELGAUGE_STATE_SLOT_SIZE,
               "the fuel gauge state does not fit its slot");
_Static_assert(EEPROM_FUELGAUGE_STATE_ADDRESS + EEPROM_STATE_SLOTS * EEPROM_FUELGAUGE_STATE_SLOT_SIZE <= EEPROM_SIZE,
               "the fuel gauge state slots do not fit the EEPROM");

_Static_assert((offsetof(Config_Data_t, device_chemistry) - offsetof(Config_Data_t, current_sensor_gain)) % sizeof(float) == 0,
               "the calibration block from current_sensor_gain to device_chemistry must hold only floats");

static Config_Data_t          *eeprom_config          = NULL;   // owned by ctrl
static FuelGauge_Data_t       *eeprom_fuelgauge       = NULL;   // owned by ctrl
static Config_Stored_t         eeprom_config_image    = {0};    // the config as it is read / written
static FuelGauge_StateStored_t eeprom_state_image     = {0};    // the state as it is read / written, the gauge keeps running meanwhile
static FuelGauge_State_t       eeprom_host_state      = {0};    // the state from the host, waits here until no write runs
static uint32_t                eeprom_model_crc       = 0;      // CRC-32 of the model loaded at boot, the saved state must match it
static bool                    eeprom_config_loaded   = false;
static bool                    eeprom_fuelgauge_loaded = false; // the gauge runs and its state is saved
static bool                    eeprom_state_loaded    = false;
static uint8_t                 eeprom_write           = EEPROM_WRITE_NONE;
static uint8_t                 eeprom_save            = EEPROM_SAVE_IDLE;   // the last config / model save
static uint8_t                 eeprom_save_block      = EEPROM_BLOCK_NONE;  // ... and what it writes
static bool                    eeprom_save_state      = false;  // ... only the fuel gauge state (host store or erase)
static uint8_t                 eeprom_final           = EEPROM_FINAL_NONE;
static uint8_t                 eeprom_state_slot      = 0;      // the slot the next state save writes (not the newest one)
static uint32_t                eeprom_state_sequence  = 0;      // sequence of the newest state saved, the next save takes one more

static uint16_t                eeprom_stream_address  = 0;
static const uint8_t          *eeprom_stream_data     = NULL;
static uint16_t                eeprom_stream_length   = 0;      // data bytes, the CRC follows
static uint16_t                eeprom_stream_pos      = 0;      // stream bytes (data + CRC) done
static uint32_t                eeprom_stream_crc      = 0;
static uint16_t                eeprom_stream_crc_size = 0;      // CRC bytes after the data, 0 for an erase
static uint8_t                 eeprom_stream_step     = EEPROM_STEP_IDLE;
static uint32_t                eeprom_stream_tick     = 0;      // HAL_GetTick when the write cycle started
static uint16_t                eeprom_page_length     = 0;
static uint8_t                 eeprom_page_data[EEPROM_PAGE_SIZE];    // the page to write, copied so it stays the same during the write
static uint8_t                 eeprom_page_check[EEPROM_PAGE_SIZE];   // the page as read from the EEPROM

static bool EEPROM_ConfigValid(const Config_Data_t *data);
static bool EEPROM_LoadConfig(void);
static bool EEPROM_LoadFuelGaugeModel(void);
static void EEPROM_LoadFuelGaugeState(void);
static uint16_t EEPROM_StateAddress(uint8_t slot);
static void EEPROM_WriteStart(uint8_t write);
static void EEPROM_WriteStep(void);
static uint32_t EEPROM_Crc32(const uint8_t *data, uint32_t length);
static bool EEPROM_ReadChecked(uint16_t address, uint8_t *data, uint16_t length, bool report_crc);
static bool EEPROM_StreamStart(uint16_t address, const uint8_t *data, uint16_t length);
static uint8_t EEPROM_StreamStep(void);


void EEPROM_Init(Config_Data_t *config, FuelGauge_Data_t *fuelgauge)
{
  eeprom_config    = config;
  eeprom_fuelgauge = fuelgauge;

  eeprom_config_loaded = EEPROM_LoadConfig();

  if(eeprom_config_loaded && EEPROM_LoadFuelGaugeModel() && FuelGauge_IsReady())
  {
    eeprom_fuelgauge_loaded = true;

    EEPROM_LoadFuelGaugeState();
  }
}
void EEPROM_Run(void)
{
  if(eeprom_write != EEPROM_WRITE_NONE)
  {
    EEPROM_WriteStep();
    return;
  }

  if(eeprom_save == EEPROM_SAVE_RUNNING)
  {
    if(eeprom_save_block == EEPROM_BLOCK_CONFIG)
    {
      EEPROM_WriteStart(EEPROM_WRITE_CONFIG);
    }
    else if(eeprom_save_block == EEPROM_BLOCK_ERASE && eeprom_save_state)
    {
      EEPROM_WriteStart(EEPROM_WRITE_STATE_ERASE);
    }
    else if(eeprom_save_block == EEPROM_BLOCK_ERASE)
    {
      EEPROM_WriteStart(EEPROM_WRITE_ERASE);
    }
    else if(eeprom_save_state)
    {
      EEPROM_WriteStart(EEPROM_WRITE_HOST_STATE);
    }
    else
    {
      EEPROM_WriteStart(EEPROM_WRITE_MODEL);
    }
  }
  else if(eeprom_final == EEPROM_FINAL_REQUESTED)
  {
    eeprom_final = EEPROM_FINAL_STARTED;
    EEPROM_WriteStart(EEPROM_WRITE_STATE);
  }
  else if(eeprom_final == EEPROM_FINAL_NONE && eeprom_fuelgauge_loaded && FuelGauge_StateChanged())
  {
    EEPROM_WriteStart(EEPROM_WRITE_STATE);
  }
}
bool EEPROM_SaveConfig(void)
{
  if(EEPROM_SaveRunning())
  {
    return false;
  }
  eeprom_save       = EEPROM_SAVE_RUNNING;
  eeprom_save_block = EEPROM_BLOCK_CONFIG;
  eeprom_save_state = false;
  return true;
}
bool EEPROM_SaveFuelGaugeModel(void)
{
  if(EEPROM_SaveRunning())
  {
    return false;
  }
  eeprom_fuelgauge_loaded = false;
  eeprom_save             = EEPROM_SAVE_RUNNING;
  eeprom_save_block       = EEPROM_BLOCK_MODEL;
  eeprom_save_state       = false;
  return true;
}
bool EEPROM_Erase(void)
{
  if(EEPROM_SaveRunning())
  {
    return false;
  }
  eeprom_fuelgauge_loaded = false;
  eeprom_save             = EEPROM_SAVE_RUNNING;
  eeprom_save_block       = EEPROM_BLOCK_ERASE;
  eeprom_save_state       = false;
  return true;
}
bool EEPROM_SaveFuelGaugeState(const FuelGauge_State_t *state)
{
  if(EEPROM_SaveRunning() || !eeprom_fuelgauge_loaded)
  {
    return false;
  }
  memcpy(&eeprom_host_state, state, sizeof(FuelGauge_State_t));
  eeprom_fuelgauge_loaded = false;
  eeprom_save             = EEPROM_SAVE_RUNNING;
  eeprom_save_block       = EEPROM_BLOCK_MODEL;
  eeprom_save_state       = true;
  return true;
}
bool EEPROM_EraseFuelGaugeState(void)
{
  if(EEPROM_SaveRunning())
  {
    return false;
  }
  eeprom_fuelgauge_loaded = false;
  eeprom_save             = EEPROM_SAVE_RUNNING;
  eeprom_save_block       = EEPROM_BLOCK_ERASE;
  eeprom_save_state       = true;
  return true;
}
void EEPROM_StopFuelGauge(void)
{
  eeprom_fuelgauge_loaded = false;
}
bool EEPROM_FinalSave(void)
{
  if(eeprom_final == EEPROM_FINAL_NONE)
  {
    eeprom_final = EEPROM_FINAL_REQUESTED;
  }
  return (eeprom_final == EEPROM_FINAL_STARTED && eeprom_write == EEPROM_WRITE_NONE);
}
bool EEPROM_ConfigLoaded(void)
{
  return eeprom_config_loaded;
}
bool EEPROM_FuelGaugeLoaded(void)
{
  return eeprom_fuelgauge_loaded;
}
bool EEPROM_SaveRunning(void)
{
  return (eeprom_save == EEPROM_SAVE_RUNNING);
}
uint8_t EEPROM_GetStatus(void)
{
  uint8_t status = 0;

  if(eeprom_config_loaded)
  {
    status |= EEPROM_STATUS_CONFIG_LOADED;
  }
  if(eeprom_fuelgauge_loaded)
  {
    status |= EEPROM_STATUS_MODEL_LOADED;
  }
  if(eeprom_state_loaded)
  {
    status |= EEPROM_STATUS_STATE_LOADED;
  }
  status |= (uint8_t)(eeprom_save << EEPROM_STATUS_SAVE_SHIFT);
  status |= (uint8_t)(eeprom_save_block << EEPROM_STATUS_BLOCK_SHIFT);
  return status;
}
static void EEPROM_WriteStart(uint8_t write)
{
  uint16_t       address;
  const uint8_t *data;
  uint16_t       length;

  if(write == EEPROM_WRITE_CONFIG)
  {
    eeprom_config_image.version = EEPROM_CONFIG_VERSION;
    memcpy(&eeprom_config_image.config, eeprom_config, sizeof(Config_Data_t));
    address = EEPROM_CONFIG_ADDRESS;
    data    = (const uint8_t *)&eeprom_config_image;
    length  = sizeof(Config_Stored_t);
  }
  else if(write == EEPROM_WRITE_MODEL)
  {
    address = EEPROM_FUELGAUGE_MODEL_ADDRESS;
    data    = (const uint8_t *)eeprom_fuelgauge;
    length  = sizeof(FuelGauge_Data_t);
  }
  else if(write == EEPROM_WRITE_ERASE)
  {
    address = 0;
    data    = NULL;
    length  = EEPROM_SIZE;
  }
  else if(write == EEPROM_WRITE_STATE_ERASE)
  {
    address = EEPROM_FUELGAUGE_STATE_ADDRESS;
    data    = NULL;
    length  = EEPROM_STATE_SLOTS * EEPROM_FUELGAUGE_STATE_SLOT_SIZE;
  }
  else if(write == EEPROM_WRITE_HOST_STATE)
  {
    eeprom_state_image.model_crc = eeprom_model_crc;
    eeprom_state_image.sequence  = eeprom_state_sequence + 1U;
    memcpy(&eeprom_state_image.state, &eeprom_host_state, sizeof(FuelGauge_State_t));
    address = EEPROM_StateAddress(eeprom_state_slot);
    data    = (const uint8_t *)&eeprom_state_image;
    length  = sizeof(FuelGauge_StateStored_t);
  }
  else
  {
    if(!eeprom_fuelgauge_loaded || !FuelGauge_IsSeeded())
    {
      return;
    }
    eeprom_state_image.model_crc = eeprom_model_crc;
    eeprom_state_image.sequence  = eeprom_state_sequence + 1U;
    FuelGauge_GetState(&eeprom_state_image.state);
    address = EEPROM_StateAddress(eeprom_state_slot);
    data    = (const uint8_t *)&eeprom_state_image;
    length  = sizeof(FuelGauge_StateStored_t);
  }

  if(EEPROM_StreamStart(address, data, length))
  {
    eeprom_write = write;
  }
  else if(write != EEPROM_WRITE_STATE)
  {
    eeprom_save = EEPROM_SAVE_FAILED;
  }
}
static void EEPROM_WriteStep(void)
{
  uint8_t result = EEPROM_StreamStep();

  if(result == EEPROM_STREAM_BUSY)
  {
    return;
  }

  if((eeprom_write == EEPROM_WRITE_STATE || eeprom_write == EEPROM_WRITE_HOST_STATE) && result == EEPROM_STREAM_DONE)
  {
    eeprom_state_sequence++;
    if(eeprom_state_slot == 0U)
    {
      eeprom_state_slot = 1U;
    }
    else
    {
      eeprom_state_slot = 0U;
    }
  }

  if(eeprom_write != EEPROM_WRITE_STATE)
  {
    if(result == EEPROM_STREAM_DONE)
    {
      eeprom_save = EEPROM_SAVE_DONE;
    }
    else
    {
      eeprom_save = EEPROM_SAVE_FAILED;
    }
  }
  eeprom_write = EEPROM_WRITE_NONE;
}
static bool EEPROM_ConfigValid(const Config_Data_t *data)
{
  const uint8_t *block = (const uint8_t *)data + offsetof(Config_Data_t, current_sensor_gain);
  uint32_t       count = (offsetof(Config_Data_t, device_chemistry) - offsetof(Config_Data_t, current_sensor_gain)) / sizeof(float);
  float          value;

  if(data->cell_count < SYS_CELL_COUNT_MIN || data->cell_count > SYS_CELL_COUNT_MAX)
  {
    return false;
  }

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
static bool EEPROM_LoadConfig(void)
{
  if(!EEPROM_ReadChecked(EEPROM_CONFIG_ADDRESS, (uint8_t *)&eeprom_config_image, sizeof(Config_Stored_t), true))
  {
    return false;
  }
  if(eeprom_config_image.version != EEPROM_CONFIG_VERSION || !EEPROM_ConfigValid(&eeprom_config_image.config))
  {
    return false;
  }

  memcpy(eeprom_config, &eeprom_config_image.config, sizeof(Config_Data_t));
  return true;
}
static bool EEPROM_LoadFuelGaugeModel(void)
{
  if(EEPROM_ReadChecked(EEPROM_FUELGAUGE_MODEL_ADDRESS, (uint8_t *)eeprom_fuelgauge, sizeof(FuelGauge_Data_t), false))
  {
    eeprom_model_crc = EEPROM_Crc32((const uint8_t *)eeprom_fuelgauge, sizeof(FuelGauge_Data_t));
    if(FuelGauge_SetData(eeprom_fuelgauge))
    {
      return true;
    }
  }

  memset(eeprom_fuelgauge, 0, sizeof(FuelGauge_Data_t));

  return false;
}
static void EEPROM_LoadFuelGaugeState(void)
{
  FuelGauge_StateStored_t newest      = {0};
  bool                    found = false;
  uint8_t                 newest_slot = 0;

  for(uint8_t slot = 0; slot < EEPROM_STATE_SLOTS; slot++)
  {
    if(!EEPROM_ReadChecked(EEPROM_StateAddress(slot), (uint8_t *)&eeprom_state_image, sizeof(FuelGauge_StateStored_t), false))
    {
      continue;
    }
    if(eeprom_state_image.model_crc != eeprom_model_crc || !FuelGauge_StateValid(&eeprom_state_image.state))
    {
      continue;
    }
    if(!found || (int32_t)(eeprom_state_image.sequence - newest.sequence) > 0)
    {
      memcpy(&newest, &eeprom_state_image, sizeof(FuelGauge_StateStored_t));
      newest_slot = slot;
      found       = true;
    }
  }

  if(found)
  {
    eeprom_state_loaded   = FuelGauge_SetState(&newest.state);
    eeprom_state_sequence = newest.sequence;
    if(newest_slot == 0U)
    {
      eeprom_state_slot = 1U;
    }
    else
    {
      eeprom_state_slot = 0U;
    }
  }
}
static uint16_t EEPROM_StateAddress(uint8_t slot)
{
  return (uint16_t)(EEPROM_FUELGAUGE_STATE_ADDRESS + (uint32_t)slot * EEPROM_FUELGAUGE_STATE_SLOT_SIZE);
}
static uint32_t EEPROM_Crc32(const uint8_t *data, uint32_t length)
{
  uint32_t crc = 0xFFFFFFFFUL;

  while(length--)
  {
    crc ^= *data++;
    for(uint8_t bit = 0; bit < 8; bit++)
    {
      crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
    }
  }

  return ~crc;
}
// Reads the data and the CRC-32 stored after it (blocking, boot only). False on a read error or a
// CRC mismatch; report_crc sets the CRC error flag on a mismatch.
static bool EEPROM_ReadChecked(uint16_t address, uint8_t *data, uint16_t length, bool report_crc)
{
  uint32_t crc_stored = 0;

  if((uint32_t)address + length + sizeof(crc_stored) > EEPROM_SIZE)
  {
    Error_Set(OPENBMS_EEPROM_READ_FAIL);
    return false;
  }

  if(!Periph_EepromRead(address, data, length) ||
     !Periph_EepromRead(address + length, (uint8_t *)&crc_stored, sizeof(crc_stored)))
  {
    return false;
  }

  if(EEPROM_Crc32(data, length) != crc_stored)
  {
    if(report_crc)
    {
      Error_Set(OPENBMS_EEPROM_CRC_FAIL);
    }
    return false;
  }

  return true;
}
static bool EEPROM_StreamStart(uint16_t address, const uint8_t *data, uint16_t length)
{
  uint16_t crc_size = 0;

  if(data != NULL)
  {
    crc_size = sizeof(eeprom_stream_crc);
  }
  if((uint32_t)address + length + crc_size > EEPROM_SIZE || eeprom_stream_step != EEPROM_STEP_IDLE)
  {
    Error_Set(OPENBMS_EEPROM_WRITE_FAIL);
    return false;
  }

  eeprom_stream_address  = address;
  eeprom_stream_data     = data;
  eeprom_stream_length   = length;
  eeprom_stream_pos      = 0;
  eeprom_stream_crc_size = crc_size;
  eeprom_stream_crc      = 0;
  if(data != NULL)
  {
    eeprom_stream_crc = EEPROM_Crc32(data, length);
  }
  eeprom_stream_step     = EEPROM_STEP_PAGE;
  return true;
}
static uint8_t EEPROM_StreamFail(OpenBMS_Error_t error)
{
  Error_Set(error);
  eeprom_stream_step = EEPROM_STEP_IDLE;
  eeprom_stream_data = NULL;
  return EEPROM_STREAM_FAIL;
}
static uint8_t EEPROM_StreamStep(void)
{
  uint8_t  transfer;
  uint16_t address = eeprom_stream_address + eeprom_stream_pos;
  uint32_t total   = (uint32_t)eeprom_stream_length + eeprom_stream_crc_size;

  if(eeprom_stream_step == EEPROM_STEP_IDLE)
  {
    return EEPROM_STREAM_FAIL;
  }

  if(eeprom_stream_step == EEPROM_STEP_COMPARE || eeprom_stream_step == EEPROM_STEP_WRITE ||
     eeprom_stream_step == EEPROM_STEP_VERIFY)
  {
    transfer = Periph_EepromTransferStatus();
    if(transfer == PER_EEPROM_BUSY)
    {
      return EEPROM_STREAM_BUSY;
    }
    if(transfer == PER_EEPROM_FAIL)
    {
      if(eeprom_stream_step == EEPROM_STEP_WRITE)
      {
        return EEPROM_StreamFail(OPENBMS_EEPROM_WRITE_FAIL);
      }
      return EEPROM_StreamFail(OPENBMS_EEPROM_READ_FAIL);
    }
  }

  switch(eeprom_stream_step)
  {
    case EEPROM_STEP_PAGE:
    {
      if(eeprom_stream_pos >= total)
      {
        eeprom_stream_step = EEPROM_STEP_IDLE;
        eeprom_stream_data = NULL;
        return EEPROM_STREAM_DONE;
      }

      eeprom_page_length = EEPROM_PAGE_SIZE - (address % EEPROM_PAGE_SIZE);
      if(eeprom_page_length > total - eeprom_stream_pos)
      {
        eeprom_page_length = total - eeprom_stream_pos;
      }
      for(uint16_t i = 0; i < eeprom_page_length; i++)
      {
        uint16_t pos = eeprom_stream_pos + i;
        if(eeprom_stream_data == NULL)
        {
          eeprom_page_data[i] = EEPROM_ERASE_VALUE;
        }
        else if(pos < eeprom_stream_length)
        {
          eeprom_page_data[i] = eeprom_stream_data[pos];
        }
        else
        {
          eeprom_page_data[i] = ((const uint8_t *)&eeprom_stream_crc)[pos - eeprom_stream_length];
        }
      }

      if(!Periph_EepromReadStart(address, eeprom_page_check, eeprom_page_length))
      {
        return EEPROM_StreamFail(OPENBMS_EEPROM_READ_FAIL);
      }
      eeprom_stream_step = EEPROM_STEP_COMPARE;
    }
    break;

    case EEPROM_STEP_COMPARE:
    {
      if(memcmp(eeprom_page_check, eeprom_page_data, eeprom_page_length) == 0)
      {
        eeprom_stream_pos  += eeprom_page_length;
        eeprom_stream_step  = EEPROM_STEP_PAGE;
      }
      else if(!Periph_EepromWriteStart(address, eeprom_page_data, eeprom_page_length))
      {
        return EEPROM_StreamFail(OPENBMS_EEPROM_WRITE_FAIL);
      }
      else
      {
        eeprom_stream_step = EEPROM_STEP_WRITE;
      }
    }
    break;

    case EEPROM_STEP_WRITE:
    {
      eeprom_stream_tick = HAL_GetTick();
      eeprom_stream_step = EEPROM_STEP_POLL;
    }
    break;

    case EEPROM_STEP_POLL:
    {
      if(Periph_EepromDeviceReady())
      {
        if(!Periph_EepromReadStart(address, eeprom_page_check, eeprom_page_length))
        {
          return EEPROM_StreamFail(OPENBMS_EEPROM_READ_FAIL);
        }
        eeprom_stream_step = EEPROM_STEP_VERIFY;
      }
      else if((HAL_GetTick() - eeprom_stream_tick) >= EEPROM_WRITE_CYCLE_TIMEOUT_MS)
      {
        return EEPROM_StreamFail(OPENBMS_EEPROM_WRITE_FAIL);
      }
    }
    break;

    case EEPROM_STEP_VERIFY:
    {
      if(memcmp(eeprom_page_check, eeprom_page_data, eeprom_page_length) != 0)
      {
        return EEPROM_StreamFail(OPENBMS_EEPROM_WRITE_FAIL);
      }
      eeprom_stream_pos  += eeprom_page_length;
      eeprom_stream_step  = EEPROM_STEP_PAGE;
    }
    break;

    default:
    {
      return EEPROM_StreamFail(OPENBMS_EEPROM_WRITE_FAIL);
    }
  }

  return EEPROM_STREAM_BUSY;
}
