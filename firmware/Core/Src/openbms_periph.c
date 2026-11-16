/**
  ******************************************************************************
  * @file           : openbms_periph.c
  * @brief          : OpenBMS Peripheral Module
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

#include "openbms_comm.h"
#include "openbms_error.h"
#include "openbms_data.h"
#include "stm32l4xx_hal.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "openbms_periph.h"

#define PER_APP_FLAG_FLASH_PAGE                   19U           // flash page with the app flag the bootloader checks
#define PER_ADS_FULL_SCALE_GAIN_16                150.0f
#define PER_ADS_FULL_SCALE_GAIN_8                 300.0f

#define PER_ADS_READ_FILT_COEF                    0.713f

#define PER_ADS_STATUS_RESET                      (1 << 10)     // STATUS bit 10: the device has reset
#define PER_ADS_STATUS_DRDY_ALL                   (0xFF)        // STATUS bits 7:0: new data on all 8 channels
#define PER_ADS_RESET_TIMEOUT_MS                  (10U)         // DRDY must go high this soon after the SYNC reset, else the ADS init fails
#define PER_ADS_SETTLE_MS                         (10U)         // wait after a change at the ADS inputs, 3-4 samples
#define PER_ADS_OFFSET_MEASURE_MS                 (500U)        // boot offset measurement: samples are averaged this long
#define PER_ADS_SETTLE_SAMPLES                    (4U)          // boot offset measurement: samples dropped first, they may still hold the old input

#define PER_NTC_ADC_MIN_COUNTS                    (20U)
#define PER_NTC_ADC_MAX_COUNTS                    (4075U)

#define PER_STM32_ADC_CHANNELS                    (4U)
#define PER_STM32_ADC_NTC                         (0U)
#define PER_STM32_ADC_VREFINT                     (1U)
#define PER_STM32_ADC_TEMP                        (2U)
#define PER_STM32_ADC_CHG_PRES                    (3U)
#define PER_STM32_ADC_FULL_SCALE                  (4095.0f)

typedef struct
{
    bool    gpio_meas_cell_voltage_enable;
    bool    gpio_meas_pack_voltage_enable;
    bool    gpio_pwr_on;
    bool    gpio_cell_balancer_enable[SYS_CELL_COUNT_MAX];

    bool    gpio_main_drv_enable;
    bool    gpio_main_fet_enable;
    bool    gpio_pre_fet_enable;

    bool    gpio_r_fet_driver_fault;
    bool    gpio_r_wake_up;

 
} Periph_GPIO_t;

typedef struct
{
    float       current_sensor_offset;                  // current sensor offset, measured, in mV across the shunt
    float       current_sensor_gain;                    // current sensor gain calibration
    float       voltage_offset[SYS_CELL_COUNT_MAX];                      // per-cell ADC offset, measured
    float       voltage_gain[SYS_CELL_COUNT_MAX];                        // per-cell ADC gain calibration
    float       ntc_beta;                               // NTC Beta value from datasheet (e.g. 3950.0)
    float       ntc_r_nominal;                          // NTC nominal resistance at 25 degC
    float       ntc_r_fixed;                            // Fixed resistor value in Ohm (e.g. 10000.0)
    float       ntc_t_nominal;                          // NTC nominal temperature in Kelvin (e.g. 298.15 = 25 degC)
    float       temperature_offset;                     // temperature sensor offset in degC
    float       shunt_resistance_mohm;                  // current shunt resistance in mOhm
    float       current_offset_ma;                      // user current offset added to every reading in mA
    float       voltage_offset_mv[SYS_CELL_COUNT_MAX];                   // per-cell user voltage offset added to every reading in mV
    uint8_t     cell_count;                             // cells in series, only these are summed into pack_voltage

} Calibration_Data_t;

ADC_HandleTypeDef           *internal_adc               = &hadc1;
I2C_HandleTypeDef           *eeprom_i2c                 = &hi2c1;
extern IWDG_HandleTypeDef   hiwdg;                                  // main.c
SPI_HandleTypeDef           *adc_spi                    = &hspi1;
TIM_HandleTypeDef           *periodic_timer             = &htim7;

static Periph_GPIO_t        periph_gpio                 = {0};
static Peripheral_Data_t    periph_data                 = {0};
static Calibration_Data_t   calibration_data            = {0};

static uint16_t             stm32_adc_dma_buf[PER_STM32_ADC_CHANNELS] = {0};

static volatile bool        ads_calibration_start       = false;   // shared with the DRDY interrupt
static volatile bool        ads_calibration_finish      = false;
static volatile bool        ads_start_measurements      = false;
static int32_t              ads_cal_val_sum[8]          = {0};
static uint32_t             ads_cal_val_count           = 0;
static volatile uint8_t     ads_cal_skip                = 0;       // samples the DRDY interrupt still drops before the offset sum
static volatile uint32_t    ads_last_good_tick          = 0;     // HAL_GetTick() of the last good sample frame
static volatile bool        per_bal_settling            = false;   // a balancer just turned off, the cell readings are still held
static volatile uint32_t    per_bal_off_tick            = 0;       // HAL_GetTick() when the last balancer turned off
static volatile bool        per_meas_settling           = false;   // MEAS_CELL just went on, the cells are not read yet
static volatile uint32_t    per_meas_on_tick            = 0;       // HAL_GetTick() when MEAS_CELL went on
#ifdef OPENBMS_TEST
static bool                 per_ads_test_standby        = false;   // CC_ADS_STANDBY: the ADS does not sample, no data timeout
static volatile bool        per_meas_test_off           = false;   // CC_MEAS_OFF: MEAS_CELL low, the cells are read anyway
#endif

static volatile bool     per_eeprom_done   = false;     // the running interrupt transfer has finished
static volatile bool     per_eeprom_error  = false;     // the running interrupt transfer failed
static bool              per_eeprom_busy   = false;     // an interrupt transfer was started and not yet reported
static uint32_t          per_eeprom_tick   = 0;         // HAL_GetTick when it started

bool Periph_EepromRead(uint16_t address, uint8_t *data, uint16_t length)
{
  uint16_t chunk;

  while(length > 0)
  {
    chunk = length;
    if(chunk > PER_EEPROM_READ_CHUNK)
    {
      chunk = PER_EEPROM_READ_CHUNK;
    }

    if(HAL_I2C_Mem_Read(eeprom_i2c, PER_EEPROM_I2C_ADDRESS, address, I2C_MEMADD_SIZE_16BIT,
                        data, chunk, PER_EEPROM_READ_CHUNK_TIMEOUT_MS) != HAL_OK)
    {
      Error_Set(OPENBMS_EEPROM_READ_FAIL);
      return false;
    }
    HAL_IWDG_Refresh(&hiwdg);

    address += chunk;
    data    += chunk;
    length  -= chunk;
  }

  return true;
}
static bool Periph_EepromStart(bool write, uint16_t address, uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef status;

  per_eeprom_done  = false;
  per_eeprom_error = false;
  per_eeprom_tick  = HAL_GetTick();
  if(write)
  {
    status = HAL_I2C_Mem_Write_IT(eeprom_i2c, PER_EEPROM_I2C_ADDRESS, address, I2C_MEMADD_SIZE_16BIT, data, length);
  }
  else
  {
    status = HAL_I2C_Mem_Read_IT(eeprom_i2c, PER_EEPROM_I2C_ADDRESS, address, I2C_MEMADD_SIZE_16BIT, data, length);
  }
  per_eeprom_busy = (status == HAL_OK);
  return per_eeprom_busy;
}
bool Periph_EepromReadStart(uint16_t address, uint8_t *data, uint16_t length)
{
  return Periph_EepromStart(false, address, data, length);
}
bool Periph_EepromWriteStart(uint16_t address, uint8_t *data, uint16_t length)
{
  return Periph_EepromStart(true, address, data, length);
}
uint8_t Periph_EepromTransferStatus(void)
{
  if(!per_eeprom_busy)
  {
    return PER_EEPROM_FAIL;
  }
  if(per_eeprom_error)
  {
    per_eeprom_busy = false;
    return PER_EEPROM_FAIL;
  }
  if(per_eeprom_done)
  {
    per_eeprom_busy = false;
    return PER_EEPROM_DONE;
  }
  if((HAL_GetTick() - per_eeprom_tick) >= PER_EEPROM_TRANSFER_TIMEOUT_MS)
  {
    HAL_I2C_DeInit(eeprom_i2c);
    HAL_I2C_Init(eeprom_i2c);
    per_eeprom_busy = false;
    return PER_EEPROM_FAIL;
  }
  return PER_EEPROM_BUSY;
}
bool Periph_EepromDeviceReady(void)
{
  return (HAL_I2C_IsDeviceReady(eeprom_i2c, PER_EEPROM_I2C_ADDRESS, PER_EEPROM_READY_TRIALS,
                                PER_EEPROM_I2C_READ_TIMEOUT) == HAL_OK);
}
// I2C1 (EEPROM) interrupt callbacks: only flags for Periph_EepromTransferStatus
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if(hi2c->Instance == I2C1)
  {
    per_eeprom_done = true;
  }
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if(hi2c->Instance == I2C1)
  {
    per_eeprom_done = true;
  }
}
void Periph_EepromI2cError(void)
{
  per_eeprom_error = true;
}
static bool Periph_AdsStandby(void)
{
  uint8_t tx_buff[30] = {0};
  uint8_t rx_buff[30] = {0};
  bool res = true;

  tx_buff[0] = 0x00;
  tx_buff[1] = 0x22;
  tx_buff[2] = 0x00;

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  if(HAL_SPI_TransmitReceive(adc_spi, tx_buff, rx_buff, 30, PER_ADS_SPI_TIMEOUT) != HAL_OK)
  {
    res = false;
  }

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
  return res;
}
static bool Periph_AdsWakeup(void)
{
  uint8_t tx_buff[30] = {0};
  uint8_t rx_buff[30] = {0};
  bool res = true;

  tx_buff[0] = 0x00;
  tx_buff[1] = 0x33;
  tx_buff[2] = 0x00;

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  if(HAL_SPI_TransmitReceive(adc_spi, tx_buff, rx_buff, 30, PER_ADS_SPI_TIMEOUT) != HAL_OK)
  {
    res = false;
  }

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

  HAL_Delay(1);

  return res;
}
static bool Periph_AdsReadReg(uint8_t reg, uint8_t* data)
{
  uint8_t tx_buff[30] = {0};
  uint16_t cmd;
  bool res = true;

  cmd = 0xA000 | ((uint16_t)reg << 7);
  tx_buff[0] = (cmd >> 8) & 0xFF;
  tx_buff[1] =  cmd       & 0xFF;

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  if(HAL_SPI_TransmitReceive(adc_spi, tx_buff, data, 30, PER_ADS_SPI_TIMEOUT) != HAL_OK)
  {
    res = false;
  }

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

  return res;
}
static bool Periph_AdsReadAdcValues(uint8_t* data)
{
  uint8_t tx_buff[30] = {0};
  bool res = true;

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  if(HAL_SPI_TransmitReceive(adc_spi, tx_buff, data, 30, PER_ADS_SPI_TIMEOUT) != HAL_OK)
  {
    res = false;
  }

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

  return res;
}
static bool Periph_AdsWriteReg(uint8_t reg, uint16_t value)
{
  uint8_t tx_buff[30] = {0};
  uint8_t rx_buff[30] = {0};
  uint16_t cmd;
  bool res = true;

  cmd = 0x6000 | ((uint16_t)reg << 7);
  tx_buff[0] = (cmd >> 8) & 0xFF;
  tx_buff[1] =  cmd       & 0xFF;
  tx_buff[2] = 0x00;

  tx_buff[3] = (value >> 8) & 0xFF;
  tx_buff[4] =  value       & 0xFF;
  tx_buff[5] = 0x00;

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  if(HAL_SPI_TransmitReceive(adc_spi, tx_buff, rx_buff, 30, PER_ADS_SPI_TIMEOUT) != HAL_OK)
  {
    res = false;
  }

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
  return res;
}
static void Periph_AdsInit(void)
{
  uint8_t  rx_data[30] = {0};
  uint32_t start_tick;

  HAL_GPIO_WritePin(ADC_SYNC_GPIO_Port, ADC_SYNC_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(ADC_SYNC_GPIO_Port, ADC_SYNC_Pin, GPIO_PIN_SET);

  start_tick = HAL_GetTick();
  while(HAL_GPIO_ReadPin(ADC_DRDY_GPIO_Port, ADC_DRDY_Pin) == GPIO_PIN_RESET)
  {
    if((HAL_GetTick() - start_tick) >= PER_ADS_RESET_TIMEOUT_MS)
    {
      Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
      return;
    }
  }

  if(!Periph_AdsStandby())
  {
    Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  if(!Periph_AdsReadReg(0x00, rx_data))
  {
    Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  if(!Periph_AdsReadReg(0x00, rx_data))
  {
    Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  if(rx_data[0] != 0x28)
  {
    Error_Set(OPENBMS_ADS131M08_ID_FAIL);
    return;
  }

  if(!Periph_AdsWriteReg(0x02, 0x0110))
  {
      Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
      return;
  }

  if(!Periph_AdsWriteReg(0x03, 0xFF1F))
  {
    Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  if(!Periph_AdsWriteReg(0x04, 0x3334))
  {
    Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  if(!Periph_AdsWriteReg(0x05, 0x3333))
  {
    Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  Periph_AdsWriteReg(0x09, 0x0001);
  HAL_GPIO_WritePin(MEAS_CELL_GPIO_Port, MEAS_CELL_Pin, GPIO_PIN_RESET);

  if(!Periph_AdsWakeup())
  {
    Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  __HAL_GPIO_EXTI_CLEAR_IT(ADC_DRDY_Pin);
  HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  ads_cal_skip          = PER_ADS_SETTLE_SAMPLES;
  ads_calibration_start = true;

  HAL_Delay(PER_ADS_OFFSET_MEASURE_MS);
  ads_calibration_finish = true;

  start_tick = HAL_GetTick();
  while(ads_calibration_start)
  {
    if((HAL_GetTick() - start_tick) >= PER_ADS_SETTLE_MS)
    {
      break;
    }
  }

  if(ads_calibration_start)
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ads_calibration_start  = false;
    ads_calibration_finish = false;
    __set_PRIMASK(primask);
    Error_Set(OPENBMS_ADS131M08_DATA_LOST);
  }

  HAL_GPIO_WritePin(CURR_CAL_GPIO_Port, CURR_CAL_Pin, GPIO_PIN_SET);

  Periph_AdsWriteReg(0x09, 0x0000);

  HAL_Delay(PER_ADS_SETTLE_MS);
  ads_last_good_tick     = HAL_GetTick();
  ads_start_measurements = true;
  
}
// ADS131M08 sample: 24-bit two's complement, most significant byte first, to int32.
// Built unsigned and then sign-extended, so no signed value is shifted.
static int32_t Periph_AdsToInt24(const uint8_t *p)
{
  int32_t value = (int32_t)(((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)p[2]);

  if(value & 0x800000)
  {
    value -= 0x1000000;
  }
  return value;
}
// CRC-16-CCITT (polynomial 0x1021, start value 0xFFFF), as the ADS131M08 sends it in MODE.CRC_TYPE = 0
static uint16_t Periph_AdsCrc16(const uint8_t *data, uint8_t length)
{
  uint16_t crc = 0xFFFF;

  for(uint8_t i = 0; i < length; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for(uint8_t bit = 0; bit < 8; bit++)
    {
      if(crc & 0x8000)
      {
        crc = (crc << 1) ^ 0x1021;
      }
      else
      {
        crc = crc << 1;
      }
    }
  }

  return crc;
}
// Checks one sample frame: SPI transfer, output CRC and STATUS. A bad frame is reported and
// dropped, a good one refreshes the data timeout.
static bool Periph_AdsFrameOk(bool spi_ok, const uint8_t *rx_data)
{
  uint16_t status;
  uint16_t crc_rx;
  bool     ok = true;

  if(!spi_ok)
  {
    Error_Set(OPENBMS_ADS131M08_READ_FAIL);
    ok = false;
  }
  else
  {
    crc_rx = ((uint16_t)rx_data[27] << 8) | rx_data[28];
    status = ((uint16_t)rx_data[0] << 8) | rx_data[1];

    if(Periph_AdsCrc16(rx_data, 27) != crc_rx)
    {
      Error_Set(OPENBMS_ADS131M08_CRC_FAIL);
      ok = false;
    }
    else if(status & PER_ADS_STATUS_RESET)
    {
      Error_Set(OPENBMS_ADS131M08_RESET);
      ok = false;
    }
    else if((status & PER_ADS_STATUS_DRDY_ALL) != PER_ADS_STATUS_DRDY_ALL)
    {
      ok = false;
    }
  }

  if(ok)
  {
    ads_last_good_tick = HAL_GetTick();
  }

  return ok;
}
static void Periph_AdsRead(void)
{
  uint8_t rx_data[30] = {0};
  int32_t raw_value;
  float voltage_mv;
  float current;
  bool    spi_ok;

  spi_ok = Periph_AdsReadAdcValues(rx_data);

  if(!ads_start_measurements && !ads_calibration_start)
  {
    return;
  }

  if(!Periph_AdsFrameOk(spi_ok, rx_data))
  {
    return;
  }


  if(ads_start_measurements)
  {
    raw_value = Periph_AdsToInt24(&rx_data[3]);
    voltage_mv = (float)raw_value * (PER_ADS_FULL_SCALE_GAIN_16 / 16777216.0f);
    if(calibration_data.shunt_resistance_mohm > 0.0f)
    {
      current = (voltage_mv - calibration_data.current_sensor_offset) / calibration_data.shunt_resistance_mohm;
      current *= calibration_data.current_sensor_gain;
      current *= 1000.0f;
      current *= -1;

      current += calibration_data.current_offset_ma;

      periph_data.pack_current += PER_ADS_READ_FILT_COEF * (current - periph_data.pack_current);
    }
  }

  if(periph_gpio.gpio_meas_cell_voltage_enable & ads_start_measurements)
  {
    bool hold = false;
    bool seed = false;
    for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
    {
      if(periph_gpio.gpio_cell_balancer_enable[i])
      {
        hold = true;
      }
    }
    if(!hold && per_bal_settling)
    {
      if((HAL_GetTick() - per_bal_off_tick) < PER_BAL_SETTLE_MS)
      {
        hold = true;
      }
      else
      {
        per_bal_settling = false;
      }
    }

    if(per_meas_settling)
    {
      if((HAL_GetTick() - per_meas_on_tick) < PER_MEAS_SETTLE_MS)
      {
        return;
      }
      seed              = true;
      per_meas_settling = false;
    }

    periph_data.pack_voltage = 0;
    for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
    {
      raw_value = Periph_AdsToInt24(&rx_data[i*3+6]);
      voltage_mv = (float)raw_value * (PER_ADS_FULL_SCALE_GAIN_8 / 16777216.0f);
      voltage_mv *= PER_CELL_VOLTAGE_RESISTANCE_FACTOR;
      voltage_mv -= calibration_data.voltage_offset[i];
      voltage_mv *= calibration_data.voltage_gain[i];
      voltage_mv += calibration_data.voltage_offset_mv[i];

      if(seed)
      {
        periph_data.cell_voltage_live[i] = voltage_mv;
        periph_data.cell_voltage[i]      = voltage_mv;
      }
      else
      {
        periph_data.cell_voltage_live[i] += PER_ADS_READ_FILT_COEF * (voltage_mv - periph_data.cell_voltage_live[i]);
      }

      if(!hold && !seed)
      {
        periph_data.cell_voltage[i] += PER_ADS_READ_FILT_COEF * (voltage_mv - periph_data.cell_voltage[i]);
      }

      if(i < calibration_data.cell_count)
      {
        periph_data.pack_voltage += periph_data.cell_voltage[i];
      }
    }
  }
  else if(ads_calibration_start)
  {
    if(ads_cal_skip > 0U)
    {
      ads_cal_skip--;
      return;
    }

    for(uint8_t i = 0; i <= 7; i++)
    {
      raw_value = Periph_AdsToInt24(&rx_data[i*3+3]);
      
      ads_cal_val_sum[i] += raw_value;
    }

    ads_cal_val_count++;

    if(ads_calibration_finish)
    {
      for(uint8_t i = 0; i <= 7; i++) 
      {
        ads_cal_val_sum[i] /= (int32_t) ads_cal_val_count;

        if(i == 0) 
        {
          calibration_data.current_sensor_offset = (float) ads_cal_val_sum[i] * (PER_ADS_FULL_SCALE_GAIN_16 / 16777216.0f);
        }
        else 
        {
          calibration_data.voltage_offset[i-1] = (float) ads_cal_val_sum[i] * (PER_ADS_FULL_SCALE_GAIN_8 / 16777216.0f);
          calibration_data.voltage_offset[i-1] *= PER_CELL_VOLTAGE_RESISTANCE_FACTOR;
        }
      }

      ads_calibration_start   = false;
      ads_calibration_finish  = false;
    }
  }
}
static float Periph_NtcGetTemperature(float r_ntc)
{
    if(r_ntc <= 0.0f)
    {
        return -273.15f;
    }

    float temp_k = 1.0f / ((1.0f / calibration_data.ntc_t_nominal) + (1.0f / calibration_data.ntc_beta) * logf(r_ntc / calibration_data.ntc_r_nominal));

    return (temp_k - 273.15f - calibration_data.temperature_offset);
}
static void Periph_Stm32AdcInit(void)
{
  HAL_ADCEx_Calibration_Start(internal_adc, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(internal_adc, (uint32_t*)stm32_adc_dma_buf, PER_STM32_ADC_CHANNELS);
}
static void Periph_Stm32AdcRead(void)
{
  if(!Error_IsSet(OPENBMS_NTC_FAULT))
  {
    if(stm32_adc_dma_buf[PER_STM32_ADC_NTC] < PER_NTC_ADC_MIN_COUNTS || stm32_adc_dma_buf[PER_STM32_ADC_NTC] > PER_NTC_ADC_MAX_COUNTS)
    {
      Error_Set(OPENBMS_NTC_FAULT);
    }
    else
    {
      float r_ntc  = calibration_data.ntc_r_fixed * ((PER_STM32_ADC_FULL_SCALE / (float)stm32_adc_dma_buf[PER_STM32_ADC_NTC]) - 1.0f);
      float temp_c = Periph_NtcGetTemperature(r_ntc);

      if(isfinite(temp_c))
      {
        periph_data.temperature_package = temp_c;
      }
      else
      {
        Error_Set(OPENBMS_NTC_FAULT);
      }
    }
  }

  uint16_t vrefint_cal = *((uint16_t*)0x1FFF75AA);

  periph_data.main_vdd_voltage_mv = (3000.0f * (float)vrefint_cal) / (float)stm32_adc_dma_buf[PER_STM32_ADC_VREFINT];

  uint16_t ts_cal1 = *((uint16_t*)0x1FFF75A8);
  uint16_t ts_cal2 = *((uint16_t*)0x1FFF75CA);

  float adc_corrected = (float)stm32_adc_dma_buf[PER_STM32_ADC_TEMP] * (periph_data.main_vdd_voltage_mv / 3000.0f);

  periph_data.temperature_stm32 = (130.0f - 30.0f) / ((float)ts_cal2 - (float)ts_cal1)
                          * (adc_corrected - (float)ts_cal1) + 30.0f;

  periph_data.sys_vcc_mv = (float)stm32_adc_dma_buf[PER_STM32_ADC_CHG_PRES] * (periph_data.main_vdd_voltage_mv / PER_STM32_ADC_FULL_SCALE)
                           * PER_CHG_PRES_DIVIDER_FACTOR;
  
}
static void Periph_TimerInit(void)
{
  HAL_TIM_Base_Start_IT(periodic_timer);
}
static void Periph_GpioCtrl(void)
{
  HAL_GPIO_WritePin(DRV_MAIN_EN_GPIO_Port, DRV_MAIN_EN_Pin, (GPIO_PinState) periph_gpio.gpio_main_drv_enable);
  HAL_GPIO_WritePin(DRV_FET_EN_GPIO_Port, DRV_FET_EN_Pin, (GPIO_PinState) periph_gpio.gpio_main_fet_enable);

  HAL_GPIO_WritePin(DRV_PRE_FET_EN_GPIO_Port, DRV_PRE_FET_EN_Pin, (GPIO_PinState) periph_gpio.gpio_pre_fet_enable);

#ifdef OPENBMS_TEST
  if(per_meas_test_off)
  {
    HAL_GPIO_WritePin(MEAS_CELL_GPIO_Port, MEAS_CELL_Pin, GPIO_PIN_RESET);
  }
  else
#endif
  HAL_GPIO_WritePin(MEAS_CELL_GPIO_Port, MEAS_CELL_Pin, (GPIO_PinState) (periph_gpio.gpio_meas_cell_voltage_enable | periph_gpio.gpio_meas_pack_voltage_enable));

  HAL_GPIO_WritePin(MEAS_BATT_GPIO_Port, MEAS_BATT_Pin, (GPIO_PinState) periph_gpio.gpio_meas_pack_voltage_enable);

  HAL_GPIO_WritePin(CELL_1_BAL_GPIO_Port, CELL_1_BAL_Pin, (GPIO_PinState) periph_gpio.gpio_cell_balancer_enable[0]);
  HAL_GPIO_WritePin(CELL_2_BAL_GPIO_Port, CELL_2_BAL_Pin, (GPIO_PinState) periph_gpio.gpio_cell_balancer_enable[1]);
  HAL_GPIO_WritePin(CELL_3_BAL_GPIO_Port, CELL_3_BAL_Pin, (GPIO_PinState) periph_gpio.gpio_cell_balancer_enable[2]);
  HAL_GPIO_WritePin(CELL_4_BAL_GPIO_Port, CELL_4_BAL_Pin, (GPIO_PinState) periph_gpio.gpio_cell_balancer_enable[3]);
  HAL_GPIO_WritePin(CELL_5_BAL_GPIO_Port, CELL_5_BAL_Pin, (GPIO_PinState) periph_gpio.gpio_cell_balancer_enable[4]);
  HAL_GPIO_WritePin(CELL_6_BAL_GPIO_Port, CELL_6_BAL_Pin, (GPIO_PinState) periph_gpio.gpio_cell_balancer_enable[5]);
  HAL_GPIO_WritePin(CELL_7_BAL_GPIO_Port, CELL_7_BAL_Pin, (GPIO_PinState) periph_gpio.gpio_cell_balancer_enable[6]);

  HAL_GPIO_WritePin(PWR_ON_GPIO_Port, PWR_ON_Pin, (GPIO_PinState) periph_gpio.gpio_pwr_on);

  periph_gpio.gpio_r_fet_driver_fault = (bool)(1 - (uint8_t)HAL_GPIO_ReadPin(DRV_FLT_GPIO_Port, DRV_FLT_Pin));

  periph_gpio.gpio_r_wake_up = (bool)HAL_GPIO_ReadPin(WAKE_UP_GPIO_Port, WAKE_UP_Pin);

  if(periph_gpio.gpio_r_fet_driver_fault)       Error_Set(OPENBMS_HW_DRV_FAULT);
}
static void Periph_SetDefaultData(void)
{
  memset(&periph_data, 0, sizeof(Peripheral_Data_t));
  memset(&calibration_data, 0, sizeof(Calibration_Data_t));
}
void Periph_Init(void)
{
  Periph_SetDefaultData();

  periph_gpio.gpio_pwr_on = true;

  Periph_Stm32AdcInit();
  Periph_AdsInit();
  Periph_TimerInit();
}
void Periph_Run(void)
{
    Periph_GpioCtrl();

#ifdef OPENBMS_TEST
    if(per_ads_test_standby)
    {
    }
    else
#endif
    {
        uint32_t last_good_tick = ads_last_good_tick;

        if((HAL_GetTick() - last_good_tick) >= PER_ADS_DATA_TIMEOUT_MS)
        {
            Error_Set(OPENBMS_ADS131M08_DATA_LOST);
        }
    }
}
void Periph_SetFET(bool state)
{  
    periph_gpio.gpio_main_fet_enable = state;

    if(state) periph_data.fet_status |= PER_FET_MAIN;
    else      periph_data.fet_status &= ~PER_FET_MAIN;
    
}
void Periph_SetPreFET(bool state)
{
    periph_gpio.gpio_pre_fet_enable = state;

    if(state) periph_data.fet_status |= PER_FET_PRE;
    else      periph_data.fet_status &= ~PER_FET_PRE;
}
void Periph_SetBalance(uint8_t mask)
{
  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    bool on = ((mask & (1U << i)) != 0);

    if(periph_gpio.gpio_cell_balancer_enable[i] && !on)
    {
      per_bal_off_tick = HAL_GetTick();
      per_bal_settling = true;
    }
    periph_gpio.gpio_cell_balancer_enable[i] = on;
  }

  periph_data.fet_status &= (uint16_t)~(0x7FU << PER_FET_BAL_SHIFT);
  periph_data.fet_status |= (uint16_t)((mask & 0x7FU) << PER_FET_BAL_SHIFT);
}
void Periph_Off(void)
{
  periph_gpio.gpio_main_fet_enable          = false;
  periph_gpio.gpio_pre_fet_enable           = false;
  periph_gpio.gpio_main_drv_enable          = false;
  periph_gpio.gpio_meas_cell_voltage_enable = false;
  periph_gpio.gpio_meas_pack_voltage_enable = false;
  for(uint8_t i = 0; i < SYS_CELL_COUNT_MAX; i++)
  {
    periph_gpio.gpio_cell_balancer_enable[i] = false;
  }
  periph_data.fet_status = 0;
}
void Periph_DriverReset(void)
{
  HAL_GPIO_WritePin(DRV_MAIN_EN_GPIO_Port, DRV_MAIN_EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(PER_DRV_RESET_MS);
  HAL_GPIO_WritePin(DRV_MAIN_EN_GPIO_Port, DRV_MAIN_EN_Pin, (GPIO_PinState) periph_gpio.gpio_main_drv_enable);
}
void Periph_Wake(void)
{
  uint32_t primask = __get_PRIMASK();

  periph_gpio.gpio_main_drv_enable = true;
  if(!periph_gpio.gpio_meas_cell_voltage_enable)
  {
    __disable_irq();
    per_meas_on_tick                          = HAL_GetTick();
    per_meas_settling                         = true;
    periph_gpio.gpio_meas_cell_voltage_enable = true;
    __set_PRIMASK(primask);
  }
}
void Periph_PowerOff(void)
{
  periph_gpio.gpio_pwr_on = false;
}
bool Periph_CellMeasureReady(void)
{
#ifdef OPENBMS_TEST
  if(per_meas_test_off)
  {
    return false;
  }
#endif
  if(periph_gpio.gpio_meas_cell_voltage_enable && !per_meas_settling)
  {
    return true;
  }
  return false;
}
void Periph_RemoveAppFlag(void)
{
  HAL_StatusTypeDef      status;
  FLASH_EraseInitTypeDef erase_init;
  uint32_t               page_error = 0;

  status = HAL_FLASH_Unlock();
  if(status != HAL_OK)
  {
  }

  erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
  erase_init.Banks       = FLASH_BANK_1;
  erase_init.Page        = PER_APP_FLAG_FLASH_PAGE;
  erase_init.NbPages     = 1;

  status = HAL_FLASHEx_Erase(&erase_init, &page_error);

  HAL_FLASH_Lock();

  if(status != HAL_OK)
  {
  }
}
bool Periph_CellMeasureOn(void)
{
#ifdef OPENBMS_TEST
  if(per_meas_test_off)
  {
    return false;
  }
#endif
  return periph_gpio.gpio_meas_cell_voltage_enable;
}
void Periph_GetData(Peripheral_Data_t *pd)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  memcpy(pd, &periph_data, sizeof(Peripheral_Data_t));
  __set_PRIMASK(primask);
}
void Periph_SetCalibration(const Config_Data_t *cfg)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  calibration_data.current_sensor_gain    = cfg->current_sensor_gain;
  memcpy(calibration_data.voltage_gain, cfg->voltage_gain, sizeof(calibration_data.voltage_gain));
  calibration_data.ntc_beta               = cfg->ntc_beta;
  calibration_data.ntc_r_nominal          = cfg->ntc_r_nominal;
  calibration_data.ntc_r_fixed            = cfg->ntc_r_fixed;
  calibration_data.ntc_t_nominal          = cfg->ntc_t_nominal;
  calibration_data.temperature_offset     = cfg->temperature_offset;
  calibration_data.shunt_resistance_mohm  = cfg->shunt_resistance_mohm;
  calibration_data.current_offset_ma      = cfg->current_offset_ma;
  memcpy(calibration_data.voltage_offset_mv, cfg->voltage_offset_mv, sizeof(calibration_data.voltage_offset_mv));
  if(cfg->cell_count > SYS_CELL_COUNT_MAX)
  {
    calibration_data.cell_count = SYS_CELL_COUNT_MAX;
  }
  else
  {
    calibration_data.cell_count = cfg->cell_count;
  }
  __set_PRIMASK(primask);
}
#ifdef OPENBMS_TEST
bool Periph_AdsTestStandby(bool standby)
{
  bool res = true;

  if(standby)
  {
    if(!per_ads_test_standby)
    {
      HAL_NVIC_DisableIRQ(ADC_DRDY_EXTI_IRQn);
      per_ads_test_standby = true;
      res = Periph_AdsStandby();
    }
  }
  else
  {
    if(per_ads_test_standby)
    {
      res = Periph_AdsWakeup();
      ads_last_good_tick   = HAL_GetTick();
      per_ads_test_standby = false;
      __HAL_GPIO_EXTI_CLEAR_IT(ADC_DRDY_Pin);
      HAL_NVIC_ClearPendingIRQ(ADC_DRDY_EXTI_IRQn);
      HAL_NVIC_EnableIRQ(ADC_DRDY_EXTI_IRQn);
    }
  }

  return res;
}
bool Periph_AdsTestStandbyOn(void)
{
  return per_ads_test_standby;
}
void Periph_MeasTestOff(bool off)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if(per_meas_test_off && !off)
  {
    per_meas_on_tick  = HAL_GetTick();
    per_meas_settling = true;
  }
  else if(!per_meas_test_off && off)
  {
    per_meas_settling = false;
  }
  per_meas_test_off = off;
  __set_PRIMASK(primask);
}
bool Periph_MeasTestOffOn(void)
{
  return per_meas_test_off;
}
const float *Periph_TestBootVoltageOffset(void)
{
  return calibration_data.voltage_offset;
}
#endif
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ADC_DRDY_Pin)
  {
    Periph_AdsRead();
  }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if(hadc->Instance == ADC1)
  {
    Periph_Stm32AdcRead();
  }
}