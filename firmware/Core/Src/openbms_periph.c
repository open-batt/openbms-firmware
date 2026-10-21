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

#define ADS_FULL_SCALE_GAIN_16                150.0f
#define ADS_FULL_SCALE_GAIN_4                 600.0f

// The ADS131M08 delivers a sample every ~4 ms (252 Hz). Current and cell voltages
// are smoothed here with a single-pole filter to a 50 Hz corner:
//   coef = 1 - exp(-2 * pi * 50 / 252) = 0.713
#define ADS_READ_FILT_COEF                    0.713f

// Sample frame checks: STATUS word in bytes 0-1 (byte 2 is padding), output CRC in bytes 27-28
#define ADS_STATUS_RESET                      (1 << 10)     // STATUS bit 10: the device has reset
#define ADS_STATUS_DRDY_ALL                   (0xFF)        // STATUS bits 7:0: new data on all 8 channels
#define ADS_DATA_TIMEOUT_MS                   (100U)        // no good sample for this long: measurement lost (~25 samples)
#define ADS_RESET_TIMEOUT_MS                  (10U)         // DRDY must go high this soon after the SYNC reset, else the ADS init fails

// NTC divider on the internal ADC (12 bit): a reading this close to either end means the NTC is open or shorted
#define NTC_ADC_MIN_COUNTS                    (20U)
#define NTC_ADC_MAX_COUNTS                    (4075U)

#define HW_CELL_VOLTAGE_RESISTANCE_FACTOR     29.3333333f

typedef struct
{
    bool    gpio_meas_cell_voltage_enable;
    bool    gpio_meas_pack_voltage_enable;
    bool    gpio_pwr_on;
    bool    gpio_cell_balancer_enable[7];

    bool    gpio_main_drv_enable;
    bool    gpio_main_fet_enable;
    bool    gpio_pre_fet_enable;

    bool    gpio_r_fet_driver_fault;
    bool    gpio_r_fet_driver_gate_fault;
    bool    gpio_r_wake_up;
    bool    gpio_r_vcc_power_good;
 
} Periph_GPIO_t;

ADC_HandleTypeDef           *internal_adc               = &hadc1;
I2C_HandleTypeDef           *eeprom_i2c                 = &hi2c1;
SPI_HandleTypeDef           *adc_spi                    = &hspi1;
TIM_HandleTypeDef           *periodic_timer             = &htim7;

static Periph_GPIO_t        periph_gpio                 = {0};
static Peripheral_Data_t    periph_data                 = {0};
static Calibration_Data_t   calibration_data            = {0};

static uint16_t             stm32_adc_dma_buf[3]        = {0};

static volatile bool        ads_calibration_start       = false;   // shared with the DRDY interrupt
static volatile bool        ads_calibration_finish      = false;
static volatile bool        ads_start_measurements      = false;
static int32_t              ads_cal_val_sum[8]          = {0};
static uint32_t             ads_cal_val_count           = 0;
static volatile uint32_t    ads_last_good_tick          = 0;     // HAL_GetTick() of the last good sample frame

bool Periph_EepromRead(uint16_t address, uint8_t *data, uint16_t length)
{
  if((uint32_t)address + length > EEPROM_SIZE)
  {
    Error_Set(OPENBMS_EEPROM_READ_FAIL);
    return false;
  }

  if(HAL_I2C_Mem_Read(eeprom_i2c, EEPROM_I2C_ADDRESS, address, I2C_MEMADD_SIZE_16BIT,
                      data, length, EEPROM_I2C_READ_TIMEOUT * (1 + length / EEPROM_PAGE_SIZE)) != HAL_OK)
  {
    Error_Set(OPENBMS_EEPROM_READ_FAIL);
    return false;
  }

  return true;
}
bool Periph_EepromWrite(uint16_t address, const uint8_t *data, uint16_t length)
{
  uint8_t  check[EEPROM_PAGE_SIZE];

  if((uint32_t)address + length > EEPROM_SIZE)
  {
    Error_Set(OPENBMS_EEPROM_WRITE_FAIL);
    return false;
  }

  // A page write must not cross a page boundary, so go page by page.
  // Pages that already hold the right data are skipped to save write cycles.
  while(length > 0)
  {
    uint16_t chunk = EEPROM_PAGE_SIZE - (address % EEPROM_PAGE_SIZE);
    if(chunk > length)
    {
      chunk = length;
    }

    if(HAL_I2C_Mem_Read(eeprom_i2c, EEPROM_I2C_ADDRESS, address, I2C_MEMADD_SIZE_16BIT,
                        check, chunk, EEPROM_I2C_READ_TIMEOUT) != HAL_OK)
    {
      Error_Set(OPENBMS_EEPROM_READ_FAIL);
      return false;
    }

    if(memcmp(check, data, chunk) != 0)
    {
      if(HAL_I2C_Mem_Write(eeprom_i2c, EEPROM_I2C_ADDRESS, address, I2C_MEMADD_SIZE_16BIT,
                           (uint8_t *)data, chunk, EEPROM_I2C_WRITE_TIMEOUT) != HAL_OK)
      {
        Error_Set(OPENBMS_EEPROM_WRITE_FAIL);
        return false;
      }

      // The EEPROM does not answer until its internal write cycle is done
      if(HAL_I2C_IsDeviceReady(eeprom_i2c, EEPROM_I2C_ADDRESS,
                               EEPROM_I2C_READY_TRIALS, EEPROM_I2C_WRITE_TIMEOUT) != HAL_OK)
      {
        Error_Set(OPENBMS_EEPROM_WRITE_FAIL);
        return false;
      }

      // Read back and verify
      if(HAL_I2C_Mem_Read(eeprom_i2c, EEPROM_I2C_ADDRESS, address, I2C_MEMADD_SIZE_16BIT,
                          check, chunk, EEPROM_I2C_READ_TIMEOUT) != HAL_OK)
      {
        Error_Set(OPENBMS_EEPROM_READ_FAIL);
        return false;
      }

      if(memcmp(check, data, chunk) != 0)
      {
        Error_Set(OPENBMS_EEPROM_WRITE_FAIL);
        return false;
      }
    }

    address += chunk;
    data    += chunk;
    length  -= chunk;
  }

  return true;
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
// Writes the data, then a CRC-32 of it in the 4 bytes right after (length + 4 bytes used).
// The CRC goes last, so a write that is cut short leaves a CRC that does not match.
bool Periph_EepromWriteChecked(uint16_t address, const uint8_t *data, uint16_t length)
{
  uint32_t crc = EEPROM_Crc32(data, length);

  if(!Periph_EepromWrite(address, data, length))
  {
    return false;
  }

  return Periph_EepromWrite(address + length, (const uint8_t *)&crc, sizeof(crc));
}
// Reads the data and its CRC and compares them. Returns false on a CRC mismatch,
// and data must not be used then (it is overwritten even when the check fails).
bool Periph_EepromReadChecked(uint16_t address, uint8_t *data, uint16_t length)
{
  uint32_t crc_stored = 0;

  if(!Periph_EepromRead(address, data, length) ||
     !Periph_EepromRead(address + length, (uint8_t *)&crc_stored, sizeof(crc_stored)))
  {
    return false;
  }

  if(EEPROM_Crc32(data, length) != crc_stored)
  {
    Error_Set(OPENBMS_EEPROM_CRC_FAIL);
    return false;
  }

  return true;
}
static bool ADS131M08_Standby(void)
{
  uint8_t tx_buff[30] = {0};
  uint8_t rx_buff[30] = {0};
  bool res = true;

  // STANDBY command = 0x0022, MSB aligned in 24-bit word
  tx_buff[0] = 0x00;
  tx_buff[1] = 0x22;
  tx_buff[2] = 0x00;

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  if(HAL_SPI_TransmitReceive(adc_spi, tx_buff, rx_buff, 30, ADS131M0_SPI_TIMEOUT) != HAL_OK)
  {
    res = false;
  }

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
  return res;
}
static bool ADS131M08_Wakeup(void)
{
  uint8_t tx_buff[30] = {0};
  uint8_t rx_buff[30] = {0};
  bool res = true;

  // WAKEUP command = 0x0033, MSB aligned in 24-bit word
  tx_buff[0] = 0x00;
  tx_buff[1] = 0x33;
  tx_buff[2] = 0x00;

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  if(HAL_SPI_TransmitReceive(adc_spi, tx_buff, rx_buff, 30, ADS131M0_SPI_TIMEOUT) != HAL_OK)
  {
    res = false;
  }

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

  // After wakeup, wait for modulators to settle (8 × tMOD)
  // tMOD = 1/fMOD = 2/fCLKIN = 2/8192000 = 244ns
  // 8 × tMOD = ~2µs, but add margin
  HAL_Delay(1);

  return res;
}
static bool ADS131M08_ReadReg(uint8_t reg, uint8_t* data)
{
  // According to datasheet, if word is set to 24 bits, then total SPI transacation has 30 bytes
  // First 3 bytes are command, followed by 24 bytes of response, followed by 3 bytes of CRC
  uint8_t tx_buff[30] = {0};
  uint16_t cmd;
  bool res = true;

  cmd = 0xA000 | ((uint16_t)reg << 7);
  tx_buff[0] = (cmd >> 8) & 0xFF;
  tx_buff[1] =  cmd       & 0xFF;

  // CS low
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  if(HAL_SPI_TransmitReceive(adc_spi, tx_buff, data, 30, ADS131M0_SPI_TIMEOUT) != HAL_OK)
  {
    res = false;
  }

  // CS high
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

  return res;
}
static bool ADS131M08_ReadADCValues(uint8_t* data)
{
  // TX: 3 bytes NULL command + 27 bytes zeros
  // RX: 3 bytes STATUS + 24 bytes channel data (CH0-CH7) + 3 bytes CRC
  uint8_t tx_buff[30] = {0};
  bool res = true;

  // CS low
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  if(HAL_SPI_TransmitReceive(adc_spi, tx_buff, data, 30, ADS131M0_SPI_TIMEOUT) != HAL_OK)
  {
    res = false;
  }

  // CS high
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

  return res;
}
static bool ADS131M08_WriteReg(uint8_t reg, uint16_t value)
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

    // CS low
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  if(HAL_SPI_TransmitReceive(adc_spi, tx_buff, rx_buff, 30, ADS131M0_SPI_TIMEOUT) != HAL_OK)
  {
    res = false;
  }

  // CS high
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
  return res;
}
static void ADS131M08_Init(void)
{
  uint8_t  rx_data[30] = {0};
  uint32_t start_tick;

  // Reset the ADS131M08 by toggling the SYNC pin low, then high
  HAL_GPIO_WritePin(ADC_SYNC_GPIO_Port, ADC_SYNC_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(ADC_SYNC_GPIO_Port, ADC_SYNC_Pin, GPIO_PIN_SET);

  // Wait for DRDY to go high indicating reset complete. A missing ADS or a stuck DRDY
  // must not hang the boot, the init fails and the board runs on without measurements.
  start_tick = HAL_GetTick();
  while(HAL_GPIO_ReadPin(ADC_DRDY_GPIO_Port, ADC_DRDY_Pin) == GPIO_PIN_RESET)
  {
    if((HAL_GetTick() - start_tick) >= ADS_RESET_TIMEOUT_MS)
    {
      Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
      return;
    }
  }

  // Standby the ADS131M08 to allow register configuration
  if(!ADS131M08_Standby())
  {
    Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  // Read ADC ID register (0x00) to verify communication
  if(!ADS131M08_ReadReg(0x00, rx_data))
  {
    Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  // To read out register, actually one more transaction is needed after writing the command, 
  // so read ID register again to get the value
  if(!ADS131M08_ReadReg(0x00, rx_data))
  {
    Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  // Check if ID is good
  if(rx_data[0] != 0x28)
  {
    Error_Set(OPENBMS_ADS131M08_ID_FAIL);
    return;
  }

  // Clear RESET bit in MODE register
  // Default MODE = 0x0510, clear bit 10 → 0x0110
  if(!ADS131M08_WriteReg(0x02, 0x0110))
  {
      Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
      return;
  }

  // Initialize CLOCK register, all channels enabled, about 250 SPS
  if(!ADS131M08_WriteReg(0x03, 0xFF1F))
  {
    Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  // Initilize GAIN1 register, PGA3 = PGA2 = PGA1 = 4, PGA0 = 16
  if(!ADS131M08_WriteReg(0x04, 0x2224))
  {
    Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  // Initilize GAIN2 register, PGA7 = PGA6 = PGA5 = PGA4 = 4
  if(!ADS131M08_WriteReg(0x05, 0x2222))
  {
    Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  /*
  // Start internal offset calibration
  ADS131M08_WriteReg(0x09, 0x0001);  // CH0_CFG MUX=01
  ADS131M08_WriteReg(0x0E, 0x0001);  // CH1_CFG MUX=01
  ADS131M08_WriteReg(0x13, 0x0001);  // CH2_CFG MUX=01
  ADS131M08_WriteReg(0x18, 0x0001);  // CH3_CFG MUX=01
  ADS131M08_WriteReg(0x1D, 0x0001);  // CH4_CFG MUX=01
  ADS131M08_WriteReg(0x22, 0x0001);  // CH5_CFG MUX=01
  ADS131M08_WriteReg(0x27, 0x0001);  // CH6_CFG MUX=01
  ADS131M08_WriteReg(0x2C, 0x0001);  // CH7_CFG MUX=01
  */

  // Wake the ADS131M08 up, the configuration is done and it starts converting
  if(!ADS131M08_Wakeup())
  {
    Error_Set(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  // ADS is configured, now the DRDY interrupt can read samples. Drop a DRDY edge
  // latched during the configuration so the first interrupt is a fresh sample.
  __HAL_GPIO_EXTI_CLEAR_IT(ADC_DRDY_Pin);
  HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  // Wait a bit to settle down, 3-4 samples
  HAL_Delay(10);
  ads_calibration_start = true;
  
  // Wait to complete calibration
  HAL_Delay(500);
  ads_calibration_finish = true;

  /*
  // Stop internal offset calibration
  ADS131M08_WriteReg(0x09, 0x0000);  // CH0_CFG MUX=01
  ADS131M08_WriteReg(0x0E, 0x0000);  // CH1_CFG MUX=01
  ADS131M08_WriteReg(0x13, 0x0000);  // CH2_CFG MUX=01
  ADS131M08_WriteReg(0x18, 0x0000);  // CH3_CFG MUX=01
  ADS131M08_WriteReg(0x1D, 0x0000);  // CH4_CFG MUX=01
  ADS131M08_WriteReg(0x22, 0x0000);  // CH5_CFG MUX=01
  ADS131M08_WriteReg(0x27, 0x0000);  // CH6_CFG MUX=01
  ADS131M08_WriteReg(0x2C, 0x0000);  // CH7_CFG MUX=01
  */

  // Wait a bit to settle down, 3-4 samples
  HAL_Delay(10);
  periph_gpio.gpio_meas_cell_voltage_enable = true;
  ads_last_good_tick     = HAL_GetTick();     // the timeout starts now
  ads_start_measurements = true;
  
}
// CRC-16-CCITT (polynomial 0x1021, start value 0xFFFF), as the ADS131M08 sends it in MODE.CRC_TYPE = 0
static uint16_t ADS131M08_Crc16(const uint8_t *data, uint8_t length)
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
// the caller drops it, so the smoothed values keep their last good value. A good frame
// refreshes the time Periph_MeasurementOk looks at.
static bool ADS131M08_FrameOk(bool spi_ok, const uint8_t *rx_data)
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

    if(ADS131M08_Crc16(rx_data, 27) != crc_rx)
    {
      Error_Set(OPENBMS_ADS131M08_CRC_FAIL);
      ok = false;
    }
    else if(status & ADS_STATUS_RESET)
    {
      // The settings (gain, rate) are back to defaults, so the values are scaled wrong
      Error_Set(OPENBMS_ADS131M08_RESET);
      ok = false;
    }
    else if((status & ADS_STATUS_DRDY_ALL) != ADS_STATUS_DRDY_ALL)
    {
      // Not every channel has a new sample
      ok = false;
    }
  }

  if(ok)
  {
    ads_last_good_tick = HAL_GetTick();
  }

  return ok;
}
static void ADS131M08_Read(void)
{
  uint8_t rx_data[30] = {0};
  int32_t raw_value;
  float voltage_mv;
  float current;
  bool    spi_ok;

  spi_ok = ADS131M08_ReadADCValues(rx_data);

  // Before the init has finished the samples are not used, and not checked either
  if(!ads_start_measurements && !ads_calibration_start)
  {
    return;
  }

  // A bad frame is dropped: the smoothed values and the offset sums keep their last good value
  if(!ADS131M08_FrameOk(spi_ok, rx_data))
  {
    return;
  }

#ifdef OPENBMS_SIMULATION
  // The cell voltages and the current come from the host (Periph_SimSetData)
  return;
#endif

  if(ads_start_measurements)
  {
    // Extract CH0 value, get battery current by dividing voltage by shunt resistance
    raw_value = ((int32_t)rx_data[3] << 24) | (rx_data[4] << 16) | (rx_data[5] << 8);
    raw_value >>= 8;
    voltage_mv = (float)raw_value * (ADS_FULL_SCALE_GAIN_16 / 16777216.0f);
    // The shunt value comes from the config. Until it is loaded it is 0, and a division by 0
    // would leave the smoothed value at inf / NaN for good.
    if(calibration_data.shunt_resistance_mohm > 0.0f)
    {
      current = (voltage_mv - calibration_data.current_sensor_offset) / calibration_data.shunt_resistance_mohm;
      current *= calibration_data.current_sensor_gain;
      current *= 1000.0f;  // Convert to mA
      current *= -1;       // Change the sign

      current -= calibration_data.current_bias_ma;

      // Smooth the new sample into the current value
      periph_data.pack_current += ADS_READ_FILT_COEF * (current - periph_data.pack_current);
    }
  }


  // Extract CH1 - CH7 values and get cell voltages
  if(periph_gpio.gpio_meas_cell_voltage_enable & ads_start_measurements)
  {
    periph_data.pack_voltage = 0;

    // Measure all cell voltages
    for(uint8_t i = 0; i < 7; i++)
    {
      raw_value = ((int32_t)rx_data[i*3+6] << 24) | (rx_data[i*3+7] << 16) | (rx_data[i*3+8] << 8);
      raw_value >>= 8;
      voltage_mv = (float)raw_value * (ADS_FULL_SCALE_GAIN_4 / 16777216.0f);
      voltage_mv *= HW_CELL_VOLTAGE_RESISTANCE_FACTOR;
      voltage_mv -= calibration_data.voltage_offset[i];
      voltage_mv *= calibration_data.voltage_gain[i];

      // Smooth the new sample into the cell voltage
      periph_data.cell_voltage[i] += ADS_READ_FILT_COEF * (voltage_mv - periph_data.cell_voltage[i]);

      periph_data.pack_voltage += periph_data.cell_voltage[i];
    }
  }
  /*
  else if(gpio_meas_pack_voltage_enable)
  {
    // Only measure pack voltage from channel 7
    raw_value = ((int32_t)rx_data[24] << 24) | (rx_data[25] << 16) | (rx_data[26] << 8);
    raw_value >>= 8;
    voltage_mv = (float)raw_value * (1200.0f / 16777216.0f);
    periph_data.voltage = voltage_mv * periph_data.batt_voltage_resistance_factor;

    // Set update flags
    pack_voltage_updated = true;
  }
  */
  else if(ads_calibration_start)
  {
    // Record offset values
    for(uint8_t i = 0; i <= 7; i++)
    {
      raw_value = ((int32_t)rx_data[i*3+3] << 24) | (rx_data[i*3+4] << 16) | (rx_data[i*3+5] << 8);
      raw_value >>= 8;
      
      ads_cal_val_sum[i] += raw_value;
    }

    ads_cal_val_count++;

    if(ads_calibration_finish)
    {
      // Get offset values now
      for(uint8_t i = 0; i <= 7; i++) 
      {
        ads_cal_val_sum[i] /= (int32_t) ads_cal_val_count;

        if(i == 0) 
        {
          calibration_data.current_sensor_offset = (float) ads_cal_val_sum[i] * (ADS_FULL_SCALE_GAIN_16 / 16777216.0f);
        }
        else 
        {
          calibration_data.voltage_offset[i-1] = (float) ads_cal_val_sum[i] * (ADS_FULL_SCALE_GAIN_4 / 16777216.0f);
          calibration_data.voltage_offset[i-1] *= HW_CELL_VOLTAGE_RESISTANCE_FACTOR;
        }
      }

      ads_calibration_start   = false;
      ads_calibration_finish  = false;
    }
  }
}
static float NTC_GetTemperature(float r_ntc)
{
    if(r_ntc <= 0.0f)
    {
        return -273.15f;
    }

    float temp_k = 1.0f / ((1.0f / calibration_data.ntc_t_nominal) + (1.0f / calibration_data.ntc_beta) * logf(r_ntc / calibration_data.ntc_r_nominal));

    // Offset subtracted, like the voltage and current offsets
    return (temp_k - 273.15f - calibration_data.temperature_offset);
}
static void STM32_ADC_Init(void)
{
  HAL_ADCEx_Calibration_Start(internal_adc, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(internal_adc, (uint32_t*)stm32_adc_dma_buf, 3);
}
static void STM32_ADC_Read(void)
{
  // A broken NTC latches OPENBMS_NTC_FAULT and the temperature is not read any more.
  // Clearing the errors starts the reading again (and sets the error again if it is still broken).
  if(!Error_IsSet(OPENBMS_NTC_FAULT))
  {
    if(stm32_adc_dma_buf[0] < NTC_ADC_MIN_COUNTS || stm32_adc_dma_buf[0] > NTC_ADC_MAX_COUNTS)
    {
      Error_Set(OPENBMS_NTC_FAULT);
    }
    else
    {
      // NTC resistance from the divider with the fixed resistor (ratiometric, VDD cancels out)
      float r_ntc = calibration_data.ntc_r_fixed / ((4095.0f / (float)stm32_adc_dma_buf[0]) - 1);

      // Get temperature now
      periph_data.temperature_package = NTC_GetTemperature(r_ntc);
    }
  }

  // ST factory calibrated VREFINT value measured at 3.0V, 30°C
  // Stored in flash at fixed address
  uint16_t vrefint_cal = *((uint16_t*)0x1FFF75AA);

  // Calculate actual VDD
  // vrefint_cal was measured at 3.0V so multiply by 3.0
  periph_data.main_vdd_voltage_mv = (3000.0f * (float)vrefint_cal) / (float)stm32_adc_dma_buf[1];

  // Calculate STM32 temperature
  uint16_t ts_cal1 = *((uint16_t*)0x1FFF75A8);
  uint16_t ts_cal2 = *((uint16_t*)0x1FFF75CA);

  // Scale UP to 3.0V reference — VDD > 3.0V means raw is artificially low
  float adc_corrected = (float)stm32_adc_dma_buf[2] * (periph_data.main_vdd_voltage_mv / 3000.0f);

  periph_data.temperature_stm32 = (130.0f - 30.0f) / ((float)ts_cal2 - (float)ts_cal1)
                          * (adc_corrected - (float)ts_cal1) + 30.0f;
  
}
static void Timer_Init(void)
{
  HAL_TIM_Base_Start_IT(periodic_timer);
}
static void GPIO_Ctrl(void)
{
  // Set main FET and driver state
  HAL_GPIO_WritePin(DRV_MAIN_EN_GPIO_Port, DRV_MAIN_EN_Pin, (GPIO_PinState) periph_gpio.gpio_main_drv_enable);
  HAL_GPIO_WritePin(DRV_FET_EN_GPIO_Port, DRV_FET_EN_Pin, (GPIO_PinState) periph_gpio.gpio_main_fet_enable);

  // Set precharge/predischarge FET state
  HAL_GPIO_WritePin(DRV_PRE_FET_EN_GPIO_Port, DRV_PRE_FET_EN_Pin, (GPIO_PinState) periph_gpio.gpio_pre_fet_enable);

  // Enable cell voltage measuring
  HAL_GPIO_WritePin(MEAS_CELL_GPIO_Port, MEAS_CELL_Pin, (GPIO_PinState) (periph_gpio.gpio_meas_cell_voltage_enable | periph_gpio.gpio_meas_pack_voltage_enable));

  // Enable pack voltage measuring
  HAL_GPIO_WritePin(MEAS_BATT_GPIO_Port, MEAS_BATT_Pin, (GPIO_PinState) periph_gpio.gpio_meas_pack_voltage_enable);

  // Set balancer
  HAL_GPIO_WritePin(CELL_1_BAL_GPIO_Port, CELL_1_BAL_Pin, (GPIO_PinState) periph_gpio.gpio_cell_balancer_enable[0]);
  HAL_GPIO_WritePin(CELL_2_BAL_GPIO_Port, CELL_2_BAL_Pin, (GPIO_PinState) periph_gpio.gpio_cell_balancer_enable[1]);
  HAL_GPIO_WritePin(CELL_3_BAL_GPIO_Port, CELL_3_BAL_Pin, (GPIO_PinState) periph_gpio.gpio_cell_balancer_enable[2]);
  HAL_GPIO_WritePin(CELL_4_BAL_GPIO_Port, CELL_4_BAL_Pin, (GPIO_PinState) periph_gpio.gpio_cell_balancer_enable[3]);
  HAL_GPIO_WritePin(CELL_5_BAL_GPIO_Port, CELL_5_BAL_Pin, (GPIO_PinState) periph_gpio.gpio_cell_balancer_enable[4]);
  HAL_GPIO_WritePin(CELL_6_BAL_GPIO_Port, CELL_6_BAL_Pin, (GPIO_PinState) periph_gpio.gpio_cell_balancer_enable[5]);
  HAL_GPIO_WritePin(CELL_7_BAL_GPIO_Port, CELL_7_BAL_Pin, (GPIO_PinState) periph_gpio.gpio_cell_balancer_enable[6]);
  
  // Set main power signal on to keep OpenBMS on
  HAL_GPIO_WritePin(PWR_ON_GPIO_Port, PWR_ON_Pin, (GPIO_PinState) periph_gpio.gpio_pwr_on);

  // Read driver fault pin, inverse state
  periph_gpio.gpio_r_fet_driver_fault = (bool)(1 - (uint8_t)HAL_GPIO_ReadPin(DRV_FLT_GPIO_Port, DRV_FLT_Pin));

  // Read driver gate fault pin, inverse state
  periph_gpio.gpio_r_fet_driver_gate_fault = (bool)(1 - (uint8_t)HAL_GPIO_ReadPin(DRV_FLT_GD_GPIO_Port, DRV_FLT_GD_Pin));

  // Read wake up pin, active high
  periph_gpio.gpio_r_wake_up = (bool)HAL_GPIO_ReadPin(WAKE_UP_GPIO_Port, WAKE_UP_Pin);

  // Read VCC power good pin
  periph_gpio.gpio_r_vcc_power_good = (bool)HAL_GPIO_ReadPin(PWR_PG_GPIO_Port, PWR_PG_Pin);

  // Hardware pin states go into the error flags like any other error
  if(periph_gpio.gpio_r_fet_driver_fault)       Error_Set(OPENBMS_HW_DRV_FAULT);
  if(periph_gpio.gpio_r_fet_driver_gate_fault)  Error_Set(OPENBMS_HW_DRV_GATE_FAULT);
  if(periph_gpio.gpio_r_wake_up)                Error_Set(OPENBMS_HW_WAKE_UP);
  if(!periph_gpio.gpio_r_vcc_power_good)        Error_Set(OPENBMS_HW_PWR_FAIL);
}
static void Periph_SetDefaultData(void)
{
  memset(&periph_data, 0, sizeof(Peripheral_Data_t));
  memset(&calibration_data, 0, sizeof(Calibration_Data_t));
}
void Periph_Init(void)
{
  // Initialize default peripheral data
  Periph_SetDefaultData();

  // Keep power on GPIO high
  periph_gpio.gpio_pwr_on = true;

  // Keep FET driver enabled (FET is still off)
  periph_gpio.gpio_main_drv_enable = true;

  STM32_ADC_Init();
  ADS131M08_Init();
  Timer_Init();
}
void Periph_Run(void)
{
    GPIO_Ctrl();
}
void Periph_SetFET(bool state)
{  
#ifdef OPENBMS_SIMULATION
    // The host simulates the load, the real FET stays off. fet_status still shows the request.
    periph_gpio.gpio_main_fet_enable = false;
#else
    periph_gpio.gpio_main_fet_enable = state;
#endif

    if(state) periph_data.fet_status |= BD_FET_MAIN;
    else      periph_data.fet_status &= ~BD_FET_MAIN;
    
}
void Periph_SetPreFET(bool state)
{
#ifdef OPENBMS_SIMULATION
    // The real pre-FET stays off as well
    periph_gpio.gpio_pre_fet_enable = false;
#else
    periph_gpio.gpio_pre_fet_enable = state;
#endif

    if(state) periph_data.fet_status |= BD_FET_PRE;
    else      periph_data.fet_status &= ~BD_FET_PRE;
}
void Periph_GetData(Peripheral_Data_t *pd)
{
  // The ADS131 DRDY and the internal ADC DMA interrupts write periph_data, so the copy
  // is done with all interrupts masked. It takes a few us, and the NVIC enables stay as they are.
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  memcpy(pd, &periph_data, sizeof(Peripheral_Data_t));
  __set_PRIMASK(primask);
}
bool Periph_MeasurementOk(void)
{
  // False when the ADS has not delivered a good sample for ADS_DATA_TIMEOUT_MS: bad frames
  // in a row, or the DRDY interrupt does not fire at all. One 32-bit read, so no masking.
#ifdef OPENBMS_SIMULATION
  // The measurements come from the host
  return true;
#else
  return (HAL_GetTick() - ads_last_good_tick) < ADS_DATA_TIMEOUT_MS;
#endif
}
#ifdef OPENBMS_SIMULATION
void Periph_SimSetData(const float *cell_mv, float current_ma)
{
  // Called from the main loop; the internal ADC interrupt also writes periph_data, so masked
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  periph_data.pack_current = current_ma;
  periph_data.pack_voltage = 0;

  for(uint8_t i = 0; i < BD_CELL_COUNT_MAX; i++)
  {
    periph_data.cell_voltage[i]  = cell_mv[i];
    periph_data.pack_voltage    += cell_mv[i];
  }
  __set_PRIMASK(primask);
}
#endif
void Periph_SetCalibration(const Config_Data_t *cfg)
{
  // The ADS131 and internal ADC interrupts read the calibration values, so they are
  // written with all interrupts masked. The offsets are measured by the peripheral
  // code itself, so they are left alone here.
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
  calibration_data.current_bias_ma        = cfg->current_bias_ma;
  __set_PRIMASK(primask);
}
void Periph_GetCalibration(Calibration_Data_t *cal)
{
  // The ADS131 and internal ADC interrupts write the measured offsets, so the copy is
  // done with all interrupts masked
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  memcpy(cal, &calibration_data, sizeof(Calibration_Data_t));
  __set_PRIMASK(primask);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ADC_DRDY_Pin)
  {
    ADS131M08_Read();
  }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if(hadc->Instance == ADC1)
  {
    STM32_ADC_Read();
  }
}