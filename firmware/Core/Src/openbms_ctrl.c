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
#include "stm32l4xx_hal_gpio.h"

ADC_HandleTypeDef   *ntc_adc         = &hadc1;
CAN_HandleTypeDef   *host_can        = &hcan1;
CRC_HandleTypeDef   *hw_crc          = &hcrc;
I2C_HandleTypeDef   *eeprom_i2c      = &hi2c1;
SMBUS_HandleTypeDef *host_smbus      = &hsmbus2;
SPI_HandleTypeDef   *adc_spi         = &hspi1;
UART_HandleTypeDef  *debug_uart      = &huart1;
uint64_t            OpenBMS_status  = 0;
OpenBMS_Data_t      OpenBMS_data    = {0};
OpenBMS_Config_t    OpenBMS_config  = {0};

static void HandleError(OpenBMS_Status_t error)
{
  OpenBMS_status |= (1ULL << error);
}
static void EEPROM_Init(void)
{
  uint8_t data[2];

  // Read first 2 bytes of EEPROM identification page to verify communication
  if(HAL_I2C_Master_Receive(eeprom_i2c, EEPROM_I2C_ADDRESS | 0x01, data, 2, EEPROM_I2C_ID_TIMEOUT) != HAL_OK)
  {
    HandleError(OPENBMS_EEPROM_ID_READ_FAIL);
    return;
  }

  // Check if the EEPROM returns the expected identification bytes
  if(data[0] != 0x20 || data[1] != 0xE0)
  {
    HandleError(OPENBMS_EEPROM_ID_VAL_FAIL);
    return;
  }
}
static void EEPROM_Read(void)
{
  uint32_t crc_stored   = 0;
  uint32_t crc_calc     = 0;
  uint8_t  data[32];
  uint16_t total_bytes  = sizeof(OpenBMS_Config_t);
  uint16_t bytes_read   = 0;
  uint8_t  *dest        = (uint8_t *)&OpenBMS_config;
  uint16_t crc_address  = EEPROM_SIZE - sizeof(uint32_t);  // Last 4 bytes of EEPROM

  // Read struct starting at address 0x0000
  while(bytes_read < total_bytes)
  {
    uint16_t chunk_size = total_bytes - bytes_read;
    if(chunk_size > 32)
    {
      chunk_size = 32;
    }

    if(HAL_I2C_Mem_Read(eeprom_i2c,
                        EEPROM_I2C_ADDRESS,
                        bytes_read,
                        I2C_MEMADD_SIZE_16BIT,
                        data,
                        chunk_size,
                        EEPROM_I2C_READ_TIMEOUT) != HAL_OK)
    {
      HandleError(OPENBMS_EEPROM_READ_FAIL);
      return;
    }

    memcpy(dest + bytes_read, data, chunk_size);
    bytes_read += chunk_size;
  }

  // Read CRC from last 4 bytes of EEPROM
  if(HAL_I2C_Mem_Read(eeprom_i2c,
                      EEPROM_I2C_ADDRESS,
                      crc_address,
                      I2C_MEMADD_SIZE_16BIT,
                      (uint8_t *)&crc_stored,
                      sizeof(uint32_t),
                      EEPROM_I2C_READ_TIMEOUT) != HAL_OK)
  {
    HandleError(OPENBMS_EEPROM_READ_FAIL);
    return;
  }

  // Calculate CRC32 over read struct and compare with stored CRC
  crc_calc = HAL_CRC_Calculate(hw_crc, (uint32_t *)&OpenBMS_config, total_bytes / sizeof(uint32_t));

  if(crc_calc != crc_stored)
  {
    HandleError(OPENBMS_EEPROM_CRC_FAIL);
    return;
  }
}
static bool ADS131M08_ReadReg(uint8_t reg, uint16_t *status, uint8_t* value)
{
  // According to datasheet, if word is set to 24 bits, then total SPI transacation has 30 bytes
  // First 3 bytes are command, followed by 24 bytes of response, followed by 3 bytes of CRC
  uint8_t tx_buff[30] = {0};
  uint8_t rx_buff[30] = {0};
  uint16_t cmd;
  bool res = true;

  cmd = 0xA000 | ((uint16_t)reg << 7);
  tx_buff[0] = (cmd >> 8) & 0xFF;
  tx_buff[1] =  cmd       & 0xFF;

  // CS low
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  if(HAL_SPI_TransmitReceive(adc_spi, tx_buff, rx_buff, 30, ADS131M0_SPI_TIMEOUT) != HAL_OK)
  {
    res = false;
  }

  // CS high
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

  // Assign status word and value from response
  *status = (rx_buff[0] << 8) | rx_buff[1];
  memcpy(value, &rx_buff[3], 24);

  return res;
}
static bool ADS131M08_WriteReg(uint8_t reg, uint16_t value)
{
  uint8_t tx_buff[6] = {0};
  uint8_t rx_buff[6] = {0};
  uint16_t cmd;
  bool res = true;

  cmd = 0x6000 | ((uint16_t)reg << 7);
  tx_buff[0] = (cmd >> 8) & 0xFF;
  tx_buff[1] =  cmd       & 0xFF;
  tx_buff[2] = 0x00;

  tx_buff[3] = 0x00;
  tx_buff[4] = (value >> 8) & 0xFF;
  tx_buff[5] =  value       & 0xFF;

    // CS low
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  if(HAL_SPI_TransmitReceive(adc_spi, tx_buff, rx_buff, 6, ADS131M0_SPI_TIMEOUT) != HAL_OK)
  {
    res = false;
  }

  // CS high
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
  return res;
}
static void ADS131M08_Init(void)
{
  uint8_t rx_data[30] = {0};
  uint16_t status;

  // Read ADC ID register (0x00) to verify communication
  if(!ADS131M08_ReadReg(0x00, &status, rx_data))
  {
    HandleError(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  // Check if ID is good
  if(rx_data[0] != 0x28)
  {
    HandleError(OPENBMS_ADS131M08_ID_FAIL);
    return;
  }

  // Initilize CLOCK register, all channels enabled, SPS set to 125
  if(!ADS131M08_WriteReg(0x03, 0xFF13))
  {
    HandleError(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  // Initilize GAIN1 register, PGA3 = PGA2 = PGA1 = 4, PGA0 = 16
  if(!ADS131M08_WriteReg(0x04, 0x2224))
  {
    HandleError(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  // Initilize GAIN2 register, PGA7 = PGA6 = PGA5 = PGA4 = 4
  if(!ADS131M08_WriteReg(0x05, 0x2222))
  {
    HandleError(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }
}
static void ADS131M08_Read(void)
{
  uint8_t rx_data[30] = {0};
  uint16_t status;
  int32_t raw_value;
  float voltage_mv;

  if(!ADS131M08_ReadReg(0x00, &status, rx_data))
  {
    HandleError(OPENBMS_ADS131M08_READ_FAIL);
  }

  // Extract CH0 value, get battery current by dividing voltage by shunt resistance
  raw_value = ((int32_t)rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8);
  raw_value >>= 8;
  voltage_mv = (float)raw_value * (150.0f / 16777216.0f);
  voltage_mv /= OpenBMS_config.shunt_resistance_mohms;
  OpenBMS_data.batt_current = voltage_mv;

  // Extract CH1 - CH7 values and get cell voltages
  for(uint8_t i = 1; i <= 7; i++)
  {
    raw_value = ((int32_t)rx_data[i*3+0] << 24) | (rx_data[i*3+1] << 16) | (rx_data[i*3+2] << 8);
    raw_value >>= 8;
    voltage_mv = (float)raw_value * (600.0f / 16777216.0f);
    voltage_mv /= OpenBMS_config.cell_voltage_resistance_factor;
    OpenBMS_data.cell_voltages[i-1] = voltage_mv;
  }

  // Set update flags
  OpenBMS_data.batt_current_updated   = true;
  OpenBMS_data.cell_voltages_updated  = true;
}
void OpenBMS_Ctrl_Init(void)
{
  EEPROM_Init();
  EEPROM_Read();

  ADS131M08_Init();
}
void OpenBMS_Ctrl_Run(void)
{

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ADC_DRDY_Pin)
  {
    ADS131M08_Read();
  }
}