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

ADC_HandleTypeDef   ntc_adc     = hadc1;
CAN_HandleTypeDef   host_can    = hcan1;
I2C_HandleTypeDef   eeprom_i2c  = hi2c1;
SMBUS_HandleTypeDef host_smbus  = hsmbus2;
SPI_HandleTypeDef   adc_spi     = hspi1;
UART_HandleTypeDef  debug_uart  = huart1;

static OpenBMS_Status_t EEPROM_Init(void)
{
  uint8_t data[2];

  // Read first 2 bytes of EEPROM identification page to verify communication
  if(HAL_I2C_Master_Receive(&eeprom_i2c, EEPROM_I2C_ADDRESS | 0x01, data, 2, EEPROM_I2C_TIMEOUT) != HAL_OK)
  {
    return OPENBMS_EEPROM_INIT_FAIL;
  }

  // Check if the EEPROM returns the expected identification bytes
  if(data[0] != 0x20 || data[1] != 0xE0)
  {
    return OPENBMS_EEPROM_INIT_FAIL;
  }

  return OPENBMS_OK;
}
static OpenBMS_Status_t ADS131M08_ReadReg(uint8_t reg, uint16_t *status, uint8_t* value)
{
  // According to datasheet, if word is set to 24 bits, then total SPI transacation has 30 bytes
  // First 3 bytes are command, followed by 24 bytes of response, followed by 3 bytes of CRC
  OpenBMS_Status_t res = OPENBMS_OK;
  uint8_t tx_buff[30] = {0};
  uint8_t rx_buff[30] = {0};
  uint16_t cmd;

  cmd = 0xA000 | ((uint16_t)reg << 7);
  tx_buff[0] = (cmd >> 8) & 0xFF;
  tx_buff[1] =  cmd       & 0xFF;

  // CS low
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  if(HAL_SPI_TransmitReceive(&adc_spi, tx_buff, rx_buff, 30, ADS131M0_SPI_TIMEOUT) != HAL_OK)
  {
    res = OPENBMS_ADS131M08_SPI_FAIL;
  }

  // CS high
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

  // Assign status word and value from response
  *status = (rx_buff[0] << 8) | rx_buff[1];
  memcpy(value, &rx_buff[3], 24);

  return res;
}
static OpenBMS_Status_t ADS131M08_WriteReg(uint8_t reg, uint16_t value)
{
  OpenBMS_Status_t res = OPENBMS_OK;
  uint8_t tx_buff[6] = {0};
  uint8_t rx_buff[6] = {0};
  uint16_t cmd;

  cmd = 0x6000 | ((uint16_t)reg << 7);
  tx_buff[0] = (cmd >> 8) & 0xFF;
  tx_buff[1] =  cmd       & 0xFF;
  tx_buff[2] = 0x00;

  tx_buff[3] = 0x00;
  tx_buff[4] = (value >> 8) & 0xFF;
  tx_buff[5] =  value       & 0xFF;

    // CS low
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  if(HAL_SPI_TransmitReceive(&adc_spi, tx_buff, rx_buff, 6, ADS131M0_SPI_TIMEOUT) != HAL_OK)
  {
    res = OPENBMS_ADS131M08_SPI_FAIL;
  }

  // CS high
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

  return res;
}
static OpenBMS_Status_t ADS131M08_Init(void)
{
  uint8_t tx_data[30] = {0};
  uint8_t rx_data[30] = {0};
  uint16_t status;

  // Read ADC ID register (0x00) to verify communication
  if(ADS131M08_ReadReg(0x00, &status, rx_data) != OPENBMS_OK)
  {
    return OPENBMS_ADS131M08_INIT_FAIL;
  }

  // Check if ID is good
  if(rx_data[0] != 0x28)
  {
    return OPENBMS_ADS131M08_INIT_FAIL;
  }

  // Initilize CLOCK register, all channels enabled, SPS set to 125
  if(ADS131M08_WriteReg(0x03, 0xFF13) != OPENBMS_OK)
  {
    return OPENBMS_ADS131M08_INIT_FAIL;
  }

  // Initilize GAIN1 register, PGA3 = PGA2 = PGA1 = 4, PGA0 = 16
  if(ADS131M08_WriteReg(0x04, 0x2224) != OPENBMS_OK)
  {
    return OPENBMS_ADS131M08_INIT_FAIL;
  }

  // Initilize GAIN2 register, PGA7 = PGA6 = PGA5 = PGA4 = 4
  if(ADS131M08_WriteReg(0x05, 0x2222) != OPENBMS_OK)
  {
    return OPENBMS_ADS131M08_INIT_FAIL;
  }


  return OPENBMS_OK;
}

OpenBMS_Status_t OpenBMS_Ctrl_Init(void)
{
  
  return OPENBMS_OK;
}
OpenBMS_Status_t OpenBMS_Ctrl_Run(void)
{
  return OPENBMS_OK;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ADC_DRDY_Pin)
    {
        // Your interrupt code here
        // e.g. set a flag, notify a task, read data
    }
}