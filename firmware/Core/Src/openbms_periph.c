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
#include "openbms_ctrl.h"
#include "openbms_periph.h"

#define ADS_FULL_SCALE_GAIN_16                150.0
#define ADS_FULL_SCALE_GAIN_4                 300.0f
#define ADS_FULL_SCALE_GAIN_2                 600.0f

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
    
    bool    run_learning;
 
} OpenBMS_Control_t;

ADC_HandleTypeDef           *internal_adc               = &hadc1;
CAN_HandleTypeDef           *host_can                   = &hcan1;
CRC_HandleTypeDef           *hw_crc                     = &hcrc;
I2C_HandleTypeDef           *eeprom_i2c                 = &hi2c1;
SMBUS_HandleTypeDef         *host_smbus                 = &hsmbus2;
SPI_HandleTypeDef           *adc_spi                    = &hspi1;

static OpenBMS_Control_t    OpenBMS_ctrl                = {0};
static uint64_t             OpenBMS_status              = 0;

static bool                 pack_current_updated        = false;
static bool                 pack_voltage_updated        = false;
static bool                 cell_voltage_updated        = false;
static bool                 main_vdd_mv_updated         = false;
static bool                 temperature_ntc_updated     = false;
static bool                 temperature_stm32_updated   = true;
static uint8_t              stm32_adc_conv_index        = 0;

static bool                 ads_calibration_start       = false;
static bool                 ads_calibration_finish      = false;
static int32_t              ads_cal_val_sum[8]          = {0};
static uint32_t             ads_cal_val_count           = 0;

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
  uint16_t total_bytes  = sizeof(OpenBMS_Data_t);
  uint16_t bytes_read   = 0;
  uint8_t  *dest        = (uint8_t *)&OpenBMS_data;
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
  crc_calc = HAL_CRC_Calculate(hw_crc, (uint32_t *)&OpenBMS_data, total_bytes / sizeof(uint32_t));

  if(crc_calc != crc_stored)
  {
    HandleError(OPENBMS_EEPROM_CRC_FAIL);
    return;
  }
}
static void EEPROM_Update(void)
{
  uint8_t  eeprom_buf[32];
  uint8_t  struct_buf[32];
  uint16_t total_bytes  = sizeof(OpenBMS_Data_t);
  uint16_t bytes_done   = 0;
  uint8_t  *src         = (uint8_t *)&OpenBMS_data;
  uint32_t crc_calc     = 0;
  uint16_t crc_address  = EEPROM_SIZE - sizeof(uint32_t);

  while(bytes_done < total_bytes)
  {
    uint16_t chunk_size = total_bytes - bytes_done;
    if(chunk_size > 32)
    {
      chunk_size = 32;
    }

    // Read 32 bytes from EEPROM
    if(HAL_I2C_Mem_Read(eeprom_i2c,
                        EEPROM_I2C_ADDRESS,
                        bytes_done,
                        I2C_MEMADD_SIZE_16BIT,
                        eeprom_buf,
                        chunk_size,
                        EEPROM_I2C_READ_TIMEOUT) != HAL_OK)
    {
      HandleError(OPENBMS_EEPROM_READ_FAIL);
      return;
   }

    // Copy corresponding chunk from struct
    memcpy(struct_buf, src + bytes_done, chunk_size);

    // Compare EEPROM chunk with struct chunk
    if(memcmp(eeprom_buf, struct_buf, chunk_size) != 0)
    {
      // Data differs — write struct chunk to EEPROM
      if(HAL_I2C_Mem_Write(eeprom_i2c,
                            EEPROM_I2C_ADDRESS,
                            bytes_done,
                            I2C_MEMADD_SIZE_16BIT,
                            struct_buf,
                            chunk_size,
                            EEPROM_I2C_WRITE_TIMEOUT) != HAL_OK)
      {
        HandleError(OPENBMS_EEPROM_WRITE_FAIL);
        return;
      }

      // M24C32 requires up to 5ms write cycle time — wait before next operation
      HAL_Delay(5);
    }

    bytes_done += chunk_size;
  }

  // Calculate new CRC32 over entire struct
  crc_calc = HAL_CRC_Calculate(&hcrc, (uint32_t *)&OpenBMS_data, sizeof(OpenBMS_Data_t));

  // Write CRC to last 4 bytes of EEPROM
  if(HAL_I2C_Mem_Write(eeprom_i2c,
                        EEPROM_I2C_ADDRESS,
                        crc_address,
                        I2C_MEMADD_SIZE_16BIT,
                        (uint8_t *)&crc_calc,
                        sizeof(uint32_t),
                        EEPROM_I2C_WRITE_TIMEOUT) != HAL_OK)
  {
    HandleError(OPENBMS_EEPROM_WRITE_FAIL);
    return;
  }

  // Wait for final CRC write cycle to complete
  HAL_Delay(5);
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
  uint8_t tx_buff[6] = {0};
  uint8_t rx_buff[6] = {0};
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

  // Read ADC ID register (0x00) to verify communication
  if(!ADS131M08_ReadReg(0x00, rx_data))
  {
    HandleError(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  // To read out register, actually one more transaction is needed after writing the command, 
  // so read ID register again to get the value
  if(!ADS131M08_ReadReg(0x00, rx_data))
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
  if(!ADS131M08_WriteReg(0x03, 0xFF1F))
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

  // Start internal offset calibration
  ADS131M08_WriteReg(0x09, 0x0001);  // CH0_CFG MUX=01
  ADS131M08_WriteReg(0x0E, 0x0001);  // CH1_CFG MUX=01
  ADS131M08_WriteReg(0x13, 0x0001);  // CH2_CFG MUX=01
  ADS131M08_WriteReg(0x18, 0x0001);  // CH3_CFG MUX=01
  ADS131M08_WriteReg(0x1D, 0x0001);  // CH4_CFG MUX=01
  ADS131M08_WriteReg(0x22, 0x0001);  // CH5_CFG MUX=01
  ADS131M08_WriteReg(0x27, 0x0001);  // CH6_CFG MUX=01
  ADS131M08_WriteReg(0x2C, 0x0001);  // CH7_CFG MUX=01

  // Wait a bit to settle down
  HAL_Delay(10);
  ads_calibration_start = true;
  
  HAL_Delay(100);
  ads_calibration_finish = true;

  // Stop internal offset calibration
  ADS131M08_WriteReg(0x09, 0x0000);  // CH0_CFG MUX=01
  ADS131M08_WriteReg(0x0E, 0x0000);  // CH1_CFG MUX=01
  ADS131M08_WriteReg(0x13, 0x0000);  // CH2_CFG MUX=01
  ADS131M08_WriteReg(0x18, 0x0000);  // CH3_CFG MUX=01
  ADS131M08_WriteReg(0x1D, 0x0000);  // CH4_CFG MUX=01
  ADS131M08_WriteReg(0x22, 0x0000);  // CH5_CFG MUX=01
  ADS131M08_WriteReg(0x27, 0x0000);  // CH6_CFG MUX=01
  ADS131M08_WriteReg(0x2C, 0x0000);  // CH7_CFG MUX=01

  HAL_Delay(10);
  OpenBMS_ctrl.gpio_meas_cell_voltage_enable = true;
  

}
static void ADS131M08_Read(void)
{
  uint8_t rx_data[30] = {0};
  int32_t raw_value;
  float voltage_mv;

  if(!ADS131M08_ReadADCValues(rx_data))
  {
    HandleError(OPENBMS_ADS131M08_READ_FAIL);
  }

  // Extract CH0 value, get battery current by dividing voltage by shunt resistance
  raw_value = ((int32_t)rx_data[3] << 24) | (rx_data[4] << 16) | (rx_data[5] << 8);
  raw_value >>= 8;
  voltage_mv = (float)raw_value * (150.0f / 16777216.0f);
  OpenBMS_data.current = voltage_mv * OpenBMS_data.shunt_resistance_mohms;

  // Set update flags
  pack_current_updated = true;

  // Extract CH1 - CH7 values and get cell voltages
  if(OpenBMS_ctrl.gpio_meas_cell_voltage_enable)
  {
    // Measure all cell voltages
    for(uint8_t i = 1; i <= 7; i++)
    {
      raw_value = ((int32_t)rx_data[i*3+3] << 24) | (rx_data[i*3+4] << 16) | (rx_data[i*3+5] << 8);
      raw_value >>= 8;
      voltage_mv = (float)raw_value * (600.0f / 16777216.0f);
      OpenBMS_data.cell_voltage[i-1] = voltage_mv * OpenBMS_data.cell_voltage_resistance_factor;
      OpenBMS_data.cell_voltage[i-1] -= OpenBMS_data.voltage_offset[i-1];
      OpenBMS_data.cell_voltage[i-1] *= OpenBMS_data.voltage_gain[i-1];
    }

    // Set update flags
    cell_voltage_updated = true;
  }
  /*
  else if(gpio_meas_pack_voltage_enable)
  {
    // Only measure pack voltage from channel 7
    raw_value = ((int32_t)rx_data[24] << 24) | (rx_data[25] << 16) | (rx_data[26] << 8);
    raw_value >>= 8;
    voltage_mv = (float)raw_value * (1200.0f / 16777216.0f);
    OpenBMS_data.voltage = voltage_mv * OpenBMS_data.batt_voltage_resistance_factor;

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
        ads_cal_val_sum[i] /= ads_cal_val_count;

        if(i == 0) 
        {
          OpenBMS_data.current_sensor_offset = (float) ads_cal_val_sum[i] * (150.0f / 16777216.0f);
          OpenBMS_data.current_sensor_offset *= OpenBMS_data.shunt_resistance_mohms;
        }
        else 
        {
          OpenBMS_data.voltage_offset[i-1] = (float) ads_cal_val_sum[i] * (600.0f / 16777216.0f);
          OpenBMS_data.voltage_offset[i-1] *= OpenBMS_data.cell_voltage_resistance_factor;
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

    float temp_k = 1.0f / ((1.0f / OpenBMS_data.ntc_t_nominal) + (1.0f / OpenBMS_data.ntc_beta) * logf(r_ntc / OpenBMS_data.ntc_r_nominal));

    return (temp_k * 10.0f);
}
static void STM32_ADC_Init(void)
{
  HAL_ADCEx_Calibration_Start(internal_adc, ADC_SINGLE_ENDED);
  HAL_ADC_Start_IT(internal_adc);
}
static void STM32_ADC_Read(void)
{
  uint32_t adc_raw = HAL_ADC_GetValue(internal_adc);

  switch(stm32_adc_conv_index)
  {
    case 0:         
    {
      // Calculate NTC voltage using actual VDD
      float r_ntc = 10000 / ((4095.0f / (float)adc_raw) - 1);

      // Get temperature now
      OpenBMS_data.temperature_package = NTC_GetTemperature(r_ntc);

      // Set flag
      temperature_ntc_updated = true;
    }
    break;

    case 1:    
    {
      // ST factory calibrated VREFINT value measured at 3.0V, 30°C
      // Stored in flash at fixed address
      uint16_t vrefint_cal = *((uint16_t*)0x1FFF75AA);

      // Calculate actual VDD
      // vrefint_cal was measured at 3.0V so multiply by 3.0
      OpenBMS_data.main_vdd_voltage_mv = (3000.0f * (float)vrefint_cal) / (float)adc_raw;

      // Set flag true
      main_vdd_mv_updated = true;
    }
    break;

    case 2:
    {
      if(main_vdd_mv_updated)
      {
        uint16_t ts_cal1 = *((uint16_t*)0x1FFF75A8);
        uint16_t ts_cal2 = *((uint16_t*)0x1FFF75CA);

        // Scale UP to 3.0V reference — VDD > 3.0V means raw is artificially low
        float adc_corrected = (float)adc_raw * (OpenBMS_data.main_vdd_voltage_mv / 3000.0f);

        OpenBMS_data.temperature_stm32 = (130.0f - 30.0f) / ((float)ts_cal2 - (float)ts_cal1)
                                            * (adc_corrected - (float)ts_cal1) + 30.0f;

        temperature_stm32_updated = true;
      }
    }
    break;
  }

  stm32_adc_conv_index++;
  if(stm32_adc_conv_index == 3) stm32_adc_conv_index = 0;
}
static void GPIO_Ctrl(void)
{
  // Set main FET and driver state
  HAL_GPIO_WritePin(DRV_MAIN_EN_GPIO_Port, DRV_MAIN_EN_Pin, (GPIO_PinState) OpenBMS_ctrl.gpio_main_drv_enable);
  HAL_GPIO_WritePin(DRV_FET_EN_GPIO_Port, DRV_FET_EN_Pin, (GPIO_PinState) OpenBMS_ctrl.gpio_main_fet_enable);

  // Set precharge/predischarge FET state
  HAL_GPIO_WritePin(DRV_PRE_FET_EN_GPIO_Port, DRV_PRE_FET_EN_Pin, (GPIO_PinState) OpenBMS_ctrl.gpio_pre_fet_enable);

  // Enable cell voltage measuring
  HAL_GPIO_WritePin(MEAS_CELL_GPIO_Port, MEAS_CELL_Pin, (GPIO_PinState) (OpenBMS_ctrl.gpio_meas_cell_voltage_enable | OpenBMS_ctrl.gpio_meas_pack_voltage_enable));

  // Enable pack voltage measuring
  HAL_GPIO_WritePin(MEAS_BATT_GPIO_Port, MEAS_BATT_Pin, (GPIO_PinState) OpenBMS_ctrl.gpio_meas_pack_voltage_enable);

  // Set balancer
  HAL_GPIO_WritePin(CELL_1_BAL_GPIO_Port, CELL_1_BAL_Pin, (GPIO_PinState) OpenBMS_ctrl.gpio_cell_balancer_enable[0]);
  HAL_GPIO_WritePin(CELL_2_BAL_GPIO_Port, CELL_2_BAL_Pin, (GPIO_PinState) OpenBMS_ctrl.gpio_cell_balancer_enable[1]);
  HAL_GPIO_WritePin(CELL_3_BAL_GPIO_Port, CELL_3_BAL_Pin, (GPIO_PinState) OpenBMS_ctrl.gpio_cell_balancer_enable[2]);
  HAL_GPIO_WritePin(CELL_4_BAL_GPIO_Port, CELL_4_BAL_Pin, (GPIO_PinState) OpenBMS_ctrl.gpio_cell_balancer_enable[3]);
  HAL_GPIO_WritePin(CELL_5_BAL_GPIO_Port, CELL_5_BAL_Pin, (GPIO_PinState) OpenBMS_ctrl.gpio_cell_balancer_enable[4]);
  HAL_GPIO_WritePin(CELL_6_BAL_GPIO_Port, CELL_6_BAL_Pin, (GPIO_PinState) OpenBMS_ctrl.gpio_cell_balancer_enable[5]);
  HAL_GPIO_WritePin(CELL_7_BAL_GPIO_Port, CELL_7_BAL_Pin, (GPIO_PinState) OpenBMS_ctrl.gpio_cell_balancer_enable[6]);
  
  // Set main power signal on to keep OpenBMS on
  HAL_GPIO_WritePin(PWR_ON_GPIO_Port, PWR_ON_Pin, (GPIO_PinState) OpenBMS_ctrl.gpio_pwr_on);

  // Read driver fault pin, inverse state
  OpenBMS_ctrl.gpio_r_fet_driver_fault = (bool)(1 - (uint8_t)HAL_GPIO_ReadPin(DRV_FLT_GPIO_Port, DRV_FLT_Pin));

  // Read driver gate fault pin, inverse state
  OpenBMS_ctrl.gpio_r_fet_driver_gate_fault = (bool)(1 - (uint8_t)HAL_GPIO_ReadPin(DRV_FLT_GD_GPIO_Port, DRV_FLT_GD_Pin));

  // Read wake up pin, active high
  OpenBMS_ctrl.gpio_r_wake_up = (bool)HAL_GPIO_ReadPin(WAKE_UP_GPIO_Port, WAKE_UP_Pin);

  // Read VCC power good pin
  OpenBMS_ctrl.gpio_r_vcc_power_good = (bool)HAL_GPIO_ReadPin(PWR_PG_GPIO_Port, PWR_PG_Pin);

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ADC_DRDY_Pin)
  {
    ADS131M08_Read();
  }
}
// This callback fires automatically when conversion completes
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if(hadc->Instance == ADC1)
  {
    STM32_ADC_Read();
  }
}
void Periph_Init(void)
{
    // Keep power on GPIO high
    OpenBMS_ctrl.gpio_pwr_on = true;

    //EEPROM_Init();
    //EEPROM_Read();

    ADS131M08_Init();
    STM32_ADC_Init();
}
void Periph_Run(void)
{
    GPIO_Ctrl();
}
void Periph_SetFET(bool state)
{
    OpenBMS_ctrl.gpio_main_drv_enable = state;
    OpenBMS_ctrl.gpio_main_fet_enable = state;
}
void Periph_SetPreFET(bool state)
{
    OpenBMS_ctrl.gpio_pre_fet_enable = state;
}
void Periph_SetLearningState(bool state)
{
    OpenBMS_ctrl.run_learning = state;
}
bool Perigh_GetLearningState(void)
{
    return OpenBMS_ctrl.run_learning;
}