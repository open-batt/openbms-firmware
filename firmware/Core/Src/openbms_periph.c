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
#include "openbms_data.h"
#include <math.h>
#include "openbms_periph.h"

#define ADS_FULL_SCALE_GAIN_16                150.0f
#define ADS_FULL_SCALE_GAIN_4                 600.0f
#define ADS_READ_FILT_COEF                    0.166f

#define HW_CELL_VOLTAGE_RESISTANCE_FACTOR     29.3333333f
#define HW_BATT_VOLTAGE_RESISTANCE_FACTOR     1
#define HW_SHUNT_RESISTANCE_MOHM              2.0f

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
CAN_HandleTypeDef           *host_can                   = &hcan1;
CRC_HandleTypeDef           *hw_crc                     = &hcrc;
I2C_HandleTypeDef           *eeprom_i2c                 = &hi2c1;
SMBUS_HandleTypeDef         *host_smbus                 = &hsmbus2;
SPI_HandleTypeDef           *adc_spi                    = &hspi1;
TIM_HandleTypeDef           *periodic_timer             = &htim7;

static Periph_GPIO_t        periph_gpio                 = {0};
static Peripheral_Data_t    periph_data                 = {0};

static uint64_t             OpenBMS_status              = 0;

static bool                 main_vdd_mv_updated         = false;
static bool                 temperature_ntc_updated     = false;
static bool                 temperature_stm32_updated   = true;
static uint8_t              stm32_adc_conv_index        = 0;
static float_t              cell_voltage_filt_prev[7]   = {0};
static float                current_filt_prev           = 0;
static float                pack_vol_filt_prev          = 0;

static bool                 ads_calibration_start       = false;
static bool                 ads_calibration_finish      = false;
static bool                 ads_start_measurements      = false;
static int32_t              ads_cal_val_sum[8]          = {0};
static uint32_t             ads_cal_val_count           = 0;

static void HandleError(OpenBMS_Status_t error)
{
  OpenBMS_status |= (1ULL << error);
}
/*
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
  uint8_t  *dest        = (uint8_t *)&bd;
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
  crc_calc = HAL_CRC_Calculate(hw_crc, (uint32_t *)&bd, total_bytes / sizeof(uint32_t));

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
  uint8_t  *src         = (uint8_t *)&bd;
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
  crc_calc = HAL_CRC_Calculate(&hcrc, (uint32_t *)&bd, sizeof(OpenBMS_Data_t));

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
*/
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
  uint8_t rx_data[30] = {0};

  // Reset the ADS131M08 by toggling the SYNC pin low, then high
  HAL_GPIO_WritePin(ADC_SYNC_GPIO_Port, ADC_SYNC_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(ADC_SYNC_GPIO_Port, ADC_SYNC_Pin, GPIO_PIN_SET);

  // Wait for DRDY to go high indicating reset complete
  while(HAL_GPIO_ReadPin(ADC_DRDY_GPIO_Port, ADC_DRDY_Pin) == GPIO_PIN_RESET);

  // Standby the ADS131M08 to allow register configuration
  if(!ADS131M08_Standby())
  {
    HandleError(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

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

  ADS131M08_ReadReg(0x02, rx_data);
  ADS131M08_ReadReg(0x02, rx_data);

  // Clear RESET bit in MODE register
  // Default MODE = 0x0510, clear bit 10 → 0x0110
  if(!ADS131M08_WriteReg(0x02, 0x0110))
  {
      HandleError(OPENBMS_ADS131M08_INIT_FAIL);
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

  // Standby the ADS131M08 to allow register configuration
  if(!ADS131M08_Wakeup())
  {
    HandleError(OPENBMS_ADS131M08_INIT_FAIL);
    return;
  }

  // Wait a bit to settle down, 3-4 samples
  HAL_Delay(10);
  ads_calibration_start = true;
  
  // Wait to complete calibration
  HAL_Delay(500);
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

  // Wait a bit to settle down, 3-4 samples
  HAL_Delay(10);
  periph_gpio.gpio_meas_cell_voltage_enable = true;
  ads_start_measurements = true;
  
}
static void ADS131M08_Read(void)
{
  uint8_t rx_data[30] = {0};
  int32_t raw_value;
  float voltage_mv;
  float current;
  float current_filtered;

  if(!ADS131M08_ReadADCValues(rx_data))
  {
    HandleError(OPENBMS_ADS131M08_READ_FAIL);
  }

  if(ads_start_measurements)
  {
    // Extract CH0 value, get battery current by dividing voltage by shunt resistance
    raw_value = ((int32_t)rx_data[3] << 24) | (rx_data[4] << 16) | (rx_data[5] << 8);
    raw_value >>= 8;
    voltage_mv = (float)raw_value * (ADS_FULL_SCALE_GAIN_16 / 16777216.0f);
    current = voltage_mv / HW_SHUNT_RESISTANCE_MOHM;
    current -= periph_data.current_sensor_offset;
    current *= 1000.0f;  // Convert to mA

    // Todo: remove hardcoded value
    current -= 28.3;
    
    current_filtered = current_filt_prev + ADS_READ_FILT_COEF * (current - current_filt_prev);
    current_filt_prev = current_filtered;

    // Assign calulcated values
    periph_data.pack_current = current;
    periph_data.pack_current_filtered = current_filtered;
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
      periph_data.cell_voltage[i] = voltage_mv * HW_CELL_VOLTAGE_RESISTANCE_FACTOR;
      periph_data.cell_voltage[i] -= periph_data.voltage_offset[i];
      periph_data.cell_voltage[i] *= periph_data.voltage_gain[i];

      // Get filtered voltage now
      periph_data.cell_voltage_filtered[i] = cell_voltage_filt_prev[i] + ADS_READ_FILT_COEF * (periph_data.cell_voltage[i] - cell_voltage_filt_prev[i]);
      cell_voltage_filt_prev[i] = periph_data.cell_voltage_filtered[i];

      periph_data.pack_voltage += periph_data.cell_voltage_filtered[i];
    }

    // Get filtered pack voltage
    periph_data.pack_voltage_filtered = pack_vol_filt_prev + ADS_READ_FILT_COEF * (periph_data.pack_voltage - pack_vol_filt_prev);
    pack_vol_filt_prev = periph_data.pack_voltage_filtered;
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
        ads_cal_val_sum[i] /= ads_cal_val_count;

        if(i == 0) 
        {
          periph_data.current_sensor_offset = (float) ads_cal_val_sum[i] * (ADS_FULL_SCALE_GAIN_16 / 16777216.0f);
          periph_data.current_sensor_offset /= HW_SHUNT_RESISTANCE_MOHM;
        }
        else 
        {
          periph_data.voltage_offset[i-1] = (float) ads_cal_val_sum[i] * (ADS_FULL_SCALE_GAIN_4 / 16777216.0f);
          periph_data.voltage_offset[i-1] *= HW_CELL_VOLTAGE_RESISTANCE_FACTOR;
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

    float temp_k = 1.0f / ((1.0f / periph_data.ntc_t_nominal) + (1.0f / periph_data.ntc_beta) * logf(r_ntc / periph_data.ntc_r_nominal));

    return (temp_k - 273.0f);
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
      periph_data.temperature_package = NTC_GetTemperature(r_ntc);

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
      periph_data.main_vdd_voltage_mv = (3000.0f * (float)vrefint_cal) / (float)adc_raw;

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
        float adc_corrected = (float)adc_raw * (periph_data.main_vdd_voltage_mv / 3000.0f);

        periph_data.temperature_stm32 = (130.0f - 30.0f) / ((float)ts_cal2 - (float)ts_cal1)
                                * (adc_corrected - (float)ts_cal1) + 30.0f;

        temperature_stm32_updated = true;
      }
    }
    break;
  }

  stm32_adc_conv_index++;
  if(stm32_adc_conv_index == 3) stm32_adc_conv_index = 0;
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

}
static void Periph_SetDefaultData(void)
{
  memset(&periph_data, 0, sizeof(Peripheral_Data_t));

  periph_data.current_sensor_gain                        = 1.0f;
  periph_data.voltage_gain[0]                            = 1.0313507f;
  periph_data.voltage_gain[1]                            = 1.0011376f;
  periph_data.voltage_gain[2]                            = 1.0244470f;
  periph_data.voltage_gain[3]                            = 0.9893198f;
  periph_data.voltage_gain[4]                            = 1.0129496f;
  periph_data.voltage_gain[5]                            = 1.0324656f;
  periph_data.voltage_gain[6]                            = 0.9872386f;
  periph_data.ntc_beta                                   = 3950.0f;
  periph_data.ntc_r_nominal                              = 10000.0f;
  periph_data.ntc_r_fixed                                = 10000.0f;
  periph_data.ntc_t_nominal                              = 298.15f;
}
void Periph_Init(void)
{
  // Initialize default peripheral data
  Periph_SetDefaultData();

  // Keep power on GPIO high
  periph_gpio.gpio_pwr_on = true;

  // Keep FET driver enabled (FET is still off)
  periph_gpio.gpio_main_drv_enable = true;

  //EEPROM_Init();
  //EEPROM_Read();

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
    periph_gpio.gpio_main_fet_enable = state;

    if(state) periph_data.fet_status |= BD_FET_MAIN;
    else      periph_data.fet_status &= ~BD_FET_MAIN;
    
}
void Periph_SetPreFET(bool state)
{
    periph_gpio.gpio_pre_fet_enable = state;

    if(state) periph_data.fet_status |= BD_FET_PRE;
    else      periph_data.fet_status &= ~BD_FET_PRE;
}
void Periph_GetData(Peripheral_Data_t *pd)
{
  // During memcpy operation, disable interrupts from ADS to prevent data corruption
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);            // ADS131 DRDY
  HAL_NVIC_DisableIRQ(ADC1_IRQn);                 // STM32 internal ADC

  memcpy(pd, &periph_data, sizeof(Peripheral_Data_t));

  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); 
  HAL_NVIC_EnableIRQ(ADC1_IRQn);
  
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        Periph_50msTimer();
    }
}