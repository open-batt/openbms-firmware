/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern ADC_HandleTypeDef hadc1;

extern CAN_HandleTypeDef hcan1;

extern I2C_HandleTypeDef hi2c1;
extern SMBUS_HandleTypeDef hsmbus2;

extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart1;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_SYNC_Pin GPIO_PIN_13
#define ADC_SYNC_GPIO_Port GPIOC
#define ADC_CS_Pin GPIO_PIN_14
#define ADC_CS_GPIO_Port GPIOC
#define ADC_DRDY_Pin GPIO_PIN_15
#define ADC_DRDY_GPIO_Port GPIOC
#define ADC_DRDY_EXTI_IRQn EXTI15_10_IRQn
#define CELL_2_BAL_Pin GPIO_PIN_0
#define CELL_2_BAL_GPIO_Port GPIOA
#define DRV_FLT_Pin GPIO_PIN_1
#define DRV_FLT_GPIO_Port GPIOA
#define DRV_FET_EN_Pin GPIO_PIN_2
#define DRV_FET_EN_GPIO_Port GPIOA
#define DRV_FLT_GD_Pin GPIO_PIN_3
#define DRV_FLT_GD_GPIO_Port GPIOA
#define MEAS_CELL_Pin GPIO_PIN_4
#define MEAS_CELL_GPIO_Port GPIOA
#define CELL_1_BAL_Pin GPIO_PIN_0
#define CELL_1_BAL_GPIO_Port GPIOB
#define PWR_PG_Pin GPIO_PIN_2
#define PWR_PG_GPIO_Port GPIOB
#define DRV_MAIN_EN_Pin GPIO_PIN_12
#define DRV_MAIN_EN_GPIO_Port GPIOB
#define WAKE_UP_Pin GPIO_PIN_13
#define WAKE_UP_GPIO_Port GPIOB
#define I2C2_INT_Pin GPIO_PIN_14
#define I2C2_INT_GPIO_Port GPIOB
#define PWR_ON_Pin GPIO_PIN_15
#define PWR_ON_GPIO_Port GPIOB
#define DRV_PRE_FET_EN_Pin GPIO_PIN_8
#define DRV_PRE_FET_EN_GPIO_Port GPIOA
#define CELL_7_BAL_Pin GPIO_PIN_15
#define CELL_7_BAL_GPIO_Port GPIOA
#define CELL_6_BAL_Pin GPIO_PIN_3
#define CELL_6_BAL_GPIO_Port GPIOB
#define MEAS_BATT_Pin GPIO_PIN_4
#define MEAS_BATT_GPIO_Port GPIOB
#define CELL_5_BAL_Pin GPIO_PIN_5
#define CELL_5_BAL_GPIO_Port GPIOB
#define CELL_4_BAL_Pin GPIO_PIN_8
#define CELL_4_BAL_GPIO_Port GPIOB
#define CELL_3_BAL_Pin GPIO_PIN_9
#define CELL_3_BAL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
