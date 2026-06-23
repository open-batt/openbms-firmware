/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUF_SIZE           256U

#define APP_FLASH_START_PAGE    20U
#define APP_FLASH_END_PAGE      127U
#define APP_FLASH_NUM_PAGES     (APP_FLASH_END_PAGE - APP_FLASH_START_PAGE + 1U)
#define APP_FLASH_START_ADDRESS 0x0800A000U
#define APP_FLAG_ADDRESS        0x08009800U

#define APP_FLAG_MAGIC          0x12345678U
#define APP_FLAG_MAGIC_PAGE     19U

#define IHEX_RECORD_DATA        0x00U
#define IHEX_RECORD_EOF         0x01U
#define IHEX_RECORD_EXT_SEG     0x02U
#define IHEX_RECORD_START_SEG   0x03U
#define IHEX_RECORD_EXT_ADDR    0x04U
#define IHEX_RECORD_START_ADDR  0x05U

#define OK                            0U  
#define ERROR_ERRASE_FAIL             1U
#define ERROR_IHEX_ERR_CHECKSUM       3U
#define ERROR_IHEX_ERR_FORMAT         4U
#define ERROR_IHEX_ERR_FLASH          5U
#define ERROR_SET_FLAG_FAIL           6U  
#define ERROR_CRC_CHECKSUM            7U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
SMBUS_HandleTypeDef hsmbus2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint8_t  uart_rx_byte                = 0;
static char     uart_rx_buf[UART_BUF_SIZE]  = {0};
static uint8_t  uart_rx_idx                 = 0;
static uint8_t  process_cmd                 = 0;
static uint32_t flash_ext_address           = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_SMBUS_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

static void               Bootloader_Start(void);

static void               UART_PrintU8(uint8_t value);
static void               UART_ProcessLine(char *line);

static HAL_StatusTypeDef  Flash_Write(uint32_t address, uint8_t *data, uint32_t length);
static uint8_t            IHEX_HexToByte(const char *hex);

static uint8_t            CMD_Erase(void);
static uint8_t            CMD_Program(const char *line);
static uint8_t            CMD_CalculateCRC(uint32_t num_bytes, uint32_t expected_crc);
static uint8_t            CMD_SetAppFlag(void);
static void               CMD_Reset(void);
static void               CheckFlagAndJumpToApp(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Bootloader_Start(void)
{
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}
static void UART_PrintU8(uint8_t value)
{
    char buf[5];
    snprintf(buf, sizeof(buf), "%u\n", value);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 100);
}
static void UART_ProcessLine(char *line)
{
    uint8_t  status;

    if(strcmp(line, "B") == 0)
    {
        // Ping command
        status = 0;
    }
    else if(strcmp(line, "E") == 0)
    {
        // Erase command
        status = CMD_Erase();
    }
    else if(line[0] == 'P' && line[1] == ',')
    {
        // Program command — P,<hex_line>
        status = CMD_Program(&line[2]);
    }
   else if(line[0] == 'C' && line[1] == ',')
   {
      char     *token;
      uint32_t  num_bytes    = 0;
      uint32_t  expected_crc = 0;

      token        = strtok(&line[2], ",");
      num_bytes    = (uint32_t)atoi(token);

      token        = strtok(NULL, ",");
      expected_crc = (uint32_t)strtoul(token, NULL, 10);

      status = CMD_CalculateCRC(num_bytes, expected_crc);
    }
    else if(strcmp(line, "A") == 0)
    {
        // Set Application Flag command
        status = CMD_SetAppFlag();
    }
    else if(strcmp(line, "R") == 0)
    {
        // Reset command
        CMD_Reset();
    }
    else
    {
        status = 255;
    }

    UART_PrintU8(status);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        if(uart_rx_byte == '\n' || uart_rx_byte == '\r')
        {
            // Null terminate and process
            uart_rx_buf[uart_rx_idx] = '\0';

            if(uart_rx_idx > 0)
            {
              process_cmd = 1;
            }

            // Reset buffer
            uart_rx_idx = 0;
        }
        else
        {
            // Add byte to buffer — guard against overflow
            if(uart_rx_idx < UART_BUF_SIZE - 1)
            {
                uart_rx_buf[uart_rx_idx++] = (char)uart_rx_byte;
            }
            else
            {
                // Buffer overflow — reset
                uart_rx_idx = 0;
            }
        }

        // Re-arm interrupt
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}
static uint8_t CMD_Erase(void)
{
    HAL_StatusTypeDef      status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t               page_error = 0;

    status = HAL_FLASH_Unlock();
    if(status != HAL_OK)
    {
        return ERROR_ERRASE_FAIL;
    }

    erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase_init.Banks       = FLASH_BANK_1;
    erase_init.Page        = APP_FLASH_START_PAGE;
    erase_init.NbPages     = APP_FLASH_NUM_PAGES;

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);

    HAL_FLASH_Lock();

    if(status != HAL_OK)
    {
        return ERROR_ERRASE_FAIL;
    }

    return OK;
}
static HAL_StatusTypeDef Flash_Write(uint32_t address, uint8_t *data, uint32_t length)
{
    HAL_StatusTypeDef status;
    uint64_t          double_word;

    // Address must be 8-byte aligned
    if(address % 8 != 0)
    {
        return HAL_ERROR;
    }

    // Length must be multiple of 8
    if(length % 8 != 0)
    {
        return HAL_ERROR;
    }

    status = HAL_FLASH_Unlock();
    if(status != HAL_OK)
    {
        return status;
    }

    for(uint32_t i = 0; i < length; i += 8)
    {
        double_word = (uint64_t)data[i]
                    | ((uint64_t)data[i+1] << 8)
                    | ((uint64_t)data[i+2] << 16)
                    | ((uint64_t)data[i+3] << 24)
                    | ((uint64_t)data[i+4] << 32)
                    | ((uint64_t)data[i+5] << 40)
                    | ((uint64_t)data[i+6] << 48)
                    | ((uint64_t)data[i+7] << 56);

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   address + i,
                                   double_word);

        if(status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return status;
        }
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}
static uint8_t IHEX_HexToByte(const char *hex)
{
    unsigned int result = 0;
    sscanf(hex, "%2x", &result);
    return (uint8_t)result;
}
static uint8_t CMD_Program(const char *line)
{
    uint8_t  byte_count;
    uint16_t address;
    uint8_t  record_type;
    uint8_t  data[32];
    uint8_t  checksum;
    uint8_t  calc_checksum = 0;
    uint32_t flash_address;

    // Check start code
    if(line[0] != ':')
    {
        return ERROR_IHEX_ERR_FORMAT;
    }

    // Parse fields
    byte_count  = IHEX_HexToByte(&line[1]);
    address     = ((uint16_t)IHEX_HexToByte(&line[3]) << 8) | IHEX_HexToByte(&line[5]);
    record_type = IHEX_HexToByte(&line[7]);

    // Parse data bytes
    for(uint8_t i = 0; i < byte_count; i++)
    {
        data[i] = IHEX_HexToByte(&line[9 + i * 2]);
    }

    // Parse checksum
    checksum = IHEX_HexToByte(&line[9 + byte_count * 2]);

    // Verify checksum
    calc_checksum += byte_count;
    calc_checksum += (address >> 8) & 0xFF;
    calc_checksum += address & 0xFF;
    calc_checksum += record_type;
    for(uint8_t i = 0; i < byte_count; i++)
    {
        calc_checksum += data[i];
    }
    calc_checksum += checksum;

    if(calc_checksum != 0x00)
    {
        return ERROR_IHEX_ERR_CHECKSUM;
    }

    // Process record type
    switch(record_type)
    {
        case IHEX_RECORD_DATA:
        {
            flash_address = (flash_ext_address << 16) | address;

            uint8_t  write_buf[8];
            uint32_t i = 0;

            while(i < byte_count)
            {
                uint8_t chunk = ((byte_count - i) >= 8) ? 8 : (byte_count - i);

                memset(write_buf, 0xFF, 8);
                memcpy(write_buf, &data[i], chunk);

                if(Flash_Write(flash_address + i, write_buf, 8) != HAL_OK)
                {
                    return ERROR_IHEX_ERR_FLASH;
                }

                i += chunk;
            }
            break;
        }

        case IHEX_RECORD_EXT_ADDR:
        {
            flash_ext_address = ((uint32_t)data[0] << 8) | data[1];
            break;
        }

        case IHEX_RECORD_EOF:
        {
            break;
        }

        default:
            break;
    }

    return OK;
}
static uint8_t CMD_CalculateCRC(uint32_t num_bytes, uint32_t expected_crc)
{
    uint32_t app_start = APP_FLASH_START_ADDRESS;
    uint32_t calculated_crc;
    
    calculated_crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)app_start, num_bytes);

    if(calculated_crc != expected_crc)
    {
        return ERROR_CRC_CHECKSUM;
    }

    return OK;
}
static uint8_t CMD_SetAppFlag(void)
{
    HAL_StatusTypeDef      status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t               page_error  = 0;
    uint64_t               double_word = (uint64_t)APP_FLAG_MAGIC | ((uint64_t)APP_FLAG_MAGIC << 32);

    // Unlock flash
    status = HAL_FLASH_Unlock();
    if(status != HAL_OK)
    {
        return ERROR_SET_FLAG_FAIL;
    }

    // Erase page 19 first
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks     = FLASH_BANK_1;
    erase_init.Page      = APP_FLAG_MAGIC_PAGE;
    erase_init.NbPages   = 1U;

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if(status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return ERROR_SET_FLAG_FAIL;
    }

    // Write magic number
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                               APP_FLAG_ADDRESS,
                               double_word);
                               
    HAL_FLASH_Lock();

    if(status != HAL_OK)
    {
        return ERROR_SET_FLAG_FAIL;
    }

    return OK;
}
static void    CheckFlagAndJumpToApp(void)
{
    uint32_t app_flag      = *(uint32_t *)APP_FLAG_ADDRESS;
    uint32_t stack_pointer = *(uint32_t *)APP_FLASH_START_ADDRESS;
    uint32_t reset_handler = *(uint32_t *)(APP_FLASH_START_ADDRESS + 4);

    // Check magic flag
    if(app_flag != APP_FLAG_MAGIC)
    {
        return;
    }

    // Check valid stack pointer — must be in RAM range
    if(stack_pointer < 0x20000000U || stack_pointer > 0x20010000U)
    {
        return;
    }

    // Check valid reset handler — must be in app flash range
    if(reset_handler < APP_FLASH_START_ADDRESS || reset_handler > 0x0803FFFFU)
    {
        return;
    }

    // Disable all interrupts
    __disable_irq();

    // Deinit HAL peripherals before jump
    HAL_UART_DeInit(&huart1);
    HAL_RCC_DeInit();
    HAL_DeInit();

    // Set vector table
    SCB->VTOR = APP_FLASH_START_ADDRESS;

    // Jump to application
    __set_MSP(stack_pointer);

    void (*app_reset_handler)(void) = (void (*)(void))reset_handler;
    app_reset_handler();
}
static void CMD_Reset(void)
{
    NVIC_SystemReset();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2C2_SMBUS_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  CheckFlagAndJumpToApp();
  Bootloader_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(process_cmd)
    {
        UART_ProcessLine(uart_rx_buf);
        process_cmd = 0;
    }
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_8;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_SMBUS_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hsmbus2.Instance = I2C2;
  hsmbus2.Init.Timing = 0x10D19CE4;
  hsmbus2.Init.AnalogFilter = SMBUS_ANALOGFILTER_ENABLE;
  hsmbus2.Init.OwnAddress1 = 22;
  hsmbus2.Init.AddressingMode = SMBUS_ADDRESSINGMODE_7BIT;
  hsmbus2.Init.DualAddressMode = SMBUS_DUALADDRESS_DISABLE;
  hsmbus2.Init.OwnAddress2 = 0;
  hsmbus2.Init.OwnAddress2Masks = SMBUS_OA2_NOMASK;
  hsmbus2.Init.GeneralCallMode = SMBUS_GENERALCALL_DISABLE;
  hsmbus2.Init.NoStretchMode = SMBUS_NOSTRETCH_DISABLE;
  hsmbus2.Init.PacketErrorCheckMode = SMBUS_PEC_ENABLE;
  hsmbus2.Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_SMBUS_SLAVE;
  hsmbus2.Init.SMBusTimeout = 0x83D093E7;
  if (HAL_SMBUS_Init(&hsmbus2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_ON_GPIO_Port, PWR_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PWR_ON_Pin */
  GPIO_InitStruct.Pin = PWR_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_ON_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
