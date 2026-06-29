#ifndef OPENBMS_CTRL_H
#define OPENBMS_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "main.h"

#define EEPROM_I2C_ADDRESS          0xA0
#define EEPROM_SIZE                 4096
#define EEPROM_I2C_ID_TIMEOUT       50
#define EEPROM_I2C_READ_TIMEOUT     10
#define EEPROM_I2C_WRITE_TIMEOUT    20

#define ADS131M0_SPI_TIMEOUT        100

typedef enum 
{
    OPENBMS_EEPROM_ID_READ_FAIL,
    OPENBMS_EEPROM_ID_VAL_FAIL,
    OPENBMS_EEPROM_READ_FAIL,
    OPENBMS_EEPROM_WRITE_FAIL,
    OPENBMS_EEPROM_CRC_FAIL,

    OPENBMS_ADS131M08_READ_FAIL,
    OPENBMS_ADS131M08_INIT_FAIL,
    OPENBMS_ADS131M08_ID_FAIL,
    
} OpenBMS_Status_t;

void OpenBMS_Ctrl_Init(void);
void OpenBMS_Ctrl_Run(void);

#ifdef __cplusplus
}
#endif

#endif // OPENBMS_CTRL_H
