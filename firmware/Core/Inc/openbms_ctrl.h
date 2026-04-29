#ifndef OPENBMS_CTRL_H
#define OPENBMS_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#define EEPROM_I2C_ADDRESS      0xA0
#define EEPROM_I2C_TIMEOUT      100
#define ADS131M0_SPI_TIMEOUT    100

typedef enum 
{
    OPENBMS_OK = 0,
    OPENBMS_EEPROM_INIT_FAIL,
    OPENBMS_ADS131M08_SPI_FAIL,
    OPENNBMS_ADS131M08_INIT_FAIL,
    
} OpenBMS_Status_t;

typedef struct 
{
    uint32_t system_flags;
    uint16_t cell_count;
    bool initialized;
} OpenBMS_CtrlContext_t;

OpenBMS_Status_t OpenBMS_Ctrl_Init(void);
OpenBMS_Status_t OpenBMS_Ctrl_Run(void);

#ifdef __cplusplus
}
#endif

#endif // OPENBMS_CTRL_H
