#ifndef OPENBMS_PERIPH_H
#define OPENBMS_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define PER_EEPROM_I2C_ADDRESS          0xA0
#define PER_EEPROM_SIZE                 4096
#define PER_EEPROM_I2C_READ_TIMEOUT     10
#define PER_EEPROM_I2C_WRITE_TIMEOUT    20
#define PER_EEPROM_I2C_READY_TRIALS     100             // ACK polling after a page write (write cycle is up to 5 ms)
#define PER_EEPROM_PAGE_SIZE            32

// CTRL_CONFIG_EEPROM_ADDRESS is in config.h

#define PER_ADS_SPI_TIMEOUT        100

void Periph_Init(void);
void Periph_Run(void);
void Periph_SetFET(bool state);
void Periph_SetPreFET(bool state);
void Periph_SetBalance(uint8_t mask);                                      // bit i = balancer FET of cell i + 1

bool Periph_EepromRead(uint16_t address, uint8_t *data, uint16_t length);
bool Periph_EepromWrite(uint16_t address, const uint8_t *data, uint16_t length);
bool Periph_EepromReadChecked(uint16_t address, uint8_t *data, uint16_t length);
bool Periph_EepromWriteChecked(uint16_t address, const uint8_t *data, uint16_t length);

void Periph_GetData(Peripheral_Data_t *pd);
void Periph_SetCalibration(const Config_Data_t *cfg);

#ifdef OPENBMS_SIMULATION
void Periph_SimSetData(const float *cell_mv, float current_ma, float temperature_c);   // the host's sample replaces the ADS and NTC measurements
#endif

#ifdef __cplusplus
}
#endif

#endif