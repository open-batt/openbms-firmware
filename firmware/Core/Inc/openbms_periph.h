#ifndef OPENBMS_PERIPH_H
#define OPENBMS_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define EEPROM_I2C_ADDRESS          0xA0
#define EEPROM_SIZE                 4096
#define EEPROM_I2C_READ_TIMEOUT     10
#define EEPROM_I2C_WRITE_TIMEOUT    20
#define EEPROM_I2C_READY_TRIALS     100             // ACK polling after a page write (write cycle is up to 5 ms)
#define EEPROM_PAGE_SIZE            32

// Config_Data_t is stored here, followed by its CRC-32 (see Periph_EepromWriteChecked)
#define CONFIG_EEPROM_ADDRESS       0x0000U

#define ADS131M0_SPI_TIMEOUT        100

void Periph_Init(void);
void Periph_Run(void);
void Periph_SetFET(bool state);
void Periph_SetPreFET(bool state);

bool Periph_EepromRead(uint16_t address, uint8_t *data, uint16_t length);
bool Periph_EepromWrite(uint16_t address, const uint8_t *data, uint16_t length);
bool Periph_EepromReadChecked(uint16_t address, uint8_t *data, uint16_t length);
bool Periph_EepromWriteChecked(uint16_t address, const uint8_t *data, uint16_t length);

void Periph_GetData(Peripheral_Data_t *pd);
void Periph_GetCalibration(Calibration_Data_t *cal);
void Periph_SetCalibration(const Config_Data_t *cfg);
bool Periph_MeasurementOk(void);                 // false when no good ADS sample came for ADS_DATA_TIMEOUT_MS

#ifdef OPENBMS_SIMULATION
void Periph_SimSetData(const float *cell_mv, float current_ma);   // the host's sample replaces the ADS measurements
#endif

#ifdef __cplusplus
}
#endif

#endif