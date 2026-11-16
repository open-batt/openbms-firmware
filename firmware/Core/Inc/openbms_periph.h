#ifndef OPENBMS_PERIPH_H
#define OPENBMS_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define PER_EEPROM_I2C_ADDRESS          0xA0
#define PER_EEPROM_I2C_READ_TIMEOUT     10              // ms, the address try of Periph_EepromDeviceReady
#define PER_EEPROM_READ_CHUNK           1024            // bytes per blocking I2C read (~92 ms at 100 kHz); the watchdog is refreshed between chunks
#define PER_EEPROM_READ_CHUNK_TIMEOUT_MS 200            // ms, limit for one blocking chunk read
#define PER_EEPROM_TRANSFER_TIMEOUT_MS  20              // ms, limit for one interrupt transfer (a page, ~6 ms at 100 kHz)
#define PER_EEPROM_READY_TRIALS         1               // address tries per Periph_EepromDeviceReady call
#define PER_BAL_SETTLE_MS               200             // the cell readings stay held this long after the last balancer turns off
#define PER_MEAS_SETTLE_MS              50              // after MEAS_CELL goes on the dividers settle this long before the cells are read
#define PER_DRV_RESET_MS                1               // DRV_MAIN_EN low this long to reset a latched TPS12000 fault (EN/UVLO below V(ENF))

#define PER_EEPROM_BUSY                 (0U)            // the interrupt transfer is running
#define PER_EEPROM_DONE                 (1U)            // finished
#define PER_EEPROM_FAIL                 (2U)            // I2C error, timeout (the I2C was restarted) or no transfer started

#define PER_ADS_SPI_TIMEOUT        100

void Periph_Init(void);
void Periph_Run(void);
void Periph_SetFET(bool state);
void Periph_SetPreFET(bool state);
void Periph_SetBalance(uint8_t mask);                                      // bit i = balancer FET of cell i + 1

bool Periph_EepromRead(uint16_t address, uint8_t *data, uint16_t length);            // blocking, only at boot (refreshes the watchdog)
bool Periph_EepromReadStart(uint16_t address, uint8_t *data, uint16_t length);       // interrupt read, then Periph_EepromTransferStatus
bool Periph_EepromWriteStart(uint16_t address, uint8_t *data, uint16_t length);      // interrupt write (within one EEPROM page), data unchanged until done
uint8_t Periph_EepromTransferStatus(void);                                           // PER_EEPROM_BUSY / DONE / FAIL of the running transfer
bool Periph_EepromDeviceReady(void);                                                 // one address try: false while the EEPROM write cycle runs
void Periph_EepromI2cError(void);                                                      // I2C1 error callback (forwarded by HAL_I2C_ErrorCallback)

void Periph_GetData(Peripheral_Data_t *pd);
void Periph_Off(void);                                                          // FETs, FET driver, balancers and voltage measuring off, pins written by the next Periph_Run
void Periph_PowerOff(void);                                                     // PWR_ON low: the 3.3 V goes off unless WAKE_UP is high, pin written by the next Periph_Run
void Periph_DriverReset(void);                                                  // resets a latched TPS12000 fault (DRV_MAIN_EN low for PER_DRV_RESET_MS, blocking)
void Periph_Wake(void);                                                         // FET driver and cell measuring on (FETs stay off), pins written by the next Periph_Run
bool Periph_CellMeasureReady(void);                                             // cell measuring is on and the cells have been read since it went on
void Periph_RemoveAppFlag(void);                                                // erases the app flag page, so the bootloader stays in after the next reset
bool Periph_CellMeasureOn(void);                                          // the cell voltage dividers are switched on (MEAS_CELL), so they drain the cells
void Periph_SetCalibration(const Config_Data_t *cfg);
#ifdef OPENBMS_TEST
bool Periph_AdsTestStandby(bool standby);                                       // true - ADS131M08 standby (DRDY interrupt off), false - wake up
bool Periph_AdsTestStandbyOn(void);                                             // the ADS is in test standby
void Periph_MeasTestOff(bool off);                                              // true - MEAS_CELL low, cells still read (zero point), false - on again
bool Periph_MeasTestOffOn(void);                                                // the dividers are off for the test
const float *Periph_TestBootVoltageOffset(void);                                // the 7 cell offsets (mV) the boot calibration measured
#endif


#ifdef __cplusplus
}
#endif

#endif
