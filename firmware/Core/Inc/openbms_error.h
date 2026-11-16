#ifndef OPENBMS_ERROR_H
#define OPENBMS_ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum
{
    OPENBMS_EEPROM_READ_FAIL,
    OPENBMS_EEPROM_WRITE_FAIL,
    OPENBMS_EEPROM_CRC_FAIL,

    OPENBMS_ADS131M08_READ_FAIL,
    OPENBMS_ADS131M08_INIT_FAIL,
    OPENBMS_ADS131M08_ID_FAIL,
    OPENBMS_ADS131M08_CRC_FAIL,     // a sample frame failed its output CRC and was dropped
    OPENBMS_ADS131M08_RESET,        // the ADS reset by itself (STATUS RESET bit), its settings are back to defaults
    OPENBMS_ADS131M08_DATA_LOST,    // no good sample for PER_ADS_DATA_TIMEOUT_MS, the measurements are stale

    OPENBMS_HW_DRV_FAULT,           // FET driver fault

    OPENBMS_UART_ERROR,             // UART error (framing, noise, overrun, DMA) - the link was restarted
    OPENBMS_UART_TX_TIMEOUT,        // a reply did not finish sending in time and was aborted

    OPENBMS_NTC_FAULT,              // NTC open or shorted: no package temperature, OTP is skipped until the errors are cleared

    OPENBMS_I2C_ERROR,              // I2C slave bus error or stuck transfer - the I2C was restarted
    OPENBMS_CAN_ERROR,              // CAN bus-off or another CAN error, or the CAN stopped - the CAN was restarted

    OPENBMS_WATCHDOG_RESET,         // the last reset was done by the independent watchdog (IWDG): the main loop hung

    OPENBMS_CELL_MISSING,           // at boot a cell (below cell_count) read less than CTRL_CELL_PRESENT_MV: not connected

} OpenBMS_Error_t;

#define ERROR_FAULT_FLAGS        ((1ULL << OPENBMS_ADS131M08_READ_FAIL) | (1ULL << OPENBMS_ADS131M08_INIT_FAIL) | \
                                  (1ULL << OPENBMS_ADS131M08_ID_FAIL)   | (1ULL << OPENBMS_ADS131M08_RESET)     | \
                                  (1ULL << OPENBMS_ADS131M08_DATA_LOST) | (1ULL << OPENBMS_HW_DRV_FAULT)        | \
                                  (1ULL << OPENBMS_NTC_FAULT)           | (1ULL << OPENBMS_CELL_MISSING))

void     Error_Set(OpenBMS_Error_t error);      // latches the error, callable from interrupts
void     Error_ClearAll(void);                  // clears every error, callable from interrupts
uint64_t Error_GetFlags(void);                  // all latched errors, bit n = error n
bool     Error_IsSet(OpenBMS_Error_t error);    // true while that error is latched

#ifdef __cplusplus
}
#endif

#endif /* OPENBMS_ERROR_H */
