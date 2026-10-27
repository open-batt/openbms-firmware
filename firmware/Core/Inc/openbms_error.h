#ifndef OPENBMS_ERROR_H
#define OPENBMS_ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// Error flags. Any module can report an error here; the flags are latched until
// they are cleared. Bit n of the 64-bit flags is the error with value n, and the
// flags are shown in the error_flags register (0x15).
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

    // Hardware pins, set by Periph_GpioCtrl through Error_Set when the pin state is seen
    OPENBMS_HW_DRV_FAULT,           // FET driver fault
    OPENBMS_HW_DRV_GATE_FAULT,      // FET driver gate fault
    OPENBMS_HW_PWR_FAIL,            // VCC power NOT good

    // Host communication
    OPENBMS_UART_ERROR,             // UART error (framing, noise, overrun, DMA) - the link was restarted
    OPENBMS_UART_TX_TIMEOUT,        // a reply did not finish sending in time and was aborted

    // Sensors
    OPENBMS_NTC_FAULT,              // NTC open or shorted: no package temperature, OTP is skipped until the errors are cleared

    // Host communication, added at the end so the bit numbers above stay the same
    OPENBMS_I2C_ERROR,              // I2C slave bus error or stuck transfer - the I2C was restarted
    OPENBMS_CAN_ERROR,              // CAN bus-off or another CAN error, or the CAN stopped - the CAN was restarted

    // System
    OPENBMS_WATCHDOG_RESET,         // the last reset was done by the independent watchdog (IWDG): the main loop hung

} OpenBMS_Error_t;

void     Error_Set(OpenBMS_Error_t error);      // latches the error, callable from interrupts
void     Error_ClearAll(void);                  // clears every error, callable from interrupts
uint64_t Error_GetFlags(void);                  // all latched errors, bit n = error n
bool     Error_IsSet(OpenBMS_Error_t error);    // true while that error is latched

#ifdef __cplusplus
}
#endif

#endif /* OPENBMS_ERROR_H */
