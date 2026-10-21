#ifndef OPENBMS_COMM_H
#define OPENBMS_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define UART_RX_BUFFER_SIZE                 2048U
#define UART_TX_TIMEOUT_MS                  50U         // a reply still sending after this is aborted
#define UART_TX_BUFFER_SIZE                 2048U

typedef enum
{
    CC_WRITE            = 0x01,
    CC_READ             = 0x02,
    CC_ACK              = 0x03,
    CC_ERROR            = 0x04,
    CC_EEPROM_CONFIG    = 0x05,
    CC_HPPC_START       = 0x06,
    CC_NORMAL_RUN       = 0x07,
    CC_SET_FET          = 0x08,
    CC_REBOOT           = 0x09,
    CC_PING             = 0x0A,
    CC_RESET_PROTECTION = 0x0B,
    CC_CLEAR_ERRORS     = 0x0C,
    CC_SET_PRE_FET      = 0x0D,
    CC_SIM_SAMPLE       = 0x0E,     // simulation build only (OPENBMS_SIMULATION in config.h)
    CC_BOOTLOADER       = 0x42,

} CommCmdType_t;

typedef enum
{
    CE_OK,
    CE_WRONG_CMD,
    CE_BAD_CRC,
    CE_NO_REG,
    CE_RO,
    CE_EEPROM,
    CE_STATE,

} CommErrorType_t;

typedef struct
{
    uint8_t  rx_buffer[UART_RX_BUFFER_SIZE];
    uint8_t  tx_buffer[UART_TX_BUFFER_SIZE];
    uint8_t  uart_rx_byte;
    volatile bool tx_wait;                             // set in SendResponse, cleared in the UART TX complete interrupt
    volatile bool frame_ready;                         // set in the UART RX interrupt, cleared in Comm_Run

} UART_Command_t;

void Comm_Init(void);
void Comm_Run(void);

#ifdef __cplusplus
}
#endif

#endif