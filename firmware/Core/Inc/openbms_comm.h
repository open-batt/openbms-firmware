#ifndef OPENBMS_COMM_H
#define OPENBMS_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define UART_RX_BUFFER_SIZE                 2048U
#define UART_TX_BUFFER_SIZE                 2048U

typedef enum
{
    CC_WRITE       = 0x01,
    CC_READ        = 0x02,
    CC_ACK         = 0x03,
    CC_ERROR       = 0x04,
    CC_CMD         = 0x05,
    CC_BOOTLOADER  = 0x42,

} CommCmdType_t;

typedef enum
{
    CE_OK,
    CE_WRONG_CMD,
    CE_BAD_CRC,
    CE_NO_REG,
    CE_RO,

} CommErrorType_t;

typedef struct
{
    uint8_t  rx_buffer[UART_RX_BUFFER_SIZE];
    uint8_t  tx_buffer[UART_TX_BUFFER_SIZE];
    uint16_t rx_length;
    uint8_t  uart_rx_byte;
    bool     frame_ready;

} UART_Command_t;

void Comm_Init(void);
void Comm_Run(void);

#endif