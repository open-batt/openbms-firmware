#ifndef OPENBMS_COMM_H
#define OPENBMS_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// Largest frame: command, address, length, up to 255 payload bytes (length is a uint8_t), checksum
#define COMM_FRAME_MAX_SIZE                 (3U + 255U + 1U)
#define COMM_RX_BUFFER_SIZE                 COMM_FRAME_MAX_SIZE
#define COMM_TX_BUFFER_SIZE                 COMM_FRAME_MAX_SIZE

// I2C slave: bytes the host reads when there is no reply (yet), or past the end of the reply
#define COMM_I2C_NOT_READY                  (0x00U)
#define COMM_I2C_FILL                       (0xFFU)

// CAN: one message carries at most 8 bytes, the status broadcast is 5 messages plus one reply
#define COMM_CAN_DATA_SIZE                  (8U)
#define COMM_CAN_TX_QUEUE_SIZE              (8U)
#define COMM_CAN_STD_ID_MASK                (0x7FFU)        // 11-bit standard ID
#define COMM_CAN_FILTER_STD_SHIFT           (5U)            // a standard ID sits in bits 15:5 of the 16-bit filter halves
#define COMM_CAN_CELLS_PER_MESSAGE          (4U)            // uint16 cell voltages in one message
#define COMM_CAN_ERROR_BYTES                (3U)            // 0x6B4 carries the low 24 error bits
#define COMM_CAN_ERROR_MASK                 (0xFFFFFFULL)
#define COMM_CAN_SOC_SCALE                  (1000.0f)       // SOC 0..1 -> 0.1 %
#define COMM_CAN_TEMP_SCALE                 (10.0f)         // degC -> 0.1 degC

// Time for a reply to leave on every port before a reboot or the bootloader jump
#define COMM_REBOOT_DELAY_MS                (100U)
#define COMM_UART_FRAME_TIMEOUT_MS          (50U)           // a frame that comes in pieces must be complete within this time

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
    CC_CLEAR_LEARNING   = 0x0F,
    CC_CAN_BROADCAST    = 0x10,     // CAN only: one argument, 0 - broadcast off, 1 - on (RAM only)
    CC_CAN_SEND_ONCE    = 0x11,     // CAN only: send the status messages once
    CC_CLEAR_FAULT      = 0x12,     // clears the fault snapshot (0x70 - 0x75)
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
    uint8_t  rx_buffer[COMM_RX_BUFFER_SIZE];
    uint8_t  tx_buffer[COMM_TX_BUFFER_SIZE];
    uint8_t  uart_rx_byte;
    volatile bool tx_wait;                             // set in Comm_SendResponse, cleared in the UART TX complete interrupt
    volatile bool frame_ready;                         // set in the UART RX interrupt, cleared in Comm_Run
    volatile uint16_t rx_count;                        // bytes of a frame that came in pieces so far, 0 = waiting for a new frame
    volatile uint32_t rx_start_ms;                     // HAL_GetTick() at the first piece

} UART_Command_t;

typedef struct
{
    uint8_t  rx_buffer[COMM_RX_BUFFER_SIZE];
    uint8_t  tx_buffer[COMM_TX_BUFFER_SIZE];
    uint8_t  rx_discard;                               // bytes that do not fit in rx_buffer, or arrive while a frame is processed
    uint8_t  tx_fill;                                  // byte sent when no reply is ready, or past the end of the reply
    volatile uint16_t rx_count;                        // bytes received in the current write
    volatile uint16_t tx_length;                       // reply length, 0 = no reply ready (yet)
    volatile uint16_t tx_index;                        // next reply byte to send, every read starts at 0
    volatile uint8_t  direction;                       // I2C_DIRECTION_TRANSMIT (host writes) or I2C_DIRECTION_RECEIVE (host reads)
    volatile bool frame_ready;                         // set at the STOP of a complete write, cleared in Comm_Run
    volatile bool transfer_active;                     // set at the address match, cleared at the STOP
    volatile uint32_t transfer_start_ms;               // HAL_GetTick() at the address match
    volatile bool restart;                             // set on a bus error, the peripheral is restarted in Comm_Run

} I2C_Command_t;

typedef struct
{
    uint32_t id;
    uint8_t  length;
    uint8_t  data[COMM_CAN_DATA_SIZE];

} CAN_Message_t;

typedef struct
{
    CAN_Message_t tx_queue[COMM_CAN_TX_QUEUE_SIZE];    // messages waiting for a free TX mailbox
    volatile uint8_t tx_head;                          // next free slot, written in the main loop
    volatile uint8_t tx_tail;                          // next message to send, written where the mailboxes are filled
    uint8_t  rx_data[COMM_CAN_DATA_SIZE];              // last command message
    volatile uint8_t rx_length;
    volatile bool command_ready;                       // set in the CAN RX interrupt, cleared in Comm_Run
    volatile bool restart;                             // set on a CAN error, the CAN is restarted in Comm_Run
    bool     broadcast_on;                             // RAM only, COMM_CAN_BROADCAST_DEFAULT after boot
    bool     send_once;                                // CC_CAN_SEND_ONCE: send the status messages at the next Comm_Run
    uint32_t broadcast_last_ms;
    uint32_t restart_last_ms;                          // HAL_GetTick() of the last restart attempt

} CAN_Command_t;

typedef enum
{
    COMM_PORT_UART,
    COMM_PORT_I2C,

} CommPort_t;

void Comm_Init(void);
void Comm_Run(void);

#ifdef __cplusplus
}
#endif

#endif