/******************************************************************************
 *
 * Module: UART
 *
 * File Name: uart_types.h
 *
 * Description: Type definitions for TM4C123GH6PM UART driver
 *
 ******************************************************************************/

#ifndef UART_TYPES_H_
#define UART_TYPES_H_

#include "Std_Types.h"

/*******************************************************************************
 *                          UART Identification                                *
 *******************************************************************************/

typedef enum {
    UART_ID_0 = 0,
    UART_ID_1 = 1,
    UART_ID_2 = 2,
    UART_ID_3 = 3,
    UART_ID_4 = 4,
    UART_ID_5 = 5,
    UART_ID_6 = 6,
    UART_ID_7 = 7,
    UART_ID_MAX
} UART_IdType;

/*******************************************************************************
 *                          UART Configuration Types                           *
 *******************************************************************************/

typedef enum {
    UART_WORD_LENGTH_5 = 0x00,
    UART_WORD_LENGTH_6 = 0x01,
    UART_WORD_LENGTH_7 = 0x02,
    UART_WORD_LENGTH_8 = 0x03
} UART_WordLengthType;

typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_ODD  = 1,
    UART_PARITY_EVEN = 2
} UART_ParityType;

typedef enum {
    UART_STOP_BITS_1 = 0,
    UART_STOP_BITS_2 = 1
} UART_StopBitsType;

/*******************************************************************************
 *                          UART Status Types                                  *
 *******************************************************************************/

typedef enum {
    UART_OK = 0,
    UART_ERROR_INVALID_ID,
    UART_ERROR_NOT_INITIALIZED,
    UART_ERROR_BUFFER_FULL,
    UART_ERROR_BUFFER_EMPTY,
    UART_ERROR_FRAMING,
    UART_ERROR_PARITY,
    UART_ERROR_OVERRUN,
    UART_ERROR_BREAK,
    UART_ERROR_TIMEOUT,
    UART_ERROR_NULL_PTR
} UART_StatusType;

typedef enum {
    UART_STATE_UNINIT = 0,
    UART_STATE_IDLE,
    UART_STATE_BUSY_TX,
    UART_STATE_BUSY_RX,
    UART_STATE_BUSY_TX_RX
} UART_StateType;

/*******************************************************************************
 *                          UART Error Flags                                   *
 *******************************************************************************/

typedef struct {
    uint8 framingError  : 1;
    uint8 parityError   : 1;
    uint8 overrunError  : 1;
    uint8 breakError    : 1;
    uint8 reserved      : 4;
} UART_ErrorFlagsType;

/*******************************************************************************
 *                          Callback Function Types                            *
 *******************************************************************************/

/* Callback for RX complete - called when data received (from ISR context) */
typedef void (*UART_RxCallbackType)(UART_IdType uartId, uint8 data);

/* Callback for TX complete - called when TX buffer becomes empty */
typedef void (*UART_TxCompleteCallbackType)(UART_IdType uartId);

/* Callback for RX buffer threshold - called when RX buffer reaches threshold */
typedef void (*UART_RxBufferCallbackType)(UART_IdType uartId, uint16 bytesAvailable);

/* Callback for errors */
typedef void (*UART_ErrorCallbackType)(UART_IdType uartId, UART_ErrorFlagsType errors);

/* Callback for idle line detection - useful for variable length messages */
typedef void (*UART_IdleCallbackType)(UART_IdType uartId);

/*******************************************************************************
 *                          Callback Configuration                             *
 *******************************************************************************/

typedef struct {
    UART_RxCallbackType         rxCallback;          /* Called for each byte received */
    UART_TxCompleteCallbackType txCompleteCallback;  /* Called when TX completes */
    UART_RxBufferCallbackType   rxBufferCallback;    /* Called at RX threshold */
    UART_ErrorCallbackType      errorCallback;       /* Called on errors */
    UART_IdleCallbackType       idleCallback;        /* Called on idle line */
    uint16                      rxBufferThreshold;   /* Threshold for rxBufferCallback */
} UART_CallbackConfigType;

/*******************************************************************************
 *                          Runtime Configuration                              *
 *******************************************************************************/

typedef struct {
    uint32              baudRate;
    UART_WordLengthType wordLength;
    UART_ParityType     parity;
    UART_StopBitsType   stopBits;
    boolean             fifoEnable;
    boolean             rxIntEnable;
    boolean             txIntEnable;
    boolean             rxTimeoutIntEnable;
    boolean             errorIntEnable;
    uint8               intPriority;
} UART_ConfigType;

/*******************************************************************************
 *                          Circular Buffer Type                               *
 *******************************************************************************/

typedef struct {
    uint8  *buffer;
    uint16  size;
    volatile uint16  head;
    volatile uint16  tail;
    volatile uint16  count;
} UART_CircularBufferType;

/*******************************************************************************
 *                          UART Handle (Internal)                             *
 *******************************************************************************/

typedef struct {
    uint32                    baseAddr;
    UART_StateType            state;
    UART_ErrorFlagsType       lastError;
    UART_CircularBufferType   rxBuffer;
    UART_CircularBufferType   txBuffer;
    UART_CallbackConfigType   callbacks;
    boolean                   initialized;
} UART_HandleType;

#endif /* UART_TYPES_H_ */
