/******************************************************************************
 *
 * Module: UART
 *
 * File Name: uart.h
 *
 * Description: Header file for TM4C123GH6PM UART driver
 *              Interrupt-driven with circular buffers and callbacks
 *
 ******************************************************************************/

#ifndef UART_H_
#define UART_H_

#include "uart_types.h"
#include "uart_cfg.h"

/*******************************************************************************
 *                          Initialization Functions                           *
 *******************************************************************************/

/**
 * @brief  Initialize all enabled UART modules based on configuration
 * @note   Must be called before any other UART function
 * @return UART_OK on success, error code otherwise
 */
UART_StatusType UART_Init(void);

/**
 * @brief  Initialize a specific UART module with runtime configuration
 * @param  uartId: UART module ID (UART_ID_0 to UART_ID_7)
 * @param  config: Pointer to configuration structure
 * @return UART_OK on success, error code otherwise
 */
UART_StatusType UART_InitWithConfig(UART_IdType uartId, const UART_ConfigType *config);

/**
 * @brief  Deinitialize a UART module
 * @param  uartId: UART module ID
 * @return UART_OK on success, error code otherwise
 */
UART_StatusType UART_DeInit(UART_IdType uartId);

/*******************************************************************************
 *                          Callback Registration                              *
 *******************************************************************************/

/**
 * @brief  Register callback functions for a UART module
 * @param  uartId: UART module ID
 * @param  callbacks: Pointer to callback configuration structure
 * @return UART_OK on success, error code otherwise
 */
UART_StatusType UART_RegisterCallbacks(UART_IdType uartId, const UART_CallbackConfigType *callbacks);

/**
 * @brief  Set RX callback only
 * @param  uartId: UART module ID
 * @param  callback: Callback function pointer (NULL to disable)
 * @return UART_OK on success
 */
UART_StatusType UART_SetRxCallback(UART_IdType uartId, UART_RxCallbackType callback);

/**
 * @brief  Set TX complete callback
 * @param  uartId: UART module ID
 * @param  callback: Callback function pointer (NULL to disable)
 * @return UART_OK on success
 */
UART_StatusType UART_SetTxCompleteCallback(UART_IdType uartId, UART_TxCompleteCallbackType callback);

/**
 * @brief  Set error callback
 * @param  uartId: UART module ID
 * @param  callback: Callback function pointer (NULL to disable)
 * @return UART_OK on success
 */
UART_StatusType UART_SetErrorCallback(UART_IdType uartId, UART_ErrorCallbackType callback);

/**
 * @brief  Set idle line callback (useful for variable length messages)
 * @param  uartId: UART module ID
 * @param  callback: Callback function pointer (NULL to disable)
 * @return UART_OK on success
 */
UART_StatusType UART_SetIdleCallback(UART_IdType uartId, UART_IdleCallbackType callback);

/*******************************************************************************
 *                          Transmit Functions                                 *
 *******************************************************************************/

/**
 * @brief  Queue a single byte for transmission (non-blocking)
 * @param  uartId: UART module ID
 * @param  data: Byte to transmit
 * @return UART_OK on success, UART_ERROR_BUFFER_FULL if TX buffer is full
 */
UART_StatusType UART_SendByte(UART_IdType uartId, uint8 data);

/**
 * @brief  Queue a buffer for transmission (non-blocking)
 * @param  uartId: UART module ID
 * @param  buffer: Pointer to data buffer
 * @param  length: Number of bytes to send
 * @return Number of bytes actually queued (may be less than length if buffer full)
 */
uint16 UART_SendBuffer(UART_IdType uartId, const uint8 *buffer, uint16 length);

/**
 * @brief  Queue a null-terminated string for transmission
 * @param  uartId: UART module ID
 * @param  str: Pointer to null-terminated string
 * @return Number of bytes queued
 */
uint16 UART_SendString(UART_IdType uartId, const char *str);

/**
 * @brief  Send an integer as ASCII string
 * @param  uartId: UART module ID
 * @param  number: Integer to send
 * @return Number of bytes queued
 */
uint16 UART_SendInteger(UART_IdType uartId, sint32 number);

/**
 * @brief  Send a float as ASCII string
 * @param  uartId: UART module ID
 * @param  number: Float to send
 * @param  decimals: Number of decimal places
 * @return Number of bytes queued
 */
uint16 UART_SendFloat(UART_IdType uartId, float32 number, uint8 decimals);

/*******************************************************************************
 *                          Receive Functions                                  *
 *******************************************************************************/

/**
 * @brief  Read a single byte from RX buffer (non-blocking)
 * @param  uartId: UART module ID
 * @param  data: Pointer to store received byte
 * @return UART_OK on success, UART_ERROR_BUFFER_EMPTY if no data available
 */
UART_StatusType UART_ReceiveByte(UART_IdType uartId, uint8 *data);

/**
 * @brief  Read multiple bytes from RX buffer (non-blocking)
 * @param  uartId: UART module ID
 * @param  buffer: Pointer to buffer to store data
 * @param  maxLength: Maximum bytes to read
 * @return Number of bytes actually read
 */
uint16 UART_ReceiveBuffer(UART_IdType uartId, uint8 *buffer, uint16 maxLength);

/**
 * @brief  Peek at next byte in RX buffer without removing it
 * @param  uartId: UART module ID
 * @param  data: Pointer to store peeked byte
 * @return UART_OK on success, UART_ERROR_BUFFER_EMPTY if no data
 */
UART_StatusType UART_PeekByte(UART_IdType uartId, uint8 *data);

/**
 * @brief  Read bytes until delimiter found or buffer full
 * @param  uartId: UART module ID
 * @param  buffer: Pointer to buffer to store data
 * @param  maxLength: Maximum bytes to read
 * @param  delimiter: Delimiter character (e.g., '\n')
 * @param  includeDelimiter: If TRUE, include delimiter in output
 * @return Number of bytes read (0 if delimiter not found)
 */
uint16 UART_ReadUntilDelimiter(UART_IdType uartId, uint8 *buffer, uint16 maxLength, 
                               uint8 delimiter, boolean includeDelimiter);

/*******************************************************************************
 *                          Status Functions                                   *
 *******************************************************************************/

/**
 * @brief  Check if data is available in RX buffer
 * @param  uartId: UART module ID
 * @return TRUE if data available, FALSE otherwise
 */
boolean UART_IsDataAvailable(UART_IdType uartId);

/**
 * @brief  Get number of bytes available in RX buffer
 * @param  uartId: UART module ID
 * @return Number of bytes available
 */
uint16 UART_GetRxCount(UART_IdType uartId);

/**
 * @brief  Get free space in TX buffer
 * @param  uartId: UART module ID
 * @return Number of bytes free in TX buffer
 */
uint16 UART_GetTxFreeSpace(UART_IdType uartId);

/**
 * @brief  Get the number of TX bytes dropped by the bounded back-pressure spin
 *
 * U23-2: UART_SendBuffer never blocks indefinitely. If the software TX buffer
 * AND the hardware FIFO stay full for UART_PRIV_TXFF_SPIN_CAP iterations, the
 * byte is discarded and this counter increments, so the loss is observable
 * rather than silent. Mirrors Can_GetIfTimeoutCount(). Expected value under
 * normal load is 0 — a non-zero reading means the producer is outrunning the
 * baud rate, not that the driver is broken.
 *
 * @param  uartId: UART module ID
 * @return Cumulative dropped-byte count (0 for an invalid uartId)
 */
uint32 UART_GetTxDroppedCount(UART_IdType uartId);

/**
 * @brief  Check if TX is complete (buffer empty and shift register empty)
 * @param  uartId: UART module ID
 * @return TRUE if transmission complete
 */
boolean UART_IsTxComplete(UART_IdType uartId);

/**
 * @brief  Get current UART state
 * @param  uartId: UART module ID
 * @return Current state (UART_STATE_UNINIT, UART_STATE_IDLE, etc.)
 */
UART_StateType UART_GetState(UART_IdType uartId);

/**
 * @brief  Get last error flags
 * @param  uartId: UART module ID
 * @return Error flags structure
 */
UART_ErrorFlagsType UART_GetLastError(UART_IdType uartId);

/**
 * @brief  Clear last error flags
 * @param  uartId: UART module ID
 */
void UART_ClearErrors(UART_IdType uartId);

/*******************************************************************************
 *                          Buffer Management                                  *
 *******************************************************************************/

/**
 * @brief  Flush (clear) RX buffer
 * @param  uartId: UART module ID
 */
void UART_FlushRxBuffer(UART_IdType uartId);

/**
 * @brief  Flush (clear) TX buffer
 * @param  uartId: UART module ID
 * @note   Data currently being transmitted will complete
 */
void UART_FlushTxBuffer(UART_IdType uartId);

/*******************************************************************************
 *                          Control Functions                                  *
 *******************************************************************************/

/**
 * @brief  Enable/disable UART module
 * @param  uartId: UART module ID
 * @param  enable: TRUE to enable, FALSE to disable
 * @return UART_OK on success
 */
UART_StatusType UART_SetEnable(UART_IdType uartId, boolean enable);

/**
 * @brief  Change baud rate at runtime
 * @param  uartId: UART module ID
 * @param  baudRate: New baud rate
 * @return UART_OK on success
 * @note   Temporarily disables UART during change
 */
UART_StatusType UART_SetBaudRate(UART_IdType uartId, uint32 baudRate);

#endif /* UART_H_ */
