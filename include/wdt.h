/******************************************************************************
 *
 * Module: Watchdog Timer (WDT)
 *
 * File Name: wdt.h
 *
 * Description: Header file for TM4C123GH6PM Hardware Watchdog Timer driver
 *              Uses the TM4C123 Watchdog Timer peripheral (WDT0 or WDT1)
 *
 * Features:
 *   - System reset on timeout (safety mechanism)
 *   - Interrupt mode with callback (for graceful handling)
 *   - Configurable timeout periods
 *   - Lock mechanism to prevent accidental reconfiguration
 *   - Debug stall support
 *
 ******************************************************************************/

#ifndef WDT_H_
#define WDT_H_

#include "wdt_types.h"
#include "wdt_regs.h"

/*******************************************************************************
 *                          Initialization Functions                           *
 *******************************************************************************/

/**
 * @brief  Initialize the watchdog timer module
 * @param  wdtId: Watchdog timer module ID (WDT_ID_0 or WDT_ID_1)
 * @param  config: Pointer to configuration structure
 * @return WDT_OK on success, error code otherwise
 * @note   Must be called before any other WDT function
 * @note   Enables clock to the watchdog peripheral
 */
WDT_StatusType WDT_Init(WDT_IdType wdtId, const WDT_ConfigType *config);

/**
 * @brief  Deinitialize the watchdog timer
 * @param  wdtId: Watchdog timer module ID
 * @return WDT_OK on success, error code otherwise
 */
WDT_StatusType WDT_DeInit(WDT_IdType wdtId);

/*******************************************************************************
 *                          Control Functions                                  *
 *******************************************************************************/

/**
 * @brief  Start the watchdog timer (begin countdown)
 * @param  wdtId: Watchdog timer module ID
 * @return WDT_OK on success, error code otherwise
 * @note   After calling this, you must call WDT_Kick() periodically
 */
WDT_StatusType WDT_Start(WDT_IdType wdtId);

/**
 * @brief  Stop the watchdog timer
 * @param  wdtId: Watchdog timer module ID
 * @return WDT_OK on success, error code otherwise
 * @note   Can only be called if watchdog is unlocked
 */
WDT_StatusType WDT_Stop(WDT_IdType wdtId);

/**
 * @brief  Kick (feed) the watchdog timer to prevent timeout
 * @param  wdtId: Watchdog timer module ID
 * @return WDT_OK on success, error code otherwise
 * @note   This reloads the counter and prevents timeout
 * @note   Must be called periodically when watchdog is running
 */
WDT_StatusType WDT_Kick(WDT_IdType wdtId);

/*******************************************************************************
 *                          Configuration Functions                            *
 *******************************************************************************/

/**
 * @brief  Lock the watchdog configuration to prevent modification
 * @param  wdtId: Watchdog timer module ID
 * @return WDT_OK on success, error code otherwise
 * @note   Once locked, registers can only be written by reset
 * @note   Provides protection against runaway code
 */
WDT_StatusType WDT_Lock(WDT_IdType wdtId);

/**
 * @brief  Unlock the watchdog configuration
 * @param  wdtId: Watchdog timer module ID
 * @return WDT_OK on success, error code otherwise
 * @note   Required before modifying configuration
 */
WDT_StatusType WDT_Unlock(WDT_IdType wdtId);

/**
 * @brief  Check if watchdog is locked
 * @param  wdtId: Watchdog timer module ID
 * @return TRUE if locked, FALSE otherwise
 */
boolean WDT_IsLocked(WDT_IdType wdtId);

/**
 * @brief  Change watchdog timeout period
 * @param  wdtId: Watchdog timer module ID
 * @param  timeoutMs: New timeout period in milliseconds
 * @return WDT_OK on success, error code otherwise
 * @note   Watchdog must be unlocked to change timeout
 */
WDT_StatusType WDT_SetTimeout(WDT_IdType wdtId, uint32 timeoutMs);

/*******************************************************************************
 *                          Status Functions                                   *
 *******************************************************************************/

/**
 * @brief  Get current watchdog state
 * @param  wdtId: Watchdog timer module ID
 * @return Current state
 */
WDT_StateType WDT_GetState(WDT_IdType wdtId);

/**
 * @brief  Get current counter value
 * @param  wdtId: Watchdog timer module ID
 * @param  value: Pointer to store counter value
 * @return WDT_OK on success, error code otherwise
 * @note   Counter counts down from load value to zero
 */
WDT_StatusType WDT_GetCurrentValue(WDT_IdType wdtId, uint32 *value);

/**
 * @brief  Get time remaining until timeout
 * @param  wdtId: Watchdog timer module ID
 * @param  timeRemainingMs: Pointer to store time remaining in milliseconds
 * @return WDT_OK on success, error code otherwise
 */
WDT_StatusType WDT_GetTimeRemaining(WDT_IdType wdtId, uint32 *timeRemainingMs);

/**
 * @brief  Check if watchdog interrupt is pending
 * @param  wdtId: Watchdog timer module ID
 * @return TRUE if interrupt is pending, FALSE otherwise
 */
boolean WDT_IsInterruptPending(WDT_IdType wdtId);

/**
 * @brief  Clear watchdog interrupt
 * @param  wdtId: Watchdog timer module ID
 * @return WDT_OK on success, error code otherwise
 * @note   Must be called in interrupt handler to clear the interrupt
 */
WDT_StatusType WDT_ClearInterrupt(WDT_IdType wdtId);

/*******************************************************************************
 *                          Interrupt Handler                                  *
 *******************************************************************************/

/**
 * @brief  Watchdog 0 interrupt handler
 * @note   Call this from the WDT0 ISR in your startup code
 */
void WDT0_IntHandler(void);

/**
 * @brief  Watchdog 1 interrupt handler
 * @note   Call this from the WDT1 ISR in your startup code
 */
void WDT1_IntHandler(void);

#endif /* WDT_H_ */
