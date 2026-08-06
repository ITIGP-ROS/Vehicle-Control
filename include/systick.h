/******************************************************************************
 *
 * Module: SysTick
 *
 * File Name: systick.h
 *
 * Description: Header file for TM4C123GH6PM SysTick driver
 *              Supports periodic interrupts, busy-wait delays, and scheduler timing
 *
 ******************************************************************************/

#ifndef SYSTICK_H_
#define SYSTICK_H_

#include "systick_types.h"
#include "systick_cfg.h"

/*******************************************************************************
 *                          Initialization Functions                           *
 *******************************************************************************/

/**
 * @brief  Initialize SysTick with period in milliseconds (interrupt mode)
 * @param  timeMs: Period in milliseconds (1 to SYSTICK_MAX_DELAY_MS)
 * @return SYSTICK_OK on success, error code otherwise
 * @note   Timer starts immediately after initialization
 */
SysTick_StatusType SysTick_Init(uint32 timeMs);

/**
 * @brief  Initialize SysTick with period in microseconds (interrupt mode)
 * @param  timeUs: Period in microseconds
 * @return SYSTICK_OK on success, error code otherwise
 * @note   Timer starts immediately after initialization
 */
SysTick_StatusType SysTick_InitUs(uint32 timeUs);

/**
 * @brief  Initialize SysTick with full configuration structure
 * @param  config: Pointer to configuration structure
 * @return SYSTICK_OK on success, error code otherwise
 */
SysTick_StatusType SysTick_InitWithConfig(const SysTick_ConfigType *config);

/**
 * @brief  De-initialize SysTick timer
 * @return SYSTICK_OK on success
 * @note   Stops timer, clears registers, disables interrupt
 */
SysTick_StatusType SysTick_DeInit(void);

/*******************************************************************************
 *                          Control Functions                                  *
 *******************************************************************************/

/**
 * @brief  Start SysTick timer
 * @return SYSTICK_OK on success, SYSTICK_ERROR_NOT_INITIALIZED if not init
 */
SysTick_StatusType SysTick_Start(void);

/**
 * @brief  Stop SysTick timer
 * @return SYSTICK_OK on success
 */
SysTick_StatusType SysTick_Stop(void);

/**
 * @brief  Reset SysTick counter to reload value
 * @return SYSTICK_OK on success
 */
SysTick_StatusType SysTick_Reset(void);

/*******************************************************************************
 *                          Callback Functions                                 *
 *******************************************************************************/

/**
 * @brief  Set callback function for SysTick interrupt
 * @param  callback: Pointer to callback function (NULL to disable)
 * @return SYSTICK_OK on success
 * @note   Callback is called from ISR context - keep it short!
 */
SysTick_StatusType SysTick_SetCallback(SysTick_CallbackType callback);

/*******************************************************************************
 *                          Delay Functions (Blocking)                         *
 *******************************************************************************/

/**
 * @brief  Blocking delay in milliseconds (busy-wait)
 * @param  timeMs: Delay time in milliseconds (1 to SYSTICK_MAX_DELAY_MS)
 * @return SYSTICK_OK on success, error code otherwise
 * @note   Uses polling mode, does not use interrupt
 * @note   For delays > SYSTICK_MAX_DELAY_MS, multiple iterations are used
 * @warning STALLS THE 1 ms TICK (T5-1). This repurposes SysTick as a one-shot
 *          polled timer: it clears CTRL, so the tick interrupt does NOT fire for
 *          the whole delay and SysTick_GetTickCount() LOSES every tick that
 *          should have elapsed (a 500 ms delay costs ~500 ticks of monotonic
 *          time). It also zeroes CURRENT, truncating the in-progress tick.
 *          => BENCH BRING-UP ONLY. Do NOT call this once anything depends on
 *          TickCount - in particular the production super-loop, whose CAN TX
 *          slots and 20 ms control gate are all derived from it. Use a
 *          SysTick_GetTickCount() delta instead (see test/bringup/main.c:58,
 *          which forbids these two functions outright).
 */
SysTick_StatusType SysTick_DelayMs(uint32 timeMs);

/**
 * @brief  Blocking delay in microseconds (busy-wait)
 * @param  timeUs: Delay time in microseconds
 * @return SYSTICK_OK on success, error code otherwise
 * @note   Uses polling mode, minimum resolution depends on clock speed
 * @warning STALLS THE 1 ms TICK (T5-1). This repurposes SysTick as a one-shot
 *          polled timer: it clears CTRL, so the tick interrupt does NOT fire for
 *          the whole delay and SysTick_GetTickCount() LOSES every tick that
 *          should have elapsed (a 500 ms delay costs ~500 ticks of monotonic
 *          time). It also zeroes CURRENT, truncating the in-progress tick.
 *          => BENCH BRING-UP ONLY. Do NOT call this once anything depends on
 *          TickCount - in particular the production super-loop, whose CAN TX
 *          slots and 20 ms control gate are all derived from it. Use a
 *          SysTick_GetTickCount() delta instead (see test/bringup/main.c:58,
 *          which forbids these two functions outright).
 */
SysTick_StatusType SysTick_DelayUs(uint32 timeUs);

/*******************************************************************************
 *                          Status Functions                                   *
 *******************************************************************************/

/**
 * @brief  Get current SysTick state
 * @return Current state (SYSTICK_STATE_UNINIT, SYSTICK_STATE_IDLE, SYSTICK_STATE_RUNNING)
 */
SysTick_StateType SysTick_GetState(void);

/**
 * @brief  Get current counter value
 * @return Current counter value (counts down from reload value)
 */
uint32 SysTick_GetCurrentValue(void);

/**
 * @brief  Get elapsed ticks since last reload
 * @return Number of ticks elapsed
 */
uint32 SysTick_GetElapsedTicks(void);

/**
 * @brief  Check if SysTick has elapsed (count flag set)
 * @return TRUE if elapsed, FALSE otherwise
 * @note   Reading this clears the count flag
 */
boolean SysTick_HasElapsed(void);

/**
 * @brief  Get system tick count (increments on each SysTick interrupt)
 * @return Total tick count since initialization
 */
uint32 SysTick_GetTickCount(void);

/*******************************************************************************
 *                          Period Configuration                               *
 *******************************************************************************/

/**
 * @brief  Set new period in milliseconds
 * @param  timeMs: New period in milliseconds
 * @return SYSTICK_OK on success, error code otherwise
 * @note   Temporarily stops timer during reconfiguration
 */
SysTick_StatusType SysTick_SetPeriodMs(uint32 timeMs);

/**
 * @brief  Set new period in microseconds
 * @param  timeUs: New period in microseconds
 * @return SYSTICK_OK on success, error code otherwise
 */
SysTick_StatusType SysTick_SetPeriodUs(uint32 timeUs);

#endif /* SYSTICK_H_ */
