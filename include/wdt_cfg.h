/******************************************************************************
 *
 * Module: Watchdog Timer (WDT)
 *
 * File Name: wdt_cfg.h
 *
 * Description: Configuration file for TM4C123 Hardware Watchdog Timer
 *
 ******************************************************************************/

#ifndef WDT_CFG_H_
#define WDT_CFG_H_

/*******************************************************************************
 *                          Watchdog Configuration                             *
 *******************************************************************************/

/**
 * @brief System clock frequency (used for timeout calculations)
 * @note  Default TM4C123 system clock is 16 MHz
 * @note  Update this if you configure PLL for higher frequency (up to 80 MHz)
 */
#define WDT_CFG_SYSTEM_CLOCK_HZ     16000000u   /* 16 MHz */

/**
 * @brief Default watchdog timeout for robot safety (in milliseconds)
 * 
 * Recommended for robot control:
 *   - 2000ms (2 seconds): Safe margin for system-level watchdog
 *   - Longer than software watchdog (200ms for comm)
 *   - Catches complete system hangs
 */
#define WDT_CFG_DEFAULT_TIMEOUT_MS  2000u       /* 2 seconds */

/**
 * @brief Enable watchdog locking after initialization
 * 
 * If TRUE: Watchdog configuration is locked after init (recommended for production)
 * If FALSE: Watchdog can be reconfigured at runtime (useful for debugging)
 */
#define WDT_CFG_ENABLE_LOCK         TRUE

/**
 * @brief Enable stall on debug halt
 * 
 * If TRUE: Watchdog stops counting when debugger halts CPU
 * If FALSE: Watchdog continues even when debugged
 * 
 * Recommended: TRUE for development, FALSE for production
 */
#define WDT_CFG_ENABLE_DEBUG_STALL  TRUE

/**
 * @brief Default watchdog mode
 * 
 * WDT_MODE_RESET: System reset on timeout (recommended for production)
 * WDT_MODE_INTERRUPT: Call interrupt handler first, then reset
 */
#define WDT_CFG_DEFAULT_MODE        WDT_MODE_RESET

/**
 * @brief Which watchdog timer to use
 * 
 * WDT_ID_0: Watchdog Timer 0
 * WDT_ID_1: Watchdog Timer 1
 * 
 * Recommendation: Use WDT0 for system watchdog
 */
#define WDT_CFG_TIMER_ID            WDT_ID_0

/**
 * @brief Interrupt priority (if using interrupt mode)
 * 
 * Priority levels: 0-7 (0 = highest priority)
 * Recommended: 0 or 1 (high priority for safety)
 */
#define WDT_CFG_INT_PRIORITY        0u

#endif /* WDT_CFG_H_ */
