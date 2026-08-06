/******************************************************************************
 *
 * Module: Watchdog Timer (WDT)
 *
 * File Name: wdt_types.h
 *
 * Description: Type definitions for TM4C123GH6PM Watchdog Timer driver
 *
 ******************************************************************************/

#ifndef WDT_TYPES_H_
#define WDT_TYPES_H_

#include "Std_Types.h"

/*******************************************************************************
 *                          Type Definitions                                   *
 *******************************************************************************/

/**
 * @brief Watchdog timer module identifiers
 */
typedef enum {
    WDT_ID_0 = 0,       /* Watchdog Timer 0 */
    WDT_ID_1 = 1,       /* Watchdog Timer 1 */
    WDT_ID_MAX
} WDT_IdType;

/**
 * @brief Watchdog timer status codes
 */
typedef enum {
    WDT_OK = 0,
    WDT_ERROR_INVALID_ID,
    WDT_ERROR_NOT_INITIALIZED,
    WDT_ERROR_INVALID_TIMEOUT,
    WDT_ERROR_NULL_PTR,
    WDT_ERROR_LOCKED
} WDT_StatusType;

/**
 * @brief Watchdog timer mode
 */
typedef enum {
    WDT_MODE_INTERRUPT = 0,     /* Generate interrupt on timeout */
    WDT_MODE_RESET = 1          /* Generate system reset on timeout */
} WDT_ModeType;

/**
 * @brief Watchdog timer interrupt type
 */
typedef enum {
    WDT_INT_STANDARD = 0,       /* Standard interrupt */
    WDT_INT_NMI = 1             /* Non-maskable interrupt */
} WDT_IntTypeType;

/**
 * @brief Watchdog timer state
 */
typedef enum {
    WDT_STATE_UNINIT = 0,       /* Not initialized */
    WDT_STATE_STOPPED,          /* Initialized but not running */
    WDT_STATE_RUNNING           /* Running and counting down */
} WDT_StateType;

/**
 * @brief Watchdog timeout callback function type
 * @param wdtId: Watchdog timer that timed out
 */
typedef void (*WDT_TimeoutCallbackType)(WDT_IdType wdtId);

/**
 * @brief Watchdog timer configuration structure
 */
typedef struct {
    uint32 timeoutMs;                       /* Timeout period in milliseconds */
    WDT_ModeType mode;                      /* Interrupt or reset mode */
    WDT_IntTypeType intType;                /* Standard interrupt or NMI */
    WDT_TimeoutCallbackType callback;       /* Callback function (only for interrupt mode) */
    boolean enableStallDebug;               /* Stall timer when debugger halts CPU */
} WDT_ConfigType;

#endif /* WDT_TYPES_H_ */
