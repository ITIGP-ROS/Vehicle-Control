/******************************************************************************
 *
 * Module: SysTick
 *
 * File Name: systick_types.h
 *
 * Description: Type definitions for TM4C123GH6PM SysTick driver
 *
 ******************************************************************************/

#ifndef SYSTICK_TYPES_H_
#define SYSTICK_TYPES_H_

#include "Std_Types.h"

/*******************************************************************************
 *                          Type Definitions                                   *
 *******************************************************************************/

/* SysTick status return type */
typedef enum {
    SYSTICK_OK = 0,
    SYSTICK_ERROR_INVALID_TIME,
    SYSTICK_ERROR_OVERFLOW,
    SYSTICK_ERROR_NOT_INITIALIZED,
    SYSTICK_ERROR_NULL_PTR
} SysTick_StatusType;

/* SysTick state type */
typedef enum {
    SYSTICK_STATE_UNINIT = 0,
    SYSTICK_STATE_IDLE,
    SYSTICK_STATE_RUNNING
} SysTick_StateType;

/* SysTick clock source type */
typedef enum {
    SYSTICK_CLOCK_PIOSC_DIV4 = 0,   /* Precision Internal Oscillator / 4 (4MHz) */
    SYSTICK_CLOCK_SYSTEM = 1         /* System Clock */
} SysTick_ClockSourceType;

/* SysTick mode type */
typedef enum {
    SYSTICK_MODE_POLLING = 0,        /* No interrupt, polling mode */
    SYSTICK_MODE_INTERRUPT = 1       /* Interrupt enabled */
} SysTick_ModeType;

/* Callback function type */
typedef void (*SysTick_CallbackType)(void);

/*******************************************************************************
 *                          Configuration Structure                            *
 *******************************************************************************/

typedef struct {
    uint32                  periodUs;       /* Period in microseconds */
    SysTick_ClockSourceType clockSource;    /* Clock source selection */
    SysTick_ModeType        mode;           /* Polling or interrupt mode */
    SysTick_CallbackType    callback;       /* Callback function (can be NULL) */
} SysTick_ConfigType;

#endif /* SYSTICK_TYPES_H_ */
