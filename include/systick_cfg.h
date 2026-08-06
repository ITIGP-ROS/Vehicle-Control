/******************************************************************************
 *
 * Module: SysTick
 *
 * File Name: systick_cfg.h
 *
 * Description: Configuration header for TM4C123GH6PM SysTick driver
 *
 ******************************************************************************/

#ifndef SYSTICK_CFG_H_
#define SYSTICK_CFG_H_

/*******************************************************************************
 *                          System Configuration                               *
 *******************************************************************************/

/* System clock frequency in Hz */
/* Change to 80000000UL if using PLL at 80MHz */
#define SYSTICK_SYSTEM_CLOCK_HZ         (16000000UL)

/*******************************************************************************
 *                          SysTick Configuration                              *
 *******************************************************************************/

/* Default tick period in microseconds for scheduler/timing base */
/* Set to 1000 for 1ms tick (typical for RTOS/scheduler) */
#define SYSTICK_DEFAULT_TICK_US         (1000UL)

/* Enable/disable interrupt mode by default */
/* 1 = Interrupt enabled, 0 = Polling mode */
#define SYSTICK_INTERRUPT_ENABLE        (1U)

/* Clock source selection */
/* 1 = System clock, 0 = PIOSC/4 (4MHz) */
#define SYSTICK_USE_SYSTEM_CLOCK        (1U)

/*******************************************************************************
 *                          Derived Constants (Do Not Modify)                  *
 *******************************************************************************/

/* Maximum reload value (24-bit counter) */
#define SYSTICK_MAX_RELOAD_VALUE        (0x00FFFFFFUL)

/* Ticks per millisecond */
#define SYSTICK_TICKS_PER_MS            (SYSTICK_SYSTEM_CLOCK_HZ / 1000UL)

/* Ticks per microsecond */
#define SYSTICK_TICKS_PER_US            (SYSTICK_SYSTEM_CLOCK_HZ / 1000000UL)

/* Maximum delay in milliseconds (to avoid overflow) */
/* For 16MHz: (0xFFFFFF + 1) / 16000 = 1048.576 ms */
#define SYSTICK_MAX_DELAY_MS            ((SYSTICK_MAX_RELOAD_VALUE + 1UL) / SYSTICK_TICKS_PER_MS)

#endif /* SYSTICK_CFG_H_ */
