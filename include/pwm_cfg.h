/******************************************************************************
 *
 * Module: PWM
 *
 * File Name: pwm_cfg.h
 *
 * Description: Configuration header for TM4C123GH6PM PWM driver
 *
 ******************************************************************************/

#ifndef PWM_CFG_H_
#define PWM_CFG_H_

/*******************************************************************************
 *                          System Configuration                               *
 *******************************************************************************/

/* System clock frequency in Hz - must match the real core clock.
 * NOTE: this is an independent copy of the same 16MHz assumption already
 * duplicated in systick_cfg.h, uart_cfg.h, qei_cfg.h and wdt_cfg.h (this is
 * the 5th copy). Unifying all of these into one source of truth is a known,
 * deferred issue - see PWM_REDESIGN.md - not addressed by this driver alone. */
#define PWM_CFG_SYSTEM_CLOCK_HZ         (16000000UL)

/* Target PWM switching frequency in Hz.
 * 20kHz is above the human hearing range, unlike the previous ~1.95kHz
 * setting (PWM_RESOLUTION=4095), which could produce an audible motor whine. */
#define PWM_CFG_FREQUENCY_HZ            (20000UL)

/*******************************************************************************
 *                          Derived Constants (Do Not Modify)                  *
 *******************************************************************************/

/* Up/down count mode: the counter covers 0 -> LOAD -> 0, i.e. 2*LOAD clock
 * cycles per period, so LOAD = PWM_clock / (2 * desired_frequency). */
#define PWM_PERIOD_LOAD                 (PWM_CFG_SYSTEM_CLOCK_HZ / (2UL * PWM_CFG_FREQUENCY_HZ))

/* PWMnLOAD is a 16-bit register (max 65535) - catch an out-of-range
 * frequency/clock combination at compile time rather than at runtime. */
#if (PWM_PERIOD_LOAD > 65535UL)
#error "PWM_PERIOD_LOAD exceeds the 16-bit PWM LOAD register width - raise PWM_CFG_FREQUENCY_HZ or check PWM_CFG_SYSTEM_CLOCK_HZ"
#endif

#endif /* PWM_CFG_H_ */
