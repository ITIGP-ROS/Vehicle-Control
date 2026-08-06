/******************************************************************************
 *
 * Module: TimerPWM (MCAL)
 *
 * File Name: timer_pwm_cfg.h
 *
 * Description: Compile-time configuration for the generic WTIMER0 Timer-PWM
 *              driver. Holds HARDWARE FACTS ONLY: which timer/pin, the system
 *              clock, the period (TAILR), ticks-per-microsecond, and the
 *              generic valid pulse-width window the driver clamps to.
 *
 *              This file is servo-agnostic. It deliberately does NOT define a
 *              "center" pulse or an angle range - those are servo-domain and
 *              live in servo_cfg.h. (Period/clock/derived math moved verbatim
 *              from the retired servo_pwm_cfg.h; values are unchanged.)
 *
 ******************************************************************************/

#ifndef TIMER_PWM_CFG_H_
#define TIMER_PWM_CFG_H_

/*******************************************************************************
 *                          Hardware Selection                                 *
 *  WTIMER0, TimerA -> WT0CCP0 = PC4. The pin is muxed to its CCP alternate    *
 *  function by the PORT driver (see PORT_PBCFG.c); this driver owns only the  *
 *  timer registers. These are documentation facts (the register addresses     *
 *  themselves live in tm4c123gh6pm_registers.h as WTIMER0_*).                  *
 *******************************************************************************/

/* (informational) the CCP output pin this timer drives: WT0CCP0 = PC4. */

/*******************************************************************************
 *                          System Configuration                               *
 *******************************************************************************/

/* Core clock in Hz - must match the real 16MHz (no PLL) system clock. This is
 * yet another copy of the same constant already in pwm_cfg.h / systick_cfg.h /
 * qei_cfg.h / uart_cfg.h; unifying them is a known, deferred repo-wide issue. */
#define TIMER_PWM_SYSTEM_CLOCK_HZ       (16000000UL)

/* PWM frame frequency: 50Hz (20ms) - the standard hobby-servo frame. (Kept in
 * the MCAL because the period is a timer fact; the servo SEMANTICS of why 50Hz
 * are documented in servo_cfg.h.) */
#define TIMER_PWM_FREQUENCY_HZ          (50UL)

/*******************************************************************************
 *                          Pulse-Width Clamp Window (margined mech. limits)   *
 *  The driver clamps every requested pulse into [MIN, MAX]: the last-line     *
 *  hard bound so the steering can NEVER be driven past its usable travel,     *
 *  regardless of upstream error. RE-MEASURED 2026-07-30 after the linkage     *
 *  rework (HWTEST 03). Both bounds are SERVO-authority limits, not linkage    *
 *  stops: the servo buzzes/stalls below ~400us and saturates at ~2500us       *
 *  (verified unresponsive out to 3000us). The servo-domain meaning lives in   *
 *  servo_cfg.h, which MUST agree with these bounds - the cross-check guard    *
 *  at the bottom of this file enforces that.                                  *
 *******************************************************************************/
#define TIMER_PWM_MIN_PULSE_US          (600U)    /* full LEFT  - servo-authority edge  */
#define TIMER_PWM_MAX_PULSE_US          (2500U)   /* full RIGHT - servo saturation      */

/* Neutral startup pulse loaded before the output is enabled, so the first PWM
 * period is well-defined. 1450us is the MEASURED wheels-straight point (front
 * wheels parallel, re-measured 2026-07-30), NOT the geometric clamp midpoint -
 * that would be 1550us. The servo self-centers to the real straight-ahead pulse
 * at boot. The servo HAL re-asserts center on top (servo.c). */
#define TIMER_PWM_INIT_PULSE_US         (1450U)

/*******************************************************************************
 *                          Derived Constants (Do Not Modify)                  *
 *******************************************************************************/

/* Timer ticks per microsecond at the (undivided) system clock: 16MHz -> 16. */
#define TIMER_PWM_TICKS_PER_US          (TIMER_PWM_SYSTEM_CLOCK_HZ / 1000000UL)

/* Full 50Hz period in timer ticks. Down-counter PWM period = (TAILR + 1)
 * ticks, so TAILR = period_ticks - 1.
 *   period_ticks = 16MHz / 50Hz = 320,000  -> 20.000ms exactly. */
#define TIMER_PWM_PERIOD_TICKS          (TIMER_PWM_SYSTEM_CLOCK_HZ / TIMER_PWM_FREQUENCY_HZ)
#define TIMER_PWM_TAILR_VALUE           (TIMER_PWM_PERIOD_TICKS - 1UL)

/* WHY a wide timer: 320,000 ticks exceeds the 16-bit (65,535) range of a
 * regular GP timer, which would force the 8-bit prescaler to be used as a
 * count EXTENSION. A wide (32/64-bit) timer split into 32-bit halves holds
 * 320,000 directly with prescaler = 0. Guard the 32-bit ceiling at compile
 * time, the same spirit as pwm_cfg.h's 16-bit LOAD guard. */
#if (TIMER_PWM_PERIOD_TICKS > 0xFFFFFFFFUL)
#error "TIMER_PWM_PERIOD_TICKS exceeds the 32-bit wide-timer width"
#endif

/*******************************************************************************
 *                          Configuration Guards                               *
 *  These fire at COMPILE time if the pulse endpoints above are edited into an  *
 *  inconsistent state - which is exactly what a re-measurement of the travel   *
 *  endpoints does. Each one protects a specific failure the driver cannot      *
 *  defend against at runtime.                                                  *
 *******************************************************************************/

/* 1. UNDERFLOW BOUND. TimerPWM_PulseUsToMatch computes
 *    (TAILR_VALUE - pulse_us * TICKS_PER_US) in unsigned 32-bit arithmetic. If a
 *    pulse ever exceeded the frame length the subtraction would WRAP to a huge
 *    match value and emit a garbage duty cycle. At 16MHz/50Hz the ceiling is
 *    ~19,999us; the current 2500us ceiling has ~8x headroom. */
#if ((TIMER_PWM_MAX_PULSE_US * TIMER_PWM_TICKS_PER_US) >= TIMER_PWM_TAILR_VALUE)
#error "TIMER_PWM_MAX_PULSE_US exceeds the PWM frame: match calculation would underflow"
#endif

/* 2. WINDOW SANITY: the clamp must be a real interval, and the boot pulse must
 *    live inside it (TimerPWM_Init clamps too, but a startup pulse that has to
 *    be clamped means the config is self-contradictory - fail the build). */
#if (TIMER_PWM_MIN_PULSE_US > TIMER_PWM_MAX_PULSE_US)
#error "TIMER_PWM_MIN_PULSE_US is above TIMER_PWM_MAX_PULSE_US: empty clamp window"
#endif

#if ((TIMER_PWM_INIT_PULSE_US < TIMER_PWM_MIN_PULSE_US) || \
     (TIMER_PWM_INIT_PULSE_US > TIMER_PWM_MAX_PULSE_US))
#error "TIMER_PWM_INIT_PULSE_US lies outside [TIMER_PWM_MIN_PULSE_US, TIMER_PWM_MAX_PULSE_US]"
#endif

/* 3. uint16 DOMAIN. The clamp in timer_pwm.c casts these bounds to uint16; a
 *    larger value would wrap and clamp to the WRONG number silently. */
#if ((TIMER_PWM_MIN_PULSE_US > 0xFFFFU) || (TIMER_PWM_MAX_PULSE_US > 0xFFFFU) || \
     (TIMER_PWM_INIT_PULSE_US > 0xFFFFU))
#error "A TIMER_PWM_*_PULSE_US value does not fit in uint16 and would wrap in the clamp"
#endif

/* 4. CROSS-LAYER SYNC with the servo domain (servo_cfg.h). The servo HAL's
 *    endpoints and this MCAL clamp are duplicated BY DESIGN (different meanings:
 *    servo travel semantics vs. last-line hardware bound) and MUST hold the same
 *    values - re-calibrating one and not the other silently de-calibrates the
 *    steering.
 *
 *    LAYERING: this MCAL deliberately does NOT #include "servo_cfg.h" - a driver
 *    including its own caller's configuration would invert the layering this
 *    module exists to keep clean (timer_pwm_cfg.h:12-15). Instead the check is
 *    conditional on servo_cfg.h ALREADY being in the translation unit, so it
 *    fires wherever both are visible (e.g. servo.c, which includes servo.h ->
 *    servo_cfg.h before timer_pwm.h).
 *    LIMITATION: in a TU that includes only the MCAL (timer_pwm.c itself) the
 *    check is silently skipped. It is a safety net, not a proof - the invariant
 *    is still ultimately maintained by the engineer editing both files. */
#ifdef SERVO_CFG_H_
#if (TIMER_PWM_MIN_PULSE_US != SERVO_MIN_PULSE_US)
#error "TIMER_PWM_MIN_PULSE_US != SERVO_MIN_PULSE_US: servo/MCAL pulse endpoints out of sync"
#endif
#if (TIMER_PWM_MAX_PULSE_US != SERVO_MAX_PULSE_US)
#error "TIMER_PWM_MAX_PULSE_US != SERVO_MAX_PULSE_US: servo/MCAL pulse endpoints out of sync"
#endif
#endif /* SERVO_CFG_H_ */

#endif /* TIMER_PWM_CFG_H_ */
