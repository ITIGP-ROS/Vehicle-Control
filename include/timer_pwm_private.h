/******************************************************************************
 *
 * Module: TimerPWM (MCAL)
 *
 * File Name: timer_pwm_private.h
 *
 * Description: Private register bit/field definitions for the generic WTIMER0
 *              Timer-PWM driver. Included by timer_pwm.c only. Register
 *              *addresses* live in tm4c123gh6pm_registers.h (WTIMER0_* block)
 *              since they are shared SoC-level registers, not private here.
 *
 *              These definitions are register-level only - this file knows
 *              nothing about "servo", "angle" or "center". (Moved verbatim
 *              from the retired servo_pwm_private.h, renamed SERVO_PWM_* ->
 *              TIMER_PWM_*; bit positions/values are byte-for-byte identical.)
 *
 ******************************************************************************/

#ifndef TIMER_PWM_PRIVATE_H_
#define TIMER_PWM_PRIVATE_H_

#include "Platform_Types.h"

/*******************************************************************************
 *                  SYSCTL Bits Touched By This Driver                         *
 *******************************************************************************/

/* SYSCTL_RCGCWTIMER bit 0 enables the WTIMER0 module clock. */
#define TIMER_PWM_RCGCWTIMER_W0_POS     (0U)
#define TIMER_PWM_RCGCWTIMER_W0_MASK    (1UL << TIMER_PWM_RCGCWTIMER_W0_POS)

/* SYSCTL_PRWTIMER bit 0 reads back WTIMER0's clock-ready status (mirrors the
 * PWM/PORT drivers' RCGC/PR ready-wait instead of a heuristic busy-loop). */
#define TIMER_PWM_PRWTIMER_W0_POS       (0U)
#define TIMER_PWM_PRWTIMER_W0_MASK      (1UL << TIMER_PWM_PRWTIMER_W0_POS)

/*******************************************************************************
 *                  GPTMCTL (WTIMER0_CTL) Bits - TimerA                        *
 *******************************************************************************/

#define TIMER_PWM_CTL_TAEN_POS          (0U)   /* TimerA enable           */
#define TIMER_PWM_CTL_TAEN_MASK         (1UL << TIMER_PWM_CTL_TAEN_POS)

#define TIMER_PWM_CTL_TAPWML_POS        (6U)   /* TimerA PWM output invert */
#define TIMER_PWM_CTL_TAPWML_MASK       (1UL << TIMER_PWM_CTL_TAPWML_POS)

/*******************************************************************************
 *                  GPTMCFG (WTIMER0_CFG)                                      *
 *******************************************************************************/

/* 0x4 = split the wide timer into two independent 32-bit halves (TimerA/B).
 * PWM mode requires this split configuration. */
#define TIMER_PWM_CFG_SPLIT_32BIT       (0x04UL)

/*******************************************************************************
 *                  GPTMTAMR (WTIMER0_TAMR) - PWM mode for TimerA              *
 *  TAMR   (bits 1:0) = 0x2  -> periodic timer mode                            *
 *  TACMR  (bit 2)    = 0    -> edge-count capture disabled                    *
 *  TAAMS  (bit 3)    = 1    -> alternate mode = PWM                           *
 *  TAMRSU (bit 10)   = 1    -> match register updates on the next TIMEOUT     *
 *******************************************************************************/

#define TIMER_PWM_TAMR_MODE_PERIODIC    (0x2UL)      /* bits 1:0 */
#define TIMER_PWM_TAMR_TACMR_POS        (2U)
#define TIMER_PWM_TAMR_TAAMS_POS        (3U)
#define TIMER_PWM_TAMR_TAMRSU_POS       (10U)

/* WHY TAMRSU = 1 (datasheet GPTMTAMR bit 10): with TAMRSU CLEAR, GPTMTAMATCHR is
 * updated "on the next cycle" - i.e. a TimerPWM_SetPulseUs() landing mid-frame
 * retimes the pulse ALREADY being emitted, truncating or stretching that one
 * pulse. With TAMRSU SET, the update lands "on the next timeout", so a new pulse
 * width only ever takes effect at a 20ms frame boundary and every emitted pulse
 * is a whole, valid one.
 *
 * Boot behaviour is unaffected: the same datasheet field states "If the timer is
 * disabled (TAEN is clear) when this bit is set, GPTMTAMATCHR ... [is] updated
 * when the timer is enabled". TimerPWM_Init() writes TAMATCHR while TAEN is
 * still clear and only then sets TAEN, so the FIRST emitted frame already
 * carries TIMER_PWM_INIT_PULSE_US. The one-frame deferral applies only to
 * SetPulseUs calls made after the output is running (<=20ms of extra latency). */
#define TIMER_PWM_TAMR_PWM_CONFIG       (TIMER_PWM_TAMR_MODE_PERIODIC | \
                                         (1UL << TIMER_PWM_TAMR_TAAMS_POS)  | \
                                         (1UL << TIMER_PWM_TAMR_TAMRSU_POS))

#endif /* TIMER_PWM_PRIVATE_H_ */
