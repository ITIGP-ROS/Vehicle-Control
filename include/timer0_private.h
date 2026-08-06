/******************************************************************************
 *
 * Module: Timer0 (MCAL)
 *
 * File Name: timer0_private.h
 *
 * Description: Private register bit/field and config-value definitions for the
 *              TIMER0 free-running microsecond-counter driver. Included by
 *              timer0.c only. Register *addresses* live in
 *              tm4c123gh6pm_registers.h (TIMER0_* block) since they are shared
 *              SoC-level registers, not private here - same convention as
 *              timer_pwm_private.h.
 *
 *              These definitions are register-level only - this file knows
 *              nothing about what a "microsecond of what" means; it is pure
 *              GPTM bring-up for a 32-bit concatenated, periodic, up-count
 *              free-running counter. Bit positions/values per TM4C123GH6PM
 *              datasheet chapter 11 (General-Purpose Timers).
 *
 ******************************************************************************/

#ifndef TIMER0_PRIVATE_H_
#define TIMER0_PRIVATE_H_

#include "Platform_Types.h"

/*******************************************************************************
 *                  SYSCTL Bits Touched By This Driver                         *
 *******************************************************************************/

/* SYSCTL_RCGCTIMER bit 0 enables the TIMER0 module clock (Run mode).
 * Datasheet Register 59 (RCGCTIMER), offset 0x604, bit R0, p.338. */
#define TIMER0_RCGCTIMER_T0_POS         (0U)
#define TIMER0_RCGCTIMER_T0_MASK        (1UL << TIMER0_RCGCTIMER_T0_POS)

/* SYSCTL_PRTIMER bit 0 reads back TIMER0's clock-ready status. Polled instead
 * of a blind delay, mirroring the TimerPWM/PORT ready-wait idiom. Datasheet
 * Register 107 (PRTIMER), offset 0xA04, bit R0, p.404. */
#define TIMER0_PRTIMER_T0_POS           (0U)
#define TIMER0_PRTIMER_T0_MASK          (1UL << TIMER0_PRTIMER_T0_POS)

/*******************************************************************************
 *                  GPTMCTL (TIMER0_CTL) Bits - TimerA                         *
 *  Datasheet Register 4 (GPTMCTL), offset 0x00C, p.736-739.                   *
 *******************************************************************************/

#define TIMER0_CTL_TAEN_POS             (0U)   /* TimerA enable (bit 0) */
#define TIMER0_CTL_TAEN_MASK            (1UL << TIMER0_CTL_TAEN_POS)

/*******************************************************************************
 *                  GPTMCFG (TIMER0_CFG)                                       *
 *  Datasheet Register 1 (GPTMCFG), offset 0x000, p.726-727.                   *
 *  Value 0x0 => for a 16/32-bit timer, selects the 32-bit (concatenated)      *
 *  timer configuration - TimerA and TimerB fused into one 32-bit counter.     *
 *******************************************************************************/

#define TIMER0_CFG_32BIT_CONCAT         (0x00000000UL)

/*******************************************************************************
 *                  GPTMTAMR (TIMER0_TAMR) - free-running up-counter           *
 *  Datasheet Register 2 (GPTMTAMR), offset 0x004, p.728-731.                  *
 *    TAMR  (bits 1:0) = 0x2  -> periodic timer mode  (auto-reloads at wrap)   *
 *    TACDIR (bit 4)   = 1    -> count UP (starts from 0x0 when enabled)        *
 *  In 32-bit concatenated mode this register controls both halves and         *
 *  GPTMTBMR is ignored (p.728). No prescaler (GPTMTAPR unavailable in          *
 *  concatenated mode, p.706/707) - so it is not touched here.                 *
 *******************************************************************************/

#define TIMER0_TAMR_MODE_PERIODIC       (0x2UL)      /* bits 1:0 */
#define TIMER0_TAMR_TACDIR_POS          (4U)         /* count direction */

#define TIMER0_TAMR_UPCOUNT_CONFIG      (TIMER0_TAMR_MODE_PERIODIC | \
                                         (1UL << TIMER0_TAMR_TACDIR_POS))   /* 0x12 */

/*******************************************************************************
 *                  GPTMTAILR (TIMER0_TAILR) - up-count top                    *
 *  Datasheet Register 10 (GPTMTAILR), offset 0x028, p.755.                    *
 *  Full 32-bit reload: up-count wraps 0xFFFFFFFF -> 0x0, giving a clean 2^32   *
 *  tick modulus (see the wrap proof in timer0.c / TIMER0_ElapsedTicks).       *
 *******************************************************************************/

#define TIMER0_TAILR_TOP                (0xFFFFFFFFUL)

/*******************************************************************************
 *                  Bring-up bounds                                            *
 *******************************************************************************/

/* GPTMTAR reset value (counters init to all 1s, datasheet 11.3.1 p.707).
 *
 * ⚠️ NO LONGER USED AS A BRING-UP SENTINEL (T25-3). Step 8 of the init used to
 * spin while GPTMTAR held this pattern, which only works out of COLD RESET: on
 * a re-init the stopped counter retains an arbitrary count instead, so that
 * test exited before the reload landed and handed back a stale stamp. Step 8 is
 * now a positive "counter observed to advance" check, which covers both entry
 * cases - see the proof in timer0.c.
 *
 * Kept because it documents the register's reset value, which is what
 * TIMER0_TICKS_INVALID (timer0.h) reports for a not-running counter: the
 * pre-init read is the honest reset pattern, not an invented code. */
#define TIMER0_TAR_RESET_SENTINEL       (0xFFFFFFFFUL)

/* Bounded spin cap for the PRTIMER ready-poll and the counter-advance window,
 * so neither can hang forever if the clock never settles (e.g. dead
 * peripheral). Ready is ~3 system clocks and an advance is 1 tick; this cap is
 * deliberately generous (~ms class even at the slowest supported SYSCLK) so it
 * never gates on optimization level. Expiry of either is now reported -
 * E_NOT_OK plus Timer0_GetInitExpiryCount() (T25-2). */
#define TIMER0_INIT_SPIN_CAP            (100000UL)

#endif /* TIMER0_PRIVATE_H_ */
