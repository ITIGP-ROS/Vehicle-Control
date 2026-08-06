/******************************************************************************
 *
 * Module: Watchdog Timer (WDT)
 *
 * File Name: wdt_regs.h
 *
 * Description: Register definitions for TM4C123GH6PM Watchdog Timer peripheral
 *              TM4C123 has two watchdog timers: WDT0 and WDT1
 *
 ******************************************************************************/

#ifndef WDT_REGS_H_
#define WDT_REGS_H_

#include "Std_Types.h"

/*******************************************************************************
 *                          Watchdog Timer Base Addresses                      *
 *******************************************************************************/

#define WDT0_BASE_ADDRESS           0x40000000u
#define WDT1_BASE_ADDRESS           0x40001000u

/*******************************************************************************
 *                          Watchdog Timer Register Offsets                    *
 *******************************************************************************/

#define WDT_LOAD_OFFSET             0x000u      /* Load register */
#define WDT_VALUE_OFFSET            0x004u      /* Current value register */
#define WDT_CTL_OFFSET              0x008u      /* Control register */
#define WDT_ICR_OFFSET              0x00Cu      /* Interrupt clear register */
#define WDT_RIS_OFFSET              0x010u      /* Raw interrupt status */
#define WDT_MIS_OFFSET              0x014u      /* Masked interrupt status */
#define WDT_TEST_OFFSET             0x418u      /* Test register */
#define WDT_LOCK_OFFSET             0xC00u      /* Lock register */
#define WDT_PERIPHID4_OFFSET        0xFD0u      /* Peripheral ID 4 */
#define WDT_PERIPHID5_OFFSET        0xFD4u      /* Peripheral ID 5 */
#define WDT_PERIPHID6_OFFSET        0xFD8u      /* Peripheral ID 6 */
#define WDT_PERIPHID7_OFFSET        0xFDCu      /* Peripheral ID 7 */
#define WDT_PERIPHID0_OFFSET        0xFE0u      /* Peripheral ID 0 */
#define WDT_PERIPHID1_OFFSET        0xFE4u      /* Peripheral ID 1 */
#define WDT_PERIPHID2_OFFSET        0xFE8u      /* Peripheral ID 2 */
#define WDT_PERIPHID3_OFFSET        0xFECu      /* Peripheral ID 3 */
#define WDT_PCELLID0_OFFSET         0xFF0u      /* PrimeCell ID 0 */
#define WDT_PCELLID1_OFFSET         0xFF4u      /* PrimeCell ID 1 */
#define WDT_PCELLID2_OFFSET         0xFF8u      /* PrimeCell ID 2 */
#define WDT_PCELLID3_OFFSET         0xFFCu      /* PrimeCell ID 3 */

/*******************************************************************************
 *                          Register Access Macros                             *
 *******************************************************************************/

/* Watchdog 0 registers */
#define WDT0_LOAD_REG               (*((volatile uint32 *)(WDT0_BASE_ADDRESS + WDT_LOAD_OFFSET)))
#define WDT0_VALUE_REG              (*((volatile uint32 *)(WDT0_BASE_ADDRESS + WDT_VALUE_OFFSET)))
#define WDT0_CTL_REG                (*((volatile uint32 *)(WDT0_BASE_ADDRESS + WDT_CTL_OFFSET)))
#define WDT0_ICR_REG                (*((volatile uint32 *)(WDT0_BASE_ADDRESS + WDT_ICR_OFFSET)))
#define WDT0_RIS_REG                (*((volatile uint32 *)(WDT0_BASE_ADDRESS + WDT_RIS_OFFSET)))
#define WDT0_MIS_REG                (*((volatile uint32 *)(WDT0_BASE_ADDRESS + WDT_MIS_OFFSET)))
#define WDT0_TEST_REG               (*((volatile uint32 *)(WDT0_BASE_ADDRESS + WDT_TEST_OFFSET)))
#define WDT0_LOCK_REG               (*((volatile uint32 *)(WDT0_BASE_ADDRESS + WDT_LOCK_OFFSET)))

/* Watchdog 1 registers */
#define WDT1_LOAD_REG               (*((volatile uint32 *)(WDT1_BASE_ADDRESS + WDT_LOAD_OFFSET)))
#define WDT1_VALUE_REG              (*((volatile uint32 *)(WDT1_BASE_ADDRESS + WDT_VALUE_OFFSET)))
#define WDT1_CTL_REG                (*((volatile uint32 *)(WDT1_BASE_ADDRESS + WDT_CTL_OFFSET)))
#define WDT1_ICR_REG                (*((volatile uint32 *)(WDT1_BASE_ADDRESS + WDT_ICR_OFFSET)))
#define WDT1_RIS_REG                (*((volatile uint32 *)(WDT1_BASE_ADDRESS + WDT_RIS_OFFSET)))
#define WDT1_MIS_REG                (*((volatile uint32 *)(WDT1_BASE_ADDRESS + WDT_MIS_OFFSET)))
#define WDT1_TEST_REG               (*((volatile uint32 *)(WDT1_BASE_ADDRESS + WDT_TEST_OFFSET)))
#define WDT1_LOCK_REG               (*((volatile uint32 *)(WDT1_BASE_ADDRESS + WDT_LOCK_OFFSET)))

/*******************************************************************************
 *                          Control Register Bits (WDTCTL)                     *
 *******************************************************************************/

#define WDT_CTL_INTEN               0x00000001u /* Interrupt enable */
#define WDT_CTL_RESEN               0x00000002u /* Reset enable */
#define WDT_CTL_INTTYPE             0x00000004u /* Interrupt type (0=Standard, 1=NMI) */
#define WDT_CTL_WRC                 0x80000000u /* Write complete (read-only) */

/*******************************************************************************
 *                          Lock Register Values                               *
 *******************************************************************************/

#define WDT_LOCK_UNLOCKED           0x00000000u /* Unlocked state */
#define WDT_LOCK_LOCKED             0x00000001u /* Locked state */
#define WDT_LOCK_UNLOCK_KEY         0x1ACCE55u  /* Key to unlock */

/*******************************************************************************
 *                          Interrupt Clear Register                           *
 *******************************************************************************/

#define WDT_ICR_CLEAR               0xFFFFFFFFu /* Write any value to clear */

/*******************************************************************************
 *                          Test Register Bits                                 *
 *******************************************************************************/

#define WDT_TEST_STALL              0x00000100u /* Stall enable for debug */

/*******************************************************************************
 *                          System Control Register Offsets                    *
 *******************************************************************************/

/* Run Mode Clock Gating Control for Watchdog */
#define SYSCTL_RCGCWD_OFFSET        0x600u
#define SYSCTL_BASE_ADDRESS         0x400FE000u
#define SYSCTL_RCGCWD_REG           (*((volatile uint32 *)(SYSCTL_BASE_ADDRESS + SYSCTL_RCGCWD_OFFSET)))

/* Watchdog Timer Run Mode Clock Gating bits */
#define SYSCTL_RCGCWD_R0            0x00000001u /* WDT0 clock gating */
#define SYSCTL_RCGCWD_R1            0x00000002u /* WDT1 clock gating */

/*******************************************************************************
 *                          Watchdog Peripheral Ready Register                 *
 *******************************************************************************/

#define SYSCTL_PRWD_OFFSET          0xA00u
#define SYSCTL_PRWD_REG             (*((volatile uint32 *)(SYSCTL_BASE_ADDRESS + SYSCTL_PRWD_OFFSET)))

/* Watchdog Timer Peripheral Ready bits */
#define SYSCTL_PRWD_R0              0x00000001u /* WDT0 peripheral ready */
#define SYSCTL_PRWD_R1              0x00000002u /* WDT1 peripheral ready */

/*******************************************************************************
 *                          Watchdog Timer Constants                           *
 *******************************************************************************/

/**
 * @brief Watchdog timer clock frequency (system clock)
 * @note  Default system clock is 16 MHz (can be configured up to 80 MHz)
 */
#define WDT_CLOCK_FREQ_HZ           16000000u   /* 16 MHz default */

/**
 * @brief Maximum watchdog timeout value (32-bit counter)
 */
#define WDT_MAX_LOAD_VALUE          0xFFFFFFFFu

/**
 * @brief Minimum practical timeout (to avoid immediate timeout)
 * @note  REVIEW 24 / W24-10 (OPEN, deferred): a load of 1 is 62.5 ns at 16 MHz,
 *        so this floor only prevents a literal zero - it is not a "practical"
 *        minimum. Left as-is by desk decision; see docs/wdt/REVIEW_24_wdt.md.
 */
#define WDT_MIN_LOAD_VALUE          0x00000001u

/**
 * @brief Largest timeout_ms whose load value still fits WDT_MAX_LOAD_VALUE
 *
 * W24-6: beyond this point the load no longer fits the 32-bit WDTLOAD register,
 * so the conversion cannot represent the request at all. WDT_Init and
 * WDT_SetTimeout reject anything above this with WDT_ERROR_INVALID_TIMEOUT
 * rather than letting it wrap to a near-instant reload - refusing the config is
 * the correct failure direction for a watchdog.
 *
 * Derived from the clock constant so it tracks it automatically:
 *   0xFFFFFFFF / (16,000,000 / 1000) = 4,294,967,295 / 16,000 = 268,435 ms
 * i.e. ~268 s (~4.5 minutes) at the current 16 MHz. Exact at the boundary:
 *   WDT_MS_TO_LOAD(268435) = 4,294,960,000 <= 0xFFFFFFFF
 *   WDT_MS_TO_LOAD(268436) = 4,294,976,000 >  0xFFFFFFFF
 */
#define WDT_MAX_TIMEOUT_MS          (WDT_MAX_LOAD_VALUE / (WDT_CLOCK_FREQ_HZ / 1000u))

/*******************************************************************************
 *                          Helper Macros                                      *
 *******************************************************************************/

/*
 * W24-5 / W24-6 - OVERFLOW SAFETY.
 *
 * All three conversions below used to evaluate their intermediate product in
 * uint32, which overflows well inside the useful range:
 *   - WDT_LOAD_TO_MS: load * 1000 wraps above load 4,294,967 (~268 ms). At the
 *     configured 2000 ms (load 32,000,000) it returned 120, so
 *     WDT_GetTimeRemaining reported 120 ms with 2000 ms actually remaining.
 *   - WDT_MS_TO_LOAD: ms * 16000 wraps above ~268,435 ms, turning a long
 *     timeout into a tiny load = a near-instant reset, the worst possible
 *     direction for a watchdog.
 *
 * The RESULT always fits uint32 across the valid range (2000 ms <-> 32,000,000);
 * only the intermediate overflowed. Both are fixed WITHOUT any 64-bit division:
 *
 *   - WDT_LOAD_TO_MS divides by the pre-divided constant (FREQ / 1000 = 16,000)
 *     instead of multiplying by 1000 first. That is EXACTLY the same value -
 *     load * 1000 / 16,000,000 == load / 16,000, and 16,000,000/1000 is exact
 *     with no remainder - but it is a plain uint32 divide that CANNOT overflow
 *     for any input. Strictly better than widening the old expression.
 *   - WDT_MS_TO_LOAD keeps the multiply and widens it to a 32x32->64 product,
 *     then narrows explicitly, so the intermediate is well-defined even if a
 *     caller skips the range check.
 *
 * ⚠️ DO NOT "simplify" these back to a uint64 divide. A 64-bit division emits a
 * call to __aeabi_uldivmod, and that libgcc object carries an .ARM.exidx section
 * which linker/tm4c123.ld has no output section for - the image then fails to
 * link with ".ARM.exidx ... overlaps section .data". That linker-script gap is a
 * separate latent defect (see docs/wdt/FIX_24_wdt.md); this module simply has no
 * need for 64-bit division, so it does not depend on that gap being closed.
 */

/**
 * @brief Calculate load value from timeout in milliseconds
 * @param timeout_ms: Timeout in milliseconds (must be <= WDT_MAX_TIMEOUT_MS)
 * @return Load value for WDTLOAD register
 * @note  Load value = timeout_ms * (WDT_CLOCK_FREQ_HZ / 1000)
 * @note  Callers must range-check against WDT_MAX_TIMEOUT_MS first; this macro
 *        narrows to uint32 and cannot represent a larger request.
 */
#define WDT_MS_TO_LOAD(timeout_ms)  ((uint32)((uint64)(timeout_ms) * (WDT_CLOCK_FREQ_HZ / 1000u)))

/**
 * @brief Calculate timeout in milliseconds from load value
 * @param load_value: Value in WDTLOAD register
 * @return Timeout in milliseconds
 */
#define WDT_LOAD_TO_MS(load_value)  ((uint32)((load_value) / (WDT_CLOCK_FREQ_HZ / 1000u)))

/**
 * @brief Calculate load value from timeout in seconds
 * @param timeout_s: Timeout in seconds (must be <= WDT_MAX_TIMEOUT_MS / 1000)
 * @return Load value for WDTLOAD register
 * @note  Same widened intermediate as WDT_MS_TO_LOAD, expressed through it so
 *        the two cannot drift. The widening makes the arithmetic well-defined;
 *        it does NOT extend the range - a timeout_s beyond WDT_MAX_TIMEOUT_MS
 *        /1000 (~268 s) still cannot be represented in the 32-bit WDTLOAD.
 *        Currently unused; kept consistent with the other two.
 */
#define WDT_SEC_TO_LOAD(timeout_s)  WDT_MS_TO_LOAD((uint64)(timeout_s) * 1000u)

#endif /* WDT_REGS_H_ */
