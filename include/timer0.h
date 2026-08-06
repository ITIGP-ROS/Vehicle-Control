/******************************************************************************
 *
 * Module: Timer0 (MCAL)
 *
 * File Name: timer0.h
 *
 * Description: Public API for the TIMER0 free-running microsecond-tick source.
 *
 *  A monotonic 32-bit up-counter clocked at the 16 MHz system clock (62.5 ns
 *  per tick, 16 ticks = 1 us), running free on the 16/32-bit GPTM TIMER0 in
 *  32-bit concatenated, periodic, up-count mode. No prescaler, no interrupt,
 *  no CCP pin - a pure internal time base.
 *
 *  This is the MCAL: it exposes ONLY raw ticks and a raw microsecond helper.
 *  It has no notion of timeouts, deadlines, or what is being timed - callers
 *  decide meaning. Timeout/elapsed math MUST stay in raw ticks (see
 *  TIMER0_US_TO_TICKS); the /16 to microseconds is display/log only.
 *
 *  Distinct peripheral from the servo's WTIMER0 (wide timer, base 0x40036000,
 *  clocked via SYSCTL_RCGCWTIMER). TIMER0 (base 0x40030000) is clocked via
 *  SYSCTL_RCGCTIMER - zero overlap with the servo path.
 *
 ******************************************************************************/

#ifndef TIMER0_H_
#define TIMER0_H_

#include "Platform_Types.h"
#include "Std_Types.h"          /* Std_ReturnType / E_OK / E_NOT_OK (T25-2) */

/*******************************************************************************
 *                          Timebase Constant                                  *
 *******************************************************************************/

/* System clock is 16 MHz with no PLL, so exactly 16 timer ticks == 1 us. */
#define TIMER0_TICKS_PER_US             (16U)

/*******************************************************************************
 *                    Not-running sentinels (T25-1)                            *
 *******************************************************************************/

/**
 * @brief  Returned by Timer0_GetTicks() when the counter is NOT running.
 *
 *  This is GPTMTAR's own reset value, i.e. literally what the register would
 *  read if the timer had never been started - so it is the honest answer, not
 *  an invented code.
 *
 *  ⚠️ IT IS NOT A RESERVED VALUE. A healthy running counter genuinely passes
 *  through 0xFFFFFFFF for one tick (62.5 ns) once every ~268.4 s. Never test a
 *  tick value against this to decide whether the timebase is alive - call
 *  Timer0_IsRunning(), which is authoritative and cannot alias. The sentinel
 *  exists so a pre-init read is defined and harmless (no bus fault), not as a
 *  detection mechanism.
 */
#define TIMER0_TICKS_INVALID            (0xFFFFFFFFUL)

/**
 * @brief  Returned by Timer0_ElapsedTicks() when the counter is NOT running.
 *
 *  Deliberately the LARGEST representable interval, because every caller uses
 *  this function the same way: `while (Timer0_ElapsedTicks(start) < cap)`. The
 *  contract caps callers at < 2^31 ticks (see i2c.h), so this value exceeds any
 *  legal cap and every bounded wait EXPIRES IMMEDIATELY on a dead timebase.
 *
 *  That direction is chosen on purpose. The pre-fix failure was the opposite: a
 *  stopped counter made elapsed stay 0 forever, so every one of i2c's bounded
 *  waits silently became UNBOUNDED - the "one hang-forever invariant" REVIEW 22
 *  signed off was conditional on timer0 running, and nothing checked it. Failing
 *  fast turns a hang into a reported timeout. See T25-1.
 */
#define TIMER0_ELAPSED_INVALID          (0xFFFFFFFFUL)

/*******************************************************************************
 *                          Compile-Time Conversion                            *
 *******************************************************************************/

/**
 * @brief  Compile-time microseconds -> raw ticks, for sizing timeout caps in
 *         the tick domain (never divide in the hot path).
 * @param  us  microseconds (unsigned). Evaluated in uint32 arithmetic.
 * @note   OVERFLOW CEILING: (us * 16) overflows uint32 when
 *         us > 268,435,455 (= 0xFFFFFFFF / 16 ~= 268.4 s). Callers must keep
 *         us below that; irrelevant for us-scale caps (e.g. an I2C per-command
 *         timeout of a few hundred us), but stated so it is not a silent trap.
 */
#define TIMER0_US_TO_TICKS(us)          (((uint32)(us)) * (uint32)TIMER0_TICKS_PER_US)

/*******************************************************************************
 *                          Function Prototypes                                *
 *******************************************************************************/

/**
 * @brief  One-time (or re-runnable) bring-up of TIMER0 as a free-running 32-bit
 *         up-counter. Enables the TIMER0 clock and waits (bounded) for ready,
 *         disables TimerA, configures 32-bit concatenated / periodic / up-count
 *         with TAILR=0xFFFFFFFF, starts it, and then waits (bounded) until the
 *         counter is OBSERVED TO ADVANCE, so the first caller-visible tick is a
 *         genuine running value.
 *
 * @return E_OK     the counter is configured AND was observed counting; every
 *                  subsequent Timer0_GetTicks()/ElapsedTicks() is live.
 *         E_NOT_OK either bounded wait expired (T25-2). The module stays marked
 *                  NOT running, Timer0_IsRunning() returns FALSE, the tick
 *                  accessors return their sentinels instead of touching the
 *                  peripheral, and Timer0_GetInitExpiryCount() is incremented.
 *
 * @note   ⚠️ CHECK THE RETURN. This used to be `void`, which made both bounded
 *         waits' expiry invisible to everyone (T25-2): a clock that never came
 *         ready fell through into register writes on a clock-gated peripheral
 *         (bus fault), and a failed start window let a caller latch a stale
 *         start stamp. Both now fail loudly instead.
 *
 * @note   Safe to call more than once, and the claim is now enforced rather
 *         than asserted (T25-3). A re-init stops a running counter, whose
 *         GPTMTAR retains an arbitrary count - NOT the 0xFFFFFFFF reset
 *         pattern - so the old "spin while GPTMTAR == 0xFFFFFFFF" skip exited
 *         immediately, before the reload landed, and handed back the STALE
 *         pre-re-init count. The positive advance check has no such hole: it
 *         cannot pass until the counter has been seen to move.
 *
 * @note   Not reentrant against a concurrent Timer0_GetTicks() (bare-metal
 *         cooperative use assumed). Across the whole call the module publishes
 *         "not running", so a concurrent reader gets the sentinel rather than a
 *         half-configured count.
 */
Std_ReturnType Timer0_FreeRunning_Init(void);

/**
 * @brief  Is the timebase live? TRUE only after a Timer0_FreeRunning_Init()
 *         that returned E_OK - i.e. the counter was actually OBSERVED counting,
 *         not merely configured.
 *
 * @return TRUE if Timer0_GetTicks()/ElapsedTicks() return real measurements.
 *
 * @note   This is the check any module whose timeouts depend on TIMER0 should
 *         make once at ITS OWN init - I2C_InitWithConfig() does exactly that and
 *         refuses to come up on a dead timebase, because i2c's entire
 *         bounded-wait guarantee is conditional on this counter running (T25-1).
 *         Prefer this over inspecting tick values: TIMER0_TICKS_INVALID is a
 *         legal running count once every ~268 s and cannot be used as a test.
 */
boolean Timer0_IsRunning(void);

/**
 * @brief  Saturating lifetime count of bounded-wait expiries inside
 *         Timer0_FreeRunning_Init() - the PRTIMER clock-ready poll and the
 *         counter-advance window (T25-2).
 * @return 0 in a healthy system. Non-zero means the TIMER0 peripheral did not
 *         come up, so the timebase is dead and every I2C timeout is affected.
 * @note   Same "a bounded wait that gives up COUNTS it" idiom as
 *         Can_GetIfTimeoutCount() (C6-3) and UART_GetTxDroppedCount() (FIX 23).
 *         Readable before or after init - the counter is BSS-zeroed.
 */
uint32 Timer0_GetInitExpiryCount(void);

/**
 * @brief  Read the current raw counter value (single atomic 32-bit read of
 *         GPTMTAR). This is the timestamp primitive.
 * @return raw tick count (0 .. 0xFFFFFFFF), incrementing at 16 MHz, or
 *         TIMER0_TICKS_INVALID if the counter is not running.
 * @note   Before a successful init this does NOT read the peripheral at all.
 *         The register lives behind a clock gate, and reading a clock-gated
 *         peripheral on this part is a BUS FAULT - which is what made the
 *         "Timer0 before I2C" ordering rule a hard crash rather than a
 *         degradation (T25-1).
 */
uint32 Timer0_GetTicks(void);

/**
 * @brief  Ticks elapsed since a previously captured start stamp, correct across
 *         a single wrap for any interval < 2^32 ticks (~268.4 s).
 * @param  startTicks  a value previously returned by Timer0_GetTicks().
 * @return (now - startTicks) modulo 2^32, in raw ticks, or
 *         TIMER0_ELAPSED_INVALID if the counter is not running - which makes
 *         every caller's bounded wait expire at once instead of never (T25-1).
 */
uint32 Timer0_ElapsedTicks(uint32 startTicks);

/**
 * @brief  Current counter value expressed in microseconds - DISPLAY/LOG ONLY.
 * @return GPTMTAR / 16, in microseconds.
 * @note   TRUNCATES the sub-microsecond remainder (integer divide by 16) and
 *         wraps every ~268.4 s just like the raw tick count. Do NOT use this in
 *         timeout/elapsed comparisons - keep those in raw ticks to avoid the
 *         truncation drift. Provided only for human-readable output.
 */
uint32 Timer0_GetMicros(void);

#endif /* TIMER0_H_ */
