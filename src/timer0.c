/******************************************************************************
 * Timer0 Driver (MCAL) - free-running 32-bit microsecond tick source
 *
 * Brings up the 16/32-bit GPTM TIMER0 (base 0x40030000) as a monotonic,
 * free-running 32-bit up-counter clocked directly by the 16 MHz system clock
 * (no PLL, no prescaler): 62.5 ns per tick, exactly 16 ticks per microsecond.
 *
 * Mode: 32-bit concatenated (GPTMCFG=0x0), periodic (auto-reload), up-count
 * (TACDIR=1) with TAILR=0xFFFFFFFF, so the counter runs 0x0 -> 0xFFFFFFFF ->
 * 0x0 forever. No interrupt, no CCP output pin - a pure internal time base.
 *
 * This layer deals ONLY in raw ticks. The /16 to microseconds exists solely in
 * Timer0_GetMicros() for display; all timeout/elapsed math stays in ticks (see
 * TIMER0_US_TO_TICKS in timer0.h and the wrap proof on Timer0_ElapsedTicks).
 *
 * Register bring-up order per the approved TIMER0 allocation+register audit
 * (datasheet chapter 11 / init sequence 11.4.1, p.721-722):
 *   1. RCGCTIMER bit0 = 1        enable TIMER0 clock          (SYSCTL 0x604)
 *   2. poll PRTIMER bit0 == 1    wait ready (bounded)         (SYSCTL 0xA04)
 *   3. GPTMCTL.TAEN = 0          disable before config        (0x00C)
 *   4. GPTMCFG = 0x00000000      32-bit concatenated          (0x000)
 *   5. GPTMTAMR = 0x00000012     periodic | up-count          (0x004)
 *   6. GPTMTAILR = 0xFFFFFFFF    full 2^32 wrap top           (0x028)
 *   7. GPTMCTL.TAEN = 1          start (counter loads 0x0)    (0x00C)
 *   8. skip the reset-value window                            (GPTMTAR 0x048)
 ******************************************************************************/

#include "Platform_Types.h"
#include "tm4c123gh6pm_registers.h"
#include "timer0.h"
#include "timer0_private.h"

/*******************************************************************************
 *                    Compile-time checks for T25-4 / T25-5                    *
 *
 * Two assumptions in this driver are load-bearing, silent, and expressed only
 * in prose: the 2^32 counter modulus that makes the wrap proof exact, and the
 * 16-ticks-per-microsecond identity that every consumer's timeout is scaled by.
 * Neither is enforced by anything, so a plausible future edit corrupts every
 * i2c timeout with no compiler complaint. These pin them.
 *
 * The build is gnu90 (no -std=, GCC 4.8), so _Static_assert is unavailable.
 * The C90 negative-array-size idiom is used instead - same as the wdt group
 * added in FIX 24. Typedefs emit no code: no runtime behaviour changes here.
 *******************************************************************************/

/* T25-4: the branchless wrap identity in Timer0_ElapsedTicks - (now - start) in
 * uint32 - is exact ONLY because the counter's modulus is exactly 2^32, i.e.
 * TAILR = 0xFFFFFFFF in up-count mode. Shorten the period and the subtraction
 * goes silently wrong for every i2c timeout. Fail the BUILD instead. */
typedef char Timer0_AssertFullPeriod[(TIMER0_TAILR_TOP == 0xFFFFFFFFUL) ? 1 : -1];

/* T25-4 (supporting): the same modulus also depends on the counter being 32-bit
 * concatenated (a 16-bit config would make it 2^16) and counting UP (down-count
 * would invert the sign of every elapsed value). Both are already expressed by
 * the existing config constants, so pin those rather than restate them. */
typedef char Timer0_Assert32BitConcat[(TIMER0_CFG_32BIT_CONCAT == 0UL) ? 1 : -1];
typedef char Timer0_AssertUpCount[((TIMER0_TAMR_UPCOUNT_CONFIG &
                                    (1UL << TIMER0_TAMR_TACDIR_POS)) != 0UL) ? 1 : -1];

/* T25-5: 16 ticks/us IS the 16 MHz system clock (a true 16 MHz post-C6-4 - MOSC
 * direct, PLL bypassed - and measured at -0.0004 % in REVIEW 25 section 4).
 *
 * This is an uncoupled restatement of that fact, and deliberately so: there is
 * no canonical system-clock constant in this tree to derive it from. The 16 MHz
 * figure is currently written out independently in NINE per-module headers
 * (pwm_cfg, wdt_regs, wdt_cfg, uart_cfg, can_cfg, i2c_cfg, systick_cfg,
 * timer_pwm_cfg, qei_cfg) and SystemClock_Init() is static inside startup and
 * exports nothing. Pulling one of those nine in here would add a cross-module
 * include purely for an assert and pick an arbitrary winner, so the literal is
 * the honest choice - this comment IS the T25-5 acknowledgement, not a paper-over.
 * The duplication itself is the known, deferred issue flagged in pwm_cfg.h:19-22.
 *
 * What the assert buys: an edit to TIMER0_TICKS_PER_US, or a future PLL that
 * raises SYSCLK without updating it, fails the BUILD instead of silently
 * re-scaling every consumer's timeout (i2c's 200 us cap would become an
 * effective 40 us at 80 MHz). IF A PLL IS EVER ADDED, THIS CONSTANT AND THIS
 * ASSERT MUST BOTH BE REVISITED - the assert will fire, and that is the point. */
typedef char Timer0_AssertTicksPerUs[(((uint32)TIMER0_TICKS_PER_US * 1000000UL)
                                      == 16000000UL) ? 1 : -1];

/*******************************************************************************
 *                          Module State (T25-1 / T25-2)                       *
 *
 * timer0 used to be the only reviewed MCAL with NO state at all - no flag, no
 * status, a void Init - while every other one (uart, wdt, adc, can, i2c) rejects
 * a command issued before its init. These two statics close that gap. Both are
 * file-static and therefore BSS-zeroed, so the pre-init reading is FALSE / 0
 * without needing a runtime initialiser.
 *******************************************************************************/

/* TRUE only between a successful Init and the next Init attempt. "Successful"
 * means the counter was OBSERVED ADVANCING, not merely configured - which is
 * the distinction that matters, because a clocked-but-stopped counter (TAEN
 * never took) reads a frozen GPTMTAR, and a frozen GPTMTAR makes every i2c
 * bounded wait silently unbounded. */
static boolean g_Timer0_Running = FALSE;

/* Saturating count of bounded-wait expiries during bring-up (T25-2). */
static uint32 g_Timer0_InitExpiryCount = 0U;

static void Timer0_CountExpiry(void)
{
    if (g_Timer0_InitExpiryCount < 0xFFFFFFFFUL)
    {
        g_Timer0_InitExpiryCount++;
    }
}

/*******************************************************************************
 *                          Initialization                                     *
 *******************************************************************************/

Std_ReturnType Timer0_FreeRunning_Init(void)
{
    uint32 spins;
    uint32 firstRead;

    /* Publish "not running" for the WHOLE bring-up window, not just on failure.
     * Between step 3 (TAEN cleared) and step 8 (advance confirmed) the counter
     * is stopped and GPTMTAR holds a stale value; a reader that slipped in
     * there would latch a start stamp that never advances. Clearing the flag up
     * front means such a reader gets TIMER0_TICKS_INVALID instead, and any wait
     * it is already inside expires immediately rather than hanging. This is
     * also what makes a RE-init safe rather than merely non-corrupting (T25-3). */
    g_Timer0_Running = FALSE;

    /* 1. Enable the TIMER0 module clock (idempotent OR - safe on re-init). */
    SYSCTL_RCGCTIMER_REG |= TIMER0_RCGCTIMER_T0_MASK;

    /* 2. Wait until the clock is ready before touching any TIMER0 register.
     * Bounded so a never-settling clock (dead peripheral) fails fast instead
     * of hanging; a healthy module reports ready within ~3 system clocks. */
    spins = TIMER0_INIT_SPIN_CAP;
    while (((SYSCTL_PRTIMER_REG & TIMER0_PRTIMER_T0_MASK) == 0UL) && (spins > 0UL))
    {
        spins--;
    }

    /* T25-2: ABORT HERE ON EXPIRY - do not fall through. The code below writes
     * GPTMCFG/GPTMTAMR/GPTMTAILR/GPTMCTL, and if PRTIMER never reported ready
     * those writes land on a peripheral whose clock never came up: a BUS FAULT,
     * reached from a function that used to have no way to say "I failed".
     * Returning before the first register write is the whole point. */
    if (spins == 0UL)
    {
        Timer0_CountExpiry();
        return E_NOT_OK;
    }

    /* 3. Disable TimerA while (re)configuring. Clearing TAEN first is the
     * datasheet ordering rule (GPTMCFG/GPTMTAMR must only change while TAEN is
     * clear, p.726/728) and is what makes a second Init() call safe: a running
     * counter is stopped before any config register is rewritten. */
    TIMER0_CTL_REG &= ~TIMER0_CTL_TAEN_MASK;

    /* 4. 32-bit concatenated timer (TimerA+TimerB fused into one 32-bit count). */
    TIMER0_CFG_REG = TIMER0_CFG_32BIT_CONCAT;

    /* 5. Periodic + up-count. No prescaler register: GPTMTAPR is unavailable in
     * concatenated mode, so it is deliberately not written. */
    TIMER0_TAMR_REG = TIMER0_TAMR_UPCOUNT_CONFIG;

    /* 6. Up-count top: wrap at the full 32-bit range. */
    TIMER0_TAILR_REG = TIMER0_TAILR_TOP;

    /* 7. Start. On the next clock the up-counter loads 0x0 and increments. */
    TIMER0_CTL_REG |= TIMER0_CTL_TAEN_MASK;

    /* 8. START-WINDOW SKIP, as a POSITIVE "the counter is advancing" check
     * (T25-3). Wait (bounded) until GPTMTAR is observed to CHANGE from whatever
     * it read the instant after TAEN was set.
     *
     * WHY NOT THE OLD TEST. This used to spin while `GPTMTAR == 0xFFFFFFFF`,
     * the reset pattern, which is correct ONLY out of cold reset. On a RE-init
     * the counter was stopped in step 3 with GPTMTAR holding an arbitrary
     * running count - almost never the sentinel - so the loop exited on its
     * FIRST evaluation, before the up-start reload of 0x0 landed. A caller
     * stamping right after that latched the STALE pre-re-init count; the
     * counter then reloaded to 0, and Timer0_ElapsedTicks(stale) returned
     * ~2^32 - stale, an enormous interval that instantly tripped every i2c cap.
     * The header advertised "safe to call more than once" the whole time.
     *
     * WHY THE POSITIVE TEST HAS NO SUCH HOLE - both entry cases converge:
     *   - firstRead is the pre-reload value (cold-reset 0xFFFFFFFF, or a stale
     *     count): the loop runs until the reload lands and the counter starts
     *     incrementing, so it exits on a genuine post-reload value.
     *   - firstRead is already post-reload (a small count): the loop exits on
     *     the very next increment, i.e. 62.5 ns later, also genuine.
     * Either way the counter has been SEEN to move before this returns, which
     * is a strictly stronger guarantee than "is not currently the sentinel" -
     * and it is the only one that distinguishes a running counter from a
     * clocked-but-frozen one (the T25-1 failure that makes i2c's bounded waits
     * unbounded). It cannot be fooled by the counter passing through firstRead
     * again: exit is on the FIRST differing read, ~one tick in.
     *
     * TERMINATION: a healthy counter increments every 62.5 ns, so this exits in
     * 1-2 iterations. If TAEN never took, it expires and we report failure. */
    firstRead = TIMER0_TAR_REG;
    spins = TIMER0_INIT_SPIN_CAP;
    while ((TIMER0_TAR_REG == firstRead) && (spins > 0UL))
    {
        spins--;
    }

    if (spins == 0UL)
    {
        /* Configured but NOT counting - the exact silent state that used to
         * make every consumer's timeout never elapse. Stay marked not-running
         * so the accessors return their fail-fast sentinels. */
        Timer0_CountExpiry();
        return E_NOT_OK;
    }

    g_Timer0_Running = TRUE;
    return E_OK;
}

boolean Timer0_IsRunning(void)
{
    return g_Timer0_Running;
}

uint32 Timer0_GetInitExpiryCount(void)
{
    return g_Timer0_InitExpiryCount;
}

/*******************************************************************************
 *                          Tick Access                                        *
 *******************************************************************************/

uint32 Timer0_GetTicks(void)
{
    /* T25-1 GATE. Reading GPTMTAR before RCGCTIMER enables the module clock is
     * a BUS FAULT on this part, so the pre-init case cannot be allowed to reach
     * the register at all - unlike the other MCALs, whose pre-init reads land
     * on a live peripheral and merely return junk. Returning the sentinel keeps
     * the failure a value, not an exception.
     *
     * Cost on the happy path is one BSS load + compare + not-taken branch. The
     * single-LDR, tear-free property this driver is valued for (REVIEW 25 §3,
     * §5.1) is untouched: the guard reads a separate word and the register
     * access below is still exactly one LDR. */
    if (g_Timer0_Running == FALSE)
    {
        return TIMER0_TICKS_INVALID;
    }

    /* Single aligned 32-bit read of GPTMTAR (RO, 0x048) -> one LDR, no tearing.
     * GPTMTAR (not GPTMTAV) is read on purpose: GPTMTAV is RW and a stray write
     * would reload the counter. This atomic single-word read is exactly why the
     * 16/32-bit GPTM is preferred over the 64-bit WTIMER (which needs a hi/lo
     * latched read). */
    return TIMER0_TAR_REG;
}

uint32 Timer0_ElapsedTicks(uint32 startTicks)
{
    /* T25-1 GATE, and the one that actually closes the hang.
     *
     * Every consumer spins `while (Timer0_ElapsedTicks(start) < cap)`. If the
     * counter is frozen, the honest answer to "how much time has passed?" is
     * unknowable - but returning 0 (what a frozen register produces) means the
     * loop NEVER exits, converting i2c's carefully bounded waits into unbounded
     * ones. Returning the maximum makes every such wait expire on its first
     * evaluation, so a dead timebase surfaces as an immediate, reported
     * I2C_ERROR_TIMEOUT instead of a silent lockup.
     *
     * Fail-fast is the right bias here: a stopped timebase is unrecoverable at
     * this layer, and a caller that gives up can at least say so. */
    if (g_Timer0_Running == FALSE)
    {
        return TIMER0_ELAPSED_INVALID;
    }

    /* WRAP PROOF (invariant 3): all values are uint32, so the subtraction is
     * evaluated modulo 2^32.
     *   - No wrap (now >= startTicks): (now - startTicks) is the plain elapsed
     *     count.
     *   - One wrap (now < startTicks numerically): (now - startTicks) in uint32
     *     arithmetic == now + 2^32 - startTicks == (true elapsed) mod 2^32,
     *     which equals the true elapsed count as long as the true elapsed is
     *     < 2^32 ticks (~268.4 s @ 16 MHz).
     * Either way the result is correct for any interval shorter than one full
     * period. No conditional/branch on wrap is needed - the modular arithmetic
     * handles it. The explicit & 0xFFFFFFFF is a no-op on uint32 kept only to
     * document the modulus. */
    return (uint32)((Timer0_GetTicks() - startTicks) & 0xFFFFFFFFUL);
}

uint32 Timer0_GetMicros(void)
{
    /* DISPLAY/LOG ONLY. Integer divide by 16 truncates the sub-us remainder;
     * value wraps every ~268.4 s like the raw ticks. Timeout math must NOT use
     * this - it stays in raw ticks to avoid the truncation drift. Divide by the
     * power-of-two TICKS_PER_US (16) is an exact shift for unsigned operands.
     *
     * Not separately gated: it inherits Timer0_GetTicks()'s guard, so a pre-init
     * call yields TIMER0_TICKS_INVALID/16 = 0x0FFFFFFF rather than a bus fault.
     * That is a nonsense timestamp on purpose and is fine for a display-only
     * accessor - anything that must KNOW should call Timer0_IsRunning(). */
    return Timer0_GetTicks() / (uint32)TIMER0_TICKS_PER_US;
}
