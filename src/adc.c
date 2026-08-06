/******************************************************************************
 * ADC Driver (MCAL) - generic ADC0 single-channel raw reader
 *
 * One software-triggered channel (AIN0/PE3) on Sample Sequencer 3, blocking
 * read, raw 12-bit count out. No averaging/filtering and no application units
 * here - those belong above the MCAL (servo_feedback.*).
 *
 * NOTE: the analog pin is put into analog mode (DEN cleared, AMSEL set) by the
 * PORT driver via the PORT_ANALOG entry for PE3 in PORT_PBCFG.c - this driver
 * deliberately does NOT hand-poke GPIO, matching the project's PORT abstraction.
 ******************************************************************************/

#include "Platform_Types.h"
#include "tm4c123gh6pm_registers.h"
#include "adc.h"
#include "adc_private.h"
#include "timer0.h"          /* BUDGET-1: the conversion wait is TIMER0-timed,
                              * not a NOP/iteration loop. Same dependency i2c.c
                              * has, for the same reason. */

/*----------------------------------------------------------------------------
 * BUDGET-1 - the conversion-complete timeout.
 *
 * SIZING, and the two constraints that pull in opposite directions:
 *
 *  LOWER BOUND - the hardware. One SS3 conversion is ~1 us (datasheet
 *  Table 24-33: TC = 1 us at the reset-default 1 Msps), and a whole healthy
 *  Adc_ReadRaw measures ~20 us including the trigger and flag handling. So
 *  200 us is ~200x the conversion and ~10x the whole call.
 *
 *  ⚠️ UPPER BOUND - PREEMPTION, and this is the part that is easy to get wrong.
 *  This is a WALL-CLOCK deadline, and wall-clock does not stop for a context
 *  switch. From B7 this wait runs inside tRosTx at priority 2, beneath tCanTx
 *  (3) and tBattery (4) - each of which can preempt it MID-WAIT. That is
 *  precisely how BUDGET-5 turned a healthy I2C bus into a 42 % failure rate: a
 *  200 us cap against a ~60 us command left only 140 us of slack, and a couple
 *  of tCanTx wakes (~20-30 us each) consumed it. A cap sized only against the
 *  hardware would repeat that mistake here and produce spurious POT_FAULTs.
 *  200 us against a ~20 us call leaves ~180 us of slack - room for roughly six
 *  preemptions inside a single conversion.
 *
 *  RESULT: Adc_ReadRaw WCET <= ~200 us (was ~62 ms), and therefore
 *  ServoFb_ReadRawFiltered (8 samples) <= ~1.6 ms (was ~500 ms) - comfortably
 *  inside the 10 ms steering task B7 creates.
 *
 *  The bias is deliberately toward a GENEROUS cap: too tight turns
 *  healthy-but-preempted into a false POT_FAULT, which is worse than a slightly
 *  larger bound on a fault path that should never be taken. Matches
 *  I2C_ABORT_CAP_US, which is 200 us for the same reasoning.
 *--------------------------------------------------------------------------*/
#define ADC_CONV_TIMEOUT_US       (200U)
#define ADC_CONV_TIMEOUT_TICKS    TIMER0_US_TO_TICKS(ADC_CONV_TIMEOUT_US)  /* = 3200 */

/*******************************************************************************
 *                          Private Variables                                  *
 *******************************************************************************/

/* Set only at the end of Adc_Init(), once SS3 is live. Guards Adc_ReadRaw()
 * against being called on an unclocked/unconfigured sequencer, where the
 * conversion-complete flag would never assert. Mirrors TimerPWM_Initialized. */
static boolean Adc_Initialized = FALSE;

void Adc_Init(void)
{
    /* Enable ADC0 module clock (one-time), then wait until it is ready. */
    SYSCTL_RCGCADC_REG |= ADC_PRIV_RCGCADC_ADC0_MASK;
    while ((SYSCTL_PRADC_REG & ADC_PRIV_PRADC_ADC0_MASK) != ADC_PRIV_PRADC_ADC0_MASK);

    /* Disable SS3 while it is being configured. */
    ADC0_ACTSS_REG &= ~ADC_PRIV_ACTSS_SS3_MASK;

    /* SS3 trigger = processor (software): clear the EM3 field. */
    ADC0_EMUX_REG = (ADC0_EMUX_REG & ~ADC_PRIV_EMUX_EM3_MASK) | ADC_PRIV_EMUX_EM3_SOFTWARE;

    /* SS3 samples the configured channel (single sample). */
    ADC0_SSMUX3_REG = (ADC0_SSMUX3_REG & ~ADC_PRIV_SSMUX3_MUX0_MASK) |
                      ((uint32)ADC_CFG_CHANNEL_AIN & ADC_PRIV_SSMUX3_MUX0_MASK);

    /* Single sample: assert interrupt + end-of-sequence on sample 0. */
    ADC0_SSCTL3_REG = ADC_PRIV_SSCTL3_CONFIG;

    /* Re-enable SS3. */
    ADC0_ACTSS_REG |= ADC_PRIV_ACTSS_SS3_MASK;

    /* Ready ONLY now: the sequencer is configured and active, so any read from
     * this point can actually complete. */
    Adc_Initialized = TRUE;
}

uint16 Adc_ReadRaw(void)
{
    uint16 result;
    uint32 startTicks;

    /* Pre-init read: the sequencer is not active, so the completion flag would
     * never assert and a wait here would never end. Fail fast instead. */
    if (Adc_Initialized == FALSE)
    {
        return (uint16)ADC_READ_INVALID;
    }

    /* ⚠️ T25-1 GUARD. The wait below is bounded by TIMER0, so a TIMER0 that is
     * not running would make it UNBOUNDED - the exact failure mode this fix
     * exists to remove. Refuse instead, returning the same sentinel a timeout
     * returns, which ServoFb_IsCountFaulted already turns into POT_FAULT.
     *
     * Not reachable today: main.c runs Timer0_FreeRunning_Init() at init step
     * 12, and the first Adc_ReadRaw is in the 0x130 transmit path long after.
     * SteeringControl_Init() (step 9) calls ServoFb_Init() -> Adc_Init() but
     * performs NO read - verified. The guard is here so that ordering stays a
     * checked property rather than a lucky one. */
    if (Timer0_IsRunning() == FALSE)
    {
        return (uint16)ADC_READ_INVALID;
    }

    /* Clear any stale completion flag, then start an SS3 conversion. */
    ADC0_ISC_REG  = ADC_PRIV_ISC_SS3_MASK;
    ADC0_PSSI_REG = ADC_PRIV_PSSI_SS3_MASK;

    /* Wait for the conversion to complete (raw interrupt status for SS3),
     * bounded by TIME on the free-running TIMER0 - NOT by an iteration count.
     * Identical idiom to every wait in i2c.c, and for the same two reasons:
     *   1. A count cap's DURATION is -O dependent; a tick cap's is not.
     *   2. A count cap's duration is not something a caller can reason about.
     *      This one is: the wait cannot exceed ADC_CONV_TIMEOUT_US, so
     *      ServoFb_ReadRawFiltered cannot exceed 8x that, which is what makes
     *      putting steering in a 10 ms task defensible. */
    startTicks = Timer0_GetTicks();
    while ((ADC0_RIS_REG & ADC_PRIV_RIS_SS3_MASK) == 0UL)
    {
        if (Timer0_ElapsedTicks(startTicks) >= ADC_CONV_TIMEOUT_TICKS)
        {
            /* Conversion never completed. Leave the flag alone - the next call
             * clears ISC before triggering anyway - and report the failure. */
            return (uint16)ADC_READ_INVALID;
        }
    }

    /* Read the 12-bit result and acknowledge the completion. */
    result = (uint16)(ADC0_SSFIFO3_REG & ADC_PRIV_FIFO_RESULT_MASK);
    ADC0_ISC_REG = ADC_PRIV_ISC_SS3_MASK;

    return result;
}
