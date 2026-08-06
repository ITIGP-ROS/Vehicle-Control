/******************************************************************************
 * Timer-PWM Driver (MCAL) - generic WTIMER0 PWM output
 *
 * Generates a 50Hz (20ms) PWM signal on WT0CCP0 = PC4 using the WIDE
 * general-purpose timer WTIMER0, TimerA, in PWM mode. Pulse width is set in
 * microseconds; this layer has no notion of "servo", "angle" or "center".
 *
 * Why a wide timer (not PWM Module 1): the 20ms period needs 320,000 timer
 * ticks at 16MHz, which overflows a 16-bit GP timer. A wide (32/64-bit) timer
 * split into 32-bit halves holds 320,000 directly with NO prescaler, and its
 * timer clock is independent of the global RCC PWMDIV that the PWM-Module-1
 * plan proved is shared with (and would break) the frozen 20kHz motors.
 *
 * Down-counter PWM, non-inverted (GPTMCTL.TAPWML = 0):
 *   - period  = (TAILR + 1) ticks
 *   - high-time = (TAILR - TAMATCHR) ticks
 * so for a pulse of P ticks: TAMATCHR = TAILR - P.
 *
 * (Register code moved verbatim from the retired servo_pwm.c; the WTIMER0
 *  configuration and the us->match arithmetic are byte-for-byte unchanged.)
 ******************************************************************************/

#include "Platform_Types.h"
#include "tm4c123gh6pm_registers.h"
#include "timer_pwm.h"
#include "timer_pwm_private.h"

/*******************************************************************************
 *                          Private Variables                                  *
 *******************************************************************************/

static boolean TimerPWM_Initialized = FALSE;

/*******************************************************************************
 *                          Private Functions                                  *
 *******************************************************************************/

/**
 * @brief  Clamp a requested pulse width into the configured travel window.
 *         The SINGLE clamp used by every path that programs the match register
 *         (both TimerPWM_Init and TimerPWM_SetPulseUs), so no caller can reach
 *         the hardware unclamped.
 */
static uint16 TimerPWM_ClampPulseUs(uint16 pulse_us)
{
    if (pulse_us < (uint16)TIMER_PWM_MIN_PULSE_US)
    {
        pulse_us = (uint16)TIMER_PWM_MIN_PULSE_US;
    }
    else if (pulse_us > (uint16)TIMER_PWM_MAX_PULSE_US)
    {
        pulse_us = (uint16)TIMER_PWM_MAX_PULSE_US;
    }
    else
    {
        /* in range */
    }

    return pulse_us;
}

/**
 * @brief  Convert a (already-clamped) pulse width in microseconds to the
 *         GPTMTAMATCHR value for the configured period.
 */
static uint32 TimerPWM_PulseUsToMatch(uint16 pulse_us)
{
    uint32 pulseTicks = (uint32)pulse_us * TIMER_PWM_TICKS_PER_US;

    /* No underflow: the largest pulse this can ever be handed is the clamp
     * ceiling TIMER_PWM_MAX_PULSE_US = 2480us = 39,680 ticks, well under
     * TAILR_VALUE = 319,999 (a 20ms frame). The compile-time guard in
     * timer_pwm_cfg.h enforces that relationship if the endpoints are ever
     * re-measured, so this subtraction cannot wrap. */
    return (uint32)TIMER_PWM_TAILR_VALUE - pulseTicks;
}

/*******************************************************************************
 *                          Initialization                                     *
 *******************************************************************************/

void TimerPWM_Init(void)
{
    /* Enable WTIMER0 module clock (one-time), then wait until it is ready. */
    SYSCTL_RCGCWTIMER_REG |= TIMER_PWM_RCGCWTIMER_W0_MASK;
    while ((SYSCTL_PRWTIMER_REG & TIMER_PWM_PRWTIMER_W0_MASK) != TIMER_PWM_PRWTIMER_W0_MASK);

    /* Disable TimerA while it is being configured. */
    WTIMER0_CTL_REG &= ~TIMER_PWM_CTL_TAEN_MASK;

    /* Split the wide timer into two 32-bit halves (required for PWM mode). */
    WTIMER0_CFG_REG = TIMER_PWM_CFG_SPLIT_32BIT;

    /* TimerA: periodic + PWM (TAAMS=1, TACMR=0, TAMR=periodic). */
    WTIMER0_TAMR_REG = TIMER_PWM_TAMR_PWM_CONFIG;

    /* Non-inverted output: high during the pulse window (TAPWML = 0). */
    WTIMER0_CTL_REG &= ~TIMER_PWM_CTL_TAPWML_MASK;

    /* 50Hz period (down-count from TAILR to 0). No prescaler in 32-bit mode. */
    WTIMER0_TAILR_REG = (uint32)TIMER_PWM_TAILR_VALUE;
    WTIMER0_TAPR_REG  = 0UL;
    WTIMER0_TAPMR_REG = 0UL;

    /* Initial pulse = TIMER_PWM_INIT_PULSE_US (1650us), the MEASURED
     * wheels-straight point - NOT the geometric midpoint of the [1020, 2480]
     * clamp window (that would be 1750us). Same statement as
     * timer_pwm_cfg.h:59-64; the two files must keep agreeing.
     * Routed through the SAME clamp as SetPulseUs so a mis-set INIT_PULSE_US
     * can never program an out-of-window match. Defined-state first period; the
     * servo HAL asserts "center" semantics on top of this. */
    WTIMER0_TAMATCHR_REG =
        TimerPWM_PulseUsToMatch(TimerPWM_ClampPulseUs((uint16)TIMER_PWM_INIT_PULSE_US));

    /* Mark initialized before enabling so a racing SetPulseUs is accepted. */
    TimerPWM_Initialized = TRUE;

    /* Enable TimerA - PWM output starts toggling on WT0CCP0 (PC4). */
    WTIMER0_CTL_REG |= TIMER_PWM_CTL_TAEN_MASK;
}

/*******************************************************************************
 *                          Pulse-Width Control                                *
 *******************************************************************************/

TimerPWM_StatusType TimerPWM_SetPulseUs(uint16 pulse_us)
{
    if (TimerPWM_Initialized == FALSE)
    {
        return TIMER_PWM_ERROR_NOT_INITIALIZED;
    }

    /* Clamp into [MIN, MAX] rather than reject (matches PWM_SetDuty policy). */
    pulse_us = TimerPWM_ClampPulseUs(pulse_us);

    WTIMER0_TAMATCHR_REG = TimerPWM_PulseUsToMatch(pulse_us);

    return TIMER_PWM_OK;
}

/*******************************************************************************
 *                          Output Enable/Disable                              *
 *******************************************************************************/

void TimerPWM_Stop(void)
{
    /* Cut the PWM output: disable TimerA (clears TAEN). The CCP pin stops
     * toggling. State flag is left as-is so SetPulseUs still validates init;
     * a stopped timer is restarted via TimerPWM_Init(). */
    WTIMER0_CTL_REG &= ~TIMER_PWM_CTL_TAEN_MASK;
}
