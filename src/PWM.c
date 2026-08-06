/******************************************************************************
 * PWM Driver
 *
 * Hardware Connections (from schematic, confirmed on real hardware):
 * - PB4 = M0PWM2 (Generator 1, Output A) -> Left Motor  (PWM_CHANNEL_LEFT)
 * - PB6 = M0PWM0 (Generator 0, Output A) -> Right Motor (PWM_CHANNEL_RIGHT)
 ******************************************************************************/

#include "Platform_Types.h"
#include "tm4c123gh6pm_registers.h"
#include "PWM.h"
#include "pwm_private.h"
#include "Common_Macros.h"

/*******************************************************************************
 *                          Private Variables                                  *
 *******************************************************************************/

/* State tracking for each channel */
static PWM_StateType PWM_ChannelState[PWM_CHANNEL_MAX] = {
    PWM_STATE_UNINIT,  /* PWM_CHANNEL_LEFT */
    PWM_STATE_UNINIT   /* PWM_CHANNEL_RIGHT */
};

/*******************************************************************************
 *                          Private Functions                                  *
 *******************************************************************************/

/**
 * @brief  Select a channel's generator register set.
 */
static PWM_GeneratorRegsType PWM_GetGeneratorRegs(PWM_ChannelType channel)
{
    PWM_GeneratorRegsType regs;

    if (channel == PWM_CHANNEL_LEFT) {
        /* PB4 = M0PWM2 = Generator 1 */
        regs.ctl = &PWM0_CTL1;
        regs.load = &PWM0_LOAD1;
        regs.cmpa = &PWM0_CMPA1;
        regs.gena = &PWM0_GENA1;
        regs.outputEnableBitPos = PWM_PRIV_OUTPUT_ENABLE_M0PWM2_POS;
    } else {
        /* PB6 = M0PWM0 = Generator 0 */
        regs.ctl = &PWM0_CTL0;
        regs.load = &PWM0_LOAD0;
        regs.cmpa = &PWM0_CMPA0;
        regs.gena = &PWM0_GENA0;
        regs.outputEnableBitPos = PWM_PRIV_OUTPUT_ENABLE_M0PWM0_POS;
    }

    return regs;
}

/**
 * @brief  Configure and start one channel's generator at 0% duty (stopped).
 * @note   Module-level setup (clock enable/ready-wait/divider) must already
 *         be done before this is called - see PWM_Init().
 */
static void PWM_InitChannel(PWM_ChannelType channel)
{
    PWM_GeneratorRegsType regs = PWM_GetGeneratorRegs(channel);

    /* Disable + clear the generator while it's configured */
    *regs.ctl = 0x00UL;

    /* Up/down count mode, keep counting while CPU is halted in debug */
    *regs.ctl |= (PWM_PRIV_GEN_CTL_MODE_MASK | PWM_PRIV_GEN_CTL_DEBUG_MASK);

    /* Period, derived from pwm_cfg.h's clock+frequency (see PWM_PERIOD_LOAD) */
    *regs.load = PWM_PERIOD_LOAD;

    /* Initial duty = 0% (stopped): CMPA = LOAD in this GENA configuration */
    *regs.cmpa = PWM_PERIOD_LOAD;

    /* Output-A actions - verified correct on hardware, see pwm_private.h */
    *regs.gena = PWM_PRIV_GENA_CONFIG;

    /* Enable the generator's counter */
    *regs.ctl |= PWM_PRIV_GEN_CTL_ENABLE_MASK;

    /* Gate this generator's output onto its pin */
    PWM0_ENABLE |= (1UL << regs.outputEnableBitPos);

    PWM_ChannelState[channel] = PWM_STATE_RUNNING;
}

/*******************************************************************************
 *                          Initialization Functions                           *
 *******************************************************************************/

void PWM_Init(void)
{
    /* Enable PWM0 module clock (one-time, module-level) */
    SYSCTL_RCGCPWM_REG |= PWM_PRIV_RCGCPWM_PWM0_MASK;

    /* Wait until the PWM0 peripheral clock is actually ready, instead of a
     * heuristic busy-loop - mirrors the PORT driver's RCGCGPIO/PRGPIO wait
     * (PWM_AUDIT.md Section 1). */
    while ((SYSCTL_PRPWM_REG & PWM_PRIV_PRPWM_PWM0_MASK) != PWM_PRIV_PRPWM_PWM0_MASK);

    /* Disable PWM clock divider - PWM clock = system clock, undivided */
    SYSCTL_RCC_REG &= ~PWM_PRIV_RCC_USEPWMDIV_MASK;

    /* Per-channel generator setup - both channels, every boot */
    PWM_InitChannel(PWM_CHANNEL_LEFT);
    PWM_InitChannel(PWM_CHANNEL_RIGHT);
}

/*******************************************************************************
 *                          Duty Cycle Control                                 *
 *******************************************************************************/

PWM_StatusType PWM_SetDuty(PWM_ChannelType channel, float32 percent)
{
    PWM_GeneratorRegsType regs;
    uint16 compareValue;

    if (channel >= PWM_CHANNEL_MAX) {
        return PWM_ERROR_INVALID_CHANNEL;
    }

    if (PWM_ChannelState[channel] == PWM_STATE_UNINIT) {
        return PWM_ERROR_NOT_INITIALIZED;
    }

    /* Clamp out-of-range duty rather than reject it - matches the original,
     * hardware-verified behavior (see PWM_REDESIGN.md for why
     * PWM_ERROR_INVALID_DUTY was removed rather than wired up). */
    if (percent < 0.0f)   percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    /* CMPA = LOAD -> output held Low all period  (0% physical duty)
     * CMPA = 0    -> output held High all period (100% physical duty)
     * Confirmed on hardware: 0%=stopped, 100%=full speed, monotonic between. */
    if (percent >= 100.0f) {
        /* CMPA must never be 0: at CMPA=0 the output is held statically
         * HIGH for the entire period with zero falling edges. The
         * H-bridge's bootstrap high-side gate driver only recharges on
         * that falling edge - with none, it bleeds down and the motor
         * stalls on a cold start (measured on hardware: 100% commanded
         * from rest stalls/buzzes without turning; 99% starts and runs
         * normally). CMPA=1 keeps one low tick per period - the minimum
         * that still recharges the bootstrap - capping the real max
         * physical duty to (LOAD-1)/LOAD = 399/400 = 99.75%, which is
         * intentional and safe. */
        compareValue = 1U;
    } else if (percent <= 0.0f) {
        compareValue = (uint16)PWM_PERIOD_LOAD;
    } else {
        /* Round to nearest rather than truncate, so 50% lands exactly on LOAD/2 */
        compareValue = (uint16)PWM_PERIOD_LOAD -
                        (uint16)(((percent / 100.0f) * (float32)PWM_PERIOD_LOAD) + 0.5f);

        /* Same bootstrap-starvation invariant as above: rounding can also
         * land exactly on 0 for inputs just under 100% (e.g. 99.9% at
         * LOAD=400) - not only the literal percent>=100 case. */
        if (compareValue == 0U) {
            compareValue = 1U;
        }
    }

    regs = PWM_GetGeneratorRegs(channel);
    *regs.cmpa = compareValue;

    return PWM_OK;
}

/*******************************************************************************
 *                          Motor Control Convenience Functions                *
 *******************************************************************************/

PWM_StatusType PWM_SetBothMotors(float32 percent) {
    PWM_StatusType status;

    status = PWM_SetDuty(PWM_CHANNEL_LEFT, percent);
    if (status != PWM_OK) {
        return status;
    }

    return PWM_SetDuty(PWM_CHANNEL_RIGHT, percent);
}

PWM_StatusType PWM_SetMotors(float32 leftPercent, float32 rightPercent) {
    PWM_StatusType status;

    status = PWM_SetDuty(PWM_CHANNEL_LEFT, leftPercent);
    if (status != PWM_OK) {
        return status;
    }

    return PWM_SetDuty(PWM_CHANNEL_RIGHT, rightPercent);
}

PWM_StatusType PWM_StopMotors(void) {
    /* Set both motors to 0% duty cycle */
    return PWM_SetBothMotors(0.0f);
}

/*******************************************************************************
 *                          State Functions                                    *
 *******************************************************************************/

PWM_StateType PWM_GetState(PWM_ChannelType channel) {
    if (channel >= PWM_CHANNEL_MAX) {
        return PWM_STATE_UNINIT;
    }

    return PWM_ChannelState[channel];
}

/*******************************************************************************
 *                          Synchronization                                    *
 *******************************************************************************/

void PWM_TriggerGlobalSync(void) {
    /* Trigger global synchronization for both generators */
    PWM0_CTL |= (1UL << PWM_PRIV_MODULE_CTL_GLOBALSYNC0_POS);
    PWM0_CTL |= (1UL << PWM_PRIV_MODULE_CTL_GLOBALSYNC1_POS);
}
