/******************************************************************************
 * Servo Driver (HAL)
 *
 * Steering-servo hardware abstraction. Speaks angle/center/disable and drives
 * the hardware ONLY through the Timer-PWM MCAL (timer_pwm.h). Contains ZERO
 * register access - mirrors how Motor.c drives PWM.c via PWM_SetDuty without
 * ever touching PWM0_* registers.
 ******************************************************************************/

#include "Platform_Types.h"
#include "servo.h"
#include "timer_pwm.h"      /* the MCAL - the ONLY way this HAL reaches hardware */

/*******************************************************************************
 *                          Private Variables                                  *
 *******************************************************************************/

/* Last COMMANDED steering angle (rad). Open-loop: this is the command, not a
 * measured pot angle. Defaults to 0 (center) until the first Servo_SetAngleRad. */
static float32 Servo_CommandedAngleRad = 0.0f;

/*******************************************************************************
 *                          Private Functions                                  *
 *******************************************************************************/

/**
 * @brief  Map a steering angle (rad) to a servo pulse width (us), clamped to the
 *         travel endpoints. CALIBRATED asymmetric map (re-measured 2026-07-30):
 *         each side uses its own linear scale that hits its endpoint exactly
 *         (servo_cfg.h / servo_feedback_cfg.h). Native sign convention is
 *         UNCHANGED: +angle -> RIGHT (higher us), -angle -> LEFT (lower us).
 *         steering_control negates once before calling, so a REP-103 +left
 *         command lands on the LEFT branch.
 *
 *         !! DO NOT MERGE THE TWO BRANCHES. They are asymmetric in BOTH terms:
 *              LEFT : angle 0.2421 rad over an 850us span (1450 -> 600)
 *              RIGHT: angle 0.3037 rad over a 1050us span (1450 -> 2500)
 *         They look mergeable whenever the two angle constants happen to match,
 *         but the pulse spans never have. Collapsing them mis-scales one side by
 *         ~24%. Each branch reaches its endpoint exactly when the commanded angle
 *         equals that side's magnitude - which is also why SC_LIMIT_LEFT/RIGHT_RAD
 *         in steering_control.c must mirror these two constants.
 */
static uint16 Servo_AngleRadToPulseUs(float32 angle_rad)
{
    float32 pulseUs;

    /* R3-1 (second, INDEPENDENT line of defence): never map a non-finite angle.
     * steering_control already rejects these, but this HAL must not trust its
     * caller - a direct Servo_SetAngleRad() user would otherwise reach
     * (uint16)(NaN + 0.5f) below, which is UNDEFINED BEHAVIOUR in C and yields 0
     * on Cortex-M4 (VCVT) -> TimerPWM clamps 0 up to MIN -> a full-LEFT lock.
     * Note both branch tests and both clamp comparisons below are false for NaN,
     * so nothing downstream would catch it.
     *
     * Returns CENTER, the one pulse this layer knows is always safe and defined.
     * (Unlike steering_control this function is pure and has no notion of a "last
     * valid" angle to hold, so holding is not available here; the layer that DOES
     * have that state implements the hold.)
     * -Inf/+Inf are caught by the same test: they are not equal to themselves
     * after the subtraction-free bounded compare below. Hand-rolled rather than
     * isfinite() because this is a -ffreestanding -fno-builtin build. */
    if ((angle_rad != angle_rad) ||                 /* NaN                       */
        (angle_rad >  SERVO_IMPLAUSIBLE_RAD) ||     /* +Inf / absurd             */
        (angle_rad < -SERVO_IMPLAUSIBLE_RAD))       /* -Inf / absurd             */
    {
        return (uint16)SERVO_CENTER_PULSE_US;
    }

    if (angle_rad >= 0.0f)
    {
        /* RIGHT half: 0..+RIGHT_MAX rad -> CENTER..MAX pulse (1650..2480). */
        pulseUs = (float32)SERVO_CENTER_PULSE_US
                + (angle_rad / SERVO_MAX_ANGLE_RIGHT_RAD)
                  * ((float32)SERVO_MAX_PULSE_US - (float32)SERVO_CENTER_PULSE_US);
    }
    else
    {
        /* LEFT half: 0..-LEFT_MAX rad -> CENTER..MIN pulse (1650..1020).
         * angle_rad is negative here, so this term subtracts from center. */
        pulseUs = (float32)SERVO_CENTER_PULSE_US
                + (angle_rad / SERVO_MAX_ANGLE_LEFT_RAD)
                  * ((float32)SERVO_CENTER_PULSE_US - (float32)SERVO_MIN_PULSE_US);
    }

    /* Final safety clamp to the travel endpoints (UNCHANGED), as float before
     * the uint16 cast so negatives can't wrap. */
    if (pulseUs < (float32)SERVO_MIN_PULSE_US)
    {
        pulseUs = (float32)SERVO_MIN_PULSE_US;
    }
    else if (pulseUs > (float32)SERVO_MAX_PULSE_US)
    {
        pulseUs = (float32)SERVO_MAX_PULSE_US;
    }
    else
    {
        /* in range */
    }

    return (uint16)(pulseUs + 0.5f);
}

/*******************************************************************************
 *                          Initialization                                     *
 *******************************************************************************/

void Servo_Init(void)
{
    /* Bring up the timer PWM primitive (it loads a neutral midpoint pulse and
     * starts the output), then assert servo "center" semantics explicitly. */
    TimerPWM_Init();
    Servo_Center();
}

/*******************************************************************************
 *                          Actuation                                          *
 *******************************************************************************/

Std_ReturnType Servo_SetAngleRad(float32 angle_rad)
{
    /* Store the command first so Servo_GetCurrentAngle reflects intent even
     * if the MCAL CLAMPS the resulting pulse. Clamping is not failure - the
     * servo is still driven, just to the endpoint - so "intent" is the right
     * thing to publish here and this ordering is unchanged. */
    Servo_CommandedAngleRad = angle_rad;

    /* S10-3: propagate the MCAL verdict instead of discarding it. The only
     * current failure is TIMER_PWM_ERROR_NOT_INITIALIZED, which writes no
     * hardware at all - so the caller learns that no pulse was produced and can
     * decline to record the angle as actuated. */
    if (TimerPWM_SetPulseUs(Servo_AngleRadToPulseUs(angle_rad)) != TIMER_PWM_OK)
    {
        return E_NOT_OK;
    }

    return E_OK;
}

float32 Servo_GetCurrentAngle(void)
{
    return Servo_CommandedAngleRad;
}

/*******************************************************************************
 *                          Failsafe States                                    *
 *******************************************************************************/

Std_ReturnType Servo_Center(void)
{
    /* Safe known state for a command-timeout: drive the explicit center pulse
     * (independent of the linear map) and record the command as 0 rad. The
     * pulse keeps running - the servo holds straight. */
    Servo_CommandedAngleRad = 0.0f;

    if (TimerPWM_SetPulseUs((uint16)SERVO_CENTER_PULSE_US) != TIMER_PWM_OK)
    {
        return E_NOT_OK;
    }

    return E_OK;
}

void Servo_Disable(void)
{
    /* Feedback-untrustworthy / pre-init: cut the output entirely. The last
     * commanded angle is left intact (it no longer reflects a live pulse). */
    TimerPWM_Stop();
}
