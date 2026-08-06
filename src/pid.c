/**
 * @file    pid.c
 * @brief   PID Controller Implementation - parallel form
 * @details Discrete-time PID, separate Tustin-discretized P/I/filtered-D
 *          terms, with setpoint weighting, derivative-on-measurement,
 *          functional back-calculation anti-windup, and NaN/Inf
 *          containment.
 *
 *          See PID_REDESIGN_REPORT.md for the full rationale, including
 *          why this moved from a single combined 2nd-order direct-form
 *          recursion to this parallel structure: setpoint weighting and
 *          derivative-on-measurement require P, I, and D to each act on
 *          a different input signal, which a single combined difference
 *          equation (derived assuming one common error input) cannot
 *          express.
 *
 * Theory (each term discretized independently via Tustin/bilinear,
 * s = (2/Ts)*(z-1)/(z+1)):
 *
 *   P[k] = Kp * (B*setpoint[k] - measurement[k])
 *
 *   I[k] = I[k-1] + (Ki*Ts/2) * (e[k] + e[k-1])         (trapezoidal)
 *          where e[k] = setpoint[k] - measurement[k]     (TRUE error,
 *          never setpoint-weighted, so steady-state error -> 0)
 *
 *   D[k] = dPole*D[k-1] + dGain*(dIn[k] - dIn[k-1])
 *          where dIn[k] = C*setpoint[k] - measurement[k]
 *          dGain = 2*Kd*N/(2+N*Ts), dPole = (2-N*Ts)/(2+N*Ts)
 *          (Tustin discretization of the continuous filtered
 *          derivative Kd*s/(1+s/N) acting on dIn; C=0 default makes
 *          this pure derivative-on-measurement)
 *
 *   u_raw[k] = P[k] + I[k] + D[k]
 *   u_limited[k] = clamp(u_raw[k])
 *
 * Anti-windup (back-calculation): satError = u_limited - u_raw is fed
 * back into the integrator (I[k] += Kc*satError) so it unwinds at a
 * controlled rate instead of stalling for a full cycle's worth of
 * overshoot while saturated.
 */

#include "pid.h"
#include <float.h>

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static void PID_CalculateCoefficients(PID_HandleType *pid);
static float32 PID_Clamp(float32 value, float32 min, float32 max);
static boolean PID_IsFinite(float32 value);

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * @brief Initialize PID controller with configuration structure
 */
PID_StatusType PID_Init(PID_HandleType *pid, const PID_ConfigType *config)
{
    if ((pid == NULL) || (config == NULL))
    {
        return PID_ERROR_NULL_PTR;
    }

#if (PID_PARAM_VALIDATION_ENABLE == 1U)
    /* Validate sampling time */
    if ((config->Ts < PID_TS_MIN) || (config->Ts > PID_TS_MAX))
    {
        return PID_ERROR_INVALID_PARAM;
    }

    /* Validate derivative filter coefficient */
    if (config->N <= 0.0f)
    {
        return PID_ERROR_INVALID_PARAM;
    }

    /* Validate limits */
    if (config->limits.min >= config->limits.max)
    {
        return PID_ERROR_INVALID_PARAM;
    }
#endif

    /* Copy configuration */
    pid->config.gains.Kp = config->gains.Kp;
    pid->config.gains.Ki = config->gains.Ki;
    pid->config.gains.Kd = config->gains.Kd;
    pid->config.Ts = config->Ts;
    pid->config.N = config->N;
    pid->config.limits.min = config->limits.min;
    pid->config.limits.max = config->limits.max;

    /* Calculate precomputed coefficients */
    PID_CalculateCoefficients(pid);

    /* Initialize state variables */
    pid->state.e_k1 = 0.0f;
    pid->state.dInput_k1 = 0.0f;
    pid->state.integral = 0.0f;
    pid->state.dFilterState = 0.0f;
    pid->state.lastP = 0.0f;
    pid->state.lastURaw = 0.0f;
    pid->state.lastOutput = 0.0f;

    pid->status = PID_STATE_READY;

    return PID_OK;
}

/**
 * @brief Initialize with individual parameters (legacy compatible)
 */
PID_StatusType PID_InitParams(PID_HandleType *pid,
                              float32 Kp, float32 Ki, float32 Kd,
                              float32 Ts, float32 N)
{
    PID_ConfigType config;

    config.gains.Kp = Kp;
    config.gains.Ki = Ki;
    config.gains.Kd = Kd;
    config.Ts = Ts;
    config.N = N;
    config.limits.min = PID_OUTPUT_MIN_DEFAULT;
    config.limits.max = PID_OUTPUT_MAX_DEFAULT;

    return PID_Init(pid, &config);
}

/**
 * @brief Compute PID control output (parallel P/I/filtered-D, with
 *        setpoint weighting, derivative-on-measurement, working
 *        back-calculation anti-windup, and NaN/Inf containment)
 */
float32 PID_Update(PID_HandleType *pid, float32 setpoint, float32 measurement)
{
    float32 error;       /* TRUE error: setpoint-measurement, drives I only */
    float32 pInput;      /* signal driving P: (B*setpoint - measurement) */
    float32 dInput;      /* signal driving D: (C*setpoint - measurement) */
    float32 pTerm;
    float32 dTerm;
    float32 integralCandidate; /* I[k] before this cycle's anti-windup correction */
    float32 u_raw;
    float32 u_limited;

    if (pid == NULL)
    {
        return 0.0f;
    }

    if (pid->status == PID_STATE_UNINIT)
    {
        return 0.0f;
    }

    if (pid->status == PID_STATE_FAULT_NAN)
    {
        /* Latched fault: hold the last known-good output and freeze all
         * state until PID_Reset() is explicitly called (see pid.h). */
        return pid->state.lastOutput;
    }

    error = setpoint - measurement;

#if (PID_SETPOINT_WEIGHT_ENABLE == 1U)
    pInput = (PID_SETPOINT_WEIGHT_B * setpoint) - measurement;
    dInput = (PID_SETPOINT_WEIGHT_C * setpoint) - measurement;
#else
    pInput = error;
    dInput = -measurement;
#endif

    /* P: stateless, recomputed fresh every cycle */
    pTerm = pid->config.gains.Kp * pInput;

    /* I: Tustin/trapezoidal integration of the TRUE error (never weighted) */
    integralCandidate = pid->state.integral + (pid->coeffs.kiTrap * (error + pid->state.e_k1));

    /* D: Tustin-discretized filtered derivative acting on dInput -
     * derivative-on-measurement when PID_SETPOINT_WEIGHT_C is 0 (default) */
    dTerm = (pid->coeffs.dPole * pid->state.dFilterState)
          + (pid->coeffs.dGain * (dInput - pid->state.dInput_k1));

    u_raw = pTerm + integralCandidate + dTerm;

    /* --- NaN/Inf containment --- */
    if (!PID_IsFinite(u_raw))
    {
        /* Do not write the non-finite value into any state. Latch the
         * fault and hold the last known-good (always-finite) output. */
        pid->status = PID_STATE_FAULT_NAN;
        return pid->state.lastOutput;
    }

    u_limited = PID_Clamp(u_raw, pid->config.limits.min, pid->config.limits.max);

    /* Update controller state */
    if (u_limited >= pid->config.limits.max)
    {
        pid->status = PID_STATE_SATURATED_HIGH;
    }
    else if (u_limited <= pid->config.limits.min)
    {
        pid->status = PID_STATE_SATURATED_LOW;
    }
    else
    {
        pid->status = PID_STATE_RUNNING;
    }

    /* Back-calculation anti-windup: correct the integrator specifically
     * (not P or D) by the saturation error, scaled by Kc, so it unwinds
     * at a controlled rate instead of stalling for a full cycle's
     * overshoot. satError is scoped to this block - when anti-windup is
     * disabled (the default), it is neither computed nor left as an
     * unused variable. */
#if (PID_ANTIWINDUP_ENABLE == 1U)
    {
        float32 satError = u_limited - u_raw;
        integralCandidate += pid->coeffs.Kc * satError;
    }
#endif

    /* Commit state for next cycle */
    pid->state.integral = integralCandidate;
    pid->state.e_k1 = error;
    pid->state.dInput_k1 = dInput;
    pid->state.dFilterState = dTerm;
    pid->state.lastP = pTerm;
    pid->state.lastURaw = u_raw;
    pid->state.lastOutput = u_limited;

    return u_limited;
}

/**
 * @brief Reset controller state
 */
PID_StatusType PID_Reset(PID_HandleType *pid)
{
    if (pid == NULL)
    {
        return PID_ERROR_NULL_PTR;
    }

    if (pid->status == PID_STATE_UNINIT)
    {
        return PID_ERROR_NOT_INITIALIZED;
    }

    /* Clear all state variables (also clears a PID_STATE_FAULT_NAN latch) */
    pid->state.e_k1 = 0.0f;
    pid->state.dInput_k1 = 0.0f;
    pid->state.integral = 0.0f;
    pid->state.dFilterState = 0.0f;
    pid->state.lastP = 0.0f;
    pid->state.lastURaw = 0.0f;
    pid->state.lastOutput = 0.0f;

    pid->status = PID_STATE_READY;

    return PID_OK;
}

/**
 * @brief Update PID gains at runtime
 */
PID_StatusType PID_SetGains(PID_HandleType *pid, const PID_GainsType *gains)
{
    if ((pid == NULL) || (gains == NULL))
    {
        return PID_ERROR_NULL_PTR;
    }

    if (pid->status == PID_STATE_UNINIT)
    {
        return PID_ERROR_NOT_INITIALIZED;
    }

    /* Update gains */
    pid->config.gains.Kp = gains->Kp;
    pid->config.gains.Ki = gains->Ki;
    pid->config.gains.Kd = gains->Kd;

    /* Recalculate coefficients with new gains */
    PID_CalculateCoefficients(pid);

    return PID_OK;
}

/**
 * @brief Update output limits at runtime
 */
PID_StatusType PID_SetLimits(PID_HandleType *pid, const PID_LimitsType *limits)
{
    if ((pid == NULL) || (limits == NULL))
    {
        return PID_ERROR_NULL_PTR;
    }

    if (pid->status == PID_STATE_UNINIT)
    {
        return PID_ERROR_NOT_INITIALIZED;
    }

#if (PID_PARAM_VALIDATION_ENABLE == 1U)
    if (limits->min >= limits->max)
    {
        return PID_ERROR_INVALID_PARAM;
    }
#endif

    pid->config.limits.min = limits->min;
    pid->config.limits.max = limits->max;

    return PID_OK;
}

/**
 * @brief Get current PID gains
 */
PID_StatusType PID_GetGains(const PID_HandleType *pid, PID_GainsType *gains)
{
    if ((pid == NULL) || (gains == NULL))
    {
        return PID_ERROR_NULL_PTR;
    }

    if (pid->status == PID_STATE_UNINIT)
    {
        return PID_ERROR_NOT_INITIALIZED;
    }

    gains->Kp = pid->config.gains.Kp;
    gains->Ki = pid->config.gains.Ki;
    gains->Kd = pid->config.gains.Kd;

    return PID_OK;
}

/**
 * @brief Get current controller state
 */
PID_StateType PID_GetState(const PID_HandleType *pid)
{
    if (pid == NULL)
    {
        return PID_STATE_UNINIT;
    }

    return pid->status;
}

/**
 * @brief Check if controller is saturated
 */
boolean PID_IsSaturated(const PID_HandleType *pid)
{
    if (pid == NULL)
    {
        return FALSE;
    }

    return ((pid->status == PID_STATE_SATURATED_HIGH) ||
            (pid->status == PID_STATE_SATURATED_LOW));
}

/**
 * @brief Get debug information
 */
PID_StatusType PID_GetDebugInfo(const PID_HandleType *pid, PID_DebugType *debug)
{
    if ((pid == NULL) || (debug == NULL))
    {
        return PID_ERROR_NULL_PTR;
    }

    if (pid->status == PID_STATE_UNINIT)
    {
        return PID_ERROR_NOT_INITIALIZED;
    }

    /* All fields are exact values from the most recently completed
     * PID_Update() cycle - no longer approximations (see
     * PID_REDESIGN_REPORT.md). */
    debug->error = pid->state.e_k1;
    debug->pTerm = pid->state.lastP;
    debug->iTerm = pid->state.integral;
    debug->dTerm = pid->state.dFilterState;
    debug->outputRaw = pid->state.lastURaw;
    debug->outputLimited = pid->state.lastOutput;
    debug->state = pid->status;

    return PID_OK;
}

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/**
 * @brief Calculate precomputed coefficients from gains and parameters
 *        (parallel-form: trapezoidal integral coefficient, filtered-
 *        derivative gain/pole, anti-windup gain)
 */
static void PID_CalculateCoefficients(PID_HandleType *pid)
{
    float32 Kd = pid->config.gains.Kd;
    float32 Ki = pid->config.gains.Ki;
    float32 Ts = pid->config.Ts;
    float32 N = pid->config.N;

    float32 NTs = N * Ts;

    /* Trapezoidal (Tustin) integration coefficient:
     * I[k] = I[k-1] + kiTrap*(e[k]+e[k-1]) */
    pid->coeffs.kiTrap = Ki * Ts * 0.5f;

    /* Tustin discretization of the continuous filtered derivative
     * Kd*s/(1+s/N): D[k] = dPole*D[k-1] + dGain*(dIn[k]-dIn[k-1]) */
    pid->coeffs.dGain = (2.0f * Kd * N) / (2.0f + NTs);
    pid->coeffs.dPole = (2.0f - NTs) / (2.0f + NTs);

#if (PID_ANTIWINDUP_ENABLE == 1U)
    /* Anti-windup gain (Kc)
     * Higher Kc = faster recovery from saturation
     */
    pid->coeffs.Kc = PID_ANTIWINDUP_KC_DEFAULT;
#else
    pid->coeffs.Kc = 0.0f;
#endif
}

/**
 * @brief Clamp value to range [min, max]
 */
static float32 PID_Clamp(float32 value, float32 min, float32 max)
{
    if (value > max)
    {
        return max;
    }
    else if (value < min)
    {
        return min;
    }
    else
    {
        return value;
    }
}

/**
 * @brief Check whether a float32 value is finite (neither NaN nor Inf)
 * @note  Implemented without <math.h> isnan()/isinf() to avoid relying
 *        on library function linkage under this project's freestanding
 *        build (-ffreestanding -fno-builtin, see platformio.ini).
 *        <float.h> is header-only (FLT_MAX is a preprocessor constant),
 *        so it carries no link-time risk.
 *        - NaN: per IEEE-754, a NaN never compares equal to itself.
 *        - Inf: any finite float32 value lies within [-FLT_MAX, FLT_MAX];
 *          +-Inf falls strictly outside that range.
 */
static boolean PID_IsFinite(float32 value)
{
    if (value != value)
    {
        return FALSE;
    }

    if ((value > FLT_MAX) || (value < -FLT_MAX))
    {
        return FALSE;
    }

    return TRUE;
}
