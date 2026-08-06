/**
 * @file    pid_types.h
 * @brief   PID Controller Type Definitions
 * @details Data types and structures for PID controller
 */

#ifndef PID_TYPES_H
#define PID_TYPES_H

#include "Platform_Types.h"
#include "Std_Types.h"
/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/**
 * @brief PID Controller Status
 */
typedef enum
{
    PID_OK = 0U,                /**< Operation successful */
    PID_ERROR_NULL_PTR,         /**< Null pointer passed */
    PID_ERROR_INVALID_PARAM,    /**< Invalid parameter value */
    PID_ERROR_NOT_INITIALIZED   /**< Controller not initialized */
} PID_StatusType;

/**
 * @brief PID Controller State
 */
typedef enum
{
    PID_STATE_UNINIT = 0U,      /**< Not initialized */
    PID_STATE_READY,            /**< Initialized and ready */
    PID_STATE_RUNNING,          /**< Actively controlling */
    PID_STATE_SATURATED_HIGH,   /**< Output at upper limit */
    PID_STATE_SATURATED_LOW,    /**< Output at lower limit */
    PID_STATE_FAULT_NAN         /**< u_raw was non-finite (NaN/Inf); output
                                  *   latched at last known-good value, all
                                  *   state frozen. Call PID_Update() does
                                  *   nothing while latched - call
                                  *   PID_Reset() to clear and resume
                                  *   (no full re-Init required). */
} PID_StateType;

/**
 * @brief PID Gains Structure
 */
typedef struct
{
    float32 Kp;                 /**< Proportional gain */
    float32 Ki;                 /**< Integral gain */
    float32 Kd;                 /**< Derivative gain */
} PID_GainsType;

/**
 * @brief PID Output Limits Structure
 */
typedef struct
{
    float32 min;                /**< Minimum output value */
    float32 max;                /**< Maximum output value */
} PID_LimitsType;

/**
 * @brief PID Precomputed Coefficients (internal use)
 * @details Coefficients for the parallel-form (P + I + filtered-D)
 *          implementation. Precomputed during Init/SetGains to avoid
 *          runtime division. NOTE: this differs from the combined
 *          2nd-order direct-form coefficients used before the
 *          parallel-form redesign (see PID_REDESIGN_REPORT.md) -
 *          setpoint weighting and derivative-on-measurement require P,
 *          I, and D to each act on a different input signal, which the
 *          old single combined difference equation could not express.
 */
typedef struct
{
    float32 kiTrap;             /**< Ki*Ts/2 - trapezoidal (Tustin) integration
                                  *   coefficient: I[k]=I[k-1]+kiTrap*(e[k]+e[k-1]) */
    float32 dGain;               /**< 2*Kd*N/(2+N*Ts) - Tustin-discretized filtered-
                                  *   derivative gain coefficient */
    float32 dPole;               /**< (2-N*Ts)/(2+N*Ts) - Tustin-discretized filtered-
                                  *   derivative recursive pole coefficient */
    float32 Kc;                 /**< Anti-windup back-calculation gain (per-handle;
                                  *   defaults from PID_ANTIWINDUP_KC_DEFAULT) */
} PID_CoeffsType;

/**
 * @brief PID Internal State Variables (parallel form)
 */
typedef struct
{
    /* --- State read by the control computation --- */
    float32 e_k1;                /**< Previous TRUE error e[k-1] = setpoint-measurement
                                   *   (never setpoint-weighted) - feeds the trapezoidal
                                   *   integral term only */
    float32 dInput_k1;           /**< Previous derivative-term input
                                   *   (C*setpoint - measurement)[k-1] - feeds the
                                   *   filtered derivative term. Reduces to "previous
                                   *   measurement, negated" when C=0 (the recommended
                                   *   default), i.e. pure derivative-on-measurement. */
    float32 integral;            /**< I[k]: the genuine, explicit integral accumulator -
                                   *   Tustin/trapezoidal integration of the TRUE error,
                                   *   with the back-calculation anti-windup correction
                                   *   (Kc*satError) folded in each cycle. This IS the
                                   *   real integral term (see PID_GetDebugInfo). */
    float32 dFilterState;        /**< D[k]: the filtered-derivative term's own recursive
                                   *   output. This IS the real derivative term. */

    /* --- State retained only for exact debug/telemetry snapshotting;
     *     not read by the control computation itself --- */
    float32 lastP;                /**< P[k] from the most recently completed cycle
                                   *   (P is stateless/recomputed fresh every cycle) */
    float32 lastURaw;            /**< u_raw[k] (pre-clamp) from the most recently
                                   *   completed cycle */
    float32 lastOutput;          /**< u_limited[k] from the most recently completed
                                   *   cycle - always finite by construction; this is
                                   *   what PID_Update() returns while latched in
                                   *   PID_STATE_FAULT_NAN */
} PID_InternalStateType;

/**
 * @brief PID Controller Configuration
 */
typedef struct
{
    PID_GainsType gains;        /**< PID gains (Kp, Ki, Kd) */
    float32 Ts;                 /**< Sampling time in seconds */
    float32 N;                  /**< Derivative filter coefficient */
    PID_LimitsType limits;      /**< Output limits */
} PID_ConfigType;

/**
 * @brief PID Controller Handle
 * @details Complete controller instance with config, state, and coefficients
 */
typedef struct
{
    PID_ConfigType config;              /**< Controller configuration */
    PID_CoeffsType coeffs;              /**< Precomputed coefficients */
    PID_InternalStateType state;        /**< Internal state variables */
    PID_StateType status;               /**< Current controller state */
} PID_HandleType;

/**
 * @brief PID Debug/Telemetry Structure (optional)
 */
typedef struct
{
    float32 error;              /**< Current error */
    float32 pTerm;              /**< Proportional contribution */
    float32 iTerm;              /**< Integral contribution */
    float32 dTerm;              /**< Derivative contribution */
    float32 outputRaw;          /**< Output before limiting */
    float32 outputLimited;      /**< Output after limiting */
    PID_StateType state;        /**< Current state */
} PID_DebugType;

#endif /* PID_TYPES_H */
