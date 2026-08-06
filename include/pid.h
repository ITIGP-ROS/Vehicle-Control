/**
 * @file    pid.h
 * @brief   PID Controller Public Interface
 * @details Discrete-time PID controller, parallel form (separate P, I,
 *          filtered-D terms), with setpoint weighting,
 *          derivative-on-measurement, and functional back-calculation
 *          anti-windup. See PID_REDESIGN_REPORT.md for the full
 *          before/after and the reasoning behind the parallel-form
 *          structure.
 *
 * @note    Each term is individually Tustin (bilinear-transform)
 *          discretized: the integral via trapezoidal integration of
 *          the true error, the derivative via a first-order filtered-
 *          derivative recursion (filter pole at N) acting on
 *          measurement (not error), to eliminate derivative kick on
 *          setpoint changes.
 *
 * Usage Example:
 * @code
 *     PID_HandleType motorPID;
 *     PID_ConfigType config = {
 *         .gains = { .Kp = 2.0f, .Ki = 0.5f, .Kd = 0.1f },
 *         .Ts = 0.01f,
 *         .N = 10.0f,
 *         .limits = { .min = -100.0f, .max = 100.0f }
 *     };
 *
 *     PID_Init(&motorPID, &config);
 *
 *     // In control loop:
 *     float32 output = PID_Update(&motorPID, setpoint, measurement);
 *     PWM_SetDutyPercent(MOTOR_A, output);
 * @endcode
 */

#ifndef PID_H
#define PID_H

#include "Platform_Types.h"
#include "pid_cfg.h"
#include "pid_types.h"

/*******************************************************************************
 * API Function Prototypes
 ******************************************************************************/

/**
 * @brief   Initialize PID controller with configuration
 * @param   pid     Pointer to PID handle
 * @param   config  Pointer to configuration structure
 * @return  PID_OK on success, error code otherwise
 */
PID_StatusType PID_Init(PID_HandleType *pid, const PID_ConfigType *config);

/**
 * @brief   Initialize PID controller with individual parameters (legacy compatible)
 * @param   pid     Pointer to PID handle
 * @param   Kp      Proportional gain
 * @param   Ki      Integral gain
 * @param   Kd      Derivative gain
 * @param   Ts      Sampling time in seconds
 * @param   N       Derivative filter coefficient
 * @return  PID_OK on success, error code otherwise
 */
PID_StatusType PID_InitParams(PID_HandleType *pid, 
                              float32 Kp, float32 Ki, float32 Kd, 
                              float32 Ts, float32 N);

/**
 * @brief   Compute PID control output
 * @param   pid          Pointer to PID handle
 * @param   setpoint     Desired target value
 * @param   measurement  Current measured value
 * @return  Control output (clamped to configured limits)
 * @note    error = setpoint - measurement is computed internally and
 *          drives the integral term (always true, unweighted error).
 *          The proportional term uses (B*setpoint - measurement) and
 *          the derivative term uses (C*setpoint - measurement) - see
 *          PID_SETPOINT_WEIGHT_B/_C in pid_cfg.h. With the default
 *          B=1, C=0, this reduces to plain error for P and pure
 *          derivative-on-measurement for D.
 * @note    If the internally computed raw output is non-finite
 *          (NaN/Inf), the controller latches into PID_STATE_FAULT_NAN,
 *          freezes all state, and returns the last known-good output
 *          on this and every subsequent call until PID_Reset() is
 *          called.
 */
float32 PID_Update(PID_HandleType *pid, float32 setpoint, float32 measurement);

/**
 * @brief   Reset controller state (clear history)
 * @param   pid     Pointer to PID handle
 * @return  PID_OK on success, error code otherwise
 * @note    Call when: changing setpoint significantly, switching modes,
 *          after emergency stop, starting a new control sequence, or
 *          to clear a PID_STATE_FAULT_NAN latch and resume control
 *          without a full re-Init.
 */
PID_StatusType PID_Reset(PID_HandleType *pid);

/**
 * @brief   Update PID gains at runtime
 * @param   pid     Pointer to PID handle
 * @param   gains   Pointer to new gains structure
 * @return  PID_OK on success, error code otherwise
 * @note    Recalculates internal coefficients, does not reset state
 */
PID_StatusType PID_SetGains(PID_HandleType *pid, const PID_GainsType *gains);

/**
 * @brief   Update output limits at runtime
 * @param   pid     Pointer to PID handle
 * @param   limits  Pointer to new limits structure
 * @return  PID_OK on success, error code otherwise
 */
PID_StatusType PID_SetLimits(PID_HandleType *pid, const PID_LimitsType *limits);

/**
 * @brief   Get current PID gains
 * @param   pid     Pointer to PID handle
 * @param   gains   Pointer to store current gains
 * @return  PID_OK on success, PID_ERROR_NOT_INITIALIZED if pid is
 *          uninitialized, error code otherwise
 */
PID_StatusType PID_GetGains(const PID_HandleType *pid, PID_GainsType *gains);

/**
 * @brief   Get current controller state
 * @param   pid     Pointer to PID handle
 * @return  Current state (UNINIT, READY, RUNNING, SATURATED_HIGH,
 *          SATURATED_LOW, or FAULT_NAN)
 */
PID_StateType PID_GetState(const PID_HandleType *pid);

/**
 * @brief   Check if controller output is saturated
 * @param   pid     Pointer to PID handle
 * @return  TRUE if saturated (high or low), FALSE otherwise
 */
boolean PID_IsSaturated(const PID_HandleType *pid);

/**
 * @brief   Get debug/telemetry data from last update
 * @param   pid     Pointer to PID handle
 * @param   debug   Pointer to store debug data
 * @return  PID_OK on success, PID_ERROR_NOT_INITIALIZED if pid is
 *          uninitialized, error code otherwise
 * @note    Useful for tuning and diagnostics over UART to Pi. All
 *          fields are exact values from the most recently completed
 *          PID_Update() cycle (not approximations).
 */
PID_StatusType PID_GetDebugInfo(const PID_HandleType *pid, PID_DebugType *debug);

/*******************************************************************************
 * Convenience Macros
 ******************************************************************************/

/** Quick initialization with default limits */
#define PID_INIT_SIMPLE(pid, kp, ki, kd, ts) \
    PID_InitParams((pid), (kp), (ki), (kd), (ts), PID_DERIVATIVE_FILTER_N)

/** Check if controller is ready to use */
#define PID_IS_READY(pid) \
    ((pid)->status != PID_STATE_UNINIT)

#endif /* PID_H */
