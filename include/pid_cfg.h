/**
 * @file    pid_cfg.h
 * @brief   PID Controller Configuration
 * @details Compile-time configuration for PID controller behavior
 */

#ifndef PID_CFG_H
#define PID_CFG_H

/*******************************************************************************
 * Output Limits Configuration
 ******************************************************************************/

/** Enable bidirectional control (-limit to +limit) vs unidirectional (0 to +limit) */
#define PID_BIDIRECTIONAL_ENABLE    (1U)

/** Default output upper limit (percentage for PWM) */
#define PID_OUTPUT_MAX_DEFAULT      (100.0f)

/** Default output lower limit */
#if (PID_BIDIRECTIONAL_ENABLE == 1U)
#define PID_OUTPUT_MIN_DEFAULT      (-100.0f)
#else
#define PID_OUTPUT_MIN_DEFAULT      (0.0f)
#endif

/*******************************************************************************
 * Anti-Windup Configuration
 ******************************************************************************/

/** Enable back-calculation anti-windup (recommended).
 *  NOW FUNCTIONAL (see PID_REDESIGN_REPORT.md) - the integrator
 *  correction (Kc * satError) is folded into the explicit integral
 *  accumulator every cycle when this is 1U. Left at 0U as the shipped
 *  default; flip to 1U after integration testing. */
#define PID_ANTIWINDUP_ENABLE       (1U)

/** Back-calculation tracking time constant (Tt)
 *  Typical value: sqrt(Ti * Td) where Ti = Kp/Ki, Td = Kd/Kp
 *  If unsure, use Ts (sampling time) as a starting point
 *  Smaller = faster recovery from saturation
 *  Stored per-handle in PID_CoeffsType.Kc, not a hardcoded literal.
 */
#define PID_ANTIWINDUP_KC_DEFAULT   (1.0f)

/*******************************************************************************
 * Setpoint Weighting Configuration
 ******************************************************************************/

/** Enable setpoint weighting to reduce overshoot.
 *  NOW FUNCTIONAL (see PID_REDESIGN_REPORT.md) - when 1U, the P term
 *  uses (B*setpoint - measurement) and the D term uses
 *  (C*setpoint - measurement) instead of plain error/-measurement.
 *  When 0U, P uses true error and D is pure derivative-on-measurement
 *  (equivalent to B=1,C=0 anyway, given the defaults below). */
#define PID_SETPOINT_WEIGHT_ENABLE  (0U)

/** Proportional setpoint weight (0.0 to 1.0, typically 0.5-1.0) */
#define PID_SETPOINT_WEIGHT_B       (1.0f)

/** Derivative setpoint weight (0.0 to 1.0, typically 0.0).
 *  0.0 = pure derivative-on-measurement (recommended - eliminates
 *  derivative kick on setpoint changes). */
#define PID_SETPOINT_WEIGHT_C       (0.0f)

/*******************************************************************************
 * Derivative Filter Configuration
 ******************************************************************************/

/** Default derivative filter coefficient (N)
 *  Higher N = less filtering, faster response, more noise
 *  Typical range: 2 to 20, common value: 10
 */
#define PID_DERIVATIVE_FILTER_N     (10.0f)

/*******************************************************************************
 * Safety Configuration
 ******************************************************************************/

/** Maximum allowed sampling time (seconds) - sanity check */
#define PID_TS_MAX                  (1.0f)

/** Minimum allowed sampling time (seconds) - sanity check */
#define PID_TS_MIN                  (0.0001f)

/** Enable input validation in Init function */
#define PID_PARAM_VALIDATION_ENABLE (1U)

#endif /* PID_CFG_H */
