/******************************************************************************
 *
 * Module: Servo (HAL)
 *
 * File Name: servo_cfg.h
 *
 * Description: Servo-DOMAIN configuration. These are steering-servo concepts -
 *              the mechanical center pulse, the travel endpoints, and the
 *              angle<->pulse mapping. They are intentionally separate from the
 *              timer hardware facts in timer_pwm_cfg.h.
 *
 *              The pulse endpoints here coincide with the MCAL's generic clamp
 *              window (TIMER_PWM_MIN/MAX_PULSE_US) by design; if you change the
 *              servo's mechanical limits, keep the two in sync.
 *
 ******************************************************************************/

#ifndef SERVO_CFG_H_
#define SERVO_CFG_H_

#include "Platform_Types.h"

/*******************************************************************************
 *                          Pulse-Width Endpoints (servo semantics)           *
 *******************************************************************************/

/* Servo pulse-width landmarks in microseconds. RE-MEASURED 2026-07-30 after the
 * steering LINKAGE REWORK (HWTEST 03; raw data in docs/servo/HWVERIFY_01_pot_reference.md).
 * 1450us = wheels-straight CENTER / failsafe, set by eye with the front wheels
 * parallel. These MUST match the MCAL clamp TIMER_PWM_MIN/MAX_PULSE_US - the
 * compile-time guard at the bottom of timer_pwm_cfg.h enforces it.
 *
 * Sign convention: higher us = RIGHT (REP-103 negative); lower us = LEFT.
 * Travel is asymmetric about center: 850us to LEFT (1450->600), 1050us to RIGHT
 * (1450->2500).
 *
 * WHY these endpoints (both are SERVO-authority limits, NOT linkage stops):
 *   LEFT  600us  - the servo buzzes/stalls at 400us and the pot noise climbs
 *                  steadily as it is approached (sd 9.5 counts at 600us, 11.5 at
 *                  550, 14.7 at 500, versus ~1.7 at center), so 600us is the
 *                  margined edge of clean authority, not the mechanical stop.
 *   RIGHT 2500us - the servo saturates here: pot response falls from 1.80
 *                  counts/us at 2300 to 1.50 at 2500 and 0.02 beyond, and pulses
 *                  up to 3000us were verified to move it no further. The LINKAGE
 *                  has more right travel; this servo cannot reach it. */
#define SERVO_MIN_PULSE_US              (600U)    /* full LEFT  (servo-authority edge) */
#define SERVO_CENTER_PULSE_US           (1450U)   /* wheels-straight center / failsafe */
#define SERVO_MAX_PULSE_US              (2500U)   /* full RIGHT (servo saturation)     */

/*******************************************************************************
 *                          Angle Mapping (servo semantics)                   *
 *******************************************************************************/

/* CALIBRATED command half-ranges (2026-07-30). These are the MAGNITUDES (both
 * positive) of the native command angle at each travel endpoint; each side has
 * its own linear scale that hits its endpoint exactly, so the pulse spans may be
 * asymmetric while the angles are not:
 *   full LEFT  = 600us   (CENTER-MIN = 850us span)  -> |angle| = 0.3037 rad
 *   full RIGHT = 2500us  (MAX-CENTER = 1050us span) -> |angle| = 0.3037 rad
 *
 * PROVENANCE, and why the two sides now differ:
 *   RIGHT 0.3037 rad = 17.4 deg = the CAD DESIGN angle (34.7 deg total
 *         lock-to-lock). The right endpoint achieves it exactly.
 *   LEFT  0.2421 rad = 13.9 deg = the ATTAINABLE angle, measured at the pot with
 *         the servo at 600us. The design angle is 0.3037 on this side too, but
 *         the servo runs out of authority before the linkage reaches it, so the
 *         wheels physically stop at 13.9 deg. We record what the hardware does,
 *         not what the drawing intends (R2-1, Option 1).
 * The CAD figure is what identified the provenance of the previous, undocumented
 * 0.218/0.300 pair (old 0.300 ~= CAD 0.3037).
 *
 * WHY BOTH the map scale here and SC_LIMIT_LEFT_RAD in steering_control.c must
 * carry the SAME left value: the map reaches MIN pulse when angle == -(this
 * constant), so this constant DEFINES which command angle means "full left";
 * the clamp must cap the command at exactly that value or the two disagree.
 * Setting both to the measured 0.2421 makes map, clamp and pot feedback agree.
 *
 * Native command sign convention (servo HAL frame): +angle -> RIGHT (higher us),
 * -angle -> LEFT (lower us). REP-103 (+left) is reconciled by the single negate
 * in steering_control before Servo_SetAngleRad -- do NOT change the sign here,
 * only the scale.
 *
 * R2-1 RESOLVED 2026-07-30 (Option 1): LEFT was 0.3037 (the CAD design angle),
 * which made a full-left command disagree with the pot by 0.0616 rad - three
 * times the 0.02 AT_TARGET deadband - leaving AT_TARGET and SATURATED unable to
 * assert at full left. LEFT now carries the attainable 0.2421, so commanded and
 * measured agree at the endpoint (err 0.0000) and both status bits work.
 *
 * ASYMMETRY IS NOW DOUBLE - do NOT "simplify" the two branches of
 * Servo_AngleRadToPulseUs into one. The angles differ (0.2421 vs 0.3037) AND the
 * pulse spans differ (850us left vs 1050us right). Merging the branches would
 * mis-scale a side badly. The two-branch structure is required. */
#define SERVO_MAX_ANGLE_LEFT_RAD        (0.2421f)  /* ATTAINABLE at full LEFT (600us), measured */
#define SERVO_MAX_ANGLE_RIGHT_RAD       (0.3037f)  /* CAD design at full RIGHT (2500us), exact  */

/* Rejection bound for the HAL's non-finite guard (R3-1, servo.c). Far outside any
 * real steering angle, so only genuinely non-finite / absurd inputs are refused;
 * ordinary out-of-travel angles still take the normal map-and-clamp path. */
#define SERVO_IMPLAUSIBLE_RAD           (1000.0f)

#endif /* SERVO_CFG_H_ */
