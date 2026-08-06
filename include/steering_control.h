/******************************************************************************
 *
 * Module: SteeringControl  (application / control-policy layer)
 *
 * File Name: steering_control.h
 *
 * Description: Thin steering policy/coordination layer over the steering HAL.
 *              Commands an angle, reads the measured angle, and builds a status
 *              bitfield. There is NO PID and NO periodic loop here: the servo's
 *              internal IC closes the position loop, so this is a stateless-ish
 *              wrapper, NOT a 20ms control task.
 *
 *              Layer: above the HAL. Includes ONLY servo.h + servo_feedback.h.
 *              NEVER touches registers, CAN, comm_data, or UART.
 *
 *              Angle convention at THIS boundary: REP-103, +left / -right
 *              (same as ServoFb_ReadAngleRad and the DBC SteeringCommand/
 *              Feedback contract). NOTE: the Servo command HAL uses the OPPOSITE
 *              sign internally; this module reconciles it in one place
 *              (see steering_control.c, SteeringControl_SetAngle).
 *
 * -----------------------------------------------------------------------------
 *  HAL PREREQUISITE: Port_Init() MUST have run before SteeringControl_Init()
 *  (PC4 servo-PWM mux + PE3 pot-analog mux are set by the PORT driver).
 *  SteeringControl_Init() then brings up the steering HAL itself (Servo_Init +
 *  ServoFb_Init) and self-centers. This module OWNS the steering HAL init
 *  (unlike velocity_control, which leaves init to the caller) because the servo
 *  + pot HAL is steering-exclusive, whereas Motor/Encoder/PWM are shared.
 * -----------------------------------------------------------------------------
 *
 ******************************************************************************/

#ifndef STEERING_CONTROL_H_
#define STEERING_CONTROL_H_

#include "Platform_Types.h"
#include "Std_Types.h"                /* Std_ReturnType - GetSnapshot REPORTS   */
#include "steering_control_types.h"   /* SteeringSnapshotType (S10-1)           */

/* Steering status bitfield. Value-compatible with the DBC SteeringFeedback
 * "steeringStatus" contract (build_dbc.py): b0..b3 as below. Defined here
 * because no C definition existed and this module must NOT include robot.h. */
#define STEERING_STATUS_AT_TARGET     (0x01U)  /* bit0: reached the CLAMPED setpoint - see below */
#define STEERING_STATUS_POT_FAULT     (0x02U)  /* bit1: pot reading implausible (HAL flag)  */
#define STEERING_STATUS_OUT_OF_RANGE  (0x04U)  /* bit2: last setpoint rejected or clamped   */
#define STEERING_STATUS_SATURATED     (0x08U)  /* bit3: measured angle at a mechanical limit */

/* ---------------------------------------------------------------------------
 *  HOW TO READ THE STATUS BITS (they are independent flags, not a state enum)
 *
 *  AT_TARGET means "the steering reached the setpoint WE ACTED ON" - which is
 *  the setpoint AFTER clamping to the travel limits, not necessarily the one you
 *  requested. So:
 *
 *    AT_TARGET alone                -> your requested angle was reached.
 *    AT_TARGET *and* OUT_OF_RANGE   -> your request was beyond travel; it was
 *                                      clamped, and the CLAMPED limit has been
 *                                      reached. This combination is normal and
 *                                      well-defined, NOT contradictory.
 *
 *  => Never interpret AT_TARGET on its own. Always read it together with
 *     OUT_OF_RANGE, or you will conclude a request was satisfied when it was
 *     actually truncated. (R3-3)
 *
 *  OUT_OF_RANGE also covers a REJECTED command: a non-finite (NaN/Inf) setpoint
 *  is refused outright, the steering HOLDS its last valid angle, and this bit is
 *  raised so the refusal is visible rather than silent (R3-1).
 *
 *  POT_FAULT suppresses the two measurement-derived bits (AT_TARGET, SATURATED),
 *  because with an implausible pot reading neither can be trusted. OUT_OF_RANGE
 *  is command-side and stays valid even then.
 * -------------------------------------------------------------------------*/

/**
 * @brief  Initialise the steering HAL (Servo_Init + ServoFb_Init) and self-center.
 * @note   Port_Init() MUST have run first (see file header). Resets the stored
 *         command to 0 (center).
 */
void SteeringControl_Init(void);

/**
 * @brief  Command a steering angle, REP-103 (+left / -right), in radians.
 *         Clamps to the validated travel limits, stores the clamped command
 *         (for AT_TARGET / OUT_OF_RANGE), and drives the servo.
 * @param  angleRad: requested angle (rad), REP-103.
 * @note   S10-3: the stored command is written ONLY after the servo HAL
 *         confirms the pulse was actually issued, so AT_TARGET can never be
 *         computed against an angle the servo never received. A call before
 *         SteeringControl_Init(), or one the HAL refuses, records NOTHING and
 *         is counted - see SteeringControl_GetPreInitCallCount().
 */
void SteeringControl_SetAngle(float32 angleRad);

/* ---------------------------------------------------------------------------
 *  READING THE STEERING BACK: SteeringControl_GetSnapshot() is THE accessor.
 *
 *  S10-1 (REVIEW 10, decided in REVIEW A4 §1, fixed 2026-08-05). This module
 *  used to publish the measured angle and the status bits through two separate
 *  getters, each performing its OWN 8-sample pot acquisition. Callers naturally
 *  wrote them back to back - jetson_comm did - and the result was a 0x130 frame
 *  whose angle came from sample B while every status bit described sample A.
 *  HV-1 caught it on hardware: dropout frames carry an angle that CONTRADICTS
 *  their own AT_TARGET/SATURATED bits, so a consumer cannot diagnose the
 *  dropout from the frame reporting it. Cost of the incoherence: 16 blocking
 *  ADC conversions per frame instead of 8.
 *
 *  There is now exactly ONE acquisition path in this module, and it publishes
 *  the whole observation at once.
 * -------------------------------------------------------------------------*/

/**
 * @brief  THE coherent read: measured angle + status bits + sequence, all from
 *         ONE pot acquisition and ONE read of the command state.
 *
 * @param  out: destination snapshot. MUST NOT be NULL.
 * @return E_OK      - `*out` holds a fresh observation and `seq` advanced.
 *         E_NOT_OK  - `out` was NULL (nothing written), or the call arrived
 *                     before SteeringControl_Init(). In the pre-Init case
 *                     `*out` IS still written, defensively: angle 0.0, status
 *                     POT_FAULT, and `seq` UNCHANGED (no acquisition happened,
 *                     so nothing may claim one did). No ADC is touched and the
 *                     call is counted - see SteeringControl_GetPreInitCallCount().
 *                     Angle 0.0 rather than "last known" because before Init
 *                     there IS no last known measurement, and POT_FAULT already
 *                     says the value carries no information.
 *
 * @note   Status semantics are unchanged - see STEERING_STATUS_* and the
 *         "HOW TO READ THE STATUS BITS" block above; in particular AT_TARGET
 *         reports the CLAMPED setpoint and must be read with OUT_OF_RANGE, and
 *         POT_FAULT suppresses the measurement-derived bits.
 * @note   Costs ONE pot acquisition (8 conversions), and `out->angle_rad` is the
 *         very sample the bits were formed on.
 * @note   S10-2 is closed by this call: OUT_OF_RANGE and AT_TARGET are computed
 *         from a SINGLE read of {sc_lastCommandRad, sc_lastWasClamped}, so the
 *         pair can no longer tear between them. Residual, by design: a
 *         preemption BETWEEN SetAngle() and this call means the snapshot
 *         describes the previous command. That is a correct "state as of now"
 *         sample, and `seq` is what lets a consumer detect it - making the two
 *         mutually atomic would need a lock, which REVIEW 10 §4 rejected in
 *         favour of publication.
 */
Std_ReturnType SteeringControl_GetSnapshot(SteeringSnapshotType *out);

/**
 * @brief  DEPRECATED - status byte only. Use SteeringControl_GetSnapshot().
 *
 * @deprecated Kept ONLY for the diagnostic/host-test callers that genuinely
 *             want the byte and nothing else (test/init_gating/native_test.c).
 *             It is now a thin wrapper over GetSnapshot and performs the same
 *             single acquisition (and advances `seq`) - there is no second
 *             acquisition path left in the module.
 *
 *             DO NOT pair it with a separate angle read to build a frame: that
 *             is exactly the S10-1 tear, and the reason its partner getter
 *             SteeringControl_GetMeasuredAngle() was REMOVED rather than
 *             deprecated. With no standalone angle getter the tear is no longer
 *             writable, which is a stronger guarantee than this comment.
 *
 * @return uint8 bitfield; POT_FAULT before Init (unchanged from before).
 */
uint8 SteeringControl_GetStatus(void);

/**
 * @brief  EXPLICIT-COMMAND re-centre: command wheels-straight and reset the
 *         stored command.
 *
 * @warning This is NOT the bus-off / command-loss failsafe action. The accepted
 *          policy is "stop driving, HOLD the wheel - do NOT re-centre"
 *          (docs/MEMORY.md, confirmed 2026-07-31): snapping the wheels straight
 *          mid-corner is its own hazard, which is the same reasoning that makes
 *          R3-1 HOLD the last valid angle when it rejects a non-finite command.
 *          `main.c`'s bus-off path deliberately calls VelocityControl_Stop()
 *          and NOT this function.
 *          DO NOT wire this into a bus-off or command-loss handler without
 *          re-opening that policy decision. (S10-5, REVIEW 10.)
 *
 * @note    "Centred" is a one-shot ACTION, not a durable state - the next
 *          SteeringControl_SetAngle() overrides it. Compare velocity V9-2.
 * @note    S10-3: init-gated like SetAngle. Because it is an explicit command
 *          and NOT the failsafe (see the warning above), gating it is safe -
 *          contrast VelocityControl_Stop(), which IS the failsafe and is
 *          therefore deliberately left ungated.
 */
void SteeringControl_Center(void);

/**
 * @brief  Number of entry-point calls refused because they arrived before
 *         SteeringControl_Init(), plus actuation attempts the servo HAL
 *         refused (S10-3).
 * @return Saturating lifetime count. MUST be 0 in a correctly-ordered system -
 *         non-zero is a startup-order defect in the caller, not a runtime
 *         condition to handle.
 */
uint32 SteeringControl_GetPreInitCallCount(void);

#endif /* STEERING_CONTROL_H_ */
