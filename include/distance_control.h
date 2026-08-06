/******************************************************************************
 *
 * Module: DistanceControl  (application / control layer — OUTER cascade loop)
 *
 * File Name: distance_control.h
 *
 * Description: Outer distance loop. Takes a forward-distance setpoint (cm) and
 *              drives the car straight to it, then stops, by commanding the
 *              EXISTING velocity_control inner loop (RPM setpoints). Its own PID
 *              (distance-error cm -> velocity-setpoint RPM) with its OWN tunable
 *              gains (NOT the velocity gains). Straight line: both wheels get the
 *              same velocity setpoint; steering is not controlled here.
 *
 *              Layer: above velocity_control. Includes ONLY pid.h + encoder.h +
 *              velocity_control.h. NEVER touches Motor directly, CAN, comm, UART,
 *              or registers. Drives the car ONLY through velocity_control's
 *              public API.
 *
 * -----------------------------------------------------------------------------
 *  CADENCE: the caller runs BOTH loops:
 *    - VelocityControl_Update()  every 20 ms  (inner; caller owns it)
 *    - DistanceControl_Update()  every 50 ms  (outer; sets the velocity setpoint)
 *  HAL + VelocityControl_Init() must be done by the caller BEFORE
 *  DistanceControl_Init().
 * -----------------------------------------------------------------------------
 *
 ******************************************************************************/

#ifndef DISTANCE_CONTROL_H_
#define DISTANCE_CONTROL_H_

#include "Platform_Types.h"

/**
 * @brief  Initialise the outer distance PID (own tunable gains) and set the
 *         current position as the distance origin. Velocity setpoint = 0.
 * @note   Call AFTER VelocityControl_Init() (and the HAL).
 */
void DistanceControl_Init(void);

/* ---------------------------------------------------------------------------
 *  ⚠️ SINGLE-WRITER INVARIANT (D11-3, REVIEW 11) — read before wiring this up.
 *
 *  distance_control is an ALTERNATIVE OWNER of the velocity setpoint: it drives
 *  VelocityControl_SetSetpoint() directly (and VelocityControl_Stop()). It is
 *  the SOLE setpoint writer only in the closed_loop_distance harness, which
 *  never runs src/main.c's super-loop.
 *
 *  NEVER run DistanceControl_Update() concurrently with the super-loop's own
 *  setpoint writes (jetson_comm's CAN RX path). Two writers race the inner
 *  loop, and velocity's V9-R3 single-writer invariant depends on there being
 *  exactly one: vc_rejectedSetpoints++ is a non-atomic read-modify-write that
 *  silently under-reports with two writers - hiding the very garbage-input
 *  evidence it exists to surface. Under the FreeRTOS port they would be two
 *  preemptible TASKS, which is worse.
 *
 *  This module is a GROUND-TESTING INSTRUMENT, not production code. It is
 *  deliberately never wired into src/main.c; it is flashed on its own via
 *  [env:closed_loop_distance] to verify velocity/steering tuning on the floor.
 * -------------------------------------------------------------------------*/

/**
 * @brief  Set the forward-distance target (cm) and capture the current position
 *         as the zero reference. Resets the outer PID integrator for the move.
 * @param  distance_cm: forward target (+forward).
 * @note   See the SINGLE-WRITER INVARIANT above: arming this module makes it
 *         the owner of the velocity setpoint.
 */
void DistanceControl_SetTarget(float32 distance_cm);

/**
 * @brief  Run ONE outer iteration (call every 50 ms): measure distance (cm) ->
 *         outer PID -> velocity setpoint (RPM, clamped) -> VelocityControl_SetSetpoint
 *         (same RPM to both wheels). On arrival, commands zero velocity.
 * @note   Does NOT call VelocityControl_Update() — the caller runs that at 20 ms.
 * @warning SINGLE-WRITER INVARIANT (see the block above): this function writes
 *          the velocity setpoint. Never call it concurrently with the
 *          super-loop's own setpoint writes.
 */
void DistanceControl_Update(void);

/**
 * @brief  TRUE when |distance error| is within the STOP TOLERANCE
 *         (DC_ARRIVAL_TOLERANCE_CM, ~0.2 cm) AND the car is moving slower than
 *         the arrival speed threshold.
 * @note   D11-4: this used to key on the 1.0 cm floor-suppression DEADBAND, so
 *         it could read TRUE a full centimetre short — the reason the
 *         instrument reported "arrived" at ~49.3 of a commanded 50.0. It is now
 *         the same condition the module itself uses to latch stopped.
 * @note   Reported LIVE (re-measured each call). If the car is nudged after
 *         settling this goes FALSE while the module keeps holding zero; that
 *         divergence is honest, not a bug.
 */
boolean DistanceControl_IsAtTarget(void);

/**
 * @brief  Hard stop: zero the velocity setpoint, VelocityControl_Stop(), and
 *         reset the outer PID. Use to abort/disengage.
 */
void DistanceControl_Stop(void);

/* Read-only observability. */
/**
 * @brief  Number of targets refused for being non-finite (NaN/Inf) or absurd
 *         (|cm| > 100 m) — D11-5. Non-zero means the commanding side sent
 *         garbage and the previous target was held. Should stay 0.
 * @return Saturating lifetime count.
 */
uint32 DistanceControl_GetRejectedTargetCount(void);

float32 DistanceControl_GetMeasuredDistanceCm(void);
float32 DistanceControl_GetVelSetpointRpm(void);

/* TEMP DIAGNOSTIC — internal target the outer PID is acting on. */
float32 DistanceControl_GetTargetCm(void);

#endif /* DISTANCE_CONTROL_H_ */
