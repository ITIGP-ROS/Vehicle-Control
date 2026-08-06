/******************************************************************************
 *
 * Module: VelocityControl  (application / control layer)
 *
 * File Name: velocity_control.h
 *
 * Description: Per-wheel closed-loop velocity controller, extracted verbatim
 *              from the validated test/closed_loop_speed_encoder/main.c. Two
 *              independent PID loops (LEFT/RIGHT) on measured wheel RPM driving
 *              signed motor speed. PURE CONTROL: it does not know where the
 *              setpoint comes from (CAN/UART/sweep) or where feedback goes.
 *
 *              Layer: above the HAL. Includes only pid/encoder/Motor. NEVER
 *              touches registers, CAN, comm_data, or UART.
 *
 * -----------------------------------------------------------------------------
 *  HAL PREREQUISITE — READ BEFORE USE
 * -----------------------------------------------------------------------------
 *  The HAL MUST already be initialised by the caller, in THIS ORDER, BEFORE
 *  calling VelocityControl_Init():
 *
 *      Port_Init()  ->  PWM_Init()  ->  (SysTick_Init)  ->  Motor_Init()  ->  Encoder_Init()
 *
 *  PWM_Init() IS REQUIRED. Motor_Init() does NOT call PWM_Init() internally --
 *  the Motor layer only calls PWM_SetDuty(), it never brings up the PWM
 *  peripheral. If PWM_Init() is omitted, PWM_SetDuty() writes to an
 *  uninitialised peripheral and THE MOTORS WILL NOT DRIVE (the encoders then
 *  read zero and the loop appears dead). SysTick is the caller's timebase for
 *  the 20 ms cadence; this module does not own it.
 *
 *  After init, the caller invokes VelocityControl_Update() once per 20 ms
 *  control period (must match Ts = 0.02 s and QEI_VELOCITY_PERIOD_MS).
 * -----------------------------------------------------------------------------
 *
 ******************************************************************************/

#ifndef VELOCITY_CONTROL_H_
#define VELOCITY_CONTROL_H_

#include "Platform_Types.h"

/**
 * @brief  Initialise both per-wheel PID controllers with the validated gains.
 * @note   Does NOT init the HAL (Motor/Encoder/PWM) -- the caller does that
 *         first, in the order documented in the file header (PWM_Init() is
 *         required). Sets both setpoints to 0 RPM.
 */
void VelocityControl_Init(void);

/**
 * @brief  Set the desired per-wheel speed, in RPM (the PID feedback domain).
 * @param  leftRpm  desired LEFT wheel speed  (signed: +fwd / -rev)
 * @param  rightRpm desired RIGHT wheel speed (signed: +fwd / -rev)
 * @note   Conversion from host rad/s to RPM is the caller's job, NOT this module's.
 * @note   V9-3: rejected (nothing latched) and counted if called before
 *         VelocityControl_Init() - see VelocityControl_GetPreInitCallCount().
 */
void VelocityControl_SetSetpoint(float32 leftRpm, float32 rightRpm);

/**
 * @brief  Run ONE control iteration (call every 20 ms): for each wheel,
 *         read measured RPM -> PID_Update -> Motor_SetSignedSpeed.
 * @note   20 ms cadence is the caller's responsibility and MUST match Ts (0.02 s)
 *         and QEI_VELOCITY_PERIOD_MS.
 * @note   V9-3: a pre-Init call is rejected and counted - no encoder read, no
 *         PID iteration, and NO motor command.
 */
void VelocityControl_Update(void);

/**
 * @brief  Hard-stop both motors, zero the setpoints, and reset both PID handles.
 * @note   Motor_Stop on both wheels, plus PID_Reset on each handle so a later
 *         restart does not carry a charged integrator (clean restart).
 * @note   V9-3: the ONE entry point that is deliberately NOT init-gated. It is
 *         the bus-off failsafe; refusing to stop because Init has not run would
 *         invert the purpose of the guard. Safe pre-Init (Motor_Stop and
 *         PID_Reset are both guarded), and it does not open the gate.
 */
void VelocityControl_Stop(void);

/**
 * @brief  Number of Update()/SetSetpoint() calls refused because they arrived
 *         before VelocityControl_Init() (V9-3).
 * @return Saturating lifetime count. MUST be 0 in a correctly-ordered system -
 *         any non-zero value is a startup-order defect in the caller, not a
 *         runtime condition to handle.
 */
/**
 * @brief  TRUE while the V9-R4 inhibit latch is set, i.e. the drive is held
 *         stopped by the failsafe or the bus-off path.
 *
 * @note   THE LATCH IS THE V9-R4 FIX (B10). Stop() sets it, Update() re-asserts
 *         Motor_Stop while it is set instead of writing a computed output, and
 *         SetSetpoint() clears it on the next ACCEPTED command. That makes the
 *         stop survive tSafety (prio 9) preempting tVelocity (prio 8) mid-Update
 *         - under ANY interleaving, without a lock, because the next Update
 *         re-asserts rather than the stop having to win a race.
 * @note   Observability only. Nothing outside velocity_control sets it.
 */
boolean VelocityControl_IsInhibited(void);

uint32 VelocityControl_GetPreInitCallCount(void);

/**
 * @brief  Number of setpoints refused for being non-finite (NaN/Inf) or absurd
 *         (R8-1). Non-zero means the commanding side sent garbage; the last
 *         valid setpoint was held. Should stay 0 in a healthy system.
 * @return Saturating lifetime count.
 */
uint32 VelocityControl_GetRejectedSetpointCount(void);

/* --- Read-only observability: the exact value fed to each PID and the exact
 *     output from the most recent VelocityControl_Update(). Lets a test main
 *     log/compare against the validated CSV without this module touching UART. */
float32 VelocityControl_GetLeftMeasuredRpm(void);
float32 VelocityControl_GetRightMeasuredRpm(void);
float32 VelocityControl_GetLeftOutput(void);
float32 VelocityControl_GetRightOutput(void);

/* V9-6 (FIX 26): VelocityControl_GetLeftSetpointRpm() was REMOVED. It was a
 * temporary accessor for the LEFT actuator RAM-buffer investigation, which
 * DIAG 29 closed (the fault was the PD6<->PC6 encoder-harness short, repaired
 * and verified). It also broke the API's left/right symmetry.
 *
 * ⚠️ The committed src/main.c still has the diagnostic RAM-buffer block that
 * called it (its gate at main.c:199 plus the buffer/dump code). That block MUST
 * be removed when the production main.c is next restored, or the build will
 * fail on an undeclared VelocityControl_GetLeftSetpointRpm. The exact extents
 * are listed in docs/velocity/FIX_26_velocity_control.md. */

#endif /* VELOCITY_CONTROL_H_ */
