/******************************************************************************
 *
 * Module: CommData (ROS comms data-prep boundary)
 *
 * File Name: comm_data.h
 *
 * Description: Thin DATA-PREPARATION boundary between the host (Jetson /
 *              ROS) and this firmware's frozen control layers. It does NOT
 *              implement any transport - there is deliberately NO CAN, NO
 *              UART framing, NO packet/protocol code here. Its only job is
 *              to make the data ready for whatever transport is added next:
 *
 *                RX (host -> us): convert per-wheel angular velocity in
 *                  rad/s into the RPM setpoints the existing, already-tuned
 *                  PID layer expects.
 *
 *                TX (us -> host): expose the raw, cumulative, signed encoder
 *                  tick count per wheel (CPR = 2464), exactly as the host
 *                  contract requires (host divides by 2464 and does
 *                  ticks*2*pi/2464 for wheel angle, and finite-differences
 *                  ticks for velocity).
 *
 *              This module owns NO global state and touches NO frozen layer
 *              internals - the TX getters call the existing Encoder HAL
 *              (Encoder_GetTotalCounts) and the RX helpers are pure
 *              functions. pid/Motor/encoder are NOT modified by this module.
 *
 * LEFT/RIGHT convention (matches the whole project):
 *   - ENCODER_LEFT  == MOTOR_LEFT   -> "left"  fields here
 *   - ENCODER_RIGHT == MOTOR_RIGHT  -> "right" fields here
 *   The host contract sends field1 = LEFT wheel, field2 = RIGHT wheel.
 *
 ******************************************************************************/

#ifndef COMM_DATA_H_
#define COMM_DATA_H_

#include "Platform_Types.h"
#include "comm_data_cfg.h"
#include "comm_data_types.h"

/*******************************************************************************
 *                       RX data prep: rad/s -> RPM                            *
 *******************************************************************************/

/**
 * @brief  Convert an angular velocity from rad/s to RPM.
 * @param  radps  Angular velocity in rad/s (signed).
 * @return Equivalent value in RPM (signed).
 * @note   Pure function. Multiplies by COMM_DATA_RADPS_TO_RPM
 *         (= 60/(2*pi), see comm_data_cfg.h for the derivation).
 */
float32 CommData_RadPerSecToRpm(float32 radps);

/**
 * @brief  Convert an angular velocity from RPM to rad/s.
 * @param  rpm  Angular velocity in RPM (signed).
 * @return Equivalent value in rad/s (signed).
 * @note   Pure function. Multiplies by COMM_DATA_RPM_TO_RADPS
 *         (= 2*pi/60). CURRENTLY UNUSED-BUT-PROVIDED: the TX side may want
 *         to report velocity in rad/s once transport exists; kept here so
 *         the inverse lives next to the forward conversion and shares the
 *         single source-of-truth constant.
 */
float32 CommData_RpmToRadPerSec(float32 rpm);

/**
 * @brief  Route the host's per-wheel rad/s command into PID-ready RPM
 *         setpoints.
 * @param  leftRadps   LEFT  wheel angular velocity setpoint in rad/s
 *                     (host contract field1).
 * @param  rightRadps  RIGHT wheel angular velocity setpoint in rad/s
 *                     (host contract field2).
 * @return Both setpoints converted to RPM, in a typed bundle.
 * @note   Pure function, no global state. LEFT stays LEFT and RIGHT stays
 *         RIGHT - the mapping is intentionally 1:1 and explicit so it
 *         matches ENCODER_LEFT/RIGHT == MOTOR_LEFT/RIGHT. The caller (a
 *         future transport/dispatch layer) is responsible for feeding
 *         .leftRpm to the LEFT-wheel PID and .rightRpm to the RIGHT-wheel
 *         PID; this module does not call PID itself.
 */
CommData_WheelSetpointsRpmType CommData_RxVelToRpm(float32 leftRadps,
                                                   float32 rightRadps);

/*******************************************************************************
 *                       TX data prep: cumulative ticks                        *
 *******************************************************************************/

/**
 * @brief  Get the LEFT wheel's raw cumulative signed encoder tick count.
 * @return Cumulative signed counts (CPR = 2464). Forward increments,
 *         reverse decrements. NOT reset by reading; NOT a per-cycle delta.
 * @note   Delegates to Encoder_GetTotalCounts(ENCODER_LEFT). No scaling is
 *         applied here - the host expects raw counts at 2464/rev.
 */
/**
 * @brief  Cumulative signed ticks for ONE wheel.
 *
 * @deprecated FOR PRODUCTION USE (B7, A4-4/V9-R1). ⚠️ CALLING THIS AND ITS TWIN
 *             BACK-TO-BACK IS THE DEFECT A4-4 NAMED:
 *
 *                 left  = CommData_GetLeftTicks();    <-- a task switch here
 *                 right = CommData_GetRightTicks();       tears the pair
 *
 *             The two QEI channels are separate hardware, so under preemption
 *             the two counts describe DIFFERENT INSTANTS - and ROS integrates
 *             them into wheel POSITIONS, making a tear a permanent, silent step
 *             in the odometry rather than a transient.
 *             ➡️ Production code MUST use CommData_GetWheelTicks(), which
 *             samples both with no task switch in between.
 *
 * @note   NOT REMOVED, unlike S10-1's GetMeasuredAngle twin, because these
 *         still have legitimate callers: the single-context bench harnesses
 *         test/closed_loop_speed_encoder (non-RTOS builds) where
 *         no task switch exists and therefore no tear is possible. Removing
 *         them would break working harnesses to prevent a hazard those
 *         harnesses cannot have.
 */
sint32 CommData_GetLeftTicks(void);

/**
 * @brief  Get the RIGHT wheel's raw cumulative signed encoder tick count.
 * @return Cumulative signed counts (CPR = 2464). Forward increments,
 *         reverse decrements. NOT reset by reading; NOT a per-cycle delta.
 * @note   Delegates to Encoder_GetTotalCounts(ENCODER_RIGHT).
 */
sint32 CommData_GetRightTicks(void);

/**
 * @brief  Get both wheels' cumulative signed tick counts in one bundle.
 * @return CommData_WheelTicksType with .leftTicks and .rightTicks.
 * @note   Transport-agnostic: just the two numbers, no framing. Reads each
 *         wheel via the getters above.
 */
CommData_WheelTicksType CommData_GetWheelTicks(void);

#endif /* COMM_DATA_H_ */
