/******************************************************************************
 *
 * Module: JetsonComm  (COMM layer, Jetson-facing)
 *
 * File Name: jetson_comm.h
 *
 * Description: The comms layer between the Jetson/ROS host and this firmware's
 *              verified control layers, over the real CAN transport. It wires
 *              the proven ROS-mimic data path (comm_data + velocity_control +
 *              steering_control) onto Can_Receive / Can_Transmit.
 *
 *                RX (host -> us):
 *                  0x100 VelocityCommand -> rad/s -> RPM -> VelocityControl
 *                  0x120 SteeringCommand -> rad (unchanged) -> SteeringControl
 *                TX (us -> host):
 *                  0x110 VelocityFeedback -> cumulative signed encoder ticks
 *                  0x130 SteeringFeedback -> measured angle (rad) + status bits
 *
 *              Layer: COMM (top of the stack). Depends DOWNWARD only on the
 *              transport (can), the generated contract (robot.h), the data
 *              boundary (comm_data) and the control layers. It owns no control
 *              loop: JetsonComm_Poll() only APPLIES setpoints; the system main
 *              still owns the 20 ms VelocityControl_Update() tick. The two
 *              feedback senders each emit EXACTLY ONE frame; one-per-tick
 *              scheduling (so they don't contend for the single CAN TX object)
 *              is the SYSTEM MAIN's job, not this module's.
 *
 ******************************************************************************/

#ifndef JETSON_COMM_H_
#define JETSON_COMM_H_

#include "Std_Types.h"

/**
 * @brief  Initialise the JetsonComm module. Near-no-op: comm_data is pure and
 *         the CAN/velocity/steering layers are initialised by the system main.
 *         Provided for symmetry with ClusterComm_Init(); owns no state.
 */
void JetsonComm_Init(void);

/**
 * @brief  Drain the CAN RX FIFO and apply every received command this tick.
 *         Loops Can_Receive() until CAN_NO_DATA; dispatches by frame id
 *         (0x100 -> velocity setpoint, 0x120 -> steering angle). A frame that
 *         arrives with an overrun is still valid and is applied. Malformed
 *         frames (unpack error or dlc != 8) are dropped.
 * @note   APPLIES SETPOINTS ONLY. Does NOT call VelocityControl_Update() - the
 *         system main owns the 20 ms control cadence. Call frequently so the
 *         RX FIFO cannot back up.
 */
void JetsonComm_Poll(void);

/**
 * @brief  Monotonic count of ACCEPTED VelocityCommand (0x100) frames.
 * @brief  Monotonic count of ACCEPTED SteeringCommand (0x120) frames.
 *
 * These are the RX-liveness signal behind the command-loss failsafe (A4-1).
 * The counters advance ONLY after a frame has been validated AND applied, so a
 * host spewing malformed frames does not register as alive - which is the whole
 * point, because a partially-crashed host is exactly the case a failsafe must
 * still catch.
 *
 * ⚠️ THIS MODULE DELIBERATELY OWNS NO TIMEOUT. It publishes counts; main.c's
 * super-loop owns the timebase and the policy, alongside the bus-off failsafe
 * that takes the same action. Do not add a deadline here - see jetson_comm.c.
 *
 * Saturating at UINT32_MAX (sticky-counter idiom). A caller must compare for
 * CHANGE, never assume a fixed increment per period.
 */
uint32 JetsonComm_GetVelocityCommandCount(void);
uint32 JetsonComm_GetSteeringCommandCount(void);

/**
 * @brief  Build and transmit ONE VelocityFeedback (0x110) frame: the per-wheel
 *         cumulative signed encoder tick counts (odometry source).
 * @return E_OK if staged for transmit; E_NOT_OK if packing failed or the CAN TX
 *         object was busy (frame dropped, no retry).
 */
Std_ReturnType JetsonComm_SendVelocityFeedback(void);

/**
 * @brief  Build and transmit ONE SteeringFeedback (0x130) frame: the measured
 *         steering angle (rad, REP-103 +left) and the status bitfield.
 * @return E_OK if staged for transmit; E_NOT_OK if packing failed or the CAN TX
 *         object was busy (frame dropped, no retry).
 */
Std_ReturnType JetsonComm_SendSteeringFeedback(void);

#endif /* JETSON_COMM_H_ */
