/******************************************************************************
 *
 * Module: JetsonComm  (COMM layer, Jetson-facing)
 *
 * File Name: jetson_comm.c
 *
 * Description: Wires the proven ROS-mimic data path onto the real CAN
 *              transport. RX unpack -> comm_data/control; TX read control/HAL
 *              -> pack -> Can_Transmit. Mirrors the cluster_comm pattern.
 *
 * -----------------------------------------------------------------------------
 *  RX COMMAND-LOSS FAILSAFE (A4-1) - IMPLEMENTED 2026-08-05, but NOT HERE.
 *
 *  This module stays TIME-AGNOSTIC on purpose. It publishes monotonic counts of
 *  ACCEPTED commands (below); main.c's super-loop watches those counts against
 *  its own timebase and trips the failsafe. Two reasons the timeout does not
 *  live in this file:
 *
 *   1. A4 verified this module is ROUTING/PACK ONLY - no control state, no
 *      timebase, no hardware. Putting a deadline here would be the first
 *      exception to that, and the layering is worth more than the convenience.
 *   2. The bus-off failsafe (C6-1) is ALREADY in main.c and takes the SAME
 *      action. "Command source is gone" deserves one implementation, not two
 *      that can drift apart.
 *
 *  POLICY (decided, not re-opened): zero velocity via VelocityControl_Stop(),
 *  and HOLD steering - do NOT re-centre (S10-5 / R3-1). See main.c.
 * -----------------------------------------------------------------------------
 *
 ******************************************************************************/

#include "jetson_comm.h"
#include "node_ping.h"   /* 0x7A0 -> latch; the echo TX is main.c's slot */
#include "encoder.h"     /* 0x140 -> Encoder_ResetAllDistances()            */
#include "cluster_comm.h"/* 0x140 -> ClusterComm_ResetTrip()                */
#include "can.h"               /* MCAL transport: Can_Receive                       */
#include "can_tx_queue.h"      /* B6: ALL transmits go through the TX queue now     */
#include "robot.h"             /* cantools: pack/unpack + ROBOT_*_FRAME_ID/_LENGTH   */
#include "comm_data.h"         /* boundary: rad/s->RPM, cumulative tick getters      */
#include "velocity_control.h"  /* control: VelocityControl_SetSetpoint (RPM)         */
#include "steering_control.h"  /* control: SetAngle / GetSnapshot (S10-1 coherent)   */

/* Expected payload length for every command/feedback frame (all are 8 bytes). */
#define JETSON_COMM_FRAME_DLC   (8U)

/*******************************************************************************
 *                          RX liveness counters (A4-1)                        *
 *                                                                             *
 * Monotonic counts of ACCEPTED commands, per channel. They are the ONLY thing
 * this module contributes to the failsafe: main.c samples them and, if neither
 * has advanced within the timeout, declares the host gone.
 *
 * ⚠️ INCREMENTED ONLY AFTER A FRAME IS FULLY VALIDATED AND APPLIED - never on
 * arrival. A malformed frame must NOT look like liveness: a Jetson emitting a
 * steady stream of wrong-DLC frames is exactly as dead, from the vehicle's
 * point of view, as one emitting nothing. Counting on arrival would defeat the
 * failsafe in precisely the case a partially-crashed host produces.
 *
 * Kept SEPARATE per channel (A4-1's recommended shape) so per-channel staleness
 * stays observable, even though the trip rule below combines them - see
 * main.c's CMD_TIMEOUT_MS block for why the combination is deliberate.
 *
 * Saturating, per the tree's sticky-counter idiom (Can_GetIfTimeoutCount).
 *******************************************************************************/
static uint32 jc_velCommandCount   = 0U;
static uint32 jc_steerCommandCount = 0U;

static void JetsonComm_CountCommand(uint32 *counter)
{
    if (*counter < 0xFFFFFFFFUL)
    {
        (*counter)++;
    }
}

/*******************************************************************************
 *                          Private Helpers (RX dispatch)                      *
 *******************************************************************************/

/* 0x100 VelocityCommand: rad/s -> RPM -> velocity setpoint. Drops malformed. */
static void JetsonComm_HandleVelocityCommand(const uint8 *data, uint8 dlc)
{
    struct robot_velocity_command_t   cmd;
    CommData_WheelSetpointsRpmType    spRpm;

    if ((dlc != JETSON_COMM_FRAME_DLC) ||
        (robot_velocity_command_unpack(&cmd, data, (size_t)dlc) != 0))
    {
        return;   /* check-and-drop: no partial apply, no retry */
    }

    /* rad/s (host domain) -> RPM (PID domain). LEFT->LEFT, RIGHT->RIGHT, no swap. */
    spRpm = CommData_RxVelToRpm(cmd.left_vel_setpoint, cmd.right_vel_setpoint);
    VelocityControl_SetSetpoint(spRpm.leftRpm, spRpm.rightRpm);

    /* Applied, so it counts as liveness (A4-1). After the apply, not before. */
    JetsonComm_CountCommand(&jc_velCommandCount);
}

/* 0x140 ResetCommand: zero the odometry TICK COUNTERS and the TRIP distance.
 *
 * ⚠️ DEFINED IN THE DBC SINCE THE BEGINNING, BUT NEVER IMPLEMENTED UNTIL NOW -
 * and it could not have been, because 0x140 is REJECTED by the command
 * acceptance filter ((0x140 & 0x7DF) = 0x140 != 0x100). It needed its own RX
 * message object, exactly like the liveness ping did. Until then TRIP only ever
 * zeroed on a power-cycle, which quietly contradicted its documented
 * "reset-relative" behaviour.
 *
 * ⚠️ THIS MUST NOT TOUCH THE LIFETIME ODOMETER. TRIP is reset-relative; ODO is
 * permanent and has no CAN path that can clear it - deliberately. ODO is safe
 * here for a structural reason rather than by remembering to skip it: it is fed
 * pure distance INCREMENTS by cluster_comm and holds no origin, so zeroing the
 * encoder counters is invisible to it (see ClusterComm_UpdateTrip).
 *
 * FIRE-AND-FORGET per the DBC: no acknowledgement frame is sent or expected.
 * Only bit 0 is meaningful; the rest of the byte is reserved. */
static void JetsonComm_HandleResetCommand(const uint8 *data, uint8 dlc)
{
    struct robot_reset_command_t cmd;

    /* ⚠️ A4-5 FIXED 2026-08-05: this was `dlc != ROBOT_RESET_COMMAND_LENGTH`,
     * i.e. DLC EXACTLY 1. ResetCommand is a 1-byte frame, but hosts and CAN
     * tooling very commonly PAD to the classic 8 bytes - and every one of those
     * frames was SILENTLY DROPPED. That would have been the SECOND silent-drop
     * path on this same frame, after the acceptance-filter bug that hid 0x140
     * for the whole project history. Two invisible failures on one frame is a
     * pattern, not bad luck.
     *
     * `>=` is safe here: only byte 0 / bit 0 carries meaning, and cantools
     * ignores trailing bytes. This also makes the RX boundary CONSISTENT - the
     * liveness ping already accepts `dlc >= LENGTH` (node_ping.c) and reset was
     * the odd one out. The 8-byte command frames keep their exact-match check,
     * because for them a short DLC means genuinely missing signal bytes. */
    if ((dlc < (uint8)ROBOT_RESET_COMMAND_LENGTH) ||
        (robot_reset_command_unpack(&cmd, data, (size_t)dlc) != 0))
    {
        return;   /* check-and-drop, same discipline as the other handlers */
    }

    if (cmd.reset == 0U)
    {
        return;   /* bit0 clear = no request; not an error */
    }

    /* ORDER MATTERS: zero the counters FIRST, then re-seed the trip origin
     * against the new (zeroed) reading. Doing it the other way round would let
     * one sample compute a delta spanning the entire pre-reset distance. */
    (void)Encoder_ResetAllDistances();
    ClusterComm_ResetTrip();
}

/* 0x120 SteeringCommand: rad (REP-103, +left) passed THROUGH UNCHANGED. */
static void JetsonComm_HandleSteeringCommand(const uint8 *data, uint8 dlc)
{
    struct robot_steering_command_t cmd;

    if ((dlc != JETSON_COMM_FRAME_DLC) ||
        (robot_steering_command_unpack(&cmd, data, (size_t)dlc) != 0))
    {
        return;   /* check-and-drop */
    }

    /* NO negate here: SteeringControl_SetAngle expects REP-103 (+left) and does
     * the single sign reconciliation to the servo HAL internally. */
    SteeringControl_SetAngle(cmd.steering_setpoint);

    /* Applied, so it counts as liveness (A4-1). After the apply, not before. */
    JetsonComm_CountCommand(&jc_steerCommandCount);
}

/*******************************************************************************
 *                          Public Functions                                   *
 *******************************************************************************/

uint32 JetsonComm_GetVelocityCommandCount(void)  { return jc_velCommandCount;   }
uint32 JetsonComm_GetSteeringCommandCount(void)  { return jc_steerCommandCount; }

void JetsonComm_Init(void)
{
    /* No private hardware/state. CAN, velocity_control and steering_control are
     * initialised by the system main; comm_data is pure. Kept for API symmetry
     * with ClusterComm_Init() and for any future per-frame scheduling state. */
}

void JetsonComm_Poll(void)
{
    uint32         id;
    uint8          data[JETSON_COMM_FRAME_DLC];
    uint8          dlc;
    Can_StatusType rx;

    /* Drain-all-per-tick: keep popping until the FIFO is empty. CAN_RX_OVERRUN
     * still delivers a valid frame, so it is dispatched (not discarded); only
     * CAN_NO_DATA ends the loop.
     *
     * ⚠️ THE ACCEPTED SET IS NO LONGER JUST {0x100, 0x120}. This comment used to
     * say "Only 0x100/0x120 can arrive (hardware filter)"; that became false on
     * 2026-08-05 when a SECOND RX message object was added for the HOST liveness
     * ping (0x7A0, exact match - it cannot pass the 0x7DF command filter, see
     * can_cfg.h). This poll is the node's ONE drain point for the RX ring
     * buffer, so it routes the ping too - but only by latching it; see below. */
    for (;;)
    {
        rx = Can_Receive(&id, data, &dlc);
        if (rx == CAN_NO_DATA)
        {
            break;
        }

        switch (id)
        {
            case ROBOT_VELOCITY_COMMAND_FRAME_ID:   /* 0x100 */
                JetsonComm_HandleVelocityCommand(data, dlc);
                break;

            case ROBOT_STEERING_COMMAND_FRAME_ID:   /* 0x120 */
                JetsonComm_HandleSteeringCommand(data, dlc);
                break;

            case ROBOT_RESET_COMMAND_FRAME_ID:      /* 0x140 */
                JetsonComm_HandleResetCommand(data, dlc);
                break;

            case ROBOT_NODE_PING_REQUEST_FRAME_ID:  /* 0x7A0, from the HOST */
                /* 🔶 UNCONDITIONAL, AND STRUCTURALLY SO. This case is routed
                 * directly to NodePing, which consults NO service - unlike the
                 * two command handlers above, which hand off to VelocityControl
                 * and SteeringControl. A fault anywhere in the control or
                 * telemetry stack therefore cannot suppress the liveness reply,
                 * which is the entire contract: silence means DEAD.
                 *
                 * It only LATCHES here - the transmit happens in the super-loop
                 * slot (main.c, now%100==51) so it cannot collide with a
                 * scheduled TX in the same millisecond. */
                NodePing_OnRequest(data, dlc);
                break;

            default:
                /* Not admitted by either RX message object; drop defensively. */
                break;
        }
    }
}

Std_ReturnType JetsonComm_SendVelocityFeedback(void)
{
    struct robot_velocity_feedback_t st;
    uint8 data[ROBOT_VELOCITY_FEEDBACK_LENGTH];
    int   packed;

    /* ⚠️ A4-4 / V9-R1 FIXED (B7): ONE coherent acquisition, not two getters.
     * This used to be
     *     st.left_ticks  = CommData_GetLeftTicks();
     *     st.right_ticks = CommData_GetRightTicks();
     * - two separate QEI reads. Latent while everything ran in the super-loop's
     * single context; a real tear the moment tRosTx became its own task, because
     * a higher-priority task can preempt between them and leave the two counts
     * describing different instants. ROS integrates this pair into wheel
     * POSITIONS, so a tear is a permanent silent step in the odometry, not a
     * transient. CommData_GetWheelTicks() now samples both with no task switch
     * in between - see the long note there. */
    CommData_WheelTicksType ticks = CommData_GetWheelTicks();

    st.left_ticks  = (int32_t)ticks.leftTicks;
    st.right_ticks = (int32_t)ticks.rightTicks;

    packed = robot_velocity_feedback_pack(data, &st, (size_t)sizeof(data));
    if (packed < 0)
    {
        return E_NOT_OK;   /* serialization failed (buffer too small) */
    }

    if (CanTxQueue_Post(ROBOT_VELOCITY_FEEDBACK_FRAME_ID, data, (uint8)packed) != CAN_OK)
    {
        return E_NOT_OK;   /* TX_BUSY / INVALID_DLC -> drop this frame, no retry */
    }

    return E_OK;
}

Std_ReturnType JetsonComm_SendSteeringFeedback(void)
{
    struct robot_steering_feedback_t st;
    uint8 data[ROBOT_STEERING_FEEDBACK_LENGTH];
    SteeringSnapshotType snap;
    int   packed;

    /* ⚠️ S10-1 FIXED 2026-08-05 - ONE acquisition per frame, not two.
     *
     * This was:
     *     status            = SteeringControl_GetStatus();        // 8 samples
     *     st.steering_angle = SteeringControl_GetMeasuredAngle(); // 8 MORE
     * i.e. two independent pot acquisitions, so the angle on the wire described
     * a different instant than the status bits beside it. HV-1 caught it live:
     * dropout frames report an angle that CONTRADICTS their own bits, which
     * means a consumer cannot diagnose the dropout from the frame reporting it.
     * 16 blocking conversions per frame, 1600/s at the 100 Hz slot, to produce
     * an incoherent result.
     *
     * The snapshot is the whole fix: one acquisition, both fields derived from
     * it. Note the caller was never at fault - the API offered no way to ask
     * for both from one sample. That was the finding; this is the API. */
    if (SteeringControl_GetSnapshot(&snap) != E_OK)
    {
        /* Pre-Init only in practice (main.c:214 runs SteeringControl_Init long
         * before the TX slot). Treated like a pack failure: DO NOT transmit.
         * Deliberate change from the old code, which would have shipped a
         * POT_FAULT frame - a frame asserting a measurement that was never
         * taken is worse than silence, and 0x130 is telemetry, so a missing
         * frame costs nothing the next 10 ms slot does not fix. */
        return E_NOT_OK;
    }

    /* The status byte is exposed in the DBC as four named 1-bit signals instead
     * of one opaque byte, so it is split here - bit for bit, no remap. The wire
     * is unchanged: at_target..saturated are b0..b3 of the same byte 4 this
     * frame always carried. Only the SOURCE changed, to the coherent snapshot. */
    st.steering_angle = snap.angle_rad;
    st.at_target    = (uint8)((snap.status & STEERING_STATUS_AT_TARGET)    ? 1U : 0U);
    st.pot_fault    = (uint8)((snap.status & STEERING_STATUS_POT_FAULT)    ? 1U : 0U);
    st.out_of_range = (uint8)((snap.status & STEERING_STATUS_OUT_OF_RANGE) ? 1U : 0U);
    st.saturated    = (uint8)((snap.status & STEERING_STATUS_SATURATED)    ? 1U : 0U);

    packed = robot_steering_feedback_pack(data, &st, (size_t)sizeof(data));
    if (packed < 0)
    {
        return E_NOT_OK;
    }

    if (CanTxQueue_Post(ROBOT_STEERING_FEEDBACK_FRAME_ID, data, (uint8)packed) != CAN_OK)
    {
        return E_NOT_OK;
    }

    return E_OK;
}
