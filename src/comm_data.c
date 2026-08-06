/******************************************************************************
 *
 * Module: CommData (ROS comms data-prep boundary)
 *
 * File Name: comm_data.c
 *
 * Description: Implementation of the data-preparation boundary layer.
 *              RX: rad/s -> RPM conversion + LEFT/RIGHT routing.
 *              TX: raw cumulative signed encoder ticks per wheel.
 *
 *              NO transport here (no CAN, no UART framing, no protocol).
 *              The frozen layers (pid/Motor/encoder) are only CALLED, never
 *              modified - the TX side reads the existing Encoder HAL.
 *
 ******************************************************************************/

#include "comm_data.h"
#include "encoder.h"        /* frozen HAL - called, never modified */

/*----------------------------------------------------------------------------
 * B7: the wheel-tick pair must be sampled with no task switch between the two
 * QEI reads (see CommData_GetWheelTicks). No-ops in every non-RTOS build, which
 * therefore keep byte-identical behaviour - there is only one context there, so
 * there was never a tear to prevent.
 *--------------------------------------------------------------------------*/
#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#define COMM_DATA_ATOMIC_ENTER()   taskENTER_CRITICAL()
#define COMM_DATA_ATOMIC_EXIT()    taskEXIT_CRITICAL()
#else
#define COMM_DATA_ATOMIC_ENTER()   do { } while (0)
#define COMM_DATA_ATOMIC_EXIT()    do { } while (0)
#endif

/*******************************************************************************
 *                       RX data prep: rad/s -> RPM                            *
 *******************************************************************************/

float32 CommData_RadPerSecToRpm(float32 radps)
{
    /* RPM = rad/s * 60/(2*pi); see comm_data_cfg.h for the derivation. */
    return radps * COMM_DATA_RADPS_TO_RPM;
}

float32 CommData_RpmToRadPerSec(float32 rpm)
{
    /* rad/s = RPM * 2*pi/60. Currently unused-but-provided (see header). */
    return rpm * COMM_DATA_RPM_TO_RADPS;
}

CommData_WheelSetpointsRpmType CommData_RxVelToRpm(float32 leftRadps,
                                                   float32 rightRadps)
{
    CommData_WheelSetpointsRpmType setpoints;

    /* Explicit 1:1 LEFT/RIGHT mapping: host field1 is the LEFT wheel,
     * field2 is the RIGHT wheel, matching ENCODER_LEFT/RIGHT ==
     * MOTOR_LEFT/RIGHT. No cross-over, no chassis kinematics here. */
    setpoints.leftRpm  = CommData_RadPerSecToRpm(leftRadps);
    setpoints.rightRpm = CommData_RadPerSecToRpm(rightRadps);

    return setpoints;
}

/*******************************************************************************
 *                       TX data prep: cumulative ticks                        *
 *******************************************************************************/

sint32 CommData_GetLeftTicks(void)
{
    /* Raw cumulative signed counts straight from the Encoder HAL. The HAL
     * (and the QEI layer beneath it) accumulates counts at CPR = 2464 and
     * does not reset on read - exactly the host contract. */
    return Encoder_GetTotalCounts(ENCODER_LEFT);
}

sint32 CommData_GetRightTicks(void)
{
    return Encoder_GetTotalCounts(ENCODER_RIGHT);
}

CommData_WheelTicksType CommData_GetWheelTicks(void)
{
    CommData_WheelTicksType ticks;

    /*------------------------------------------------------------------------
     * A4-4 / V9-R1 FIXED HERE (B7, 2026-08-06). THE VELOCITY-SIDE TWIN OF S10-1.
     *
     * This function USED TO BE exactly the two getters back-to-back:
     *
     *     ticks.leftTicks  = CommData_GetLeftTicks();
     *     ticks.rightTicks = CommData_GetRightTicks();
     *
     * which is why REVIEW A4 recorded that adopting it was "presentational, NOT
     * a fix" - it tore identically to the call site it was meant to replace.
     *
     * THE DEFECT: the two QEI channels are separate hardware read one after the
     * other. In the super-loop that was a single context, so the pair was
     * always self-consistent and the tear was LATENT. The moment B7 peels
     * tRosTx into its own task, a higher-priority task (tBattery 6, tRosRx 7,
     * tVelocity 8, tSafety 9) can preempt BETWEEN the two reads - and then the
     * left and right counts in one 0x110 frame describe DIFFERENT INSTANTS.
     *
     * WHY THAT MATTERS MORE THAN IT LOOKS: ROS integrates this pair into wheel
     * POSITIONS. A torn pair is not a transient glitch that averages out - it
     * is a permanent step in the odometry, injected silently, with no flag. At
     * 2 m/s a 1 ms tear is ~2 mm of phantom differential rotation, i.e. a
     * phantom heading change, and it accumulates.
     *
     * THE FIX: sample both channels with NO TASK SWITCH BETWEEN THEM. That is
     * all "atomic" can mean here - they are two independent peripherals, so
     * there is no single instruction that reads both; what must be excluded is
     * a context switch in the middle.
     *
     * ⚠️ A CRITICAL SECTION, NOT A MUTEX, and it is genuinely cheap: the body
     * is two Encoder_GetTotalCounts() calls, each a couple of QEI register
     * reads - well under a microsecond with interrupts masked. It cannot be
     * held across a context switch, cannot block a task, and cannot invert
     * priorities. A mutex here would be absurd for a sub-microsecond read.
     *
     * NO `seq` FIELD, deliberately - unlike S10-1 and imu_service. `seq` earns
     * its place when a reader needs to detect STALENESS (has this sample been
     * refreshed?). These are two free-running monotonic counters: every read is
     * current by construction, and the only defect possible was the tear, which
     * the critical section removes outright. Adding a counter would be
     * ceremony, not safety.
     *----------------------------------------------------------------------*/
    COMM_DATA_ATOMIC_ENTER();
    ticks.leftTicks  = Encoder_GetTotalCounts(ENCODER_LEFT);
    ticks.rightTicks = Encoder_GetTotalCounts(ENCODER_RIGHT);
    COMM_DATA_ATOMIC_EXIT();

    return ticks;
}
