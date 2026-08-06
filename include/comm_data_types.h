/******************************************************************************
 *
 * Module: CommData (ROS comms data-prep boundary)
 *
 * File Name: comm_data_types.h
 *
 * Description: Public types used by the comm_data boundary API. These are
 *              plain data bundles - just the numbers, no framing, no
 *              transport metadata. They exist so a future transport layer
 *              has a clean, typed hand-off for the data this module
 *              prepares.
 *
 ******************************************************************************/

#ifndef COMM_DATA_TYPES_H_
#define COMM_DATA_TYPES_H_

#include "Platform_Types.h"

/*******************************************************************************
 *                       RX: per-wheel RPM setpoints                           *
 *
 * Result of converting the host's per-wheel rad/s command into the RPM
 * units the existing PID layer is tuned for. left/right follow the
 * project's ENCODER_LEFT / ENCODER_RIGHT == MOTOR_LEFT / MOTOR_RIGHT
 * convention (see comm_data.h).
 ******************************************************************************/
typedef struct {
    float32 leftRpm;    /* LEFT wheel setpoint in RPM  */
    float32 rightRpm;   /* RIGHT wheel setpoint in RPM */
} CommData_WheelSetpointsRpmType;

/*******************************************************************************
 *                       TX: per-wheel cumulative ticks                        *
 *
 * Raw, cumulative, signed encoder counts per wheel (CPR = 2464). The host
 * finite-differences these for velocity and uses ticks*2*pi/2464 for wheel
 * angle, so these are ABSOLUTE accumulators - NOT per-cycle deltas.
 ******************************************************************************/
typedef struct {
    sint32 leftTicks;   /* LEFT  wheel cumulative signed counts */
    sint32 rightTicks;  /* RIGHT wheel cumulative signed counts */
} CommData_WheelTicksType;

#endif /* COMM_DATA_TYPES_H_ */
