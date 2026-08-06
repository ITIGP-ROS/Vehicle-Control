#ifndef STEERING_CONTROL_TYPES_H_
#define STEERING_CONTROL_TYPES_H_

/******************************************************************************
 *
 * Module: SteeringControl (application / control-policy layer)
 *
 * File Name: steering_control_types.h
 *
 * Description: Public types for the steering control layer: the published
 *              feedback SNAPSHOT.
 *
 *              WHY A SNAPSHOT TYPE EXISTS AT ALL (S10-1, REVIEW A4 §1). The
 *              angle and the status bits are not two independent answers - they
 *              are two views of ONE pot acquisition. Publishing them through
 *              two separate getters made it possible (and, on hardware,
 *              routine) to pair an angle from sample B with status bits from
 *              sample A: HV-1 captured 0x130 frames whose steering_angle
 *              CONTRADICTS their own AT_TARGET/SATURATED bits. This struct is
 *              the type-level statement that the two travel together.
 *
 *              Same shape, same reason, as Ina226_ReadAll's
 *              {bus, shunt, current, flags, seq} and BatteryStatusType.
 *
 ******************************************************************************/

#include "Platform_Types.h"

/*******************************************************************************
 *                          Feedback snapshot                                  *
 *******************************************************************************/

/**
 * One coherent observation of the steering. Every field is derived from a
 * SINGLE ServoFb acquisition and a SINGLE read of the command state, taken
 * inside one SteeringControl_GetSnapshot() call - so no two fields can describe
 * different instants.
 */
typedef struct
{
    /* Measured angle, REP-103 (+left / -right), radians. From the SAME pot
     * sample the status bits below were derived from. NOT clamped - it is the
     * true measurement, exactly as SteeringControl_GetMeasuredAngle() used to
     * return it.
     *
     * Published even when POT_FAULT is set: the value is then untrustworthy but
     * it is still the sample the fault verdict was formed on, which is what
     * makes a dropout diagnosable from the frame that reports it. */
    float32 angle_rad;

    /* STEERING_STATUS_* bitfield (steering_control.h). AT_TARGET and SATURATED
     * are derived from `angle_rad` itself; POT_FAULT from the same raw count;
     * OUT_OF_RANGE from one read of the command state. */
    uint8   status;

    /* Increments once per SUCCESSFUL acquisition (E_OK). It does NOT advance on
     * a refused call, so a repeated value means "no new observation happened",
     * not "nothing changed".
     *
     * WHY IT IS HERE BEFORE THERE IS AN RTOS: under the super-loop the caller
     * acquires synchronously and staleness is impossible, so `seq` is
     * currently write-only. It exists because after the port `tRosTx` reads a
     * snapshot produced by another task, and an unchanged `seq` across two
     * reads is the ONLY way to tell a stale read from a fresh identical one.
     * Adding it later would be a wire/API change; adding it now is free.
     *
     * WRAPS at 255 by design - it is a change detector, not a counter. A
     * consumer must compare for INEQUALITY, never ordering. */
    uint8   seq;
} SteeringSnapshotType;

#endif /* STEERING_CONTROL_TYPES_H_ */
