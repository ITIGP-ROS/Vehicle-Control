#ifndef IMU_SERVICE_H_
#define IMU_SERVICE_H_

/******************************************************************************
 *
 * Module: ImuService  (application / service layer)
 *
 * File Name: imu_service.h
 *
 * Description: Application-layer wrapper over the MPU6050 HAL. Owns the board's
 *              fixed IMU configuration, a periodic non-blocking Update(), the
 *              latest sample in engineering units, a health flag with a
 *              RATE-LIMITED fault recovery, and the shared sample-sequence
 *              counter the DBC frames need.
 *
 *              Layer: above the HAL. Includes ONLY mpu6050.h. NEVER touches
 *              i2c.c, registers, CAN, comm_data or UART. Consumers
 *              (the super-loop, jetson_comm) talk to THIS and never to
 *              mpu6050.c or i2c.c.
 *
 *              Shape deliberately mirrors velocity_control / steering_control:
 *              X_Init() / X_Update() / X_Get*(), so a scheduler slot can drop it
 *              in and a comm layer can pull the latest sample at pack time.
 *
 * -----------------------------------------------------------------------------
 *  CADENCE LIVES AT THE CALL SITE. ImuService_Update() contains NO delay and
 *  does exactly ONE sensor read per call. The caller schedules it (~50 Hz, i.e.
 *  every 20 ms) - the same discipline the HAL follows, and the reason the old
 *  in-driver 2 ms busy-wait was removed.
 *
 *  PREREQUISITES, in this order, before ImuService_Init():
 *      Port_Init()  ->  Timer0_FreeRunning_Init()  ->  I2C_Init()
 *  Timer0 MUST precede I2C: every I2C command is TIMER0-tick capped, and with
 *  TIMER0 stopped the cap never elapses and the first read hangs forever.
 * -----------------------------------------------------------------------------
 *
 *  SIGN CONVENTION - EXACTLY ONE TRANSFORM SITE, AND IT IS HERE.
 *  The DBC contract (robot.dbc 0x160 gyro_z, and HANDOFF_README) states: "The
 *  TIVA pre-inverts this to the robot frame; the Jetson applies no host-side
 *  inversion." Read that as a POLICY - the Tiva, not the host, owns delivering
 *  the robot-frame sign - not as an instruction to negate unconditionally.
 *
 *  For gyro Z that transform is the IDENTITY: an MPU6050 mounted chip-up is
 *  already right-handed with +Z up, so raw gyro-Z is already positive-for-CCW,
 *  which is REP-103.
 *  HARDWARE-VERIFIED 2026-08-01 (FIX_26): chip-up, CCW-from-top gives +gz.
 *
 *  ⚠️ UPDATED B13c (2026-08-07): the other four axes are NO LONGER pass-through.
 *  The module is physically mounted 180 deg yawed from the robot frame (MEASURED
 *  +X = REAR, +Y = RIGHT, +Z = up), so ImuService_Latch now NEGATES accel X/Y and
 *  gyro X/Y. A 180 deg yaw maps (x,y,z) -> (-x,-y,z), so gyro Z and accel Z are
 *  deliberately untouched - yaw rate is invariant under a yaw rotation, which is
 *  precisely why FIX_26's gz result stayed valid and why the mount error hid for
 *  so long. See the long block in ImuService_Latch for the measurements.
 *
 *  The transform still lives in exactly ONE place (ImuService_Latch), so a
 *  remount - chip-down, or the IMU rotated onto another axis - is a one-line
 *  change there. DO NOT add an inversion downstream (in jetson_comm, at pack
 *  time, or on the host) - same discipline as steering_control's one-site rule.
 *
 ******************************************************************************/

#include "Std_Types.h"

/**
 * @brief  Bring up the IMU with this board's fixed configuration.
 * @details Wraps MPU6050_Init() with the ranges/bus from imu_service_cfg.h and
 *          clears the sample state. Safe to call again - that is exactly what
 *          the internal fault recovery does.
 * @note    Does NOT hang if the sensor is absent; the service simply reports
 *          unhealthy and retries on its own backoff schedule.
 */
void ImuService_Init(void);

/**
 * @brief  Take at most ONE sensor sample. Non-blocking, no delay inside.
 * @details Call periodically at ~50 Hz. On success the latest sample and the
 *          sequence counter advance and the service is healthy. On failure the
 *          service goes unhealthy and HOLDS the last good sample rather than
 *          emitting garbage, and re-initialisation is attempted at most once
 *          every IMU_REINIT_BACKOFF_CALLS calls - never inline on every failed
 *          read (REVIEW 22 R22-3: an inline re-init is 6 capped I2C commands and
 *          would blow the super-loop's 1 ms CAN-tx slot).
 */
void ImuService_Update(void);

/**
 * @brief  TRUE if the most recent ImuService_Update() produced a valid sample.
 * @note   While FALSE the Get* accessors still return the last GOOD sample, so
 *         a consumer that ignores this flag publishes stale - not garbage - data.
 *         Check it before trusting a sample for fusion.
 */
boolean ImuService_IsHealthy(void);

/**
 * @brief  Latest acceleration in m/s^2, ready for the 0x150 codec.
 * @details Already corrected by IMU_ACCEL_SCALE_CORR (see imu_service_cfg.h).
 *          NULL pointers are ignored individually, so a caller may request a
 *          subset.
 */
void ImuService_GetAccel(float32 *ax, float32 *ay, float32 *az);

/**
 * @brief  Latest angular rate in rad/s, ready for the 0x160 codec.
 * @details Z is ALREADY inverted to the robot frame here (the one and only
 *          negate - see the sign-convention block above). X and Y are raw.
 */
void ImuService_GetGyro(float32 *gx, float32 *gy, float32 *gz);

/**
 * @brief  Latest die temperature in degrees Celsius.
 */
float32 ImuService_GetTemp(void);

/**
 * @brief  TRUE once at least ONE good sample has ever been latched.
 * @details B13 GATE - a CONSUMER MUST CHECK THIS BEFORE PUBLISHING.
 *          Before the first successful read the held sample is all ZEROES and
 *          the sequence is 0. Those zeroes are indistinguishable, on the wire,
 *          from a real reading of "no acceleration at all" - and because BOTH
 *          0x150 and 0x160 would carry the same sequence, they would pass the
 *          Jetson's pairing check and be FUSED AS REAL DATA. A vehicle in free
 *          fall is the only thing that legitimately reads 0 g.
 *
 *          IsHealthy() is NOT a substitute: it goes FALSE on a single failed
 *          read and stays FALSE for up to IMU_REINIT_BACKOFF_CALLS, so gating
 *          publication on it would blank ~500 ms of frames for one glitch.
 *          The right behaviour there is to keep publishing the HELD sample with
 *          its FROZEN sequence.
 *
 *          ⚠️ A FROZEN SEQUENCE IS DETECTABLE, BUT THE HOST DOES NOT CURRENTLY
 *          DETECT IT. can_comms.cpp stores `expected_seq_`/`seq_initialized_`
 *          (can_comms.hpp:47-48) and NEVER READS THEM - verified by grep - so
 *          today it compares only accel-seq against gyro-seq, which a held
 *          sample still satisfies. It will therefore fuse a stale sample as
 *          fresh for as long as the fault lasts. That is a ROS-side follow-up,
 *          not a reason to publish zeroes or go silent here: a frozen counter is
 *          at least diagnosable from a capture, which zeroes are not.
 *
 *          So: HasSample() gates whether to publish AT ALL; IsHealthy() only
 *          tells you whether the newest sample is fresh.
 * @note    Latches TRUE and never clears except through ImuService_Init().
 */
boolean ImuService_HasSample(void);

/**
 * @brief  TRUE while the IMU should be reported as freshly RE-INITIALISED.
 * @details Maps to the DBC's 0x160 `imu_reset` bit: "1 = the IMU was
 *          re-initialised/crashed and recovered. The Jetson forces gyro Z to 0
 *          and holds its existing calibration while this is set."
 *
 *          SET on the unhealthy -> healthy EDGE, and only when a good sample had
 *          already been seen before the fault. The first-ever sample after boot
 *          is deliberately NOT a reset: the host has no calibration to protect
 *          yet, so announcing one would be noise.
 *
 *          HELD for IMU_RESET_HOLD_SAMPLES samples rather than a single frame,
 *          because the signal is worthless if the host misses it. The host
 *          drains at 30 Hz and keeps only the LAST frame of each cycle, so a
 *          one-frame pulse at 50 Hz is genuinely likely to be dropped. The hold
 *          is a level, which is exactly how the DBC words it ("WHILE this is
 *          set"), not an event.
 */
boolean ImuService_GetResetFlag(void);

/**
 * @brief  Free-running sample counter, 0..255, incremented once per GOOD sample.
 * @details ONE counter for the whole service, by design. The Jetson pairs
 *          0x150 and 0x160 by comparing their sequence bytes and REJECTS the
 *          read when they differ, so both frames built from one sample MUST
 *          carry this same value. Do not keep a second counter per frame.
 *          (Note the byte offset differs between the two frames - byte 6 in
 *          ImuAccel, byte 7 in ImuGyroFlags - but the VALUE must match.)
 *          Wraps 255 -> 0; consumers must treat it as modulo-256.
 */
uint8 ImuService_GetSequence(void);

#endif /* IMU_SERVICE_H_ */
