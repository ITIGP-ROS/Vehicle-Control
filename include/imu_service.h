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
 *  For the CURRENT chip-up mounting that transform is the IDENTITY: an MPU6050
 *  mounted chip-up is already right-handed with +Z up, so raw gyro-Z is already
 *  positive-for-CCW, which is REP-103. All six axes are therefore pass-through
 *  today (accel additionally carries the scale correction).
 *  HARDWARE-VERIFIED 2026-08-01 (FIX_26): chip-up, CCW-from-top gives +gz.
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
