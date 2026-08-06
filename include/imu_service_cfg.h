#ifndef IMU_SERVICE_CFG_H_
#define IMU_SERVICE_CFG_H_

/*******************************************************************************
 * ImuService compile-time configuration.
 *
 * Board-level knobs for the IMU application service. Everything here is a value
 * a bring-up or a re-calibration might legitimately change; the service logic
 * itself lives in imu_service.c.
 *******************************************************************************/

#include "Platform_Types.h"
#include "i2c_types.h"          /* I2C_ID_1                                    */
#include "mpu6050_types.h"      /* MPU6050_ACCEL_RANGE_*, MPU6050_GYRO_RANGE_* */

/*******************************************************************************
 *                          Sensor / bus selection                             *
 *******************************************************************************/

/* The MPU6050 is on I2C1 (PA6=SCL, PA7=SDA) at 400 kHz - a DIFFERENT bus from
 * the INA226's I2C0/PB2-PB3. See PINOUT.md. */
#define IMU_I2C_ID              I2C_ID_1

/* +/-2 g fits int16 x0.001 on the DBC 0x150 wire without saturating, and gives
 * the best resolution for a slow rover that never sees high g. */
#define IMU_ACCEL_RANGE         MPU6050_ACCEL_RANGE_2G

/* +/-250 dps: quiet and high-resolution. A rover yaw rate is far below this. */
#define IMU_GYRO_RANGE          MPU6050_GYRO_RANGE_250DPS

/*******************************************************************************
 *                          Fault handling (R22-3)                             *
 *******************************************************************************/

/* Re-init attempts are RATE LIMITED. MPU6050_Init costs 6 capped I2C commands,
 * which must never run inline on every failed read inside the cooperative
 * super-loop - that is exactly the pattern REVIEW 22 R22-3 says not to copy from
 * the bench, because it would blow the 1 ms CAN-tx slot budget.
 *
 * At the intended ~50 Hz Update() cadence, 25 calls ~= 500 ms between attempts,
 * so a disconnected sensor costs 6 commands twice a second instead of 6 commands
 * every 20 ms. Reads while unhealthy are FREE: the HAL short-circuits to
 * MPU6050_ERROR_NOT_INITIALIZED without touching the bus. */
#define IMU_REINIT_BACKOFF_CALLS    (25U)

/*******************************************************************************
 *                          Accelerometer scale correction                     *
 *
 * Per-axis multiplier applied to the accelerometer AFTER the HAL's datasheet
 * scaling, to correct a part whose real sensitivity differs from the nominal
 * 16384 LSB/g.
 *
 * WHY THIS EXISTS: the fitted part measures 8.860 m/s^2 of gravity at rest
 * against a true 9.80665 - 9.7 % low. That is orientation-INDEPENDENT (it is the
 * vector magnitude, so tilt cannot explain it) and it is NOT a software scaling
 * bug: the range encodings were verified as AFS_SEL=0 / FS_SEL=0, so 16384 LSB/g
 * and 131 LSB/dps do apply. The MPU6050 datasheet allows only +/-3 % initial
 * calibration tolerance, so 9.7 % is out of spec - consistent with a clone part
 * (many still report WHO_AM_I 0x68) or one needing calibration. Left
 * uncorrected it would bias 0x150 ImuAccel, which feeds odometry fusion.
 *
 * RE-CALIBRATION PROCEDURE (compile-time; there is deliberately NO flash/NVM
 * persistence - this repo has no NVM abstraction):
 *   1. Set all three factors back to 1.0f and rebuild.
 *   2. Hold the board STILL and LEVEL (Z axis up, pointing at the sky).
 *   3. Run the bench and trigger ImuCal_MeasureGravity(); it averages samples
 *      and prints  corr = 9.80665 / ||measured||.
 *   4. Copy the printed factor in below, rebuild, and re-run: ||accel|| should
 *      now read ~9.807 and accelZ ~+9.807 when level.
 *
 * SCALAR vs PER-AXIS: this is deliberately a single scalar replicated across all
 * three axes, derived from the gravity MAGNITUDE. A magnitude measurement can
 * only ever constrain one number - resolving three independent axis gains needs
 * three orientations (+Z up, +X up, +Y up). Only go per-axis if a still-and-
 * level capture actually shows axis-specific error; do not invent three numbers
 * from one measurement.
 *
 * MEASURED 2026-08-01 via ImuCal_MeasureGravity(), 200 samples, board still:
 *     mean ||a|| = 8.8714 m/s^2  =  90.4 % of g   =>  corr = 9.80665/8.8714
 *                                                        = 1.1054
 * Cross-check: an independent 553-sample capture before this routine existed
 * gave ||a|| = 8.860 => 1.1068, i.e. the two agree to ~0.13 %, so the figure is
 * repeatable and not an artefact of one capture.
 *******************************************************************************/
#define IMU_ACCEL_SCALE_CORR    { 1.1054f, 1.1054f, 1.1054f }

/* Nominal gravity used by the calibration routine and by the cfg maths above. */
#define IMU_GRAVITY_MPS2        (9.80665f)

#endif /* IMU_SERVICE_CFG_H_ */
