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
 *                          imu_reset hold window (B13)                        *
 *
 * How many GOOD samples keep 0x160's `imu_reset` bit asserted after the IMU
 * recovers from a fault.
 *
 * WHY THIS IS A LEVEL AND NOT A ONE-FRAME PULSE. The DBC words the bit as a
 * state - "the Jetson forces gyro Z to 0 and holds its existing calibration
 * WHILE this is set" - and the host cannot see a pulse reliably anyway: its
 * read_sensor_values() drains the whole socket per cycle and keeps only the
 * LAST frame of each ID, at 30 Hz against our 50 Hz. A single flagged frame is
 * therefore MORE LIKELY TO BE DISCARDED THAN SEEN. Announcing a recovery that
 * the consumer never observes is the same as not announcing it.
 *
 * 25 samples = 500 ms at the 50 Hz cadence ~= 15 host cycles, so the bit cannot
 * be missed, and it matches IMU_REINIT_BACKOFF_CALLS's timescale above - the
 * two windows describe the same fault episode from opposite ends.
 *******************************************************************************/
#define IMU_RESET_HOLD_SAMPLES      (25U)

/*******************************************************************************
 *                          Accelerometer scale correction                     *
 *
 * Per-axis multiplier applied to the accelerometer AFTER the HAL's datasheet
 * scaling, to correct a part whose real sensitivity differs from the nominal
 * 16384 LSB/g.
 *
 * ⚠️⚠️ THIS FACTOR IS PER-MODULE, NOT PER-DESIGN. It MUST be re-measured every
 * time the IMU is physically swapped - it corrects THAT part's gain error, and
 * two clones can be wrong in OPPOSITE directions. Proven on this bench: the
 * module fitted until 2026-08-14 read 0.904 g and needed 1.1054; the one fitted
 * after it reads 1.128 g and needs 0.8868. Carrying the old number over left
 * 0x150 publishing gravity as 12.22 m/s^2 (+24.7 %) with nothing else wrong.
 * If you see |a| far from 9.81 after a swap, come HERE first - the id list in
 * mpu6050_cfg.h decides whether the part is ACCEPTED, this decides whether it is
 * SCALED.
 *
 * WHY THIS EXISTS: the part fitted in 2026-08 measures 8.860 m/s^2 of gravity at rest
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
 * MEASURED 2026-08-01 via ImuCal_MeasureGravity(), 200 samples, board still, on
 * the module fitted at the time (WHO_AM_I 0x68):
 *     mean ||a|| = 8.8714 m/s^2  =  90.4 % of g   =>  corr = 9.80665/8.8714
 *                                                        = 1.1054
 * Cross-check: an independent 553-sample capture before this routine existed
 * gave ||a|| = 8.860 => 1.1068, i.e. the two agree to ~0.13 %, so the figure is
 * repeatable and not an artefact of one capture.
 *
 * ===== CURRENT VALUE - RE-MEASURED 2026-08-14 FOR THE SWAPPED-IN MODULE =====
 * That module was replaced; the new one reports WHO_AM_I 0x70 (see
 * mpu6050_cfg.h) and has the OPPOSITE gain error - it reads HIGH, not low:
 *     299 samples decoded from live 0x150 ImuAccel frames, vehicle stationary,
 *     mean ||a|| = 12.2247 m/s^2 (min 12.129, max 12.324) with corr 1.1054
 *          => raw ||a|| = 12.2247/1.1054 = 11.059 m/s^2 = 1.1277 g
 *          => corr = 1.1054 x 9.80665/12.2247 = 0.8868
 * ℹ️ METHOD NOTE: this used the CAN stream, not ImuCal_MeasureGravity() (that
 * routine prints over UART1, and no USB-TTL dongle was attached - see the
 * 2026-08-14 block in docs/MEMORY.md). It is arithmetically the same procedure:
 * measure ||a||, scale the EXISTING factor by 9.80665/||a||, which is why step 1
 * above (reset to 1.0f) could be skipped. The magnitude is orientation-
 * independent, so the board did not need to be levelled for it.
 *******************************************************************************/
#define IMU_ACCEL_SCALE_CORR    { 0.8868f, 0.8868f, 0.8868f }

/*******************************************************************************
 *                          Gyroscope scale correction                         *
 *
 * Per-axis multiplier applied to the gyroscope AFTER the HAL's datasheet scaling
 * (131 LSB/dps at FS_SEL=0), for the same reason the accel has one: this part's
 * real sensitivity is not the nominal figure. ADDED 2026-08-14 - before that the
 * gyro had NO correction path at all and 131 LSB/dps was taken on trust.
 *
 * MEASURED 2026-08-14, gravity-referenced, 7 moves: the gyro reads
 *     +12.4 % HIGH  (ratio 1.1244, sd 0.0117)  =>  corr = 1/1.1244 = 0.8893
 * 🔑 Compare IMU_ACCEL_SCALE_CORR = 0.8868 above. The two INDEPENDENT sensors
 * agree to 0.3 %, so this is ONE device-wide gain error on the 0x70 part, not
 * two coincidences - which is also why a single scalar is the right shape here.
 *
 * ===== HOW IT WAS MEASURED, because the obvious method does NOT work =====
 * A gyro measures RATE, so verifying its GAIN needs a known ANGLE. Hand-turning
 * the module "exactly 360 deg" was tried twice and BOTH SETS WERE DISCARDED:
 * the readings were self-inconsistent (ratios 0.49 and 0.66) because the
 * reference itself - a human saying "that was a full circle" - cannot be
 * trusted or checked from inside the data.
 * ⚠️ SO DO NOT RE-VERIFY THIS WITH A HAND-TURNED PROTRACTOR ANGLE. Ask GRAVITY
 * instead: at rest the accelerometer (now good to 0.08 %) gives absolute
 * orientation, so the angle between two rests is known to ~0.1 deg NO MATTER
 * how sloppy the move was. Per move:
 *     applied  = angle between the two rest gravity vectors
 *     measured = | integral of (omega - bias) dt |, as a VECTOR NET
 * ⚠️ VECTOR NET, NOT PATH LENGTH: path counts wobble as rotation. One logged
 * move had 154.5 deg of path for a 25.1 deg net - path ratio 6.17 (garbage),
 * net ratio 1.132 (in line with every other move).
 *
 * ⚠️ WHAT IS MEASURED: gy directly (every usable move was pitch-dominated, gz
 * share <= 10 %). gz is INFERRED: FS_SEL is a SINGLE GLOBAL field in
 * GYRO_CONFIG bits 4:3 covering all three axes, so a range-level error cannot be
 * per-axis, and the accel's independent +12.7 % shows the error is device-wide.
 * Residual risk is per-axis silicon spread (typically a few %). A direct gz
 * capture is the open item - see docs/MEMORY.md 2026-08-14.
 *
 * ⚠️ PER-MODULE, exactly like the accel factor: RE-MEASURE ON EVERY IMU SWAP.
 * This corrects THIS part, not the design.
 *
 * NOT a bias/offset knob. The rest bias (gx +0.0332, gy +0.0371, gz -0.0078
 * rad/s on this part) is deliberately NOT removed here - the Tiva sends raw by
 * design and the ROS side owns de-biasing. A scale error and an offset error are
 * different faults; do not fold one into the other.
 *******************************************************************************/
#define IMU_GYRO_SCALE_CORR     { 0.8893f, 0.8893f, 0.8893f }

/* Nominal gravity used by the calibration routine and by the cfg maths above. */
#define IMU_GRAVITY_MPS2        (9.80665f)

#endif /* IMU_SERVICE_CFG_H_ */
