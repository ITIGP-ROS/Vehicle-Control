#ifndef MPU6050_CFG_H_
#define MPU6050_CFG_H_

/*******************************************************************************
 * MPU6050 compile-time configuration.
 *
 * Follows the house 4-file split documented in CLAUDE.md
 * (X.h / X_types.h / X_cfg.h / X_private.h): everything a board bring-up might
 * legitimately need to change lives HERE, not buried in mpu6050.c.
 *
 * Deliberately NOT moved here: MPU6050_GRAVITY_MPS2 and MPU6050_DEG2RAD. Those
 * are mathematical constants, not board configuration - putting them in a cfg
 * header would invite someone to "tune" them.
 *******************************************************************************/

#include "i2c.h"        /* I2C_CAP_DEFAULT_TICKS */

/*******************************************************************************
 *                          Bus / device addressing                            *
 *******************************************************************************/

/* 7-bit I2C address. 0x68 with AD0 tied LOW, 0x69 with AD0 tied HIGH. */
#define MPU6050_ADDRESS                 0x68

/* Per-command TIMER0-tick cap handed to every I2C_Read/I2C_Write (see i2c.h).
 * REQUIRES Timer0_FreeRunning_Init() to have run, or the wait never elapses. */
#define MPU6050_I2C_CAP_TICKS           I2C_CAP_DEFAULT_TICKS

/*******************************************************************************
 *                          Register map                                       *
 *******************************************************************************/

#define MPU6050_REG_SMPLRT_DIV          0x19
#define MPU6050_REG_CONFIG              0x1A    /* DLPF_CFG lives here          */
#define MPU6050_REG_GYRO_CONFIG         0x1B
#define MPU6050_REG_ACCEL_CONFIG        0x1C
#define MPU6050_REG_ACCEL_XOUT_H        0x3B
#define MPU6050_REG_PWR_MGMT_1          0x6B
#define MPU6050_REG_WHO_AM_I            0x75

/*******************************************************************************
 *                          Device identity                                    *
 *******************************************************************************/

/* WHO_AM_I values accepted as "this is our sensor", as a brace initializer.
 *
 * A genuine InvenSense MPU6050 returns 0x68. Clones and close derivatives on
 * otherwise pin- and register-compatible breakout boards report different ids,
 * so a hard == 0x68 rejects a working part with a misleading NOT_FOUND. The
 * default stays strict - only the genuine id - but adding a board's id here is
 * now a one-line change instead of a driver edit.
 *
 * Commonly seen on compatible parts (opt in deliberately, they are NOT all
 * register-identical - verify before trusting the scale factors):
 *     0x70  MPU6500 / some MPU6050 clones
 *     0x71  MPU9250
 *     0x72  MPU9255
 *     0x98  some far-eastern MPU6050 clones
 * e.g. to accept a 0x70 clone:  #define MPU6050_WHO_AM_I_ACCEPTED_IDS { 0x68, 0x70 }
 *
 * 2026-08-14: 0x70 ADDED. The IMU module was physically swapped and the new
 * part returns WHO_AM_I = 0x70 (MPU6500-family / clone silicon on an otherwise
 * MPU6050-pin-compatible breakout). MEASURED, not guessed: with the strict
 * { 0x68 } list the bringup harness read whoami=0x70 with i2c status I2C_OK -
 * i.e. the part answers cleanly at 0x68 and was rejected purely on identity, so
 * MPU6050_Init returned NOT_FOUND, ImuService never got a sample, and the
 * HasSample gate held 0x150/0x160 silent (0 frames on CAN).
 *
 * ⚠️ WHAT THIS ONE LINE DOES *NOT* PROMISE - the part is register-compatible
 * for everything this driver touches, but not identical:
 *   - the REGISTER MAP is compatible and that much IS verified on this part: the
 *     burst from 0x3B, PWR_MGMT_1 0x6B, CONFIG 0x1A, GYRO_CONFIG 0x1B,
 *     ACCEL_CONFIG 0x1C and SMPLRT_DIV 0x19 all took effect - data flows at the
 *     configured 50 Hz and the gyro rest bias is sane (~0.03 rad/s).
 *   - ⚠️ THE ACCEL GAIN IS NOT. Measured 2026-08-14 over 299 CAN samples at
 *     rest, this part's raw output is 1.1277 g where it should read 1.000 g,
 *     i.e. an effective ~18475 LSB/g against the 16384 this driver assumes
 *     (+12.7 %). The driver's scale constant is NOT changed for that - the
 *     per-part gain error is what IMU_ACCEL_SCALE_CORR (imu_service_cfg.h)
 *     exists to absorb, and it still holds the OLD module's factor. See the
 *     2026-08-14 block in docs/MEMORY.md before trusting 0x150 accel.
 *   - the gyro's 131 LSB/dps is UNVERIFIED here: a rest reading constrains the
 *     OFFSET, not the SCALE. Verifying it needs a known rotation.
 *   - the temperature formula differs on genuine MPU6500 silicon
 *     (/333.87 + 21.0 vs the /340 + 36.53 in mpu6050.c). Harmless HERE only
 *     because temp is not published on 0x150/0x160 - do not start trusting
 *     MPU6050_DataType.temp on this board without re-deriving it.
 *   - the ACCEL digital LPF on a 6500 lives in ACCEL_CONFIG2 (0x1D), which this
 *     driver never writes. MPU6050_DLPF_CFG_VALUE (CONFIG 0x1A) still sets the
 *     GYRO bandwidth and the 1 kHz base the divider needs, so the 50 Hz output
 *     rate is unaffected, but accel bandwidth sits at the part's reset default
 *     rather than the ~44 Hz this cfg intends.
 */
#define MPU6050_WHO_AM_I_ACCEPTED_IDS   { 0x68, 0x70 }

/*******************************************************************************
 *                          Filtering and sample rate                          *
 *******************************************************************************/

/* DLPF_CFG, low 3 bits of MPU6050_REG_CONFIG. Sets the on-chip low-pass
 * bandwidth AND - critically - the gyro base sample rate that SMPLRT_DIV then
 * divides:
 *
 *   DLPF | accel BW / delay | gyro BW / delay | gyro base
 *     0  | 260 Hz / 0.0 ms  | 256 Hz / 0.98ms |   8 kHz    <- reset default
 *     1  | 184 Hz / 2.0 ms  | 188 Hz / 1.9 ms |   1 kHz
 *     2  |  94 Hz / 3.0 ms  |  98 Hz / 2.8 ms |   1 kHz
 *     3  |  44 Hz / 4.9 ms  |  42 Hz / 4.8 ms |   1 kHz    <- chosen
 *     4  |  21 Hz / 8.5 ms  |  20 Hz / 8.3 ms |   1 kHz
 *     5  |  10 Hz / 13.8ms  |  10 Hz / 13.4ms |   1 kHz
 *     6  |   5 Hz / 19.0ms  |   5 Hz / 18.6ms |   1 kHz
 *
 * SOURCES, so the next reader knows what is verified and what is quoted:
 *   - The base-rate column is confirmed in Tiva_DataSheet/MPU6050-DataSheet.pdf,
 *     "Gyroscope Sample Rate, Fast  DLPFCFG=0            8 kHz" and
 *     "Gyroscope Sample Rate, Slow  DLPFCFG=1,2,3,4,5,6  1 kHz".
 *     The same datasheet gives the programmable LPF range as 5..256 Hz (gyro)
 *     and 5..260 Hz (accel), consistent with DLPF=0 being the unfiltered end.
 *   - The per-setting BW/delay columns are NOT in that datasheet; they come from
 *     the companion "MPU-6000/MPU-6050 Register Map and Descriptions" (which the
 *     datasheet itself points to), and that document is not in this repo.
 *
 * 3 is chosen for a ~50 Hz consumer: ~44/42 Hz of bandwidth is comfortably
 * above the signal of interest on a slow rover while removing the ~260 Hz of
 * unfiltered noise the reset default lets through. It also drops the gyro base
 * to 1 kHz, which makes the divider arithmetic below intuitive.
 *
 * TRADE-OFF, stated explicitly: filtering costs latency. DLPF=3 adds ~4.9 ms
 * (accel) / ~4.8 ms (gyro) of group delay. That is acceptable for odometry
 * fusion at 50 Hz (a 20 ms period); it would NOT be acceptable inside a fast
 * inner control loop. Anyone tightening the loop should revisit this. */
#define MPU6050_DLPF_CFG_VALUE          0x03

/* Output rate = gyro_base / (1 + SMPLRT_DIV).
 * With DLPF enabled (any value 1..6) gyro_base is 1 kHz, so:
 *     1000 / (1 + 19) = 50 Hz
 * matching the ~50 Hz IMU cadence the bench and the DBC 0x150/0x160 consumers
 * expect. There is no point sampling faster than the filter's bandwidth.
 *
 * NOTE this replaces the old 0x07, whose comment claimed "1kHz/8 = 125Hz". That
 * was wrong twice over: the DLPF was never configured, so per the datasheet the
 * base was 8 kHz (DLPFCFG=0) and the true rate was 8000/8 = 1000 Hz, not 125 Hz.
 * Confirmed against Tiva_DataSheet/MPU6050-DataSheet.pdf, not from memory. */
#define MPU6050_SMPLRT_DIV_VALUE        19

#endif /* MPU6050_CFG_H_ */
