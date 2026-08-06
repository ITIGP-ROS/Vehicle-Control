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
 */
#define MPU6050_WHO_AM_I_ACCEPTED_IDS   { 0x68 }

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
