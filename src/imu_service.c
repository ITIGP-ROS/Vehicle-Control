/******************************************************************************
 * imu_service.c - application-layer IMU service over the MPU6050 HAL.
 * See imu_service.h for the contract (cadence, sign convention, sequence).
 ******************************************************************************/

#include "imu_service.h"
#include "imu_service_cfg.h"
#include "mpu6050.h"

/*******************************************************************************
 *                          Private State                                      *
 *******************************************************************************/

/* This board's fixed IMU configuration - the service owns it so no consumer
 * has to know the bus or the ranges. */
static const MPU6050_ConfigType ImuService_Config = {
    .i2cId      = IMU_I2C_ID,
    .accelRange = IMU_ACCEL_RANGE,
    .gyroRange  = IMU_GYRO_RANGE
};

/* Per-axis accelerometer scale correction (see imu_service_cfg.h for why this
 * part needs one and how to re-measure it). */
static const float32 ImuService_AccelCorr[3] = IMU_ACCEL_SCALE_CORR;

/* Latest GOOD sample, in engineering units, already corrected and already
 * sign-conventioned. Held across a fault so consumers get stale-but-valid data
 * rather than garbage. */
static MPU6050_DataType ImuService_Sample;

static boolean ImuService_Healthy   = FALSE;
static uint8   ImuService_Sequence  = 0U;

/* Calls since the last re-init attempt. Only counts while unhealthy. */
static uint16  ImuService_BackoffCount = 0U;

/*******************************************************************************
 *                          Private Helpers                                    *
 *******************************************************************************/

/* Latch a raw HAL sample as the service's current sample.
 *
 * This is the ONE place the outgoing sample is shaped, and it does exactly two
 * things beyond copying:
 *   1. applies the accelerometer scale correction, and
 *   2. applies the sensor->robot-frame transform on gyro Z (currently the
 *      IDENTITY for this chip-up mounting - see the note at that line).
 *
 * Both are done HERE, at latch, rather than in the getters, so that (a) there is
 * exactly one application site to audit - matching steering_control's
 * one-transform discipline - and (b) the sample held across a fault is already
 * the corrected, correctly-signed one, so a stale read is consistent with a
 * fresh read. */
static void ImuService_Latch(const MPU6050_DataType *raw)
{
    ImuService_Sample.accelX = raw->accelX * ImuService_AccelCorr[0];
    ImuService_Sample.accelY = raw->accelY * ImuService_AccelCorr[1];
    ImuService_Sample.accelZ = raw->accelZ * ImuService_AccelCorr[2];

    ImuService_Sample.gyroX  =  raw->gyroX;
    ImuService_Sample.gyroY  =  raw->gyroY;

    /* Gyro Z is PASS-THROUGH. This is still the single, audited site for the
     * sensor->robot-frame Z transform (matching steering_control's one-transform
     * discipline); it is just the IDENTITY for this mounting. An MPU6050 mounted
     * CHIP-UP is already right-handed with +Z up, so raw gyro-Z is already
     * positive-for-CCW = REP-103. HANDOFF_README's "TIVA pre-inverts to the robot
     * frame" is a POLICY (the Tiva, not the host, owns the robot-frame sign), and
     * for this orientation that policy resolves to no change.
     *
     * VERIFIED on hardware 2026-08-01 (FIX_26): chip-up, CCW-from-top yields +gz,
     * CW yields -gz. The previous `-raw->gyroZ` produced exactly the opposite on
     * both directions (CCW peaked ~-0.5, CW ~+0.5 rad/s), i.e. the negate turned a
     * already-correct sign into a wrong one.
     *
     * If the IMU is ever remounted (chip-down, or rotated onto another axis) this
     * ONE line is where the sign is re-derived - do not scatter negates downstream. */
    ImuService_Sample.gyroZ  =  raw->gyroZ;

    ImuService_Sample.temp   =  raw->temp;
}

/*******************************************************************************
 *                          Public Functions                                   *
 *******************************************************************************/

void ImuService_Init(void)
{
    MPU6050_StatusType s;

    ImuService_Sample.accelX = 0.0f;
    ImuService_Sample.accelY = 0.0f;
    ImuService_Sample.accelZ = 0.0f;
    ImuService_Sample.gyroX  = 0.0f;
    ImuService_Sample.gyroY  = 0.0f;
    ImuService_Sample.gyroZ  = 0.0f;
    ImuService_Sample.temp   = 0.0f;

    ImuService_Sequence      = 0U;
    ImuService_BackoffCount  = 0U;

    /* An absent sensor must not hang or trap the caller here: MPU6050_Init
     * returns a status and the service simply starts unhealthy, then retries on
     * its own backoff schedule from Update(). */
    s = MPU6050_Init(&ImuService_Config);
    ImuService_Healthy = (s == MPU6050_OK) ? TRUE : FALSE;
}

void ImuService_Update(void)
{
    MPU6050_DataType   raw;
    MPU6050_StatusType s;

    /* Exactly ONE read attempt per call. No retry loop, no delay - the caller
     * owns the cadence. */
    s = MPU6050_ReadData(&raw);

    if (s == MPU6050_OK)
    {
        ImuService_Latch(&raw);
        ImuService_Sequence++;              /* free-running, wraps 255 -> 0 */
        ImuService_Healthy      = TRUE;
        ImuService_BackoffCount = 0U;
        return;
    }

    /* --- fault path ------------------------------------------------------
     * Mark unhealthy and KEEP the last good sample (do not zero it, do not
     * publish the failed read).
     *
     * Recovery is rate limited. After a fault the HAL has forced itself to
     * NOT_INITIALIZED, so the ReadData above short-circuits without touching
     * the bus - failing calls are cheap. The expensive part is MPU6050_Init
     * (6 capped I2C commands), so it runs at most once per
     * IMU_REINIT_BACKOFF_CALLS calls instead of on every failed read. That is
     * REVIEW 22 R22-3: the bench could afford an inline re-init because its
     * 20 ms delay absorbed it; the super-loop cannot, because 6 capped commands
     * would overrun the 1 ms CAN-tx slot. */
    ImuService_Healthy = FALSE;

    ImuService_BackoffCount++;
    if (ImuService_BackoffCount >= IMU_REINIT_BACKOFF_CALLS)
    {
        ImuService_BackoffCount = 0U;
        (void)MPU6050_Init(&ImuService_Config);   /* ONE attempt, then wait */
    }
}

boolean ImuService_IsHealthy(void)
{
    return ImuService_Healthy;
}

void ImuService_GetAccel(float32 *ax, float32 *ay, float32 *az)
{
    /* Each pointer is optional so a caller can ask for a subset. */
    if (ax != NULL_PTR) { *ax = ImuService_Sample.accelX; }
    if (ay != NULL_PTR) { *ay = ImuService_Sample.accelY; }
    if (az != NULL_PTR) { *az = ImuService_Sample.accelZ; }
}

void ImuService_GetGyro(float32 *gx, float32 *gy, float32 *gz)
{
    if (gx != NULL_PTR) { *gx = ImuService_Sample.gyroX; }
    if (gy != NULL_PTR) { *gy = ImuService_Sample.gyroY; }
    if (gz != NULL_PTR) { *gz = ImuService_Sample.gyroZ; }
}

float32 ImuService_GetTemp(void)
{
    return ImuService_Sample.temp;
}

uint8 ImuService_GetSequence(void)
{
    return ImuService_Sequence;
}
