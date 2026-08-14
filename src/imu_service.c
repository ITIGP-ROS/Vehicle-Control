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

/* Per-axis gyroscope scale correction (2026-08-14). Same story as the accel one
 * directly above - this part's real LSB/dps is not the nominal 131 - and it is
 * applied at the SAME single site, ImuService_Latch. See imu_service_cfg.h for
 * the measured figure and, more importantly, for how to re-measure it (with
 * gravity as the reference; a hand-turned "360 degrees" is NOT a usable one). */
static const float32 ImuService_GyroCorr[3] = IMU_GYRO_SCALE_CORR;

/* Latest GOOD sample, in engineering units, already corrected and already
 * sign-conventioned. Held across a fault so consumers get stale-but-valid data
 * rather than garbage. */
static MPU6050_DataType ImuService_Sample;

static boolean ImuService_Healthy   = FALSE;
static uint8   ImuService_Sequence  = 0U;

/* Calls since the last re-init attempt. Only counts while unhealthy. */
static uint16  ImuService_BackoffCount = 0U;

/* B13. TRUE once a first good sample has ever been latched - the publish gate.
 * Without it the all-zero boot state would go out as a VALID, correctly-paired
 * reading of 0 g. See the long note on ImuService_HasSample() in the header. */
static boolean ImuService_HasSampleFlag = FALSE;

/* B13. Samples still to be published with 0x160's imu_reset bit SET. A level,
 * not a pulse - see ImuService_GetResetFlag() in the header for why. */
static uint16  ImuService_ResetHold = 0U;

/*******************************************************************************
 *                          Private Helpers                                    *
 *******************************************************************************/

/* Latch a raw HAL sample as the service's current sample.
 *
 * This is the ONE place the outgoing sample is shaped, and it does exactly three
 * things beyond copying:
 *   1. applies the accelerometer scale correction,
 *   2. applies the 180 deg MOUNT YAW correction (B13c - see the block below), and
 *   3. applies the sensor->robot-frame transform on gyro Z (the IDENTITY - see
 *      the note at that line; do NOT change it).
 *
 * All are done HERE, at latch, rather than in the getters, so that (a) there is
 * exactly one application site to audit - matching steering_control's
 * one-transform discipline - and (b) the sample held across a fault is already
 * the corrected, correctly-signed one, so a stale read is consistent with a
 * fresh read.
 *
 * =====================================================================
 *  THE 180 DEG MOUNT YAW CORRECTION (B13c, 2026-08-07) - A MOUNT FACT,
 *  NOT AN ARBITRARY SIGN CHOICE.
 * =====================================================================
 * The MPU6050 is physically mounted rotated 180 deg about +Z relative to the
 * robot frame that the DBC, the URDF and the odometry all assume. MEASURED on
 * hardware (B13b, 2026-08-07), decoding the live 0x150/0x160 frames:
 *
 *     nose UP, held        -> accelX -4.390  (26.5 deg tilt)  => sensor +X = REAR
 *     left side UP, held   -> accelY -5.617  (34.0 deg tilt)  => sensor +Y = RIGHT
 *     right side DOWN      -> accelY -10.75                   => +Y = RIGHT (again)
 *
 * (An accelerometer at rest reads +g on whichever axis points UP. accelX going
 * NEGATIVE when the nose rose therefore means +X points at the REAR.)
 * The frame is right-handed and self-consistent: rear x right = up. REP-103
 * wants +X forward, +Y left, +Z up, so the sensor is a clean 180 deg yaw off.
 *
 * A 180 deg rotation about +Z maps (x, y, z) -> (-x, -y, z). Hence EXACTLY FOUR
 * sign flips: accelX, accelY, gyroX, gyroY.
 *
 * ⚠️⚠️ gyroZ AND accelZ ARE DELIBERATELY NOT TOUCHED.
 *   - YAW RATE IS INVARIANT UNDER A YAW ROTATION. gyroZ is already correct, and
 *     that invariance is exactly WHY this mount error hid for so long: every
 *     earlier check looked either at |a| (a magnitude, blind to direction) or at
 *     gz (the one component a 180 deg yaw cannot change). Negating gyroZ here
 *     would BREAK the FIX_26-verified CCW -> +gz contract.
 *   - +Z already maps to +Z (up = up), so accelZ needs no change either.
 *
 * ⚠️ The correction lives HERE and NOWHERE ELSE. Do not add a sign flip in
 * jetson_comm, at pack time, in the DBC, or in the URDF - the DBC contract is
 * "the TIVA delivers the robot frame", and this function is what MAKES that
 * true. If the module is ever physically remounted, this block is the single
 * place to re-derive; delete the negates rather than adding compensating ones.
 * ===================================================================== */
static void ImuService_Latch(const MPU6050_DataType *raw)
{
    /* Scale correction AND the 180 deg mount yaw, together: X and Y negate,
     * Z does not. The negate wraps the corrected value, so the magnitude is
     * untouched - |a| still reads ~9.81 and the scale calibration is unaffected. */
    ImuService_Sample.accelX = -(raw->accelX * ImuService_AccelCorr[0]);
    ImuService_Sample.accelY = -(raw->accelY * ImuService_AccelCorr[1]);
    ImuService_Sample.accelZ =  (raw->accelZ * ImuService_AccelCorr[2]);

    /* Same 180 deg yaw on the rate axes: roll/pitch rates flip, yaw rate does not.
     * The 2026-08-14 gyro scale correction rides along here, exactly as the accel
     * correction does above: a MULTIPLY commutes with the negate, so the sign
     * convention below is completely unaffected by it. */
    ImuService_Sample.gyroX  = -(raw->gyroX * ImuService_GyroCorr[0]);
    ImuService_Sample.gyroY  = -(raw->gyroY * ImuService_GyroCorr[1]);

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
     * ONE line is where the sign is re-derived - do not scatter negates downstream.
     *
     * ⚠️⚠️ B13c: THE 180 DEG MOUNT-YAW CORRECTION ABOVE DOES **NOT** APPLY HERE,
     * and that is not an oversight. Yaw rate is INVARIANT under a rotation about
     * yaw, so a 180 deg mount yaw leaves gyroZ alone while flipping X and Y.
     * If you are "making this consistent" with the accelX/accelY negates, STOP -
     * you would be re-introducing the exact bug FIX_26 removed.
     *
     * ⚠️ 2026-08-14: the gyro SCALE correction is applied here too, and it does
     * NOT disturb any of the above. It is a positive multiplier - it changes the
     * MAGNITUDE only, never the sign - so the FIX_26 "CCW from top yields +gz"
     * contract still holds exactly as verified. This line remains the identity
     * for the mount TRANSFORM; the multiply is a per-part gain fix, a separate
     * concern that just happens to live at the same single site. */
    ImuService_Sample.gyroZ  =  raw->gyroZ * ImuService_GyroCorr[2];

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
    ImuService_HasSampleFlag = FALSE;
    ImuService_ResetHold     = 0U;

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
        /* B13 - THE RECOVERY EDGE, sampled BEFORE Healthy is overwritten.
         *
         * "Recovered" means: we were unhealthy, AND we had already produced a
         * good sample at some earlier point. The second half is what excludes
         * the first-ever sample after boot - coming up for the first time is
         * not a re-initialisation, and the host has no calibration to protect
         * yet. Announcing one there would train the consumer to ignore the bit. */
        boolean recovered = ((ImuService_Healthy == FALSE) &&
                             (ImuService_HasSampleFlag != FALSE)) ? TRUE : FALSE;

        ImuService_Latch(&raw);
        ImuService_Sequence++;              /* free-running, wraps 255 -> 0 */
        ImuService_Healthy       = TRUE;
        ImuService_HasSampleFlag = TRUE;
        ImuService_BackoffCount  = 0U;

        /* RE-ARM on every recovery rather than only when the window is idle, so
         * a sensor that keeps dropping out keeps the bit asserted throughout
         * instead of letting it lapse between two closely-spaced faults. */
        if (recovered != FALSE)
        {
            ImuService_ResetHold = IMU_RESET_HOLD_SAMPLES;
        }
        else if (ImuService_ResetHold > 0U)
        {
            ImuService_ResetHold--;
        }
        else
        {
            /* steady state - nothing to do */
        }

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

boolean ImuService_HasSample(void)
{
    return ImuService_HasSampleFlag;
}

boolean ImuService_GetResetFlag(void)
{
    return (ImuService_ResetHold > 0U) ? TRUE : FALSE;
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
