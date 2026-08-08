#include "mpu6050.h"
#include "mpu6050_cfg.h"   /* address, register map, WHO_AM_I id, rate, I2C cap */
#include "i2c.h"

/*******************************************************************************
 *                          Private Macros                                     *
 *******************************************************************************/

/* Conversion constants stay HERE, not in mpu6050_cfg.h: they are mathematical
 * facts, not board configuration - nothing about a wiring change should ever
 * make someone want to edit them. */
#define MPU6050_GRAVITY_MPS2        9.80665f
#define MPU6050_DEG2RAD             0.0174532925f

/*******************************************************************************
 *                          Private Variables                                  *
 *******************************************************************************/

static boolean g_MPU6050_Initialized = FALSE;
static I2C_IdType g_MPU6050_I2cId = I2C_ID_0;

/* Last raw transport error behind an MPU6050_ERROR_I2C_FAILED. The HAL keeps a
 * coarse status enum (callers should not have to reason about the I2C layer),
 * but flattening NO_ACK / TIMEOUT / BUS_STUCK / BUS_BUSY / ARBITRATION_LOST into
 * one value throws away the single most useful diagnostic bit: whether nothing
 * answered (wiring / address / power) or the bus misbehaved (pull-ups / timing).
 * Cached here and exposed via MPU6050_GetLastI2cError() so a bench can report
 * the cause without widening the HAL's own status enum. */
static I2C_StatusType g_MPU6050_LastI2cError = I2C_OK;

/* WHO_AM_I ids this build accepts, and the byte the device actually returned on
 * the most recent Init. The latter is what makes a NOT_FOUND actionable: without
 * it "not found" cannot distinguish a dead bus from a present-but-unlisted
 * clone. Exposed via MPU6050_GetLastWhoAmI(). */
static const uint8 g_MPU6050_AcceptedIds[] = MPU6050_WHO_AM_I_ACCEPTED_IDS;
static uint8 g_MPU6050_LastWhoAmI = 0x00;

static float32 g_MPU6050_AccelScale = 1.0f;
static float32 g_MPU6050_GyroScale = 1.0f;

/*******************************************************************************
 *                          Private Helper Functions                           *
 *******************************************************************************/

/*=============================================================================
 * BUS RECOVERY - the HAL half of i2c.h's §3 contract.
 *
 * i2c.h assigns the split explicitly, and this is the side it gives us:
 *
 *   "the MCAL owns the mechanics, the HAL owns ALL policy (how many steps,
 *    how many retries, when to give up)"
 *   "HAL drives them ONLY on I2C_ERROR_BUS_STUCK (see the integration sketch):
 *      RecoverBegin(id);
 *      for (n=0; n<I2C_RECOVER_MAX_STEPS; n++) if (RecoverStep(id)) break;
 *      status = RecoverEnd(id);"
 *
 * 🔴 Until 2026-08-08 NOTHING in the production image implemented this side of
 * the contract - the primitives were reachable only from test/bringup, and only
 * for the INA226's bus. The result was measured, not theorised: a BUS_STUCK was
 * correctly DETECTED (SDA held low mid-burst) and then thrown away, because the
 * only thing the HAL did was I2C_Reset(), which re-inits the MASTER and cannot
 * touch a SLAVE holding the line. The IMU stayed wedged across MCU resets and a
 * reflash. See the DIAG block in docs/MEMORY.md.
 *
 * ⚠️ THE GUARD IS PART OF THE CONTRACT, not an optimisation: BUS_STUCK is the
 * ONLY status allowed in here. i2c.h is explicit that a TIMEOUT means the bus is
 * already electrically clean, and that "SCL toggling cannot revive a dead slave,
 * so a TIMEOUT must NEVER be escalated into this recovery path". Callers below
 * enforce that; do not relax it.
 *
 * ⚠️ PER-i2cId, so this cannot disturb the other bus. Every primitive takes the
 * id and only ever touches that module's own registers and its own two pins
 * (I2C_Module[]); g_MPU6050_I2cId is I2C1/PA6/PA7. The INA226 on I2C0/PB2/PB3 is
 * untouched by anything here - the two devices share this MECHANISM, not a bus.
 *
 * COST - MEASURED, not estimated, because the estimate was too kind. The
 * recovery ITSELF is ~185 us (Begin ~15 us + up to 9 steps x 15 us + End ~35 us,
 * every wait TIMER0-timed). But the whole fault path around it is what tImu
 * actually pays: the failing command burns its 200 us cap plus a 200 us abort,
 * then the 14-byte burst is retried in full. On the bench that took tImu's worst
 * observed Update from 924 us (healthy) to ~2.65 ms on a recovering cycle -
 * roughly +1.7 ms, NOT +185 us.
 *
 * That is affordable here and was verified, not assumed: tImu's period is 20 ms
 * so a 2.7 ms run is 13 % of its own slot, and although tImu outranks tCanTx a
 * 7 min soak with 64 recoveries held tx queue peak=2, qfull=0, txfail=0,
 * txtmo=0, vel_ivl=20/20ms, skip=0, with all six CAN frames at nominal rate.
 * The HEALTHY path is completely unchanged - none of this executes unless a
 * command actually fails.
 *===========================================================================*/
static uint8 g_MPU6050_LastRecoverPulses = 0U;
static uint16 g_MPU6050_RecoverCount = 0U;

uint8  MPU6050_GetLastRecoverPulses(void) { return g_MPU6050_LastRecoverPulses; }
uint16 MPU6050_GetRecoverCount(void)      { return g_MPU6050_RecoverCount; }

static I2C_StatusType MPU6050_RecoverBus(void)
{
    uint8 n;
    I2C_StatusType st;

    I2C_RecoverBegin(g_MPU6050_I2cId);
    for (n = 0U; n < I2C_RECOVER_MAX_STEPS; n++)
    {
        if (I2C_RecoverStep(g_MPU6050_I2cId) != 0U)
        {
            break;      /* SDA released - stop early, exactly as i2c.h says */
        }
    }
    st = I2C_RecoverEnd(g_MPU6050_I2cId);

    /* Pulses actually EMITTED, not the loop index: breaking at n means n+1
     * pulses went out, and running to completion means the full cap. Reporting
     * the raw n printed "0 pulses" for the common case of the very first pulse
     * freeing the line, which reads as "recovery did nothing".
     *
     * The count is informative, not cosmetic. 1-4 is the healthy signature -
     * recovery now fires on the FIRST BUS_STUCK, before the slave has been
     * abandoned long, and sometimes the line is already free by the time we look
     * (the mode where the MASTER latched BUSBSY, not the slave holding SDA).
     * The bench probe measured 4-7 on a bus that had been wedged for minutes.
     * A count drifting upward means the underlying electrical trigger - which is
     * deliberately NOT fixed here, because it was never isolated - is worsening. */
    g_MPU6050_LastRecoverPulses = (n < I2C_RECOVER_MAX_STEPS) ? (uint8)(n + 1U) : (uint8)n;
    g_MPU6050_RecoverCount++;

    return st;
}

/* TRUE if this transport status is one the contract lets us recover from. */
static boolean MPU6050_IsRecoverable(I2C_StatusType s)
{
    return (boolean)(s == I2C_ERROR_BUS_STUCK);
}

static MPU6050_StatusType MPU6050_WriteRegister(uint8 regAddr, uint8 data) {
    /* cap_ticks: per-command TIMER0-tick timeout owned by the caller (see i2c.h).
     * REQUIRES Timer0_FreeRunning_Init() to have run - otherwise the wait hangs
     * forever (GPTMTAR static). MPU6050_Init documents this ordering dependency. */
    I2C_StatusType s = I2C_Write(g_MPU6050_I2cId, MPU6050_ADDRESS, regAddr, &data, 1,
                                 MPU6050_I2C_CAP_TICKS);

    if (MPU6050_IsRecoverable(s) != FALSE) {
        if (MPU6050_RecoverBus() == I2C_OK) {
            s = I2C_Write(g_MPU6050_I2cId, MPU6050_ADDRESS, regAddr, &data, 1,
                          MPU6050_I2C_CAP_TICKS);      /* ONE retry, never a loop */
        }
    }

    if (s != I2C_OK) {
        g_MPU6050_LastI2cError = s;          /* keep the real cause for diagnostics */
        return MPU6050_ERROR_I2C_FAILED;
    }
    return MPU6050_OK;
}

static MPU6050_StatusType MPU6050_ReadRegisters(uint8 regAddr, uint8 *data, uint16 length) {
    I2C_StatusType s = I2C_Read(g_MPU6050_I2cId, MPU6050_ADDRESS, regAddr, data, length,
                                MPU6050_I2C_CAP_TICKS);

    /* 🔴 DIAG defect 1+2, closed HERE and deliberately at this level.
     *
     * Every I2C access this HAL makes funnels through these two wrappers - the
     * WHO_AM_I probe in MPU6050_Init, the six config writes, and ReadRaw's
     * 14-byte burst. Putting the recovery here therefore closes BOTH measured
     * defects with one call site:
     *   - defect 1: a live BUS_STUCK now actually reaches the SCL-toggle
     *     recovery instead of being flattened and dropped; and
     *   - defect 2: MPU6050_Init's own WHO_AM_I read can now un-wedge the bus.
     *     That matters because once ReadRaw marks the device uninitialised,
     *     ReadData short-circuits on NOT_INITIALIZED and never touches the bus
     *     again - so Init, driven by imu_service's backoff, is the ONLY path
     *     left. Before this fix it died at the entry BUSBSY guard having
     *     attempted nothing, which is what made the wedge permanent.
     *
     * ONE retry, not a loop: if a freshly-cleared bus still fails, the fault is
     * not a stranded slave and pulsing again will not help. The caller's own
     * backoff owns any further attempt. */
    if (MPU6050_IsRecoverable(s) != FALSE) {
        if (MPU6050_RecoverBus() == I2C_OK) {
            s = I2C_Read(g_MPU6050_I2cId, MPU6050_ADDRESS, regAddr, data, length,
                         MPU6050_I2C_CAP_TICKS);
        }
    }

    if (s != I2C_OK) {
        g_MPU6050_LastI2cError = s;          /* keep the real cause for diagnostics */
        return MPU6050_ERROR_I2C_FAILED;
    }
    return MPU6050_OK;
}

I2C_StatusType MPU6050_GetLastI2cError(void) {
    return g_MPU6050_LastI2cError;
}

uint8 MPU6050_GetLastWhoAmI(void) {
    return g_MPU6050_LastWhoAmI;
}

/* TRUE if the id is in this build's accepted set (see mpu6050_cfg.h). */
static boolean MPU6050_IsAcceptedId(uint8 id) {
    uint8 i;
    for (i = 0U; i < (uint8)(sizeof(g_MPU6050_AcceptedIds) / sizeof(g_MPU6050_AcceptedIds[0])); i++) {
        if (g_MPU6050_AcceptedIds[i] == id) {
            return TRUE;
        }
    }
    return FALSE;
}

static void MPU6050_CalculateScales(MPU6050_AccelRangeType accelRange, MPU6050_GyroRangeType gyroRange) {
    switch (accelRange) {
        case MPU6050_ACCEL_RANGE_2G:  g_MPU6050_AccelScale = 16384.0f; break;
        case MPU6050_ACCEL_RANGE_4G:  g_MPU6050_AccelScale = 8192.0f;  break;
        case MPU6050_ACCEL_RANGE_8G:  g_MPU6050_AccelScale = 4096.0f;  break;
        case MPU6050_ACCEL_RANGE_16G: g_MPU6050_AccelScale = 2048.0f;  break;
    }
    
    switch (gyroRange) {
        case MPU6050_GYRO_RANGE_250DPS:  g_MPU6050_GyroScale = 131.0f; break;
        case MPU6050_GYRO_RANGE_500DPS:  g_MPU6050_GyroScale = 65.5f;  break;
        case MPU6050_GYRO_RANGE_1000DPS: g_MPU6050_GyroScale = 32.8f;  break;
        case MPU6050_GYRO_RANGE_2000DPS: g_MPU6050_GyroScale = 16.4f;  break;
    }
}

/*******************************************************************************
 *                          Public Functions                                   *
 *******************************************************************************/

/* STARTUP ORDER (hard dependency): every I2C command below is TIMER0-tick capped.
 * Timer0_FreeRunning_Init() AND I2C_Init() MUST have run before MPU6050_Init() is
 * called. If TIMER0 is not free-running, GPTMTAR is static, the per-command cap
 * never elapses, and the very first WHO_AM_I read hangs forever instead of timing
 * out. This is the one ordering invariant the caller's bring-up sequence must
 * guarantee (see i2c.h "REQUIRES Timer0_FreeRunning_Init()"). */
MPU6050_StatusType MPU6050_Init(const MPU6050_ConfigType *config) {
    if (config == NULL_PTR) return MPU6050_ERROR_NULL_PTR;
    
    g_MPU6050_I2cId = config->i2cId;
    
    /* 1. Check WHO_AM_I register to verify device presence */
    uint8 whoAmI = 0;
    if (MPU6050_ReadRegisters(MPU6050_REG_WHO_AM_I, &whoAmI, 1) != MPU6050_OK) {
        return MPU6050_ERROR_I2C_FAILED;
    }
    
    /* Record what the part actually answered BEFORE judging it, so a rejected
     * id is still reportable (MPU6050_GetLastWhoAmI). */
    g_MPU6050_LastWhoAmI = whoAmI;

    if (MPU6050_IsAcceptedId(whoAmI) == FALSE) {
        return MPU6050_ERROR_NOT_FOUND;
    }
    
    /* 2. Wake up MPU6050 (clear sleep bit in PWR_MGMT_1) */
    if (MPU6050_WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x00) != MPU6050_OK) {
        return MPU6050_ERROR_I2C_FAILED;
    }
    
    /* 3. Digital low-pass filter. MUST be written BEFORE the sample-rate
     * divider is meaningful: DLPF_CFG selects the gyro BASE rate that
     * SMPLRT_DIV then divides (8 kHz when disabled, 1 kHz when enabled), as
     * well as the accel/gyro bandwidth. Leaving it at reset - which this driver
     * previously did - left the sensors unfiltered at ~260 Hz AND made the
     * divider divide 8 kHz instead of 1 kHz. See MPU6050_DLPF_CFG_VALUE. */
    if (MPU6050_WriteRegister(MPU6050_REG_CONFIG, MPU6050_DLPF_CFG_VALUE) != MPU6050_OK) {
        return MPU6050_ERROR_I2C_FAILED;
    }

    /* 4. Sample rate = gyro_base / (1 + SMPLRT_DIV) = 1000 / (1 + 19) = 50 Hz,
     * matching the IMU cadence the consumers actually want. */
    if (MPU6050_WriteRegister(MPU6050_REG_SMPLRT_DIV, MPU6050_SMPLRT_DIV_VALUE) != MPU6050_OK) {
        return MPU6050_ERROR_I2C_FAILED;
    }
    
    /* 5. Configure Accel Range */
    if (MPU6050_WriteRegister(MPU6050_REG_ACCEL_CONFIG, (uint8)config->accelRange) != MPU6050_OK) {
        return MPU6050_ERROR_I2C_FAILED;
    }
    
    /* 6. Configure Gyro Range */
    if (MPU6050_WriteRegister(MPU6050_REG_GYRO_CONFIG, (uint8)config->gyroRange) != MPU6050_OK) {
        return MPU6050_ERROR_I2C_FAILED;
    }
    
    /* Pre-calculate scaling factors */
    MPU6050_CalculateScales(config->accelRange, config->gyroRange);
    
    g_MPU6050_Initialized = TRUE;
    
    return MPU6050_OK;
}

MPU6050_StatusType MPU6050_DeInit(void) {
    /* Put device to sleep */
    MPU6050_WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x40);
    g_MPU6050_Initialized = FALSE;
    return MPU6050_OK;
}

MPU6050_StatusType MPU6050_ReadRaw(MPU6050_RawDataType *data) {
    if (data == NULL_PTR) return MPU6050_ERROR_NULL_PTR;
    if (!g_MPU6050_Initialized) return MPU6050_ERROR_NOT_INITIALIZED;
    
    uint8 buffer[14];
    
    /* Burst read starting from ACCEL_XOUT_H (14 bytes total) */
    if (MPU6050_ReadRegisters(MPU6050_REG_ACCEL_XOUT_H, buffer, 14) != MPU6050_OK) {

        /* [SELF-HEALING] Responsibility split (see i2c.h):
         *   - MCAL owns the bus mechanics. I2C_Reset() is now a pure peripheral
         *     re-init that re-applies the cached bus config, and the I2C command
         *     path already runs auto-abort/SCL-toggle recovery internally. So the
         *     HAL must NOT re-drive the bus or re-apply the OLD hardcoded 0x00
         *     ranges here (that silently de-calibrated the scales after a fault).
         *   - This HAL owns SENSOR policy: after a bus reset the MPU still needs
         *     re-configuring from the SAME config the last Init used. We do NOT
         *     have that config cached here, so we mark the device uninitialized
         *     and force the caller to re-run MPU6050_Init() with its real config.
         *     A read after a fault therefore fails LOUDLY (NOT_INITIALIZED on the
         *     next call) instead of silently coming back at wrong scales. */
        I2C_Reset(g_MPU6050_I2cId);
        g_MPU6050_Initialized = FALSE;
        return MPU6050_ERROR_I2C_FAILED;
    }
    
    /* Combine High and Low bytes */
    data->accelX = (sint16)((buffer[0] << 8) | buffer[1]);
    data->accelY = (sint16)((buffer[2] << 8) | buffer[3]);
    data->accelZ = (sint16)((buffer[4] << 8) | buffer[5]);
    data->temp   = (sint16)((buffer[6] << 8) | buffer[7]);
    data->gyroX  = (sint16)((buffer[8] << 8) | buffer[9]);
    data->gyroY  = (sint16)((buffer[10] << 8) | buffer[11]);
    data->gyroZ  = (sint16)((buffer[12] << 8) | buffer[13]);
    
    return MPU6050_OK;
}

MPU6050_StatusType MPU6050_ReadData(MPU6050_DataType *data) {
    if (data == NULL_PTR) return MPU6050_ERROR_NULL_PTR;
    
    MPU6050_RawDataType rawData;
    MPU6050_StatusType status = MPU6050_ReadRaw(&rawData);
    
    if (status != MPU6050_OK) return status;
    
    /* Convert raw to proper units (Accel = m/s^2, Gyro = rad/s) */
    data->accelX = ((float32)rawData.accelX / g_MPU6050_AccelScale) * MPU6050_GRAVITY_MPS2;
    data->accelY = ((float32)rawData.accelY / g_MPU6050_AccelScale) * MPU6050_GRAVITY_MPS2;
    data->accelZ = ((float32)rawData.accelZ / g_MPU6050_AccelScale) * MPU6050_GRAVITY_MPS2;
    
    data->gyroX = ((float32)rawData.gyroX / g_MPU6050_GyroScale) * MPU6050_DEG2RAD;
    data->gyroY = ((float32)rawData.gyroY / g_MPU6050_GyroScale) * MPU6050_DEG2RAD;
    data->gyroZ = ((float32)rawData.gyroZ / g_MPU6050_GyroScale) * MPU6050_DEG2RAD;
    
    data->temp = ((float32)rawData.temp / 340.0f) + 36.53f;

    /* NOTE: the old ~2 ms blocking spin here (for(volatile d<20000)) was REMOVED.
     * It stalled the cooperative super-loop and broke the one-CAN-transmit-per-1ms
     * -tick invariant in main.c. The MPU-holds-SDA-low-on-back-to-back-reads quirk
     * it worked around is a CADENCE problem, not a delay-inside-the-driver problem:
     * the caller must not poll ReadData() faster than the frame is produced. At the
     * intended ~50 Hz IMU rate (20 ms period) consecutive reads are naturally spaced
     * far beyond the quirk window, so no in-driver delay is needed. Do NOT reintroduce
     * a spin here - if a tighter poll is ever required, space it with a TIMER0 gate
     * at the call site, not a NOP loop inside the MCAL/HAL. */

    return MPU6050_OK;
}
