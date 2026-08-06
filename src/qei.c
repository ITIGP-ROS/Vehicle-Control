/******************************************************************************
 *
 * Module: QEI (Quadrature Encoder Interface)
 *
 * File Name: qei.c
 *
 * Description: Source file for TM4C123GH6PM QEI driver
 *              Designed for DC motor encoder feedback
 *
 ******************************************************************************/

#include "qei.h"
#include "qei_private.h"
//#include <math.h>

/*******************************************************************************
 *                    Compile-time check for V9-5 (REVIEW 09)                  *
 *
 * The sensor-domain sign for the mirror-mounted pair is compensated EXACTLY
 * ONCE, by QEI0_SWAP_SIGNALS on the right channel (hardware QEICTL.SWAP,
 * hand-spin verified - see the block comment at qei_cfg.h:170-185). But THREE
 * independent switches can flip that sign:
 *
 *   1. QEI0/QEI1_SWAP_SIGNALS      - the sanctioned one (1 / 0)
 *   2. QEI0/QEI1_INVERT_DIRECTION  - a second inversion, unused (0 / 0)
 *   3. Encoder_Handles[].config.invertDirection - a third, runtime (FALSE)
 *
 * Setting either currently-zero compile-time switch DOUBLE-compensates the
 * right wheel, which puts the velocity PID into POSITIVE FEEDBACK. qei_cfg.h
 * warns that getting this wrong "silently inverts every position/velocity sign
 * for one motor"; this pins switch 2 at BUILD time so the warning cannot be
 * missed. (Switch 3 is a runtime field and cannot be pinned here - see the
 * notes at Encoder_Init and the runtime setter in encoder.c.)
 *
 * If a remount ever genuinely needs a sign change, make it in ONE place and
 * update this assert deliberately.
 *
 * gnu90 / GCC 4.8, so _Static_assert is unavailable: C90 negative-array-size
 * idiom, same as FIX 24 / FIX 25. Emits no code.
 *******************************************************************************/

typedef char QEI_AssertNoDoubleInvert[(((QEI0_INVERT_DIRECTION) == 0U) &&
                                       ((QEI1_INVERT_DIRECTION) == 0U)) ? 1 : -1];

/*******************************************************************************
 *                          Private Types                                      *
 *******************************************************************************/

typedef struct {
    uint32              baseAddr;
    QEI_StateType       state;
    uint32              lastPosition;
    sint32              totalCounts;
    float32             totalDistance;
    boolean             distanceInitialized;
    /* Velocity-SIGN state, owned SOLELY by QEI_GetVelocityRPM (Option 1 hybrid, see
     * QEI_VELOCITY_RESIDUAL_AUDIT.md). Kept SEPARATE from lastPosition/totalCounts/
     * totalDistance above (those belong to QEI_UpdateDistance / odometry) so the
     * two mechanisms never alias. Advanced exactly once per completed hardware
     * velocity window (gated on the QEISPEED velocity-timer RIS flag). */
    uint32              lastVelPosition;   /* QEIPOS snapshot at last completed velocity window */
    float32             lastVelSign;       /* cached sign (+-1), reused by intra-window callers  */
    boolean             velSignInit;       /* FALSE until the first windowed sample is taken      */
    QEI_CallbackConfigType callbacks;
} QEI_ChannelHandleType;

/*******************************************************************************
 *                          Private Variables                                  *
 *******************************************************************************/

static QEI_ChannelHandleType QEI_Channels[QEI_CHANNEL_MAX] = {
    {
        .baseAddr = QEI0_BASE_ADDR,
        .state = QEI_STATE_UNINIT,
        .lastPosition = 0,
        .totalCounts = 0,
        .totalDistance = 0.0f,
        .distanceInitialized = FALSE
    },
    {
        .baseAddr = QEI1_BASE_ADDR,
        .state = QEI_STATE_UNINIT,
        .lastPosition = 0,
        .totalCounts = 0,
        .totalDistance = 0.0f,
        .distanceInitialized = FALSE
    }
};

/* Odometry state */
static QEI_OdometryType QEI_Odometry = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static sint32 QEI_OdometryLastCounts[2] = {0, 0};
static boolean QEI_OdometryInitialized = FALSE;

/*******************************************************************************
 *                          Private Functions                                  *
 *******************************************************************************/

/**
 * @brief  Configure QEI module registers
 */
static void QEI_ConfigureModule(QEI_ChannelType channel)
{
    uint32 base = QEI_Channels[channel].baseAddr;
    uint32 ctl = 0;
    
    /* Disable QEI during configuration */
    QEI_CTL(base) = 0;
    
    /* Build control register value */
    
    /* Quadrature mode (not clock/direction) */
    ctl |= QEI_CTL_SIGMODE_QUADRATURE;
    
    /* Enable velocity capture, and select x4 decode (both edges of both
     * PhA/PhB) - see qei_private.h for why this is CAPMODE, not VELEN */
    ctl |= QEI_CTL_VELEN_MASK;
    ctl |= QEI_CTL_CAPMODE_BOTH_PHASES;
    
    /* Reset mode: INDEX (RESMODE=0, the value ctl already has - this OR
     * is a no-op, kept only for symmetry/clarity). This encoder has no
     * index/Z channel (QEI_INDEX_ENABLE=0 in qei_cfg.h), so QEIPOS never
     * auto-resets in hardware either way - it free-runs across the full
     * 32-bit range (QEI_MAXPOS_VALUE=0xFFFFFFFF) and wraps there, with
     * software (QEI_CalculateDelta) tracking signed multi-revolution
     * totals across that wrap. A MAXPOS-reset, wrap-per-revolution
     * scheme was considered (see the commented-out QEI_MAXPOS_VALUE in
     * qei_cfg.h) but is NOT what's active - don't assume otherwise. */
    ctl |= QEI_CTL_RESMODE_INDEX;
    
    /* Stall during debug */
    ctl |= QEI_CTL_STALLEN_MASK;
    
    /* Velocity divider = 1 (no predivision) */
    ctl |= QEI_CTL_VELDIV_1;
    
#if (QEI_FILTER_ENABLE == 1U)
    ctl |= QEI_CTL_FILTEN_MASK;
    ctl |= ((uint32)QEI_FILTER_COUNT << QEI_CTL_FILTCNT_POS);
#endif

    /* Channel-specific swap configuration */
    if (channel == QEI_CHANNEL_0)
    {
#if (QEI0_SWAP_SIGNALS == 1U)
        ctl |= QEI_CTL_SWAP_MASK;
#endif
#if (QEI0_INVERT_DIRECTION == 1U)
        ctl |= QEI_CTL_INVA_MASK;
#endif
    }
    else
    {
#if (QEI1_SWAP_SIGNALS == 1U)
        ctl |= QEI_CTL_SWAP_MASK;
#endif
#if (QEI1_INVERT_DIRECTION == 1U)
        ctl |= QEI_CTL_INVA_MASK;
#endif
    }
    
#if (QEI_INDEX_ENABLE == 1U)
#if (QEI_INDEX_RESET_ENABLE == 1U)
    ctl &= ~QEI_CTL_RESMODE_MASK;  /* Reset on index */
#endif
#endif
    
    /* Set maximum position - full 32-bit free-run (0xFFFFFFFF), see the
     * RESMODE comment above for why this is not "counts per rev - 1" */
    QEI_MAXPOS(base) = QEI_MAXPOS_VALUE;
    
    /* Set velocity timer load value */
    QEI_LOAD(base) = QEI_VELOCITY_LOAD_VALUE;
    
    /* Reset position counter */
    QEI_POS(base) = 0;
    
    /* Clear any pending interrupts */
    QEI_ISC(base) = QEI_INT_ALL_MASK;
    
    /* Apply control register */
    QEI_CTL(base) = ctl;
    
    /* Enable QEI */
    QEI_CTL(base) |= QEI_CTL_ENABLE_MASK;
}

/**
 * @brief  Calculate signed delta counts between two raw QEIPOS samples
 * @note   Valid ONLY for this driver's full-width free-run scheme
 *         (QEI_MAXPOS_VALUE = 0xFFFFFFFF, RESMODE = INDEX - see
 *         qei_cfg.h/QEI_ConfigureModule). With MAXPOS spanning the full
 *         32-bit range, plain two's-complement subtraction already
 *         yields the correct signed delta across the 0xFFFFFFFF<->0
 *         wrap for any realistic single-update movement (anything under
 *         +/-2^31 counts between calls) - no extra wrap-detection is
 *         needed or correct here.
 *         This function previously took maxPos/dir parameters and had
 *         explicit "wrapped" branches comparing delta against
 *         +/-0x7FFFFFFF. Both were dead code: a sint32 can never exceed
 *         0x7FFFFFFF (its own type's maximum), so the first branch could
 *         never execute, and the second only at the single exact value
 *         INT32_MIN - so they never actually fired. Removed rather than
 *         kept (see QEI_REFACTOR.md). This simple form does NOT
 *         generalize to a smaller MAXPOS (e.g. a wrap-per-revolution
 *         scheme) - if that's ever reintroduced, this function needs to
 *         change too, not just QEI_MAXPOS_VALUE.
 */
static sint32 QEI_CalculateDelta(uint32 current, uint32 last)
{
    return (sint32)(current - last);
}

/*******************************************************************************
 *                          Initialization Functions                           *
 *******************************************************************************/

QEI_StatusType QEI_Init(void)
{
    QEI_StatusType status = QEI_OK;
    
#if (QEI0_ENABLED == 1U)
    /* Clock enable+wait is done inside QEI_InitChannel() - no need to
     * repeat it here too */
    status = QEI_InitChannel(QEI_CHANNEL_0);
    if (status != QEI_OK)
    {
        return status;
    }
#endif

#if (QEI1_ENABLED == 1U)
    status = QEI_InitChannel(QEI_CHANNEL_1);
    if (status != QEI_OK)
    {
        return status;
    }
#endif
    
    return status;
}

QEI_StatusType QEI_InitChannel(QEI_ChannelType channel)
{
    if (channel >= QEI_CHANNEL_MAX)
    {
        return QEI_ERROR_INVALID_CHANNEL;
    }
    
    /* Enable clock if not already */
    SYSCTL_RCGCQEI |= (1U << channel);
    while (!(SYSCTL_PRQEI & (1U << channel)));
    
    /* Configure the QEI module */
    QEI_ConfigureModule(channel);
    
    /* Initialize tracking state */
    QEI_Channels[channel].lastPosition = 0;
    QEI_Channels[channel].totalCounts = 0;
    QEI_Channels[channel].totalDistance = 0.0f;
    QEI_Channels[channel].distanceInitialized = FALSE;

    /* Velocity-sign state (independent of the odometry state above). QEIPOS was
     * just reset to 0 in QEI_ConfigureModule, so the baseline is a known 0. */
    QEI_Channels[channel].lastVelPosition = 0;
    QEI_Channels[channel].lastVelSign = 1.0f;
    QEI_Channels[channel].velSignInit = FALSE;

    QEI_Channels[channel].state = QEI_STATE_RUNNING;
    
    return QEI_OK;
}

QEI_StatusType QEI_DeInit(void)
{
    uint8 i;
    
    for (i = 0; i < QEI_CHANNEL_MAX; i++)
    {
        if (QEI_Channels[i].state != QEI_STATE_UNINIT)
        {
            QEI_CTL(QEI_Channels[i].baseAddr) = 0;
            QEI_Channels[i].state = QEI_STATE_UNINIT;
        }
    }
    
    /* Disable clocks */
    SYSCTL_RCGCQEI = 0;
    
    return QEI_OK;
}

/*******************************************************************************
 *                          Position Functions                                 *
 *******************************************************************************/

uint32 QEI_GetPosition(QEI_ChannelType channel)
{
    if (channel >= QEI_CHANNEL_MAX)
    {
        return 0;
    }
    
    return QEI_POS(QEI_Channels[channel].baseAddr);
}

QEI_StatusType QEI_SetPosition(QEI_ChannelType channel, uint32 position)
{
    if (channel >= QEI_CHANNEL_MAX)
    {
        return QEI_ERROR_INVALID_CHANNEL;
    }
    
    QEI_POS(QEI_Channels[channel].baseAddr) = position;
    QEI_Channels[channel].lastPosition = position;
    
    return QEI_OK;
}

QEI_StatusType QEI_ResetPosition(QEI_ChannelType channel)
{
    if (channel >= QEI_CHANNEL_MAX)
    {
        return QEI_ERROR_INVALID_CHANNEL;
    }
    
    /* Reset hardware position counter */
    QEI_POS(QEI_Channels[channel].baseAddr) = 0;

    /* Also reset software tracking variables, so totals stay consistent
     * with the hardware register this just zeroed */
    QEI_Channels[channel].lastPosition = 0;
    QEI_Channels[channel].totalCounts = 0;
    QEI_Channels[channel].totalDistance = 0.0f;
    QEI_Channels[channel].distanceInitialized = TRUE;
    
    return QEI_OK;
}

float32 QEI_GetPositionDegrees(QEI_ChannelType channel)
{
    uint32 pos;
    
    if (channel >= QEI_CHANNEL_MAX)
    {
        return 0.0f;
    }
    
    pos = QEI_POS(QEI_Channels[channel].baseAddr);
    
    return ((float32)pos / (float32)QEI_COUNTS_PER_REV) * 360.0f;
}

float32 QEI_GetPositionRadians(QEI_ChannelType channel)
{
    uint32 pos;
    
    if (channel >= QEI_CHANNEL_MAX)
    {
        return 0.0f;
    }
    
    pos = QEI_POS(QEI_Channels[channel].baseAddr);
    
    return ((float32)pos / (float32)QEI_COUNTS_PER_REV) * QEI_2PI;
}

QEI_StatusType QEI_GetPositionData(QEI_ChannelType channel, QEI_PositionType *position)
{
    QEI_ChannelHandleType *handle;
    uint32 rawPos;
    
    if (channel >= QEI_CHANNEL_MAX)
    {
        return QEI_ERROR_INVALID_CHANNEL;
    }
    
    if (position == NULL_PTR)
    {
        return QEI_ERROR_NULL_PTR;
    }
    
    handle = &QEI_Channels[channel];
    rawPos = QEI_POS(handle->baseAddr);
    
    position->rawPosition = rawPos;
    position->totalCounts = handle->totalCounts;
    position->revolutions = (float32)handle->totalCounts / (float32)QEI_COUNTS_PER_REV;
    position->degrees = ((float32)rawPos / (float32)QEI_COUNTS_PER_REV) * 360.0f;
    position->radians = ((float32)rawPos / (float32)QEI_COUNTS_PER_REV) * QEI_2PI;
    position->distanceMm = handle->totalDistance;
    position->distanceCm = handle->totalDistance / 10.0f;
    position->distanceM = handle->totalDistance / 1000.0f;
    
    return QEI_OK;
}

/*******************************************************************************
 *                          Velocity Functions                                 *
 *******************************************************************************/

uint32 QEI_GetVelocityRaw(QEI_ChannelType channel)
{
    if (channel >= QEI_CHANNEL_MAX)
    {
        return 0;
    }
    
    return QEI_SPEED(QEI_Channels[channel].baseAddr);
}

float32 QEI_GetVelocityRPM(QEI_ChannelType channel)
{
    /*
     * Hybrid magnitude+sign (Option 1, see QEI_VELOCITY_RESIDUAL_AUDIT.md).
     *
     * MAGNITUDE from QEISPEED: the hardware edge count for the last COMPLETE
     * velocity window (datasheet p.1320). This is stateless and read-only, so
     * it is immune to how many software callers read it or when - unlike a
     * delta-over-one-window-load computed in software, which over/under-reports
     * whenever a caller's read cadence doesn't align exactly with the hardware
     * window (confirmed root cause of the multi-rate-caller magnitude wobble:
     * 50 Hz control reads vs 10 Hz 0x200 telemetry reads racing the same
     * window-gated snapshot).
     *
     * SIGN from the window-gated NET QEIPOS delta (kept from Option B, see
     * QEI_VELOCITY_SIGN_AUDIT.md): QEISTAT.DIRECTION is an INSTANTANEOUS
     * last-transition flag that dithers on a single edge, which previously
     * paired a windowed magnitude with a glitchy sign and drove a limit cycle.
     * QEIPOS increments/decrements on forward/reverse edges, so the sign of its
     * net delta over a full window is glitch-free. The sign is recomputed
     * EXACTLY ONCE per completed window (RIS-gated) and cached; extra
     * intra-window callers reuse the cached sign so a second reader cannot
     * consume/truncate the window.
     */
    uint32 base;
    uint32 speed;
    uint32 load;
    uint32 pos;
    sint32 delta;
    float32 rpmMag;
    float32 sign;

    if (channel >= QEI_CHANNEL_MAX)
    {
        return 0.0f;
    }

    base  = QEI_Channels[channel].baseAddr;
    speed = QEI_SPEED(base);

    /* No motion in the last complete window -> no sign to resolve. */
    if (speed == 0U)
    {
        return 0.0f;
    }

    /* Magnitude: original pre-fix formula, driven by the hardware-latched
     * QEISPEED edge count (always exactly one window's worth). */
    load   = QEI_LOAD(base) + 1;
    rpmMag = ((float32)QEI_SYSTEM_CLOCK_HZ * (float32)speed * 60.0f) /
             ((float32)load * (float32)QEI_COUNTS_PER_REV);

    /* Sign: only advance the snapshot when a NEW velocity window has
     * completed; otherwise reuse the cached sign from this same window. */
    if ((QEI_RIS(base) & QEI_INT_TIMER_MASK) != 0U)
    {
        pos = QEI_POS(base);

        /* Signed net counts over the window. Full 32-bit free-run
         * (MAXPOS=0xFFFFFFFF), so plain two's-complement subtraction is the
         * correct signed delta across wrap - identical reasoning to
         * QEI_CalculateDelta. */
        delta = (sint32)(pos - QEI_Channels[channel].lastVelPosition);
        QEI_Channels[channel].lastVelPosition = pos;

        /* Clear the window-complete flag (bit 1 only; ERROR/INDEX untouched). */
        QEI_ISC(base) = QEI_INT_TIMER_MASK;

        if (!QEI_Channels[channel].velSignInit)
        {
            /* First windowed sample: baseline was a known 0 but the first
             * delta spans an undefined interval, so default to + and start
             * real differencing next window. */
            QEI_Channels[channel].velSignInit = TRUE;
            sign = 1.0f;
        }
        else if (delta > 0)
        {
            sign = 1.0f;
        }
        else if (delta < 0)
        {
            sign = -1.0f;
        }
        else
        {
            /* delta == 0 with speed != 0 (gross vs net during dither): hold
             * the previous sign rather than guessing. */
            sign = QEI_Channels[channel].lastVelSign;
        }

        QEI_Channels[channel].lastVelSign = sign;
    }
    else
    {
        sign = QEI_Channels[channel].lastVelSign;
    }

    return rpmMag * sign;
}

float32 QEI_GetVelocityRadPerSec(QEI_ChannelType channel)
{
    float32 rpm = QEI_GetVelocityRPM(channel);
    
    /* rad/s = RPM * 2π / 60 */
    return rpm * QEI_2PI / 60.0f;
}

float32 QEI_GetVelocityDegPerSec(QEI_ChannelType channel)
{
    float32 rpm = QEI_GetVelocityRPM(channel);
    
    /* deg/s = RPM * 360 / 60 = RPM * 6 */
    return rpm * 6.0f;
}

float32 QEI_GetVelocityMmPerSec(QEI_ChannelType channel)
{
    float32 rpm = QEI_GetVelocityRPM(channel);
    
    /* mm/s = RPM * circumference_mm / 60 */
    return rpm * QEI_WHEEL_CIRCUMFERENCE_MM / 60.0f;
}

float32 QEI_GetVelocityCmPerSec(QEI_ChannelType channel)
{
    return QEI_GetVelocityMmPerSec(channel) / 10.0f;
}

float32 QEI_GetVelocityMPerSec(QEI_ChannelType channel)
{
    return QEI_GetVelocityMmPerSec(channel) / 1000.0f;
}

QEI_StatusType QEI_GetVelocityData(QEI_ChannelType channel, QEI_VelocityType *velocity)
{
    float32 rpm;
    
    if (channel >= QEI_CHANNEL_MAX)
    {
        return QEI_ERROR_INVALID_CHANNEL;
    }
    
    if (velocity == NULL_PTR)
    {
        return QEI_ERROR_NULL_PTR;
    }
    
    velocity->rawSpeed = QEI_SPEED(QEI_Channels[channel].baseAddr);
    velocity->direction = QEI_GetDirection(channel);
    
    rpm = QEI_GetVelocityRPM(channel);
    if (rpm < 0) rpm = -rpm;  /* Store absolute values */
    
    velocity->rpm = rpm;
    velocity->rps = rpm / 60.0f;
    velocity->degPerSec = rpm * 6.0f;
    velocity->radPerSec = rpm * QEI_2PI / 60.0f;
    velocity->mmPerSec = rpm * QEI_WHEEL_CIRCUMFERENCE_MM / 60.0f;
    velocity->cmPerSec = velocity->mmPerSec / 10.0f;
    velocity->mPerSec = velocity->mmPerSec / 1000.0f;
    
    return QEI_OK;
}

/*******************************************************************************
 *                          Direction Functions                                *
 *******************************************************************************/

QEI_DirectionType QEI_GetDirection(QEI_ChannelType channel)
{
    uint32 stat;
    
    if (channel >= QEI_CHANNEL_MAX)
    {
        return QEI_DIR_FORWARD;
    }
    
    stat = QEI_STAT(QEI_Channels[channel].baseAddr);
    
    return (stat & QEI_STAT_DIRECTION_MASK) ? QEI_DIR_REVERSE : QEI_DIR_FORWARD;
}

boolean QEI_IsForward(QEI_ChannelType channel)
{
    return (QEI_GetDirection(channel) == QEI_DIR_FORWARD);
}

boolean QEI_IsReverse(QEI_ChannelType channel)
{
    return (QEI_GetDirection(channel) == QEI_DIR_REVERSE);
}

/*******************************************************************************
 *                          Distance Tracking Functions                        *
 *******************************************************************************/

QEI_StatusType QEI_ResetDistance(QEI_ChannelType channel)
{
    if (channel >= QEI_CHANNEL_MAX)
    {
        return QEI_ERROR_INVALID_CHANNEL;
    }
    
    QEI_Channels[channel].lastPosition = QEI_POS(QEI_Channels[channel].baseAddr);
    QEI_Channels[channel].totalCounts = 0;
    QEI_Channels[channel].totalDistance = 0.0f;
    QEI_Channels[channel].distanceInitialized = TRUE;
    
    return QEI_OK;
}

QEI_StatusType QEI_UpdateDistance(QEI_ChannelType channel)
{
    QEI_ChannelHandleType *handle;
    uint32 currentPos;
    sint32 delta;

    if (channel >= QEI_CHANNEL_MAX)
    {
        return QEI_ERROR_INVALID_CHANNEL;
    }

    handle = &QEI_Channels[channel];

    if (!handle->distanceInitialized)
    {
        QEI_ResetDistance(channel);
        return QEI_OK;
    }

    currentPos = QEI_POS(handle->baseAddr);

    /* Calculate signed delta (see QEI_CalculateDelta's own comment for
     * why maxPos/direction are not, and should not be, inputs here) */
    delta = QEI_CalculateDelta(currentPos, handle->lastPosition);
    
    /* Update tracking */
    handle->lastPosition = currentPos;
    handle->totalCounts += delta;
    handle->totalDistance += (float32)delta * QEI_MM_PER_COUNT;
    
    return QEI_OK;
}

float32 QEI_GetDistanceMm(QEI_ChannelType channel)
{
    if (channel >= QEI_CHANNEL_MAX)
    {
        return 0.0f;
    }
    
    /* Update distance before returning */
    QEI_UpdateDistance(channel);
    
    return QEI_Channels[channel].totalDistance;
}

float32 QEI_GetDistanceCm(QEI_ChannelType channel)
{
    return QEI_GetDistanceMm(channel) / 10.0f;
}

float32 QEI_GetDistanceM(QEI_ChannelType channel)
{
    return QEI_GetDistanceMm(channel) / 1000.0f;
}

sint32 QEI_GetTotalCounts(QEI_ChannelType channel)
{
    if (channel >= QEI_CHANNEL_MAX)
    {
        return 0;
    }
    
    QEI_UpdateDistance(channel);
    
    return QEI_Channels[channel].totalCounts;
}

float32 QEI_GetTotalRevolutions(QEI_ChannelType channel)
{
    if (channel >= QEI_CHANNEL_MAX)
    {
        return 0.0f;
    }
    
    QEI_UpdateDistance(channel);
    
    return (float32)QEI_Channels[channel].totalCounts / (float32)QEI_COUNTS_PER_REV;
}

/*******************************************************************************
 *                          Odometry Functions                                 *
 *******************************************************************************/

QEI_StatusType QEI_ResetOdometry(void)
{
    QEI_ResetDistance(QEI_CHANNEL_0);
    QEI_ResetDistance(QEI_CHANNEL_1);
    
    QEI_Odometry.x = 0.0f;
    QEI_Odometry.y = 0.0f;
    QEI_Odometry.theta = 0.0f;
    QEI_Odometry.distanceTotal = 0.0f;
    QEI_Odometry.linearVelocity = 0.0f;
    QEI_Odometry.angularVelocity = 0.0f;
    
    QEI_OdometryLastCounts[0] = QEI_Channels[0].totalCounts;
    QEI_OdometryLastCounts[1] = QEI_Channels[1].totalCounts;
    QEI_OdometryInitialized = TRUE;
    
    return QEI_OK;
}

QEI_StatusType QEI_UpdateOdometry(void)
{
    sint32 leftCounts, rightCounts;
    sint32 deltaLeft, deltaRight;
    float32 distLeft, distRight;
    float32 distCenter, deltaTheta;
    
    if (!QEI_OdometryInitialized)
    {
        QEI_ResetOdometry();
        return QEI_OK;
    }
    
    /* Update distance for both encoders */
    QEI_UpdateDistance(QEI_CHANNEL_0);
    QEI_UpdateDistance(QEI_CHANNEL_1);
    
    /* Get current counts */
    leftCounts = QEI_Channels[0].totalCounts;
    rightCounts = QEI_Channels[1].totalCounts;
    
    /* Calculate delta counts since last update */
    deltaLeft = leftCounts - QEI_OdometryLastCounts[0];
    deltaRight = rightCounts - QEI_OdometryLastCounts[1];
    
    /* Update last counts */
    QEI_OdometryLastCounts[0] = leftCounts;
    QEI_OdometryLastCounts[1] = rightCounts;
    
    /* Convert counts to distance (mm) */
    distLeft = (float32)deltaLeft * QEI_MM_PER_COUNT;
    distRight = (float32)deltaRight * QEI_MM_PER_COUNT;
    
    /* Differential drive kinematics */
    distCenter = (distLeft + distRight) / 2.0f;
    deltaTheta = (distRight - distLeft) / QEI_WHEEL_BASE_MM;
    
    /* Update heading */
    QEI_Odometry.theta += deltaTheta;
    
    /* Normalize theta to [-π, π] */
    while (QEI_Odometry.theta > QEI_PI)
    {
        QEI_Odometry.theta -= QEI_2PI;
    }
    while (QEI_Odometry.theta < -QEI_PI)
    {
        QEI_Odometry.theta += QEI_2PI;
    }
    
    /* Update position using mid-point integration */
    /* Note: For small deltaTheta, cos ≈ 1 and sin ≈ deltaTheta */
    /* This simplification avoids needing math.h in embedded context */
    {
        float32 avgTheta = QEI_Odometry.theta - deltaTheta / 2.0f;
        float32 cosTheta, sinTheta;
        
        /* Simple Taylor series approximation for small angles */
        /* For better accuracy, use lookup table or CORDIC */
        /* cos(x) ≈ 1 - x²/2, sin(x) ≈ x - x³/6 */
        float32 x2 = avgTheta * avgTheta;
        cosTheta = 1.0f - x2 / 2.0f + x2 * x2 / 24.0f;
        sinTheta = avgTheta * (1.0f - x2 / 6.0f + x2 * x2 / 120.0f);
        
        QEI_Odometry.x += distCenter * cosTheta;
        QEI_Odometry.y += distCenter * sinTheta;
    }
    
    /* Update total distance */
    if (distCenter > 0)
    {
        QEI_Odometry.distanceTotal += distCenter;
    }
    else
    {
        QEI_Odometry.distanceTotal -= distCenter;
    }
    
    /* Update velocities (from encoder velocity readings) */
    float32 velLeft = QEI_GetVelocityMmPerSec(QEI_CHANNEL_0);
    float32 velRight = QEI_GetVelocityMmPerSec(QEI_CHANNEL_1);
    
    QEI_Odometry.linearVelocity = (velLeft + velRight) / 2.0f;
    QEI_Odometry.angularVelocity = (velRight - velLeft) / QEI_WHEEL_BASE_MM;
    
    return QEI_OK;
}

QEI_StatusType QEI_GetOdometry(QEI_OdometryType *odometry)
{
    if (odometry == NULL_PTR)
    {
        return QEI_ERROR_NULL_PTR;
    }
    
    *odometry = QEI_Odometry;
    
    return QEI_OK;
}

QEI_StatusType QEI_SetOdometry(float32 x, float32 y, float32 theta)
{
    QEI_Odometry.x = x;
    QEI_Odometry.y = y;
    QEI_Odometry.theta = theta;
    
    return QEI_OK;
}

/*******************************************************************************
 *                          Status Functions                                   *
 *******************************************************************************/

QEI_StateType QEI_GetState(QEI_ChannelType channel)
{
    if (channel >= QEI_CHANNEL_MAX)
    {
        return QEI_STATE_UNINIT;
    }
    
    return QEI_Channels[channel].state;
}

boolean QEI_HasError(QEI_ChannelType channel)
{
    if (channel >= QEI_CHANNEL_MAX)
    {
        return FALSE;
    }
    
    return (QEI_STAT(QEI_Channels[channel].baseAddr) & QEI_STAT_ERROR_MASK) != 0;
}

QEI_StatusType QEI_ClearError(QEI_ChannelType channel)
{
    if (channel >= QEI_CHANNEL_MAX)
    {
        return QEI_ERROR_INVALID_CHANNEL;
    }
    
    /* Error clears on read, but we can also clear via ISC */
    QEI_ISC(QEI_Channels[channel].baseAddr) = QEI_INT_ERROR_MASK;
    
    return QEI_OK;
}

boolean QEI_IsStationary(QEI_ChannelType channel)
{
    if (channel >= QEI_CHANNEL_MAX)
    {
        return TRUE;
    }
    
    return (QEI_SPEED(QEI_Channels[channel].baseAddr) == 0);
}

/*******************************************************************************
 *                          Configuration Functions                            *
 *******************************************************************************/

QEI_StatusType QEI_SetVelocityPeriod(QEI_ChannelType channel, uint32 periodMs)
{
    uint32 loadValue;
    /* Largest periodMs for which (QEI_SYSTEM_CLOCK_HZ * periodMs) still
     * fits in uint32. Exceeding it silently wraps loadValue to garbage
     * (at 16 MHz, periodMs >= 269 overflows). periodMs == 0 is rejected
     * too, since it would underflow the "- 1" below to 0xFFFFFFFF - an
     * equally nonsensical LOAD value. */
    uint32 maxPeriodMs = 0xFFFFFFFFUL / QEI_SYSTEM_CLOCK_HZ;

    if (channel >= QEI_CHANNEL_MAX)
    {
        return QEI_ERROR_INVALID_CHANNEL;
    }

    if ((periodMs == 0U) || (periodMs > maxPeriodMs))
    {
        return QEI_ERROR_INVALID_CONFIG;
    }

    loadValue = (QEI_SYSTEM_CLOCK_HZ * periodMs / 1000UL) - 1;

    QEI_LOAD(QEI_Channels[channel].baseAddr) = loadValue;

    return QEI_OK;
}

QEI_StatusType QEI_SetMaxPosition(QEI_ChannelType channel, uint32 maxPos)
{
    if (channel >= QEI_CHANNEL_MAX)
    {
        return QEI_ERROR_INVALID_CHANNEL;
    }
    
    QEI_MAXPOS(QEI_Channels[channel].baseAddr) = maxPos;
    
    return QEI_OK;
}

QEI_StatusType QEI_SetFilter(QEI_ChannelType channel, boolean enable, uint8 count)
{
    uint32 base;
    uint32 ctl;
    
    if (channel >= QEI_CHANNEL_MAX)
    {
        return QEI_ERROR_INVALID_CHANNEL;
    }
    
    base = QEI_Channels[channel].baseAddr;
    ctl = QEI_CTL(base);
    
    /* Clear filter bits */
    ctl &= ~(QEI_CTL_FILTEN_MASK | QEI_CTL_FILTCNT_MASK);
    
    if (enable)
    {
        ctl |= QEI_CTL_FILTEN_MASK;
        ctl |= ((uint32)(count & 0xF) << QEI_CTL_FILTCNT_POS);
    }
    
    QEI_CTL(base) = ctl;
    
    return QEI_OK;
}

/* Interrupt handlers removed - QEI_INTERRUPT_ENABLE is 0, the NVIC was
 * never enabled for either module (QEI_EnableNVIC was dead code, see
 * QEI_REFACTOR.md), and the callback-registration API these handlers
 * depended on has been removed. Velocity/position are polled instead.
 * If interrupt-driven QEI is reintroduced later, this is the place to
 * add QEI0_Handler/QEI1_Handler back, along with NVIC setup and a
 * callback-registration mechanism. */
