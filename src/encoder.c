/******************************************************************************
 *
 * Module: Encoder
 *
 * File Name: encoder.c
 *
 * Description: Source file for Encoder driver abstraction layer
 *              Provides high-level encoder interface over QEI hardware
 *              Designed for DC motor feedback in differential drive robot
 *
 ******************************************************************************/

#include "encoder.h"
#include "qei.h"

/*******************************************************************************
 *                          Constants                                          *
 *******************************************************************************/

/* NOTE: ENCODER_PI/ENCODER_2PI/ENCODER_RAD_TO_DEG/ENCODER_DEG_TO_RAD and
 * ENCODER_DEFAULT_WHEEL_BASE_MM were removed here (ENCODER_REFACTOR.md) -
 * they only fed Encoder_UpdateDerivedParams() and the odometry subsystem,
 * both removed. ENCODER_2PI/ENCODER_DEG_TO_RAD were already unused even
 * before this pass. */

/*******************************************************************************
 *                          Private Types                                      *
 *******************************************************************************/

typedef struct {
    Encoder_ConfigType      config;
    Encoder_StateType       state;
    boolean                 initialized;
} Encoder_HandleType;

/*******************************************************************************
 *                          Private Variables                                  *
 *******************************************************************************/

static Encoder_HandleType Encoder_Handles[ENCODER_ID_MAX] = {
    {
        .config = ENCODER_CONFIG_DEFAULT,
        .state = ENCODER_STATE_UNINIT,
        .initialized = FALSE
    },
    {
        .config = ENCODER_CONFIG_DEFAULT,
        .state = ENCODER_STATE_UNINIT,
        .initialized = FALSE
    }
};

/* Module initialization flag */

static boolean Encoder_ModuleInitialized = FALSE;

/*******************************************************************************
 *                          Private Function Prototypes                        *
 *******************************************************************************/

static QEI_ChannelType Encoder_GetQeiChannel(Encoder_IdType encoderId);
static float32 Encoder_ApplyDirectionSign(Encoder_IdType encoderId, float32 value);

/*******************************************************************************
 *                          Private Functions                                  *
 *******************************************************************************/

/**
 * @brief  Map encoder ID to QEI channel
 */
static QEI_ChannelType Encoder_GetQeiChannel(Encoder_IdType encoderId)
{
    switch (encoderId)
    {
        case ENCODER_ID_LEFT:
            return QEI_CHANNEL_1;
        case ENCODER_ID_RIGHT:
            return QEI_CHANNEL_0;
        default:
            return QEI_CHANNEL_0;
    }
}

/**
 * @brief  Apply direction sign based on configuration
 */
static float32 Encoder_ApplyDirectionSign(Encoder_IdType encoderId, float32 value)
{
    if (Encoder_Handles[encoderId].config.invertDirection)
    {
        return -value;
    }
    return value;
}

/*******************************************************************************
 *                          Initialization Functions                           *
 *******************************************************************************/

Encoder_StatusType Encoder_Init(void)
{
    QEI_StatusType qeiStatus;
    uint8 i;
    
    /* Initialize underlying QEI driver */
    qeiStatus = QEI_Init();
    if (qeiStatus != QEI_OK)
    {
        return ENCODER_ERROR_NOT_INITIALIZED;
    }
    
    /* Initialize each encoder handle. countsPerRev/wheelDiameterMm are no
     * longer tracked here - every reported value comes from QEI's own
     * fixed, compile-time constants (qei_cfg.h: QEI_COUNTS_PER_REV=2464,
     * 11 PPR motor-shaft x 56:1 gear x 4 decode; QEI_WHEEL_DIAMETER_MM=
     * 65mm), not from a per-instance copy at this layer (see
     * ENCODER_AUDIT.md / ENCODER_REFACTOR.md). */
    for (i = 0; i < ENCODER_ID_MAX; i++)
    {
        /* V9-5: invertDirection is the THIRD switch that can flip the sensor
         * sign (the other two are QEI0/1_SWAP_SIGNALS and
         * QEI0/1_INVERT_DIRECTION, pinned by an assert in qei.c). The
         * mirror-mount is already compensated exactly once, by the hardware
         * SWAP on the right channel, so this MUST stay FALSE - setting it
         * double-compensates that wheel into positive feedback under the
         * velocity PID. Being a runtime field it cannot be pinned at build
         * time; keeping it hardcoded here is what guarantees it. */
        Encoder_Handles[i].config.invertDirection = FALSE;
        Encoder_Handles[i].config.velocityPeriodMs = 20;
        Encoder_Handles[i].config.filterEnable = TRUE;
        Encoder_Handles[i].config.filterCount = 3;

        Encoder_Handles[i].state = ENCODER_STATE_STOPPED;
        Encoder_Handles[i].initialized = TRUE;
    }

    Encoder_ModuleInitialized = TRUE;

    return ENCODER_OK;
}

Encoder_StatusType Encoder_DeInit(void)
{
    uint8 i;
    
    QEI_DeInit();
    
    for (i = 0; i < ENCODER_ID_MAX; i++)
    {
        Encoder_Handles[i].state = ENCODER_STATE_UNINIT;
        Encoder_Handles[i].initialized = FALSE;
    }
    
    Encoder_ModuleInitialized = FALSE;
    
    return ENCODER_OK;
}

/*******************************************************************************
 *                          Velocity Functions                                 *
 *******************************************************************************/

float32 Encoder_GetRPM(Encoder_IdType encoderId)
{
    float32 rpm;
    
    if (encoderId >= ENCODER_ID_MAX || !Encoder_Handles[encoderId].initialized)
    {
        return 0.0f;
    }
    
    rpm = QEI_GetVelocityRPM(Encoder_GetQeiChannel(encoderId));
    
    return Encoder_ApplyDirectionSign(encoderId, rpm);
}

float32 Encoder_GetRadPerSec(Encoder_IdType encoderId)
{
    float32 radPerSec;
    
    if (encoderId >= ENCODER_ID_MAX || !Encoder_Handles[encoderId].initialized)
    {
        return 0.0f;
    }
    
    radPerSec = QEI_GetVelocityRadPerSec(Encoder_GetQeiChannel(encoderId));
    
    return Encoder_ApplyDirectionSign(encoderId, radPerSec);
}

float32 Encoder_GetLinearVelocityM(Encoder_IdType encoderId)
{
    float32 mPerSec;

    if (encoderId >= ENCODER_ID_MAX || !Encoder_Handles[encoderId].initialized)
    {
        return 0.0f;
    }

    /* Was derived from the now-removed Encoder_GetLinearVelocityMm(); calls
     * QEI directly instead. Same QEI_GetVelocityMmPerSec() value, same
     * sign handling - only the intermediate mm-public-function is gone. */
    mPerSec = QEI_GetVelocityMmPerSec(Encoder_GetQeiChannel(encoderId)) / 1000.0f;

    return Encoder_ApplyDirectionSign(encoderId, mPerSec);
}

Encoder_StatusType Encoder_GetVelocity(Encoder_IdType encoderId, Encoder_VelocityType *velocity)
{
    QEI_VelocityType qeiVelocity;
    QEI_StatusType qeiStatus;
    Encoder_HandleType *handle;
    float32 sign;

    if (encoderId >= ENCODER_ID_MAX)
    {
        return ENCODER_ERROR_INVALID_ID;
    }
    
    if (velocity == NULL_PTR)
    {
        return ENCODER_ERROR_NULL_PTR;
    }
    
    handle = &Encoder_Handles[encoderId];
    
    if (!handle->initialized)
    {
        return ENCODER_ERROR_NOT_INITIALIZED;
    }
    
    qeiStatus = QEI_GetVelocityData(Encoder_GetQeiChannel(encoderId), &qeiVelocity);
    if (qeiStatus != QEI_OK)
    {
        return ENCODER_ERROR_INVALID_ID;
    }

    /* Apply direction inversion if configured */
    sign = handle->config.invertDirection ? -1.0f : 1.0f;

    velocity->rpm = qeiVelocity.rpm * sign;
    velocity->radPerSec = qeiVelocity.radPerSec * sign;
    velocity->mPerSec = qeiVelocity.mPerSec * sign;
    
    /* Direction is inverted if inversion is configured */
    if (handle->config.invertDirection)
    {
        velocity->direction = (qeiVelocity.direction == QEI_DIR_FORWARD) ? 
                              ENCODER_DIR_REVERSE : ENCODER_DIR_FORWARD;
    }
    else
    {
        velocity->direction = (qeiVelocity.direction == QEI_DIR_FORWARD) ? 
                              ENCODER_DIR_FORWARD : ENCODER_DIR_REVERSE;
    }
    
    return ENCODER_OK;
}

/*******************************************************************************
 *                          Position Functions                                 *
 *******************************************************************************/

sint32 Encoder_GetTotalCounts(Encoder_IdType encoderId)
{
    sint32 counts;
    
    if (encoderId >= ENCODER_ID_MAX || !Encoder_Handles[encoderId].initialized)
    {
        return 0;
    }
    
    counts = QEI_GetTotalCounts(Encoder_GetQeiChannel(encoderId));
    
    if (Encoder_Handles[encoderId].config.invertDirection)
    {
        counts = -counts;
    }
    
    return counts;
}

float32 Encoder_GetRevolutions(Encoder_IdType encoderId)
{
    float32 revs;
    
    if (encoderId >= ENCODER_ID_MAX || !Encoder_Handles[encoderId].initialized)
    {
        return 0.0f;
    }
    
    revs = QEI_GetTotalRevolutions(Encoder_GetQeiChannel(encoderId));
    
    return Encoder_ApplyDirectionSign(encoderId, revs);
}

float32 Encoder_GetAngleRadians(Encoder_IdType encoderId)
{
    float32 radians;

    if (encoderId >= ENCODER_ID_MAX || !Encoder_Handles[encoderId].initialized)
    {
        return 0.0f;
    }

    radians = QEI_GetPositionRadians(Encoder_GetQeiChannel(encoderId));

    /* E-1 (REVIEW 07/08): this getter used to return the QEI value RAW while
     * every sibling applied invertDirection, so with that flag set it would
     * have been the only one reporting a backwards angle. Latent today - the
     * flag is FALSE everywhere - but the asymmetry is exactly the kind that
     * stays invisible until someone enables it. All getters now agree.
     *
     * CAVEAT (separate, NOT fixed here - see REVIEW 08 finding E-2): the value
     * underneath is QEIPOS read as a RAW uint32 free-running position, so this
     * is an unbounded absolute angle that jumps to a huge positive number after
     * reverse motion past zero - it is not a wrapped 0..2pi heading. Applying
     * the sign makes it CONSISTENT with its siblings; it does not make the
     * underlying quantity a well-formed angle. This function currently has no
     * callers. */
    return Encoder_ApplyDirectionSign(encoderId, radians);
}

Encoder_StatusType Encoder_GetPosition(Encoder_IdType encoderId, Encoder_PositionType *position)
{
    QEI_PositionType qeiPosition;
    QEI_StatusType qeiStatus;
    Encoder_HandleType *handle;
    float32 sign;

    if (encoderId >= ENCODER_ID_MAX)
    {
        return ENCODER_ERROR_INVALID_ID;
    }
    
    if (position == NULL_PTR)
    {
        return ENCODER_ERROR_NULL_PTR;
    }
    
    handle = &Encoder_Handles[encoderId];
    
    if (!handle->initialized)
    {
        return ENCODER_ERROR_NOT_INITIALIZED;
    }
    
    qeiStatus = QEI_GetPositionData(Encoder_GetQeiChannel(encoderId), &qeiPosition);
    if (qeiStatus != QEI_OK)
    {
        return ENCODER_ERROR_INVALID_ID;
    }

    sign = handle->config.invertDirection ? -1.0f : 1.0f;

    position->totalCounts = (sint32)(qeiPosition.totalCounts * sign);
    position->revolutions = qeiPosition.revolutions * sign;
    position->radians = qeiPosition.radians;
    position->distanceM = qeiPosition.distanceM * sign;

    return ENCODER_OK;
}

/*******************************************************************************
 *                          Distance Functions                                 *
 *******************************************************************************/

float32 Encoder_GetDistanceM(Encoder_IdType encoderId)
{
    float32 distance;

    if (encoderId >= ENCODER_ID_MAX || !Encoder_Handles[encoderId].initialized)
    {
        return 0.0f;
    }

    /* Was derived from the now-removed Encoder_GetDistanceMm(); calls QEI
     * directly instead. Same QEI_GetDistanceMm() value, same sign
     * handling - only the intermediate mm-public-function is gone. */
    distance = QEI_GetDistanceMm(Encoder_GetQeiChannel(encoderId)) / 1000.0f;

    return Encoder_ApplyDirectionSign(encoderId, distance);
}

float32 Encoder_GetDistanceCm(Encoder_IdType encoderId)
{
    /* KEPT despite being a "redundant unit" - control_distance.c calls
     * this directly (8 call sites, verified by grep before removing
     * anything - see ENCODER_REFACTOR.md). Derived from the kept
     * Encoder_GetDistanceM() rather than from the removed
     * Encoder_GetDistanceMm() - numerically equivalent (distanceM*100 ==
     * the old distanceMm/10) within float32 rounding. */
    return Encoder_GetDistanceM(encoderId) * 100.0f;
}

Encoder_StatusType Encoder_ResetDistance(Encoder_IdType encoderId)
{
    if (encoderId >= ENCODER_ID_MAX)
    {
        return ENCODER_ERROR_INVALID_ID;
    }
    
    if (!Encoder_Handles[encoderId].initialized)
    {
        return ENCODER_ERROR_NOT_INITIALIZED;
    }
    
    QEI_ResetDistance(Encoder_GetQeiChannel(encoderId));
    
    return ENCODER_OK;
}

Encoder_StatusType Encoder_ResetAllDistances(void)
{
    Encoder_StatusType status;

    status = Encoder_ResetDistance(ENCODER_ID_LEFT);
    if (status != ENCODER_OK)
    {
        return status;
    }

    status = Encoder_ResetDistance(ENCODER_ID_RIGHT);
    if (status != ENCODER_OK)
    {
        return status;
    }

    return ENCODER_OK;
}

float32 Encoder_GetAverageDistanceCm(void)
{
    /* Mean of the two drive wheels' cumulative tick counts, converted to cm via
     * the single-source qei_cfg constant. Reads hardware counts directly
     * (Encoder_GetTotalCounts -> QEI_GetTotalCounts), so it needs no periodic
     * Encoder_UpdateDistance. Additive helper for the outer distance loop. */
    float32 sumCounts = (float32)Encoder_GetTotalCounts(ENCODER_ID_LEFT)
                      + (float32)Encoder_GetTotalCounts(ENCODER_ID_RIGHT);

    return (sumCounts * 0.5f) * QEI_CM_PER_COUNT;
}

/*******************************************************************************
 *                          Direction Functions                                *
 *******************************************************************************/

Encoder_DirectionType Encoder_GetDirection(Encoder_IdType encoderId)
{
    QEI_DirectionType qeiDir;
    Encoder_DirectionType dir;
    
    if (encoderId >= ENCODER_ID_MAX || !Encoder_Handles[encoderId].initialized)
    {
        return ENCODER_DIR_FORWARD;
    }
    
    qeiDir = QEI_GetDirection(Encoder_GetQeiChannel(encoderId));
    
    dir = (qeiDir == QEI_DIR_FORWARD) ? ENCODER_DIR_FORWARD : ENCODER_DIR_REVERSE;
    
    /* Apply inversion */
    if (Encoder_Handles[encoderId].config.invertDirection)
    {
        dir = (dir == ENCODER_DIR_FORWARD) ? ENCODER_DIR_REVERSE : ENCODER_DIR_FORWARD;
    }
    
    return dir;
}

boolean Encoder_IsForward(Encoder_IdType encoderId)
{
    return (Encoder_GetDirection(encoderId) == ENCODER_DIR_FORWARD);
}

boolean Encoder_IsReverse(Encoder_IdType encoderId)
{
    return (Encoder_GetDirection(encoderId) == ENCODER_DIR_REVERSE);
}

boolean Encoder_IsStationary(Encoder_IdType encoderId)
{
    if (encoderId >= ENCODER_ID_MAX || !Encoder_Handles[encoderId].initialized)
    {
        return TRUE;
    }
    
    return QEI_IsStationary(Encoder_GetQeiChannel(encoderId));
}

/* NOTE: the odometry subsystem that used to live here (Encoder_Update/
 * Get/Reset/SetOdometry, GetX/Y/Heading/HeadingDegrees/RobotVelocity/
 * RobotAngularVelocity, and the Encoder_Odometry cache) was removed -
 * see ENCODER_REFACTOR.md. It belongs on the Orin (ros2_control's
 * Ackermann controller), and the implementation it called into
 * (QEI_UpdateOdometry et al.) was differential-drive kinematics, wrong
 * for this Ackermann robot. QEI's own odometry functions are untouched
 * (out of scope) - this HAL simply no longer calls them. */

/*******************************************************************************
 *                          Update Functions                                   *
 *******************************************************************************/

Encoder_StatusType Encoder_UpdateDistance(Encoder_IdType encoderId)
{
    Encoder_HandleType *handle;
    Encoder_StateType newState;
    
    if (encoderId >= ENCODER_ID_MAX)
    {
        return ENCODER_ERROR_INVALID_ID;
    }
    
    handle = &Encoder_Handles[encoderId];
    
    if (!handle->initialized)
    {
        return ENCODER_ERROR_NOT_INITIALIZED;
    }
    
    QEI_UpdateDistance(Encoder_GetQeiChannel(encoderId));
    
    /* Update state based on motion */
    if (QEI_IsStationary(Encoder_GetQeiChannel(encoderId)))
    {
        newState = ENCODER_STATE_STOPPED;
    }
    else if (Encoder_GetDirection(encoderId) == ENCODER_DIR_FORWARD)
    {
        newState = ENCODER_STATE_FORWARD;
    }
    else
    {
        newState = ENCODER_STATE_REVERSE;
    }
    
    handle->state = newState;
    
    return ENCODER_OK;
}

Encoder_StatusType Encoder_UpdateAllDistances(void)
{
    Encoder_StatusType status;

    status = Encoder_UpdateDistance(ENCODER_ID_LEFT);
    if (status != ENCODER_OK)
    {
        return status;
    }

    status = Encoder_UpdateDistance(ENCODER_ID_RIGHT);
    if (status != ENCODER_OK)
    {
        return status;
    }

    return ENCODER_OK;
}

/*******************************************************************************
 *                          Status Functions                                   *
 *******************************************************************************/

Encoder_StateType Encoder_GetState(Encoder_IdType encoderId)
{
    if (encoderId >= ENCODER_ID_MAX)
    {
        return ENCODER_STATE_UNINIT;
    }
    
    return Encoder_Handles[encoderId].state;
}

boolean Encoder_HasError(Encoder_IdType encoderId)
{
    if (encoderId >= ENCODER_ID_MAX || !Encoder_Handles[encoderId].initialized)
    {
        return FALSE;
    }
    
    return QEI_HasError(Encoder_GetQeiChannel(encoderId));
}

Encoder_StatusType Encoder_ClearError(Encoder_IdType encoderId)
{
    if (encoderId >= ENCODER_ID_MAX)
    {
        return ENCODER_ERROR_INVALID_ID;
    }
    
    if (!Encoder_Handles[encoderId].initialized)
    {
        return ENCODER_ERROR_NOT_INITIALIZED;
    }
    
    QEI_ClearError(Encoder_GetQeiChannel(encoderId));
    
    if (Encoder_Handles[encoderId].state == ENCODER_STATE_ERROR)
    {
        Encoder_Handles[encoderId].state = ENCODER_STATE_STOPPED;
    }
    
    return ENCODER_OK;
}

/*******************************************************************************
 *                          Configuration Functions                            *
 *******************************************************************************/

/* NOTE: Encoder_SetCountsPerRev/SetWheelDiameter/SetWheelBase and the
 * velocity/direction callback registration functions that used to live
 * here were removed - see ENCODER_REFACTOR.md and the matching note in
 * encoder.h. The Set* config functions updated local state nothing read
 * (every reported value comes from QEI's fixed, compile-time constants);
 * the callbacks stored a function pointer that was never invoked, and
 * the QEI-level callback API they needed no longer exists. */

/* ⚠️ V9-5: this is the THIRD sensor-sign switch (see the note in Encoder_Init).
 * It has NO callers today and must not gain one while a velocity loop is
 * running: the mirror-mount is already compensated exactly once by the hardware
 * QEI SWAP, so flipping this double-compensates that wheel and puts the
 * velocity PID into POSITIVE FEEDBACK - mid-run, with no build-time warning.
 * The other two switches are pinned by an assert in qei.c; a runtime field
 * cannot be. Left in place deliberately (removing it is an encoder-API
 * question, out of scope for FIX 26) - but treat calling it as a sign change
 * that needs the qei_cfg.h hand-spin re-verification. */
Encoder_StatusType Encoder_SetInvertDirection(Encoder_IdType encoderId, boolean invert)
{
    if (encoderId >= ENCODER_ID_MAX)
    {
        return ENCODER_ERROR_INVALID_ID;
    }

    Encoder_Handles[encoderId].config.invertDirection = invert;

    return ENCODER_OK;
}

Encoder_StatusType Encoder_SetVelocityPeriod(Encoder_IdType encoderId, uint32 periodMs)
{
    if (encoderId >= ENCODER_ID_MAX)
    {
        return ENCODER_ERROR_INVALID_ID;
    }
    
    if (!Encoder_Handles[encoderId].initialized)
    {
        return ENCODER_ERROR_NOT_INITIALIZED;
    }
    
    Encoder_Handles[encoderId].config.velocityPeriodMs = periodMs;
    
    QEI_SetVelocityPeriod(Encoder_GetQeiChannel(encoderId), periodMs);
    
    return ENCODER_OK;
}

Encoder_StatusType Encoder_SetFilter(Encoder_IdType encoderId, boolean enable, uint8 count)
{
    if (encoderId >= ENCODER_ID_MAX)
    {
        return ENCODER_ERROR_INVALID_ID;
    }
    
    if (!Encoder_Handles[encoderId].initialized)
    {
        return ENCODER_ERROR_NOT_INITIALIZED;
    }
    
    Encoder_Handles[encoderId].config.filterEnable = enable;
    Encoder_Handles[encoderId].config.filterCount = count;
    
    QEI_SetFilter(Encoder_GetQeiChannel(encoderId), enable, count);
    
    return ENCODER_OK;
}
