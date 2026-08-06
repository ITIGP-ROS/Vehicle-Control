/******************************************************************************
 *
 * Module: Encoder
 *
 * File Name: encoder.h
 *
 * Description: Header file for Encoder driver abstraction layer
 *              Provides high-level encoder interface over QEI hardware
 *              Designed for DC motor feedback in differential drive robot
 *
 ******************************************************************************/

#ifndef ENCODER_H_
#define ENCODER_H_

#include "Std_Types.h"

/*******************************************************************************
 *                          Encoder Identification                             *
 *******************************************************************************/

typedef enum {
    ENCODER_ID_LEFT = 0,        /* Left motor encoder */
    ENCODER_ID_RIGHT = 1,       /* Right motor encoder */
    ENCODER_ID_MAX
} Encoder_IdType;

/* Convenient aliases */
#define ENCODER_LEFT        ENCODER_ID_LEFT
#define ENCODER_RIGHT       ENCODER_ID_RIGHT
#define ENCODER_B           ENCODER_ID_LEFT
#define ENCODER_A           ENCODER_ID_RIGHT

/*******************************************************************************
 *                          Encoder Status Types                               *
 *******************************************************************************/

typedef enum {
    ENCODER_OK = 0,
    ENCODER_ERROR_INVALID_ID,
    ENCODER_ERROR_NOT_INITIALIZED,
    ENCODER_ERROR_NULL_PTR,
    ENCODER_ERROR_INVALID_CONFIG,
    ENCODER_ERROR_PHASE
} Encoder_StatusType;

/* NOTE: ENCODER_STATE_ERROR is currently unreachable - Encoder_UpdateDistance()
 * only ever sets STOPPED/FORWARD/REVERSE based on motion, regardless of
 * QEI_HasError()'s result (see ENCODER_AUDIT.md Section 7). Left in place
 * rather than removed: Encoder_HasError()/Encoder_ClearError() work
 * independently of this state field, and wiring an actual ERROR
 * transition into Encoder_UpdateDistance() is a behavior change beyond
 * this cleanup pass's scope, not a dead-code removal. */
typedef enum {
    ENCODER_STATE_UNINIT = 0,
    ENCODER_STATE_STOPPED,
    ENCODER_STATE_FORWARD,
    ENCODER_STATE_REVERSE,
    ENCODER_STATE_ERROR
} Encoder_StateType;

/*******************************************************************************
 *                          Direction Types                                    *
 *******************************************************************************/

typedef enum {
    ENCODER_DIR_FORWARD = 0,    /* Counter incrementing */
    ENCODER_DIR_REVERSE = 1     /* Counter decrementing */
} Encoder_DirectionType;

/*******************************************************************************
 *                          Encoder Configuration                              *
 *******************************************************************************/

/*
 * NOTE: this struct previously also carried countsPerRev/wheelDiameterMm/
 * gearRatio fields. Removed (ENCODER_REFACTOR.md) - every Get* function
 * that reports a physical unit delegates straight to a QEI_Get* function,
 * which uses qei_cfg.h's own fixed, compile-time constants (2464 counts/
 * rev, 65mm wheel). Those three fields were written by Set* functions but
 * never read by anything that produced a reported value - see
 * ENCODER_AUDIT.md Section 2/3 for the full trace. If per-instance,
 * runtime-configurable physical constants are ever genuinely needed, that
 * capability has to be added to the QEI MCAL first (it has no setter for
 * wheel diameter or wheel base today) - it cannot be retrofitted at this
 * layer alone.
 */
typedef struct {
    boolean         invertDirection;    /* Invert direction sense */
    uint32          velocityPeriodMs;   /* Velocity measurement period in ms */
    boolean         filterEnable;       /* Enable input filter */
    uint8           filterCount;        /* Filter sample count (0-15) */
} Encoder_ConfigType;

/*******************************************************************************
 *                          Velocity Data Structure                            *
 *******************************************************************************/

/* Trimmed to the kept SI/diagnostic unit set (ENCODER_REFACTOR.md) - rps/
 * degPerSec/mmPerSec/cmPerSec removed along with their standalone getters. */
typedef struct {
    float32             rpm;            /* Speed in RPM (diagnostics) */
    float32             radPerSec;      /* Angular velocity in rad/s (SI, ROS) */
    float32             mPerSec;        /* Linear velocity in m/s (chassis-level) */
    Encoder_DirectionType direction;    /* Current direction */
} Encoder_VelocityType;

/*******************************************************************************
 *                          Position Data Structure                            *
 *******************************************************************************/

/* Trimmed to the kept unit set (ENCODER_REFACTOR.md) - degrees/distanceMm/
 * distanceCm removed along with their standalone getters. */
typedef struct {
    sint32              totalCounts;    /* Total accumulated counts (signed) */
    float32             revolutions;    /* Total revolutions (signed) */
    float32             radians;        /* Current angle in radians (0-2π) */
    float32             distanceM;      /* Distance traveled in meters */
} Encoder_PositionType;

/*******************************************************************************
 *                          Default Configuration                              *
 *******************************************************************************/

#define ENCODER_CONFIG_DEFAULT { \
    .invertDirection = FALSE,    \
    .velocityPeriodMs = 20,      \
    .filterEnable = TRUE,        \
    .filterCount = 3             \
}

/*******************************************************************************
 *                          Initialization Functions                           *
 *******************************************************************************/

/**
 * @brief  Initialize all encoders with default configuration
 * @return ENCODER_OK on success
 * @note   Must be called after Port_Init()
 */
Encoder_StatusType Encoder_Init(void);

/**
 * @brief  De-initialize encoder driver
 * @return ENCODER_OK on success
 */
Encoder_StatusType Encoder_DeInit(void);

/*******************************************************************************
 *                          Velocity Functions                                 *
 *******************************************************************************/

/**
 * @brief  Get velocity in RPM
 * @param  encoderId: Encoder ID
 * @return Velocity in RPM (signed: negative = reverse)
 */
float32 Encoder_GetRPM(Encoder_IdType encoderId);

/**
 * @brief  Get velocity in radians per second
 * @param  encoderId: Encoder ID
 * @return Angular velocity in rad/s (signed)
 */
float32 Encoder_GetRadPerSec(Encoder_IdType encoderId);

/**
 * @brief  Get linear velocity in m/s
 * @param  encoderId: Encoder ID
 * @return Linear velocity in m/s (signed)
 */
float32 Encoder_GetLinearVelocityM(Encoder_IdType encoderId);

/**
 * @brief  Get complete velocity data structure
 * @param  encoderId: Encoder ID
 * @param  velocity: Pointer to velocity structure to fill
 * @return ENCODER_OK on success
 */
Encoder_StatusType Encoder_GetVelocity(Encoder_IdType encoderId, Encoder_VelocityType *velocity);

/*******************************************************************************
 *                          Position Functions                                 *
 *******************************************************************************/

/**
 * @brief  Get total accumulated counts (with overflow handling)
 * @param  encoderId: Encoder ID
 * @return Total counts (signed)
 */
sint32 Encoder_GetTotalCounts(Encoder_IdType encoderId);

/**
 * @brief  Get total revolutions since last reset
 * @param  encoderId: Encoder ID
 * @return Total revolutions (signed)
 */
float32 Encoder_GetRevolutions(Encoder_IdType encoderId);

/**
 * @brief  Get current angle in radians (0-2π)
 * @param  encoderId: Encoder ID
 * @return Current angle in radians
 */
float32 Encoder_GetAngleRadians(Encoder_IdType encoderId);

/**
 * @brief  Get complete position data structure
 * @param  encoderId: Encoder ID
 * @param  position: Pointer to position structure to fill
 * @return ENCODER_OK on success
 */
Encoder_StatusType Encoder_GetPosition(Encoder_IdType encoderId, Encoder_PositionType *position);

/*******************************************************************************
 *                          Distance Functions                                 *
 *******************************************************************************/

/**
 * @brief  Get distance traveled in centimeters
 * @param  encoderId: Encoder ID
 * @return Distance in cm (signed)
 * @note   KEPT despite being a "redundant unit" by the same standard as
 *         the removed mm/cm velocity getters - control_distance.c calls
 *         this directly (8 call sites). Removing it would have broken
 *         that caller. Derived from Encoder_GetDistanceM() internally.
 */
float32 Encoder_GetDistanceCm(Encoder_IdType encoderId);

/**
 * @brief  Get distance traveled in meters
 * @param  encoderId: Encoder ID
 * @return Distance in meters (signed)
 */
float32 Encoder_GetDistanceM(Encoder_IdType encoderId);

/**
 * @brief  Reset distance tracking to zero
 * @param  encoderId: Encoder ID
 * @return ENCODER_OK on success
 */
Encoder_StatusType Encoder_ResetDistance(Encoder_IdType encoderId);

/**
 * @brief  Reset all encoder distances to zero
 * @return ENCODER_OK on success
 */
Encoder_StatusType Encoder_ResetAllDistances(void);

/**
 * @brief  Average forward distance of the two drive wheels, in cm.
 * @return Mean of the LEFT/RIGHT cumulative tick counts converted via
 *         QEI_CM_PER_COUNT. Cadence-free (reads hardware counts directly,
 *         no Encoder_UpdateDistance needed). Signed: +forward.
 */
float32 Encoder_GetAverageDistanceCm(void);

/*******************************************************************************
 *                          Direction Functions                                *
 *******************************************************************************/

/**
 * @brief  Get current direction
 * @param  encoderId: Encoder ID
 * @return ENCODER_DIR_FORWARD or ENCODER_DIR_REVERSE
 */
Encoder_DirectionType Encoder_GetDirection(Encoder_IdType encoderId);

/**
 * @brief  Check if encoder is moving forward
 * @param  encoderId: Encoder ID
 * @return TRUE if forward
 */
boolean Encoder_IsForward(Encoder_IdType encoderId);

/**
 * @brief  Check if encoder is moving reverse
 * @param  encoderId: Encoder ID
 * @return TRUE if reverse
 */
boolean Encoder_IsReverse(Encoder_IdType encoderId);

/**
 * @brief  Check if encoder is stationary (velocity ≈ 0)
 * @param  encoderId: Encoder ID
 * @return TRUE if stationary
 */
boolean Encoder_IsStationary(Encoder_IdType encoderId);

/* NOTE: the odometry (x/y/theta) surface that used to live here was
 * removed (ENCODER_REFACTOR.md) - it belongs on the Orin (ros2_control's
 * Ackermann controller), and the implementation it called into
 * (QEI_UpdateOdometry et al.) was differential-drive kinematics, which is
 * wrong for this Ackermann robot. QEI's own odometry functions are
 * untouched (out of scope) - this HAL simply no longer calls them. */

/*******************************************************************************
 *                          Update Functions                                   *
 *******************************************************************************/

/**
 * @brief  Update distance tracking for an encoder
 * @param  encoderId: Encoder ID
 * @return ENCODER_OK on success
 * @note   Call periodically (faster than position overflow rate)
 */
Encoder_StatusType Encoder_UpdateDistance(Encoder_IdType encoderId);

/**
 * @brief  Update distance tracking for all encoders
 * @return ENCODER_OK on success
 */
Encoder_StatusType Encoder_UpdateAllDistances(void);

/*******************************************************************************
 *                          Status Functions                                   *
 *******************************************************************************/

/**
 * @brief  Get encoder state
 * @param  encoderId: Encoder ID
 * @return Current state
 */
Encoder_StateType Encoder_GetState(Encoder_IdType encoderId);

/**
 * @brief  Check for encoder error (phase error)
 * @param  encoderId: Encoder ID
 * @return TRUE if error detected
 */
boolean Encoder_HasError(Encoder_IdType encoderId);

/**
 * @brief  Clear encoder error
 * @param  encoderId: Encoder ID
 * @return ENCODER_OK on success
 */
Encoder_StatusType Encoder_ClearError(Encoder_IdType encoderId);

/*******************************************************************************
 *                          Configuration Functions                            *
 *******************************************************************************/

/* NOTE: Encoder_SetCountsPerRev/SetWheelDiameter/SetWheelBase were
 * removed here (ENCODER_REFACTOR.md) - they updated local state nothing
 * read; every reported value comes from QEI's fixed, compile-time
 * constants (qei_cfg.h), which these had no way to influence. The
 * velocity-callback/direction-callback registration functions that used
 * to live here were also removed - they stored a function pointer that
 * was never invoked, and the QEI-level callback API they would have
 * needed to wire into no longer exists (removed in the QEI refactor). */

/**
 * @brief  Set direction inversion
 * @param  encoderId: Encoder ID
 * @param  invert: TRUE to invert direction
 * @return ENCODER_OK on success
 */
Encoder_StatusType Encoder_SetInvertDirection(Encoder_IdType encoderId, boolean invert);

/**
 * @brief  Set velocity measurement period
 * @param  encoderId: Encoder ID
 * @param  periodMs: Period in milliseconds
 * @return ENCODER_OK on success
 */
Encoder_StatusType Encoder_SetVelocityPeriod(Encoder_IdType encoderId, uint32 periodMs);

/**
 * @brief  Enable/disable input filter
 * @param  encoderId: Encoder ID
 * @param  enable: TRUE to enable filter
 * @param  count: Filter count (0-15)
 * @return ENCODER_OK on success
 */
Encoder_StatusType Encoder_SetFilter(Encoder_IdType encoderId, boolean enable, uint8 count);

#endif /* ENCODER_H_ */
