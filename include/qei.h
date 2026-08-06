/******************************************************************************
 *
 * Module: QEI (Quadrature Encoder Interface)
 *
 * File Name: qei.h
 *
 * Description: Header file for TM4C123GH6PM QEI driver
 *              Designed for DC motor encoder feedback
 *
 ******************************************************************************/

#ifndef QEI_H_
#define QEI_H_

#include "qei_types.h"
#include "qei_cfg.h"

/*******************************************************************************
 *                          Initialization Functions                           *
 *******************************************************************************/

/**
 * @brief  Initialize all enabled QEI modules
 * @return QEI_OK on success
 * @note   GPIO pins must be configured by Port driver before calling this
 */
QEI_StatusType QEI_Init(void);

/**
 * @brief  Initialize a specific QEI channel
 * @param  channel: QEI channel (ENCODER_A or ENCODER_B)
 * @return QEI_OK on success
 */
QEI_StatusType QEI_InitChannel(QEI_ChannelType channel);

/**
 * @brief  De-initialize QEI module
 * @return QEI_OK on success
 */
QEI_StatusType QEI_DeInit(void);

/*******************************************************************************
 *                          Position Functions                                 *
 *******************************************************************************/

/**
 * @brief  Get raw position counter value
 * @param  channel: QEI channel
 * @return Raw position (0 to MAXPOS)
 */
uint32 QEI_GetPosition(QEI_ChannelType channel);

/**
 * @brief  Set position counter value
 * @param  channel: QEI channel
 * @param  position: Position value to set
 * @return QEI_OK on success
 */
QEI_StatusType QEI_SetPosition(QEI_ChannelType channel, uint32 position);

/**
 * @brief  Reset position counter to zero
 * @param  channel: QEI channel
 * @return QEI_OK on success
 */
QEI_StatusType QEI_ResetPosition(QEI_ChannelType channel);

/**
 * @brief  Get position in degrees (0-360)
 * @param  channel: QEI channel
 * @return Position in degrees
 */
float32 QEI_GetPositionDegrees(QEI_ChannelType channel);

/**
 * @brief  Get position in radians (0-2π)
 * @param  channel: QEI channel
 * @return Position in radians
 */
float32 QEI_GetPositionRadians(QEI_ChannelType channel);

/**
 * @brief  Get complete position data structure
 * @param  channel: QEI channel
 * @param  position: Pointer to position structure to fill
 * @return QEI_OK on success
 */
QEI_StatusType QEI_GetPositionData(QEI_ChannelType channel, QEI_PositionType *position);

/*******************************************************************************
 *                          Velocity Functions                                 *
 *******************************************************************************/

/**
 * @brief  Get raw velocity register value (counts per period)
 * @param  channel: QEI channel
 * @return Raw velocity value
 */
uint32 QEI_GetVelocityRaw(QEI_ChannelType channel);

/**
 * @brief  Get velocity in RPM
 * @param  channel: QEI channel
 * @return Velocity in RPM (signed: negative = reverse)
 */
float32 QEI_GetVelocityRPM(QEI_ChannelType channel);

/**
 * @brief  Get velocity in radians per second
 * @param  channel: QEI channel
 * @return Angular velocity in rad/s
 */
float32 QEI_GetVelocityRadPerSec(QEI_ChannelType channel);

/**
 * @brief  Get velocity in degrees per second
 * @param  channel: QEI channel
 * @return Angular velocity in deg/s
 */
float32 QEI_GetVelocityDegPerSec(QEI_ChannelType channel);

/**
 * @brief  Get linear velocity in mm/s (based on wheel diameter)
 * @param  channel: QEI channel
 * @return Linear velocity in mm/s
 */
float32 QEI_GetVelocityMmPerSec(QEI_ChannelType channel);

/**
 * @brief  Get linear velocity in cm/s
 * @param  channel: QEI channel
 * @return Linear velocity in cm/s
 */
float32 QEI_GetVelocityCmPerSec(QEI_ChannelType channel);

/**
 * @brief  Get linear velocity in m/s
 * @param  channel: QEI channel
 * @return Linear velocity in m/s
 */
float32 QEI_GetVelocityMPerSec(QEI_ChannelType channel);

/**
 * @brief  Get complete velocity data structure
 * @param  channel: QEI channel
 * @param  velocity: Pointer to velocity structure to fill
 * @return QEI_OK on success
 */
QEI_StatusType QEI_GetVelocityData(QEI_ChannelType channel, QEI_VelocityType *velocity);

/*******************************************************************************
 *                          Direction Functions                                *
 *******************************************************************************/

/**
 * @brief  Get current direction
 * @param  channel: QEI channel
 * @return QEI_DIR_FORWARD or QEI_DIR_REVERSE
 */
QEI_DirectionType QEI_GetDirection(QEI_ChannelType channel);

/**
 * @brief  Check if encoder is moving forward
 * @param  channel: QEI channel
 * @return TRUE if forward, FALSE if reverse
 */
boolean QEI_IsForward(QEI_ChannelType channel);

/**
 * @brief  Check if encoder is moving reverse
 * @param  channel: QEI channel
 * @return TRUE if reverse, FALSE if forward
 */
boolean QEI_IsReverse(QEI_ChannelType channel);

/*******************************************************************************
 *                          Distance Tracking Functions                        *
 *******************************************************************************/

/**
 * @brief  Reset distance tracker to zero
 * @param  channel: QEI channel
 * @return QEI_OK on success
 * @note   Call this before starting distance measurement
 */
QEI_StatusType QEI_ResetDistance(QEI_ChannelType channel);

/**
 * @brief  Update distance tracking (call periodically from main loop or ISR)
 * @param  channel: QEI channel
 * @return QEI_OK on success
 * @note   This accumulates distance including direction changes
 */
QEI_StatusType QEI_UpdateDistance(QEI_ChannelType channel);

/**
 * @brief  Get total distance traveled in millimeters
 * @param  channel: QEI channel
 * @return Total distance in mm (signed based on net direction)
 */
float32 QEI_GetDistanceMm(QEI_ChannelType channel);

/**
 * @brief  Get total distance traveled in centimeters
 * @param  channel: QEI channel
 * @return Total distance in cm
 */
float32 QEI_GetDistanceCm(QEI_ChannelType channel);

/**
 * @brief  Get total distance traveled in meters
 * @param  channel: QEI channel
 * @return Total distance in meters
 */
float32 QEI_GetDistanceM(QEI_ChannelType channel);

/**
 * @brief  Get total encoder counts since reset
 * @param  channel: QEI channel
 * @return Total counts (signed)
 */
sint32 QEI_GetTotalCounts(QEI_ChannelType channel);

/**
 * @brief  Get total revolutions since reset
 * @param  channel: QEI channel
 * @return Total revolutions (signed)
 */
float32 QEI_GetTotalRevolutions(QEI_ChannelType channel);

/*******************************************************************************
 *                          Dual Encoder Functions (Odometry)                  *
 *******************************************************************************/

/**
 * @brief  Reset odometry to zero (both encoders)
 * @return QEI_OK on success
 */
QEI_StatusType QEI_ResetOdometry(void);

/**
 * @brief  Update odometry calculation (call periodically)
 * @return QEI_OK on success
 * @note   Updates x, y, theta based on differential drive kinematics
 */
QEI_StatusType QEI_UpdateOdometry(void);

/**
 * @brief  Get current odometry data
 * @param  odometry: Pointer to odometry structure to fill
 * @return QEI_OK on success
 */
QEI_StatusType QEI_GetOdometry(QEI_OdometryType *odometry);

/**
 * @brief  Set odometry position manually
 * @param  x: X position in mm
 * @param  y: Y position in mm
 * @param  theta: Heading in radians
 * @return QEI_OK on success
 */
QEI_StatusType QEI_SetOdometry(float32 x, float32 y, float32 theta);

/*******************************************************************************
 *                          Status Functions                                   *
 *******************************************************************************/

/**
 * @brief  Get QEI channel state
 * @param  channel: QEI channel
 * @return Current state
 */
QEI_StateType QEI_GetState(QEI_ChannelType channel);

/**
 * @brief  Check for encoder error (phase error)
 * @param  channel: QEI channel
 * @return TRUE if error detected
 */
boolean QEI_HasError(QEI_ChannelType channel);

/**
 * @brief  Clear error flag
 * @param  channel: QEI channel
 * @return QEI_OK on success
 */
QEI_StatusType QEI_ClearError(QEI_ChannelType channel);

/**
 * @brief  Check if encoder is stationary (velocity = 0)
 * @param  channel: QEI channel
 * @return TRUE if stationary
 */
boolean QEI_IsStationary(QEI_ChannelType channel);

/*******************************************************************************
 *                          Configuration Functions                            *
 *******************************************************************************/

/**
 * @brief  Set velocity measurement period
 * @param  channel: QEI channel
 * @param  periodMs: Period in milliseconds
 * @return QEI_OK on success
 */
QEI_StatusType QEI_SetVelocityPeriod(QEI_ChannelType channel, uint32 periodMs);

/**
 * @brief  Set maximum position value
 * @param  channel: QEI channel
 * @param  maxPos: Maximum position value
 * @return QEI_OK on success
 */
QEI_StatusType QEI_SetMaxPosition(QEI_ChannelType channel, uint32 maxPos);

/**
 * @brief  Enable/disable input filter
 * @param  channel: QEI channel
 * @param  enable: TRUE to enable filter
 * @param  count: Filter count (0-15)
 * @return QEI_OK on success
 */
QEI_StatusType QEI_SetFilter(QEI_ChannelType channel, boolean enable, uint8 count);

/*******************************************************************************
 *                          Legacy Compatibility                               *
 *******************************************************************************/

/* For backward compatibility with old ENCODER API */
#define ENCODER_Init(motor)                     QEI_InitChannel(motor)
#define Reset_DistanceTracker(motor)            QEI_ResetDistance(motor)
#define Compute_Travelled_Distance_CM(motor)    QEI_GetDistanceCm(motor)
#define Compute_RPM(motor)                      QEI_GetVelocityRPM(motor)

/*******************************************************************************
 *      DIAGNOSTIC ONLY - TEMPORARY (see QEI_VELOCITY_RESIDUAL_AUDIT.md)       *
 *      Remove this declaration (and its definition/call sites) once the      *
 *      residual velocity-magnitude wobble investigation is complete.         *
 *******************************************************************************/

/**
 * @brief  DIAGNOSTIC ONLY: read-and-clear the LEFT-channel (QEI_CHANNEL_1)
 *         internals recorded by the most recent QEI_GetVelocityRPM(QEI_CHANNEL_1)
 *         call(s). Read-only observability - adds no logic to the velocity path.
 * @param  rpmMag: unsigned magnitude from QEISPEED, before sign (out, nullable)
 * @param  delta: QEIPOS_now - lastVelPosition captured the last time the
 *         velocity-timer RIS gate opened (out, nullable)
 * @param  sign: sign applied to rpmMag for the last call, +1.0f/-1.0f (out, nullable)
 * @param  gateOpened: TRUE if the RIS timer flag fired (new window) on the
 *         last call, FALSE if a cached sign was reused (out, nullable)
 * @param  calls: number of QEI_GetVelocityRPM(QEI_CHANNEL_1) calls since the
 *         last read of this parameter; cleared as a side effect of this call
 *         so the caller can see "reads per control tick" (out, nullable)
 */
void QEI_GetLeftVelDiag(float32 *rpmMag, sint32 *delta, float32 *sign,
                         boolean *gateOpened, uint32 *calls);

#endif /* QEI_H_ */
