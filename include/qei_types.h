/******************************************************************************
 *
 * Module: QEI (Quadrature Encoder Interface)
 *
 * File Name: qei_types.h
 *
 * Description: Type definitions for TM4C123GH6PM QEI driver
 *
 ******************************************************************************/

#ifndef QEI_TYPES_H_
#define QEI_TYPES_H_

#include "Std_Types.h"

/*******************************************************************************
 *                          QEI Channel Identification                         *
 *******************************************************************************/

/*
 * Channel-to-wheel mapping - hardware-verified (see QEI_AUDIT.md /
 * QEI_REFACTOR.md hand-spin test). This was previously labeled backwards
 * here (QEI_CHANNEL_0 as "Motor A"/left, QEI_CHANNEL_1 as "Motor B"/
 * right) - that labeling directly contradicted qei_cfg.h's
 * QEI0_SWAP_SIGNALS/QEI1_SWAP_SIGNALS comments, which had it right. The
 * hand-spin test confirmed qei_cfg.h's comments were correct:
 *   QEI_CHANNEL_0 = RIGHT motor (PD6/PD7)
 *   QEI_CHANNEL_1 = LEFT  motor (PC5/PC6)
 */
typedef enum {
    QEI_CHANNEL_0 = 0,      /* QEI0 - RIGHT motor encoder (PD6/PD7) */
    QEI_CHANNEL_1 = 1,      /* QEI1 - LEFT motor encoder (PC5/PC6) */
    QEI_CHANNEL_MAX
} QEI_ChannelType;

/* Motor aliases for readability */
typedef QEI_ChannelType EncoderID;
#ifndef ENCODER_A
#define ENCODER_A           QEI_CHANNEL_0
#endif
#ifndef ENCODER_B
#define ENCODER_B           QEI_CHANNEL_1
#endif
#ifndef ENCODER_LEFT
#define ENCODER_LEFT        QEI_CHANNEL_1
#endif
#ifndef ENCODER_RIGHT
#define ENCODER_RIGHT       QEI_CHANNEL_0
#endif

/* Legacy compatibility */
typedef QEI_ChannelType MotorID;
#define MOTOR_A             QEI_CHANNEL_0
#define MOTOR_B             QEI_CHANNEL_1

/*******************************************************************************
 *                          QEI Status Types                                   *
 *******************************************************************************/

typedef enum {
    QEI_OK = 0,
    QEI_ERROR_INVALID_CHANNEL,
    QEI_ERROR_NOT_INITIALIZED,
    QEI_ERROR_NULL_PTR,
    QEI_ERROR_INVALID_CONFIG
} QEI_StatusType;

typedef enum {
    QEI_STATE_UNINIT = 0,
    QEI_STATE_IDLE,
    QEI_STATE_RUNNING
} QEI_StateType;

/*******************************************************************************
 *                          Direction Types                                    *
 *******************************************************************************/

typedef enum {
    QEI_DIR_FORWARD = 0,    /* Counter incrementing */
    QEI_DIR_REVERSE = 1     /* Counter decrementing */
} QEI_DirectionType;

/*******************************************************************************
 *                          Velocity Data Structure                            *
 *******************************************************************************/

typedef struct {
    uint32              rawSpeed;       /* Raw speed register value (counts per period) */
    QEI_DirectionType   direction;      /* Current direction */
    float32             rpm;            /* Speed in RPM */
    float32             rps;            /* Speed in revolutions per second */
    float32             degPerSec;      /* Speed in degrees per second */
    float32             radPerSec;      /* Speed in radians per second */
    float32             mmPerSec;       /* Linear speed in mm/s */
    float32             cmPerSec;       /* Linear speed in cm/s */
    float32             mPerSec;        /* Linear speed in m/s */
} QEI_VelocityType;

/*******************************************************************************
 *                          Position Data Structure                            *
 *******************************************************************************/

typedef struct {
    uint32              rawPosition;    /* Raw position register value */
    sint32              totalCounts;    /* Total accumulated counts (with overflow) */
    float32             revolutions;    /* Total revolutions */
    float32             degrees;        /* Position in degrees (0-360) */
    float32             radians;        /* Position in radians (0-2π) */
    float32             distanceMm;     /* Distance traveled in mm */
    float32             distanceCm;     /* Distance traveled in cm */
    float32             distanceM;      /* Distance traveled in meters */
} QEI_PositionType;

/*******************************************************************************
 *                          Odometry Data Structure                            *
 *******************************************************************************/

typedef struct {
    float32             x;              /* X position in mm */
    float32             y;              /* Y position in mm */
    float32             theta;          /* Heading angle in radians */
    float32             distanceTotal;  /* Total distance traveled in mm */
    float32             linearVelocity; /* Linear velocity in mm/s */
    float32             angularVelocity;/* Angular velocity in rad/s */
} QEI_OdometryType;

/*******************************************************************************
 *                          Callback Types                                     *
 *******************************************************************************/

/* Callback for position events (index pulse, position match) */
typedef void (*QEI_PositionCallbackType)(QEI_ChannelType channel, uint32 position);

/* Callback for velocity events */
typedef void (*QEI_VelocityCallbackType)(QEI_ChannelType channel, uint32 velocity);

/* Callback for direction change */
typedef void (*QEI_DirectionCallbackType)(QEI_ChannelType channel, QEI_DirectionType direction);

/* Callback for errors */
typedef void (*QEI_ErrorCallbackType)(QEI_ChannelType channel, uint32 errorFlags);

/*******************************************************************************
 *                          Callback Configuration                             *
 *******************************************************************************/

typedef struct {
    QEI_PositionCallbackType    positionCallback;
    QEI_VelocityCallbackType    velocityCallback;
    QEI_DirectionCallbackType   directionCallback;
    QEI_ErrorCallbackType       errorCallback;
} QEI_CallbackConfigType;

/*******************************************************************************
 *                          Channel Configuration                              *
 *******************************************************************************/

typedef struct {
    uint32              maxPosition;        /* Maximum position count (MAXPOS) */
    uint32              velocityPeriod;     /* Velocity measurement period (LOAD) */
    boolean             filterEnable;       /* Enable input filter */
    uint8               filterCount;        /* Filter sample count (0-15) */
    boolean             swapSignals;        /* Swap A/B inputs */
    boolean             invertDirection;    /* Invert direction sense */
    boolean             indexEnable;        /* Enable index signal */
    boolean             indexResetEnable;   /* Reset position on index */
    boolean             velocityEnable;     /* Enable velocity measurement */
    boolean             captureEnable;      /* Enable capture mode */
} QEI_ChannelConfigType;

#endif /* QEI_TYPES_H_ */
