/******************************************************************************
 *
 * Module: Motor
 *
 * File Name: Motor.h
 *
 * Description: High-level DC Motor driver abstraction layer
 *              Provides convenient motor control over PWM/GPIO
 *              Compatible with your refactored PWM driver
 *
 * Hardware Mapping:
 * - Left Motor  (MOTOR_LEFT)  → PB4 (PWM), PD3 (Direction)
 * - Right Motor (MOTOR_RIGHT) → PB6 (PWM), PD2 (Direction)
 *
 ******************************************************************************/

#ifndef MOTOR_H_
#define MOTOR_H_

#include "Std_Types.h"
#include "PWM.h"

/*******************************************************************************
 *                          Motor Identification                               *
 *******************************************************************************/

typedef enum {
    MOTOR_LEFT = 0,             /* Left motor - PB4 PWM, PD3 Direction */
    MOTOR_RIGHT = 1,            /* Right motor - PB6 PWM, PD2 Direction */
    MOTOR_MAX
} Motor_IdType;

/*******************************************************************************
 *                          Motor Status Types                                 *
 *******************************************************************************/

typedef enum {
    MOTOR_OK = 0,
    MOTOR_ERROR_INVALID_ID,
    MOTOR_ERROR_INVALID_SPEED,
    MOTOR_ERROR_NOT_INITIALIZED
} Motor_StatusType;

typedef enum {
    MOTOR_STATE_UNINIT = 0,
    MOTOR_STATE_STOPPED,
    MOTOR_STATE_RUNNING
} Motor_StateType;

/**
 * @brief  Motor rotation direction, relative to the CHASSIS, not the
 *         motor's raw electrical polarity.
 *
 * @note   MIRROR-MOUNT COMPENSATION (verified on hardware - do not
 *         "simplify" away): the two motors are mounted as mirror images
 *         of each other, so the SAME electrical DIR level drives them in
 *         OPPOSITE physical/chassis senses. The private
 *         Motor_SetDirectionGPIO() in Motor.c deliberately writes the
 *         right motor's DIR pin inverted relative to the left motor's,
 *         so that MOTOR_DIR_FORWARD on BOTH motors propels the chassis
 *         straight forward (and MOTOR_DIR_REVERSE on both propels it
 *         straight backward). This asymmetry is intentional and was
 *         physically verified - removing it would make the chassis spin
 *         in place instead of driving straight.
 */
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_REVERSE = 1
} Motor_DirectionType;

/*******************************************************************************
 *                          Initialization Functions                           *
 *******************************************************************************/

/**
 * @brief  Initialize motor driver
 * @return MOTOR_OK on success
 * @note   Must be called after Port_Init() and PWM_Init()
 */
Motor_StatusType Motor_Init(void);

/*******************************************************************************
 *                          Basic Motor Control                                *
 *******************************************************************************/

/**
 * @brief  Set motor speed (maintains current direction)
 * @param  motorId: MOTOR_LEFT or MOTOR_RIGHT
 * @param  speedPercent: Speed 0.0 to 100.0%
 * @return MOTOR_OK on success
 */
Motor_StatusType Motor_SetSpeed(Motor_IdType motorId, float32 speedPercent);

/**
 * @brief  Set motor direction
 * @param  motorId: MOTOR_LEFT or MOTOR_RIGHT
 * @param  direction: MOTOR_DIR_FORWARD or MOTOR_DIR_REVERSE
 * @return MOTOR_OK on success
 */
Motor_StatusType Motor_SetDirection(Motor_IdType motorId, Motor_DirectionType direction);

/**
 * @brief  Set motor speed with direction in one call
 * @param  motorId: MOTOR_LEFT or MOTOR_RIGHT
 * @param  speedPercent: Speed 0.0 to 100.0%
 * @param  direction: MOTOR_DIR_FORWARD or MOTOR_DIR_REVERSE
 * @return MOTOR_OK on success
 */
Motor_StatusType Motor_SetSpeedWithDirection(Motor_IdType motorId, 
                                              float32 speedPercent, 
                                              Motor_DirectionType direction);

/**
 * @brief  Set motor using signed speed (-100 to +100)
 * @param  motorId: MOTOR_LEFT or MOTOR_RIGHT
 * @param  signedSpeed: -100.0 (full reverse) to +100.0 (full forward)
 * @return MOTOR_OK on success
 */
Motor_StatusType Motor_SetSignedSpeed(Motor_IdType motorId, float32 signedSpeed);

/**
 * @brief  Stop a specific motor (speed = 0%)
 * @param  motorId: MOTOR_LEFT or MOTOR_RIGHT
 * @return MOTOR_OK on success
 */
Motor_StatusType Motor_Stop(Motor_IdType motorId);

/*******************************************************************************
 *                          Dual Motor Control (Robot Movement)                *
 *******************************************************************************/

/**
 * @brief  Drive both motors forward at same speed
 * @param  speedPercent: Speed 0.0 to 100.0%
 * @return MOTOR_OK on success
 * @note   Relies on the mirror-mount DIR compensation described on
 *         Motor_DirectionType - verified to drive the chassis straight.
 */
Motor_StatusType Motor_DriveForward(float32 speedPercent);

/**
 * @brief  Drive both motors reverse at same speed
 * @param  speedPercent: Speed 0.0 to 100.0%
 * @return MOTOR_OK on success
 * @note   Relies on the mirror-mount DIR compensation described on
 *         Motor_DirectionType - verified to drive the chassis straight.
 */
Motor_StatusType Motor_DriveReverse(float32 speedPercent);

/**
 * @brief  Set individual motor speeds with signed values
 * @param  leftSpeed: Left motor -100.0 to +100.0
 * @param  rightSpeed: Right motor -100.0 to +100.0
 * @return MOTOR_OK on success
 */
Motor_StatusType Motor_SetSpeeds(float32 leftSpeed, float32 rightSpeed);

/**
 * @brief  Turn left (right motor faster)
 * @param  speedPercent: Base speed
 * @param  turnRate: Turn sharpness 0.0 (gentle) to 100.0 (spin)
 * @return MOTOR_OK on success
 */
Motor_StatusType Motor_TurnLeft(float32 speedPercent, float32 turnRate);

/**
 * @brief  Turn right (left motor faster)
 * @param  speedPercent: Base speed
 * @param  turnRate: Turn sharpness 0.0 (gentle) to 100.0 (spin)
 * @return MOTOR_OK on success
 */
Motor_StatusType Motor_TurnRight(float32 speedPercent, float32 turnRate);

/**
 * @brief  Spin in place (motors in opposite directions)
 * @param  speedPercent: Spin speed
 * @param  direction: MOTOR_DIR_FORWARD=clockwise, MOTOR_DIR_REVERSE=counter-clockwise
 * @return MOTOR_OK on success
 */
Motor_StatusType Motor_Spin(float32 speedPercent, Motor_DirectionType direction);

/**
 * @brief  Stop all motors
 * @return MOTOR_OK on success
 */
Motor_StatusType Motor_StopAll(void);

/*******************************************************************************
 *                          Status Functions                                   *
 *******************************************************************************/

/**
 * @brief  Get motor state
 * @param  motorId: MOTOR_LEFT or MOTOR_RIGHT
 * @return Current motor state
 */
Motor_StateType Motor_GetState(Motor_IdType motorId);

/**
 * @brief  Get current motor direction
 * @param  motorId: MOTOR_LEFT or MOTOR_RIGHT
 * @return Current direction
 */
Motor_DirectionType Motor_GetDirection(Motor_IdType motorId);

/**
 * @brief  Get current motor speed
 * @param  motorId: MOTOR_LEFT or MOTOR_RIGHT
 * @return Current speed 0.0 to 100.0%, negative on error
 */
float32 Motor_GetSpeed(Motor_IdType motorId);

/**
 * @brief  Check if motor is running
 * @param  motorId: MOTOR_LEFT or MOTOR_RIGHT
 * @return TRUE if motor speed > 0
 */
boolean Motor_IsRunning(Motor_IdType motorId);

#endif /* MOTOR_H_ */
