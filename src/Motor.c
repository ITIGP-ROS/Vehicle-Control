/******************************************************************************
 *
 * Module: Motor
 *
 * File Name: Motor.c
 *
 * Description: Implementation of high-level DC Motor driver
 *              Abstraction layer over PWM and GPIO
 *
 * Hardware Mapping:
 * - Left Motor  (MOTOR_LEFT)  → PB4 (PWM via PWM_CHANNEL_LEFT), PD3 (DIR)
 * - Right Motor (MOTOR_RIGHT) → PB6 (PWM via PWM_CHANNEL_RIGHT), PD2 (DIR)
 *
 * Note: the PWM driver's channel enum is named directly after LEFT/RIGHT,
 * matching this module's own naming one-to-one - no bridging required.
 *
 ******************************************************************************/

#include "Motor.h"
#include "PWM.h"
#include "PORT.h"

/*******************************************************************************
 *                          Hardware Pin Mapping                               *
 *******************************************************************************/

/* Direction GPIO pins - PORT driver pin IDs (see PORT_CFG.h), written via
 * Port_WritePin() rather than direct register access. */
#define MOTOR_LEFT_DIR_PIN      PORT_D_PIN_3    /* PD3 - Left motor direction */
#define MOTOR_RIGHT_DIR_PIN     PORT_D_PIN_2    /* PD2 - Right motor direction */

#define MOTOR_LEFT_PWM_CHANNEL  PWM_CHANNEL_LEFT
#define MOTOR_RIGHT_PWM_CHANNEL PWM_CHANNEL_RIGHT


/* Maximum duty-cycle percentage. This layer is open-loop: "speed" here
 * means PWM duty %, not a physical velocity unit (RPM/m/s) - the PID/
 * velocity layer that converts a real velocity target into a duty % is a
 * future layer above this one. */
#define MOTOR_MAX_SPEED              100.0f

/*******************************************************************************
 *                          Private Variables                                  *
 *******************************************************************************/

typedef struct {
    Motor_StateType     state;
    Motor_DirectionType direction;
    float32             speedPercent;
} Motor_InfoType;

static Motor_InfoType Motor_Status[MOTOR_MAX] = {
    { MOTOR_STATE_UNINIT, MOTOR_DIR_FORWARD, 0.0f },  /* Left motor */
    { MOTOR_STATE_UNINIT, MOTOR_DIR_FORWARD, 0.0f }   /* Right motor */
};

/*******************************************************************************
 *                          Private Functions                                  *
 *******************************************************************************/

/**
 * @brief  Set GPIO direction pin
 *
 * MIRROR-MOUNT COMPENSATION (verified on hardware - do not "simplify"
 * away, see Motor_DirectionType in Motor.h for the full rationale): the
 * two motors are mounted as mirror images of each other, so the SAME
 * electrical DIR level drives them in OPPOSITE physical/chassis senses.
 * The right motor's DIR pin is therefore deliberately written INVERTED
 * relative to the left motor's, so that MOTOR_DIR_FORWARD on BOTH motors
 * propels the chassis straight forward. Verified DIR levels per command
 * (do not change without re-verifying on hardware):
 *   LEFT  (PD3): forward -> STD_HIGH, reverse -> STD_LOW
 *   RIGHT (PD2): forward -> STD_LOW,  reverse -> STD_HIGH  (inverted)
 */
static void Motor_SetDirectionGPIO(Motor_IdType motorId, Motor_DirectionType direction)
{
    if (motorId == MOTOR_LEFT) {
        /* Left motor - PD3 */
        if (direction == MOTOR_DIR_FORWARD) {
            Port_WritePin(MOTOR_LEFT_DIR_PIN, STD_HIGH);
        } else {
            Port_WritePin(MOTOR_LEFT_DIR_PIN, STD_LOW);
        }
    } else {
        /* Right motor - PD2 - inverted, see comment above */
        if (direction == MOTOR_DIR_FORWARD) {
            Port_WritePin(MOTOR_RIGHT_DIR_PIN, STD_LOW);
        } else {
            Port_WritePin(MOTOR_RIGHT_DIR_PIN, STD_HIGH);
        }
    }
}

/**
 * @brief  Validate motor ID
 */
static boolean Motor_IsValidId(Motor_IdType motorId)
{
    return (motorId < MOTOR_MAX);
}

/**
 * @brief  Clamp speed to valid range
 */
static float32 Motor_ClampSpeed(float32 speed)
{
    if (speed < 0.0f) return 0.0f;
    if (speed > MOTOR_MAX_SPEED) return MOTOR_MAX_SPEED;
    return speed;
}

/*******************************************************************************
 *                          Initialization Functions                           *
 *******************************************************************************/

Motor_StatusType Motor_Init(void)
{
    /* Initialize both motors to stopped state */
    Motor_Status[MOTOR_LEFT].state = MOTOR_STATE_STOPPED;
    Motor_Status[MOTOR_LEFT].direction = MOTOR_DIR_FORWARD;
    Motor_Status[MOTOR_LEFT].speedPercent = 0.0f;
    
    Motor_Status[MOTOR_RIGHT].state = MOTOR_STATE_STOPPED;
    Motor_Status[MOTOR_RIGHT].direction = MOTOR_DIR_FORWARD;
    Motor_Status[MOTOR_RIGHT].speedPercent = 0.0f;
    
    /* Set both direction pins to forward */
    Motor_SetDirectionGPIO(MOTOR_LEFT, MOTOR_DIR_FORWARD);
    Motor_SetDirectionGPIO(MOTOR_RIGHT, MOTOR_DIR_FORWARD);
    
    /* Set both motors to 0% speed */
    PWM_SetDuty(MOTOR_LEFT_PWM_CHANNEL, 0.0f);
    PWM_SetDuty(MOTOR_RIGHT_PWM_CHANNEL, 0.0f);
    
    return MOTOR_OK;
}

/*******************************************************************************
 *                          Basic Motor Control                                *
 *******************************************************************************/

Motor_StatusType Motor_SetSpeed(Motor_IdType motorId, float32 speedPercent)
{
    PWM_StatusType pwmStatus;
    
    /* Validate motor ID */
    if (!Motor_IsValidId(motorId)) {
        return MOTOR_ERROR_INVALID_ID;
    }
    
    /* Check initialization */
    if (Motor_Status[motorId].state == MOTOR_STATE_UNINIT) {
        return MOTOR_ERROR_NOT_INITIALIZED;
    }
    
    /* Clamp speed to valid range */
    speedPercent = Motor_ClampSpeed(speedPercent);
    
    /* Set PWM duty cycle */
    if (motorId == MOTOR_LEFT) {
        pwmStatus = PWM_SetDuty(MOTOR_LEFT_PWM_CHANNEL, speedPercent);
    } else {
        pwmStatus = PWM_SetDuty(MOTOR_RIGHT_PWM_CHANNEL, speedPercent);
    }
    
    if (pwmStatus != PWM_OK) {
        return MOTOR_ERROR_INVALID_SPEED;
    }
    
    /* Update state */
    Motor_Status[motorId].speedPercent = speedPercent;
    Motor_Status[motorId].state = (speedPercent > 0.0f) ? MOTOR_STATE_RUNNING : MOTOR_STATE_STOPPED;
    
    return MOTOR_OK;
}

Motor_StatusType Motor_SetDirection(Motor_IdType motorId, Motor_DirectionType direction)
{
    /* Validate motor ID */
    if (!Motor_IsValidId(motorId)) {
        return MOTOR_ERROR_INVALID_ID;
    }
    
    /* Check initialization */
    if (Motor_Status[motorId].state == MOTOR_STATE_UNINIT) {
        return MOTOR_ERROR_NOT_INITIALIZED;
    }
    
    /* Set GPIO direction pin */
    Motor_SetDirectionGPIO(motorId, direction);
    
    /* Update state */
    Motor_Status[motorId].direction = direction;
    
    return MOTOR_OK;
}

Motor_StatusType Motor_SetSpeedWithDirection(Motor_IdType motorId, 
                                              float32 speedPercent, 
                                              Motor_DirectionType direction)
{
    Motor_StatusType status;

    /* M-1: when the DIRECTION actually CHANGES, drop the duty to 0 BEFORE
     * flipping the DIR pin.
     *
     * The old order was DIR-then-duty, so a sign-flipping command (e.g. the
     * velocity PID crossing zero, +40% -> -40%) left the PREVIOUS duty applied
     * in the NEWLY REVERSED direction for the few microseconds between the two
     * register writes - a hard H-bridge reversal with no dead-time, at 20 kHz
     * where the whole PWM period is only 50 us.
     *
     * Dropping the duty first makes the flip happen with the bridge idle. No
     * explicit dead-time delay is added: PWM_SetDuty writes CMPA, which the
     * generator applies from the next PWM cycle, and the DIR GPIO write that
     * follows is only a handful of instructions - the ordering alone removes the
     * reversed-drive window. A timed dead-time would mean busy-waiting in the
     * control path, which is a worse trade for this hardware.
     *
     * Unchanged direction (the overwhelmingly common case: every steady-state
     * update) skips the extra write entirely, so there is no added cost or
     * duty glitch on a normal speed change. */
    if (Motor_Status[motorId].direction != direction)
    {
        (void)Motor_SetSpeed(motorId, 0.0f);       /* idle the bridge first */
    }

    status = Motor_SetDirection(motorId, direction);
    if (status != MOTOR_OK) {
        return status;
    }

    /* Then apply the new magnitude */
    return Motor_SetSpeed(motorId, speedPercent);
}

Motor_StatusType Motor_SetSignedSpeed(Motor_IdType motorId, float32 signedSpeed)
{
    Motor_DirectionType direction;
    float32 absoluteSpeed;
    
    /* Determine direction and absolute speed */
    if (signedSpeed >= 0.0f) {
        direction = MOTOR_DIR_FORWARD;
        absoluteSpeed = signedSpeed;
    } else {
        direction = MOTOR_DIR_REVERSE;
        absoluteSpeed = -signedSpeed;
    }
    
    /* Clamp to valid range */
    absoluteSpeed = Motor_ClampSpeed(absoluteSpeed);
    
    /* Set speed with direction */
    return Motor_SetSpeedWithDirection(motorId, absoluteSpeed, direction);
}

Motor_StatusType Motor_Stop(Motor_IdType motorId)
{
    return Motor_SetSpeed(motorId, 0.0f);
}

/*******************************************************************************
 *                          Dual Motor Control (Robot Movement)                *
 *******************************************************************************/

Motor_StatusType Motor_DriveForward(float32 speedPercent)
{
    Motor_StatusType status;
    
    /* Set both motors forward */
    status = Motor_SetSpeedWithDirection(MOTOR_LEFT, speedPercent, MOTOR_DIR_FORWARD);
    if (status != MOTOR_OK) {
        return status;
    }
    
    status = Motor_SetSpeedWithDirection(MOTOR_RIGHT, speedPercent, MOTOR_DIR_FORWARD);
    return status;
}

Motor_StatusType Motor_DriveReverse(float32 speedPercent)
{
    Motor_StatusType status;
    
    /* Set both motors reverse */
    status = Motor_SetSpeedWithDirection(MOTOR_LEFT, speedPercent, MOTOR_DIR_REVERSE);
    if (status != MOTOR_OK) {
        return status;
    }
    
    status = Motor_SetSpeedWithDirection(MOTOR_RIGHT, speedPercent, MOTOR_DIR_REVERSE);
    return status;
}

Motor_StatusType Motor_SetSpeeds(float32 leftSpeed, float32 rightSpeed)
{
    Motor_StatusType status;
    
    /* Set left motor */
    status = Motor_SetSignedSpeed(MOTOR_LEFT, leftSpeed);
    if (status != MOTOR_OK) {
        return status;
    }
    
    /* Set right motor */
    status = Motor_SetSignedSpeed(MOTOR_RIGHT, rightSpeed);
    return status;
}

Motor_StatusType Motor_TurnLeft(float32 speedPercent, float32 turnRate)
{
    float32 leftSpeed, rightSpeed;
    
    /* Clamp inputs */
    speedPercent = Motor_ClampSpeed(speedPercent);
    turnRate = Motor_ClampSpeed(turnRate);
    
    /* Calculate differential speeds:
     * turnRate = 0   → both same speed (straight)
     * turnRate = 50  → left at 50%, right at 100%
     * turnRate = 100 → left at 0%, right at 100% (spin)
     */
    rightSpeed = speedPercent;
    leftSpeed = speedPercent * (100.0f - turnRate) / 100.0f;
    
    /* Set speeds (both forward) */
    return Motor_SetSpeeds(leftSpeed, rightSpeed);
}

Motor_StatusType Motor_TurnRight(float32 speedPercent, float32 turnRate)
{
    float32 leftSpeed, rightSpeed;
    
    /* Clamp inputs */
    speedPercent = Motor_ClampSpeed(speedPercent);
    turnRate = Motor_ClampSpeed(turnRate);
    
    /* Calculate differential speeds */
    leftSpeed = speedPercent;
    rightSpeed = speedPercent * (100.0f - turnRate) / 100.0f;
    
    /* Set speeds (both forward) */
    return Motor_SetSpeeds(leftSpeed, rightSpeed);
}

Motor_StatusType Motor_Spin(float32 speedPercent, Motor_DirectionType direction)
{
    Motor_StatusType status;
    
    speedPercent = Motor_ClampSpeed(speedPercent);
    
    if (direction == MOTOR_DIR_FORWARD) {
        /* Clockwise spin: Left forward, Right reverse */
        status = Motor_SetSpeedWithDirection(MOTOR_LEFT, speedPercent, MOTOR_DIR_FORWARD);
        if (status != MOTOR_OK) return status;
        
        status = Motor_SetSpeedWithDirection(MOTOR_RIGHT, speedPercent, MOTOR_DIR_REVERSE);
    } else {
        /* Counter-clockwise spin: Left reverse, Right forward */
        status = Motor_SetSpeedWithDirection(MOTOR_LEFT, speedPercent, MOTOR_DIR_REVERSE);
        if (status != MOTOR_OK) return status;
        
        status = Motor_SetSpeedWithDirection(MOTOR_RIGHT, speedPercent, MOTOR_DIR_FORWARD);
    }
    
    return status;
}

Motor_StatusType Motor_StopAll(void)
{
    Motor_StatusType status;
    
    status = Motor_Stop(MOTOR_LEFT);
    if (status != MOTOR_OK) {
        return status;
    }
    
    status = Motor_Stop(MOTOR_RIGHT);
    return status;
}

/*******************************************************************************
 *                          Status Functions                                   *
 *******************************************************************************/

Motor_StateType Motor_GetState(Motor_IdType motorId)
{
    if (!Motor_IsValidId(motorId)) {
        return MOTOR_STATE_UNINIT;
    }
    
    return Motor_Status[motorId].state;
}

Motor_DirectionType Motor_GetDirection(Motor_IdType motorId)
{
    if (!Motor_IsValidId(motorId)) {
        return MOTOR_DIR_FORWARD;
    }
    
    return Motor_Status[motorId].direction;
}

float32 Motor_GetSpeed(Motor_IdType motorId)
{
    if (!Motor_IsValidId(motorId)) {
        return -1.0f;
    }
    
    return Motor_Status[motorId].speedPercent;
}

boolean Motor_IsRunning(Motor_IdType motorId)
{
    if (!Motor_IsValidId(motorId)) {
        return FALSE;
    }
    
    return (Motor_Status[motorId].speedPercent > 0.0f);
}
