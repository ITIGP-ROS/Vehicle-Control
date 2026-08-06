/******************************************************************************
 * 
 * Module: Motor Calibration
 * File: motor_calibration.h
 * 
 * Description: Hardware-specific speed compensation layer
 *              Handles motor-to-motor variations without touching PID
 * 
 * Architecture:
 *   Application → PID → Motor Calibration → Motor Driver → Hardware
 * 
 ******************************************************************************/

#ifndef MOTOR_CALIBRATION_H_
#define MOTOR_CALIBRATION_H_

#include "Platform_Types.h"
#include "Motor.h"

/*******************************************************************************
 *                          Configuration                                      *
 *******************************************************************************/

/* Manual per-motor calibration REMOVED - both factors neutralized to
 * 1.00 (no-op). A fixed multiplier was the wrong mechanism for
 * compensating inter-motor differences; the future closed-loop PID
 * layer will handle this correctly instead. MotorCal_* below are kept
 * so upper layers keep compiling, but they no longer scale anything. */
#define MOTOR_CAL_LEFT_FACTOR       1.00f   /* no-op - manual calibration removed pending closed-loop PID */
#define MOTOR_CAL_RIGHT_FACTOR      1.00f   /* no-op - manual calibration removed pending closed-loop PID */

/*******************************************************************************
 *                          Public Functions                                   *
 *******************************************************************************/

/**
 * @brief  Set motor speed with automatic calibration
 * @param  motorId: MOTOR_LEFT or MOTOR_RIGHT
 * @param  speedPercent: Desired speed (0-100%)
 * @return Motor status
 * 
 * @note   Automatically applies hardware compensation factors
 *         Application code doesn't need to know about calibration
 */
static inline Motor_StatusType MotorCal_SetSpeed(Motor_IdType motorId, float32 speedPercent)
{
    float32 calibratedSpeed;
    
    /* Apply motor-specific calibration */
    switch (motorId) {
        case MOTOR_LEFT:
            calibratedSpeed = speedPercent * MOTOR_CAL_LEFT_FACTOR;
            break;
            
        case MOTOR_RIGHT:
            calibratedSpeed = speedPercent * MOTOR_CAL_RIGHT_FACTOR;
            break;
            
        default:
            calibratedSpeed = speedPercent;
            break;
    }
    
    /* Send calibrated speed to motor driver */
    return Motor_SetSpeed(motorId, calibratedSpeed);
}

/**
 * @brief  Set signed motor speed with calibration
 * @param  motorId: MOTOR_LEFT or MOTOR_RIGHT  
 * @param  signedSpeed: Speed with direction (-100 to +100%)
 * @return Motor status
 */
static inline Motor_StatusType MotorCal_SetSignedSpeed(Motor_IdType motorId, float32 signedSpeed)
{
    float32 calibratedSpeed;
    
    /* Apply motor-specific calibration */
    switch (motorId) {
        case MOTOR_LEFT:
            calibratedSpeed = signedSpeed * MOTOR_CAL_LEFT_FACTOR;
            break;
            
        case MOTOR_RIGHT:
            calibratedSpeed = signedSpeed * MOTOR_CAL_RIGHT_FACTOR;
            break;
            
        default:
            calibratedSpeed = signedSpeed;
            break;
    }
    
    /* Send calibrated speed to motor driver */
    return Motor_SetSignedSpeed(motorId, calibratedSpeed);
}

#endif /* MOTOR_CALIBRATION_H_ */

/******************************************************************************
 * USAGE EXAMPLE:
 ******************************************************************************/

/*
// In distance_control.c or any application:

#include "motor_calibration.h"  // ← Add this header

// BEFORE (direct motor control):
Motor_SetSpeed(MOTOR_LEFT, pidOutput);
Motor_SetSpeed(MOTOR_RIGHT, pidOutput);

// AFTER (calibrated motor control):
MotorCal_SetSpeed(MOTOR_LEFT, pidOutput);   // ← Automatically compensated
MotorCal_SetSpeed(MOTOR_RIGHT, pidOutput);  // ← Automatically compensated

// That's it! The calibration happens transparently.
// Your PID driver remains unchanged.
// Your distance control logic remains unchanged.
// Only the final motor command is adjusted.
*/

/******************************************************************************
 * WHY THIS IS THE CORRECT APPROACH:
 ******************************************************************************/

/*
 * ✅ Separation of Concerns:
 *    - PID driver: Generic control algorithm
 *    - Motor calibration: Hardware-specific compensation
 *    - Application: Business logic
 * 
 * ✅ Single Responsibility:
 *    - Each module does ONE thing well
 *    - Easy to test independently
 *    - Easy to maintain
 * 
 * ✅ Portability:
 *    - PID driver works on ANY system
 *    - Motor calibration specific to YOUR robot
 *    - Change robots? Just update calibration values
 * 
 * ✅ Maintainability:
 *    - All calibration in ONE place
 *    - Easy to tune (change #define values)
 *    - No code changes in multiple files
 * 
 * ✅ Reusability:
 *    - PID driver can be used in other projects
 *    - Motor driver remains generic
 *    - Calibration layer is optional (can bypass if needed)
 */
