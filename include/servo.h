/******************************************************************************
 *
 * Module: Servo (HAL)
 *
 * File Name: servo.h
 *
 * Description: Steering-servo Hardware Abstraction Layer.
 *
 *  Talks in servo concepts - angle (rad), center, disable - and drives the
 *  hardware ONLY through the Timer-PWM MCAL (timer_pwm.h). This layer contains
 *  ZERO register access, exactly as Motor.c drives PWM.c without ever touching
 *  PWM0_* registers.
 *
 *  Scope (Phase 1, open-loop): this commands the servo. It does NOT read pot
 *  feedback, run a steering control loop, or implement the failsafe-timeout
 *  policy - those are higher layers that will CALL Servo_Center()/Disable().
 *
 ******************************************************************************/

#ifndef SERVO_H_
#define SERVO_H_

#include "Platform_Types.h"
#include "Std_Types.h"      /* Std_ReturnType - actuation now REPORTS (S10-3) */
#include "servo_cfg.h"

/*******************************************************************************
 *                          Function Prototypes                                *
 *******************************************************************************/

/**
 * @brief  Initialize the steering servo: brings up the Timer-PWM MCAL and
 *         self-centers the servo at boot (1.5ms). Call ONCE at boot, after
 *         Port_Init() (the WT0CCP0/PC4 mux) and typically after PWM_Init().
 */
void Servo_Init(void);

/**
 * @brief  Command the steering angle in radians (the ONLY public actuation
 *         verb). Maps the angle onto the pulse range with a CALIBRATED
 *         asymmetric per-side scale (0 rad -> center; +SERVO_MAX_ANGLE_RIGHT_RAD
 *         -> full RIGHT, -SERVO_MAX_ANGLE_LEFT_RAD -> full LEFT), drives the
 *         MCAL, and stores the commanded angle for Servo_GetCurrentAngle().
 * @param  angle_rad: requested steering angle (rad), in the servo native frame
 *         (+angle -> RIGHT / -angle -> LEFT). Out-of-range values are clamped to
 *         the travel endpoints by the mapping + MCAL clamp.
 * @return E_OK     the MCAL accepted the pulse - the servo really was commanded.
 *         E_NOT_OK the MCAL refused it (today: TimerPWM not initialised). NO
 *                  pulse was written, and the caller must NOT record the angle
 *                  as actuated.
 * @note   S10-3: this used to be `void` and swallowed the MCAL status with an
 *         explicit `(void)TimerPWM_SetPulseUs(...)`, so even post-init a refused
 *         write was unknowable to every layer above. The HAL still presents a
 *         simple angle verb; it just no longer LIES about whether it worked.
 *         Std_ReturnType rather than the MCAL's own enum keeps TimerPWM's type
 *         out of the app layer - the caller needs "did it actuate?", not which
 *         register objected.
 */
Std_ReturnType Servo_SetAngleRad(float32 angle_rad);

/**
 * @brief  Return the last COMMANDED steering angle in radians.
 * @note   This is the open-loop command, NOT a measured pot angle. Closed-loop
 *         measured-angle feedback is a later layer; until then "current" means
 *         "last commanded".
 * @return Last commanded angle (rad). Defaults to 0 (center) until first set.
 */
float32 Servo_GetCurrentAngle(void);

/**
 * @brief  Command center (0 rad / 1.5ms). This is the safe KNOWN state for a
 *         command-timeout: the pulse keeps running at center so the servo
 *         holds straight. Contrast Servo_Disable().
 * @return E_OK / E_NOT_OK, same contract as Servo_SetAngleRad() (S10-3).
 */
Std_ReturnType Servo_Center(void);

/**
 * @brief  Disable the servo output entirely (cut the pulse via TimerPWM_Stop).
 *         Use when feedback is untrustworthy (e.g. POT_FAULT) or pre-init,
 *         where holding any commanded position would be unsafe. Contrast
 *         Servo_Center(), which keeps a valid center pulse running.
 */
void Servo_Disable(void);

#endif /* SERVO_H_ */
