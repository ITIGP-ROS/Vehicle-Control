#ifndef PWM_H
#define PWM_H

#include "Platform_Types.h"
#include "pwm_cfg.h"

/*******************************************************************************
 *                          Pin Mapping Configuration                          *
 *******************************************************************************/
/*
 * Hardware Connections (from schematic, confirmed on real hardware):
 * - PB4 = M0PWM2 (Module 0, Generator 1, Output A) -> Left Motor  (PWM_CHANNEL_LEFT)
 * - PB6 = M0PWM0 (Module 0, Generator 0, Output A) -> Right Motor (PWM_CHANNEL_RIGHT)
 */

/*******************************************************************************
 *                          Type Definitions                                   *
 *******************************************************************************/

/* PWM Channel Type - one entry per physically wired motor, named to match
 * the wiring directly. This is the PWM driver's OWN channel identifier and
 * is intentionally not related to Motor.h's Motor_IdType - callers must go
 * through Motor.c's explicit channel mapping rather than relying on the two
 * enums sharing values. */
typedef enum {
    PWM_CHANNEL_LEFT = 0,       /* PB4 - M0PWM2 - Generator 1 */
    PWM_CHANNEL_RIGHT = 1,      /* PB6 - M0PWM0 - Generator 0 */
    PWM_CHANNEL_MAX
} PWM_ChannelType;

/* PWM Status Type */
typedef enum {
    PWM_OK = 0,
    PWM_ERROR_INVALID_CHANNEL,
    PWM_ERROR_NOT_INITIALIZED
} PWM_StatusType;

/* PWM State Type */
typedef enum {
    PWM_STATE_UNINIT = 0,
    PWM_STATE_IDLE,
    PWM_STATE_RUNNING
} PWM_StateType;

/*******************************************************************************
 *                          Function Prototypes                                *
 *******************************************************************************/

/* Initialization Functions */

/**
 * @brief  One-time PWM module initialization. Enables the PWM0 peripheral
 *         clock, waits for it to be ready, configures the clock divider,
 *         then configures and starts BOTH generators (LEFT and RIGHT) at
 *         0% duty (stopped).
 * @note   Call ONCE at boot - unlike the previous per-channel PWM_Init(),
 *         this single call brings up both channels.
 */
void PWM_Init(void);

/* Duty Cycle Control */

/**
 * @brief  Set one channel's PWM duty cycle.
 * @param  channel: PWM_CHANNEL_LEFT or PWM_CHANNEL_RIGHT
 * @param  percent: 0.0 (stopped) to 100.0 (full speed). Out-of-range values
 *         are clamped into [0,100] rather than rejected.
 * @return PWM_OK, PWM_ERROR_INVALID_CHANNEL, or PWM_ERROR_NOT_INITIALIZED
 */
PWM_StatusType PWM_SetDuty(PWM_ChannelType channel, float32 percent);

/* Motor Control Convenience Functions */
PWM_StatusType PWM_SetBothMotors(float32 percent);
PWM_StatusType PWM_SetMotors(float32 leftPercent, float32 rightPercent);
PWM_StatusType PWM_StopMotors(void);

/* State Functions */
PWM_StateType PWM_GetState(PWM_ChannelType channel);

/* Synchronization */
void PWM_TriggerGlobalSync(void);

#endif // PWM_H
