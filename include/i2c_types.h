#ifndef I2C_TYPES_H_
#define I2C_TYPES_H_

#include "Std_Types.h"

/*******************************************************************************
 *                          I2C Identification                                 *
 *******************************************************************************/

typedef enum {
    I2C_ID_0 = 0,
    I2C_ID_1 = 1,
    I2C_ID_2 = 2,
    I2C_ID_3 = 3,
    I2C_ID_MAX
} I2C_IdType;

/*******************************************************************************
 *                          I2C Configuration Types                            *
 *******************************************************************************/

typedef enum {
    I2C_SPEED_STANDARD = 0, /* 100 kbps */
    I2C_SPEED_FAST = 1      /* 400 kbps */
} I2C_SpeedModeType;

/*******************************************************************************
 *                          I2C Status Types                                   *
 *******************************************************************************/

typedef enum {
    I2C_OK = 0,
    I2C_ERROR_INVALID_ID,
    I2C_ERROR_NOT_INITIALIZED,
    I2C_ERROR_BUS_BUSY,
    I2C_ERROR_ARBITRATION_LOST,
    I2C_ERROR_NO_ACK,
    I2C_ERROR_TIMEOUT,        /* command capped out; auto-abort cleaned the bus
                              * (STOP asserted, BUSBSY cleared) -> simple retry   */
    I2C_ERROR_NULL_PTR,
    I2C_ERROR_BUS_STUCK       /* command capped out AND auto-abort could not clear
                              * BUSBSY (SDA held low) -> HAL must run SCL-toggle
                              * recovery (I2C_RecoverBegin/Step/End)              */
} I2C_StatusType;

/*******************************************************************************
 *                          Runtime Configuration                              *
 *******************************************************************************/

typedef struct {
    I2C_SpeedModeType speedMode;
} I2C_ConfigType;

#endif /* I2C_TYPES_H_ */
