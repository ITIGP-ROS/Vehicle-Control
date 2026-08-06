#ifndef MPU6050_H_
#define MPU6050_H_

#include "mpu6050_types.h"

/*******************************************************************************
 *                          Function Prototypes                                *
 *******************************************************************************/

/**
 * @brief Initialize the MPU6050 sensor
 * @param config: Pointer to configuration structure
 * @return MPU6050_OK on success
 */
MPU6050_StatusType MPU6050_Init(const MPU6050_ConfigType *config);

/**
 * @brief  Raw I2C status behind the most recent MPU6050_ERROR_I2C_FAILED.
 * @details The HAL deliberately keeps a coarse status enum, so every transport
 *          failure surfaces as MPU6050_ERROR_I2C_FAILED. That hides the one bit
 *          that actually helps when bringing a board up: whether nothing ACKed
 *          (I2C_ERROR_NO_ACK -> wiring, address or power) or the bus itself
 *          misbehaved (I2C_ERROR_TIMEOUT / BUS_STUCK -> pull-ups, timing, SDA
 *          held low). This getter exposes that cause without widening the enum.
 * @return  The cached I2C_StatusType; I2C_OK if no transport error has occurred.
 * @note    Sticky: only updated on failure, never cleared on success. Read it
 *          immediately after a call returns MPU6050_ERROR_I2C_FAILED.
 */
I2C_StatusType MPU6050_GetLastI2cError(void);

/**
 * @brief  The WHO_AM_I byte the device returned during the most recent Init.
 * @details Makes MPU6050_ERROR_NOT_FOUND actionable: on its own that status
 *          cannot distinguish "nothing sensible on the bus" from "a working but
 *          unlisted clone answered". The accepted-id set lives in
 *          mpu6050_cfg.h (MPU6050_WHO_AM_I_ACCEPTED_IDS); if the value reported
 *          here is a known-compatible clone id, add it there.
 * @return  The raw byte read from WHO_AM_I; 0x00 if Init never got that far
 *          (i.e. the read itself failed - check MPU6050_GetLastI2cError()).
 */
uint8 MPU6050_GetLastWhoAmI(void);

/**
 * @brief De-initialize the MPU6050 sensor
 */
MPU6050_StatusType MPU6050_DeInit(void);

/**
 * @brief Read raw data from the MPU6050 sensor
 * @param data: Pointer to data structure to store results
 */
MPU6050_StatusType MPU6050_ReadRaw(MPU6050_RawDataType *data);

/**
 * @brief Read converted floating point data from the MPU6050 sensor
 * @param data: Pointer to data structure to store results
 */
MPU6050_StatusType MPU6050_ReadData(MPU6050_DataType *data);

#endif /* MPU6050_H_ */
