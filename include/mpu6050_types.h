#ifndef MPU6050_TYPES_H_
#define MPU6050_TYPES_H_

#include "Std_Types.h"
#include "i2c_types.h"

/*******************************************************************************
 *                          MPU6050 Status Types                               *
 *******************************************************************************/

typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERROR_I2C_FAILED,
    MPU6050_ERROR_NOT_FOUND,
    MPU6050_ERROR_NULL_PTR,
    MPU6050_ERROR_NOT_INITIALIZED
} MPU6050_StatusType;

/*******************************************************************************
 *                          MPU6050 Data Types                                 *
 *******************************************************************************/

/* Raw Data Structure */
typedef struct {
    sint16 accelX;
    sint16 accelY;
    sint16 accelZ;
    sint16 gyroX;
    sint16 gyroY;
    sint16 gyroZ;
    sint16 temp;
} MPU6050_RawDataType;

/* Floating Point Data Structure */
typedef struct {
    float32 accelX; /* m/s^2 */
    float32 accelY; /* m/s^2 */
    float32 accelZ; /* m/s^2 */
    float32 gyroX;  /* rad/s */
    float32 gyroY;  /* rad/s */
    float32 gyroZ;  /* rad/s */
    float32 temp;   /* Celsius */
} MPU6050_DataType;

/*******************************************************************************
 *                          MPU6050 Configuration Types                        *
 *******************************************************************************/

typedef enum {
    MPU6050_ACCEL_RANGE_2G = 0x00,
    MPU6050_ACCEL_RANGE_4G = 0x08,
    MPU6050_ACCEL_RANGE_8G = 0x10,
    MPU6050_ACCEL_RANGE_16G = 0x18
} MPU6050_AccelRangeType;

typedef enum {
    MPU6050_GYRO_RANGE_250DPS = 0x00,
    MPU6050_GYRO_RANGE_500DPS = 0x08,
    MPU6050_GYRO_RANGE_1000DPS = 0x10,
    MPU6050_GYRO_RANGE_2000DPS = 0x18
} MPU6050_GyroRangeType;

typedef struct {
    I2C_IdType i2cId;
    MPU6050_AccelRangeType accelRange;
    MPU6050_GyroRangeType gyroRange;
} MPU6050_ConfigType;

#endif /* MPU6050_TYPES_H_ */
