#ifndef MPU6050_H
#define MPU6050_H


#include <Wire.h>

#define MPU6050_ADDR 0x68
#define MPU6050_ACCEL_XOUT_H 0x3B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define CONFIG 0x1A

typedef enum {
    ACCEL_RANGE_2G = 0x00,
    ACCEL_RANGE_4G = 0x08,
    ACCEL_RANGE_8G = 0x10,
    ACCEL_RANGE_16G = 0x18
} AccelRange_t;
typedef enum {
    GYRO_RANGE_250DPS = 0x00,
    GYRO_RANGE_500DPS = 0x08,
    GYRO_RANGE_1000DPS = 0x10,
    GYRO_RANGE_2000DPS = 0x18
} GyroRange_t;

typedef enum {
    ACCEL_OFFS_X = 0,
    ACCEL_OFFS_Y = 2,
    ACCEL_OFFS_Z = 4,
    TEMP_OFFS = 6,
    GYRO_OFFS_X = 8,
    GYRO_OFFS_Y = 10,
    GYRO_OFFS_Z = 12
} RegisterOffsets_t;
void MPU6050_Init();
void MPU6050_SetGyroRange(GyroRange_t range);
void MPU6050_SetAccelRange(AccelRange_t range);
void MPU6050_ConfigFilter();
void MPU6050_ReadAccelRaw(int16_t* ax, int16_t* ay, int16_t* az);
void MPU6050_ReadGyroRaw(int16_t* gx, int16_t* gy, int16_t* gz);
void MPU6050_ReadTempRaw(int16_t* temp);
void MPU6050_ReadAllRaw(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* temp);
void MPU6050_ConvertAccelRaw(float* ax, float* ay, float* az, int16_t ax_raw, int16_t ay_raw, int16_t az_raw, AccelRange_t range);
void MPU6050_ConvertGyroRaw(float* gx, float* gy, float* gz, int16_t gx_raw, int16_t gy_raw, int16_t gz_raw, GyroRange_t range);
void MPU6050_ConvertTempRaw(float* temp, int16_t rawTemp);
void MPU6050_Calibrate(float* gx_offset, float* gy_offset, float* gz_offset, uint8_t n_samples, GyroRange_t gyroRange);
void MPU6050_ConvertGyroRawOffset(float* gx, float* gy, float* gz, int16_t gx_raw, int16_t gy_raw, int16_t gz_raw, GyroRange_t range, float gx_offset, float gy_offset, float gz_offset);
#endif