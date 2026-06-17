#include "Wire.h"
#include "mpu6050.h"

void MPU6050_Init()
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0x00); // Set to zero (wakes up the MPU-6050)
    Wire.endTransmission();
}
void MPU6050_SetAccelRange(AccelRange_t range)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_CONFIG);
    Wire.write(range); // Set the accelerometer range
    Wire.endTransmission();
}
void MPU6050_SetGyroRange(GyroRange_t range)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(GYRO_CONFIG);
    Wire.write(range); // Set the gyroscope range
    Wire.endTransmission();
}
void MPU6050_ConfigFilter()
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(CONFIG);
    Wire.write(0x03);
    Wire.endTransmission();
}
void MPU6050_ReadAccelRaw(int16_t* ax, int16_t* ay, int16_t* az)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_XOUT_H);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, 6); // Request 6 bytes for ax, ay, az
    *ax = Wire.read() << 8 | Wire.read();
    *ay = Wire.read() << 8 | Wire.read();
    *az = Wire.read() << 8 | Wire.read();
    Wire.endTransmission();
}
void MPU6050_ReadGyroRaw(int16_t* gx, int16_t* gy, int16_t* gz)
{   
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_XOUT_H + GYRO_OFFS_X); // Gyro data starts at 0x43
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, 6); // Request 6 bytes for gx, gy, gz
    *gx = Wire.read() << 8 | Wire.read();
    *gy = Wire.read() << 8 | Wire.read();
    *gz = Wire.read() << 8 | Wire.read();
    Wire.endTransmission();
}
void MPU6050_ReadTempRaw(int16_t* temp)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_XOUT_H + TEMP_OFFS); // Temp data starts at 0x41
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, 2); // Request 2 bytes for temp
    *temp = Wire.read() << 8 | Wire.read();
    Wire.endTransmission();
}
void MPU6050_ReadAllRaw(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* temp)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_XOUT_H); // Start reading from the first data register
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, 14); // Request 14 bytes for all data
    *ax = Wire.read() << 8 | Wire.read();
    *ay = Wire.read() << 8 | Wire.read();
    *az = Wire.read() << 8 | Wire.read();
    *temp = Wire.read() << 8 | Wire.read();
    *gx = Wire.read() << 8 | Wire.read();
    *gy = Wire.read() << 8 | Wire.read();
    *gz = Wire.read() << 8 | Wire.read();
    Wire.endTransmission();
}
void MPU6050_ConvertAccelRaw(float* ax, float* ay, float* az, int16_t ax_raw, int16_t ay_raw, int16_t az_raw, AccelRange_t range)
{
    float scaleFactor = 0.0;
    switch (range) {
        case ACCEL_RANGE_2G:
            scaleFactor = 16384.0;
            break;
        case ACCEL_RANGE_4G:
            scaleFactor = 8192.0;
            break;
        case ACCEL_RANGE_8G:
            scaleFactor = 4096.0;
            break;
        case ACCEL_RANGE_16G:
            scaleFactor = 2048.0;
            break;
    }
    *ax = ax_raw / scaleFactor;
    *ay = ay_raw / scaleFactor;
    *az = az_raw / scaleFactor;
}
void MPU6050_ConvertGyroRaw(float* gx, float* gy, float* gz, int16_t gx_raw, int16_t gy_raw, int16_t gz_raw, GyroRange_t range)
{
    float scaleFactor = 0.0;
    switch (range) {
        case GYRO_RANGE_250DPS:
            scaleFactor = 131.0;
            break;
        case GYRO_RANGE_500DPS:
            scaleFactor = 65.5;
            break;
        case GYRO_RANGE_1000DPS:
            scaleFactor = 32.8;
            break;
        case GYRO_RANGE_2000DPS:
            scaleFactor = 16.4;
            break;
    }
    *gx = gx_raw / scaleFactor;
    *gy = gy_raw / scaleFactor;
    *gz = gz_raw / scaleFactor;
}
void MPU6050_ConvertGyroRawOffset(float* gx, float* gy, float* gz, int16_t gx_raw, int16_t gy_raw, int16_t gz_raw, GyroRange_t range, float gx_offset, float gy_offset, float gz_offset)
{
    float scaleFactor = 0.0;
    switch (range) {
        case GYRO_RANGE_250DPS:
            scaleFactor = 131.0;
            break;
        case GYRO_RANGE_500DPS:
            scaleFactor = 65.5;
            break;
        case GYRO_RANGE_1000DPS:
            scaleFactor = 32.8;
            break;
        case GYRO_RANGE_2000DPS:
            scaleFactor = 16.4;
            break;
    }
    *gx = (gx_raw - gx_offset) / scaleFactor;
    *gy = (gy_raw - gy_offset) / scaleFactor;
    *gz = (gz_raw - gz_offset) / scaleFactor;
}
void MPU6050_ConvertTempRaw(float* temp, int16_t rawTemp)
{
    *temp = (rawTemp / 340.0) + 36.53;
}


void MPU6050_Calibrate(float* gx_offset, float* gy_offset, float* gz_offset, uint8_t n_samples, GyroRange_t gyroRange)
{
    int16_t gx_raw, gy_raw, gz_raw;
    long sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for(uint8_t i = 0; i < n_samples; i++) {
        MPU6050_ReadGyroRaw(&gx_raw, &gy_raw, &gz_raw);


        sum_gx += gx_raw;
        sum_gy += gy_raw;
        sum_gz += gz_raw;

        delay(2);
    }
    *gx_offset = (float)sum_gx / n_samples;
    *gy_offset = (float)sum_gy / n_samples;
    *gz_offset = (float)sum_gz / n_samples;
}