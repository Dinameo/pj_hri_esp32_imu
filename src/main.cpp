#include <Arduino.h>
#include "udp.h"
#include "mpu6050.h"

int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, temp_raw;
float ax, ay, az, gx, gy, gz, temp;
float gx_offset, gy_offset, gz_offset;

#define SDA 27
#define SCL 14

// =====================
// Vị trí gốc end-effector trong Unity
// =====================
const float CENTER_X = 0.0f;
const float CENTER_Y = 0.0f;
const float CENTER_Z = 0.0f;

float x = CENTER_X;
float y = CENTER_Y;
float z = CENTER_Z;

// =====================
// Góc IMU
// =====================
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

float roll0 = 0.0f;
float pitch0 = 0.0f;
float yaw0 = 0.0f;

bool originSet = false;

// Đợi filter ổn định rồi mới lấy góc gốc
int stableCount = 0;
const int STABLE_SAMPLES = 50;

unsigned long lastTime;

// =====================
// Thông số air mouse
// =====================

// Yaw trái/phải -> X robot
const float SCALE_X = 5.0f;   // mm / độ

// Roll lên/xuống -> Z robot
const float SCALE_Z = 5.0f;   // mm / độ

// Nếu sau này muốn dùng pitch cho Y robot thì tăng SCALE_Y
const float SCALE_Y = 0.0f;

// Đảo chiều nếu chạy ngược
// Đảo chiều nếu chạy ngược
const float X_SIGN = -1.0f;
const float Y_SIGN = 1.0f;
const float Z_SIGN = 1.0f;

// Giới hạn vùng làm việc
const float X_MIN = -1300.0f;
const float X_MAX = 1300.0f;

const float Y_MIN = -1300.0f;
const float Y_MAX = 1300.0f;

const float Z_MIN = -1500.0f;
const float Z_MAX = 1500.0f;

// Lọc gyro
const float GYRO_ALPHA = 0.9f;

// Complementary filter
const float BETA = 0.98f;

// Deadzone góc
const float ANGLE_DEADZONE = 1.0f;

// =====================
// Hàm phụ
// =====================
float ApplyDeadzone(float value, float threshold)
{
    if (fabs(value) < threshold)
    {
        return 0.0f;
    }

    return value;
}

float NormalizeAngle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

void ResetOrigin()
{
    roll0 = roll;
    pitch0 = pitch;
    yaw0 = yaw;

    originSet = true;

    x = CENTER_X;
    y = CENTER_Y;
    z = CENTER_Z;

    Serial.println("Origin reset!");
    Serial.print("roll0 = "); Serial.println(roll0, 2);
    Serial.print("pitch0 = "); Serial.println(pitch0, 2);
    Serial.print("yaw0 = "); Serial.println(yaw0, 2);
}

void setup()
{
    Serial.begin(115200);

    Wire.begin(SDA, SCL);

    MPU6050_Init();
    MPU6050_SetAccelRange(ACCEL_RANGE_16G);
    MPU6050_SetGyroRange(GYRO_RANGE_2000DPS);
    MPU6050_ConfigFilter();

    Serial.println("Calibrating MPU6050 gyro...");
    MPU6050_Calibrate(&gx_offset, &gy_offset, &gz_offset,
                      (uint8_t)1000,
                      GYRO_RANGE_2000DPS);

    Serial.println("MPU6050 Ready");

    ConnectWiFi();
    Serial.println("UDP Ready");

    lastTime = millis();
}

void loop()
{
    float gx_new, gy_new, gz_new;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    if (dt <= 0.0f || dt > 0.05f)
    {
        dt = 0.02f;
    }

    // =====================
    // Đọc MPU6050
    // =====================
    MPU6050_ReadAllRaw(&ax_raw, &ay_raw, &az_raw,
                       &gx_raw, &gy_raw, &gz_raw,
                       &temp_raw);

    MPU6050_ConvertAccelRaw(&ax, &ay, &az,
                            ax_raw, ay_raw, az_raw,
                            ACCEL_RANGE_16G);

    MPU6050_ConvertGyroRawOffset(&gx_new, &gy_new, &gz_new,
                                  gx_raw, gy_raw, gz_raw,
                                  GYRO_RANGE_2000DPS,
                                  gx_offset, gy_offset, gz_offset);

    MPU6050_ConvertTempRaw(&temp, temp_raw);

    // =====================
    // Lọc gyro
    // =====================
    gx = GYRO_ALPHA * gx + (1.0f - GYRO_ALPHA) * gx_new;
    gy = GYRO_ALPHA * gy + (1.0f - GYRO_ALPHA) * gy_new;
    gz = GYRO_ALPHA * gz + (1.0f - GYRO_ALPHA) * gz_new;

    // =====================
    // Góc từ accel
    // Với hướng IMU:
    // X sang phải, Y vào màn hình, Z xuống
    // roll quanh X: dùng để điều khiển Z robot
    // yaw quanh Z: dùng để điều khiển X robot
    // =====================
    float roll_acc  = atan2(ay, az) * 180.0f / PI;
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;

    // =====================
    // Complementary filter
    // =====================
    roll  = BETA * (roll  + gx * dt) + (1.0f - BETA) * roll_acc;
    pitch = BETA * (pitch + gy * dt) + (1.0f - BETA) * pitch_acc;

    // MPU6050 không có từ kế nên yaw sẽ trôi từ từ
    yaw += gz * dt;
    yaw = NormalizeAngle(yaw);

    // =====================
    // Đợi filter ổn định rồi lấy góc hiện tại làm tâm
    // =====================
    if (!originSet)
    {
        stableCount++;

        if (stableCount >= STABLE_SAMPLES)
        {
            ResetOrigin();
        }
    }

    // Có thể gửi ký tự 'r' trên Serial Monitor để reset tâm
    if (Serial.available())
    {
        char c = Serial.read();

        if (c == 'r' || c == 'R')
        {
            ResetOrigin();
        }
    }

    // =====================
    // Tính độ lệch góc so với tâm
    // =====================
    float yaw_delta = NormalizeAngle(yaw - yaw0);
    float roll_delta = NormalizeAngle(roll - roll0);
    float pitch_delta = NormalizeAngle(pitch - pitch0);

    yaw_delta = ApplyDeadzone(yaw_delta, ANGLE_DEADZONE);
    roll_delta = ApplyDeadzone(roll_delta, ANGLE_DEADZONE);
    pitch_delta = ApplyDeadzone(pitch_delta, ANGLE_DEADZONE);

    // =====================
    // Air mouse mapping đúng theo hướng đặt IMU
    // =====================

    // Xoay trái/phải quanh Z -> robot đi X
    x = CENTER_X + X_SIGN * yaw_delta * SCALE_X;

    // Ngẩng/chúi quanh X -> robot đi Z
    z = CENTER_Z + Z_SIGN * roll_delta * SCALE_Z;

    // Tạm khóa Y
    // Nếu muốn dùng pitch để điều khiển Y thì sửa SCALE_Y > 0
    y = CENTER_Y + Y_SIGN * pitch_delta * SCALE_Y;

    // Giới hạn vùng làm việc
    x = constrain(x, X_MIN, X_MAX);
    y = constrain(y, Y_MIN, Y_MAX);
    z = constrain(z, Z_MIN, Z_MAX);

    // =====================
    // Khóa góc robot
    // Chỉ điều khiển vị trí end-effector
    // =====================
    float sendRoll = 0.0f;
    float sendPitch = 0.0f;
    float sendYaw = 0.0f;

    // =====================
    // Gửi sang Unity
    // Giữ thứ tự của bạn: y, x, z, roll, pitch, yaw
    // =====================
    String msg =
        String(x, 3) + "," +
        String(y, 3) + "," +
        String(z, 3) + "," +
        String(sendRoll, 3) + "," +
        String(sendPitch, 3) + "," +
        String(sendYaw, 3);

    SendUDPData(msg.c_str());

    // =====================
    // Debug
    // =====================
    Serial.print("roll: "); Serial.print(roll, 2);
    Serial.print(" pitch: "); Serial.print(pitch, 2);
    Serial.print(" yaw: "); Serial.print(yaw, 2);

    Serial.print(" | dYaw: "); Serial.print(yaw_delta, 2);
    Serial.print(" dRoll: "); Serial.print(roll_delta, 2);
    Serial.print(" dPitch: "); Serial.print(pitch_delta, 2);

    Serial.print(" | X: "); Serial.print(x, 2);
    Serial.print(" Y: "); Serial.print(y, 2);
    Serial.print(" Z: "); Serial.print(z, 2);

    Serial.print(" | UDP: ");
    Serial.println(msg);

    delay(20);
}