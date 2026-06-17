#include <Arduino.h>
#include "udp.h"
#include "mpu6050.h"  
int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, temp_raw;
float ax, ay, az, gx, gy, gz, temp;
float gx_offset, gy_offset, gz_offset;

#define SDA 21
#define SCL 22


float vx = 0.0f, vy = 0.0f, vz = 0.0f;

float dx=0, dy=0, dz=0, roll=0, pitch=0, yaw=0;
unsigned long lastTime;
void setup()
{
    Serial.begin(115200);
    Wire.begin(SDA, SCL); // SDA, SCL
    MPU6050_Init();
    MPU6050_SetAccelRange(ACCEL_RANGE_16G);
    MPU6050_SetGyroRange(GYRO_RANGE_2000DPS);
    MPU6050_ConfigFilter();

	Serial.println("Calibrating MPU6050...");
	MPU6050_Calibrate(&gx_offset, &gy_offset, &gz_offset, (uint8_t) 1000, GYRO_RANGE_2000DPS);
    Serial.println("MPU6050 Ready");


	ConnectWiFi();
	Serial.println("UDP Ready");

	lastTime = millis();
}
void loop()
{
	float gx_new, gy_new, gz_new;
	const float alpha = 0.9f; 
	unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
	if(dt <= 0 || dt > 0.05f) dt = 0.02f;
	MPU6050_ReadAllRaw(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw, &temp_raw);
	MPU6050_ConvertAccelRaw(&ax, &ay, &az, ax_raw, ay_raw, az_raw, ACCEL_RANGE_16G);
	MPU6050_ConvertGyroRawOffset(&gx_new, &gy_new, &gz_new, gx_raw, gy_raw, gz_raw, GYRO_RANGE_2000DPS, gx_offset, gy_offset, gz_offset);
	MPU6050_ConvertTempRaw(&temp, temp_raw);


	gx = alpha * gx + (1.0 - alpha) * gx_new;
	gy = alpha * gy + (1.0 - alpha) * gy_new;
	gz = alpha * gz + (1.0 - alpha) * gz_new;

	float roll_acc  = atan2(ay, az) * 180.0 / PI;
	float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

	float beta = 0.98;

	roll  = beta * (roll  + gx * dt) + (1.0 - beta) * roll_acc;
	pitch = beta * (pitch + gy * dt) + (1.0 - beta) * pitch_acc;
	yaw   += gz * dt;

	float ax_ms2 = ax * 9.81f * 1000.0f;
	float ay_ms2 = ay * 9.81f * 1000.0f;
	float az_ms2 = az * 9.81f * 1000.0f;

	// Chặn nhiễu nhỏ
	if (fabs(ax_ms2) < 500) ax_ms2 = 0;
	if (fabs(ay_ms2) < 500) ay_ms2 = 0;
	if (fabs(az_ms2) < 500) az_ms2 = 0;

	// Cộng vận tốc
	vx += ax_ms2 * dt;
	vy += ay_ms2 * dt;
	vz += az_ms2 * dt;

	// Nếu không có gia tốc thì giảm vận tốc dần
	if (ax_ms2 == 0) vx *= 0.8f;
	if (ay_ms2 == 0) vy *= 0.8f;
	if (az_ms2 == 0) vz *= 0.8f;

	// Giới hạn vận tốc để khỏi bay
	vx = constrain(vx, -300.0f, 300.0f);
	vy = constrain(vy, -300.0f, 300.0f);
	vz = constrain(vz, -300.0f, 300.0f);

	// Tính delta
	dx = vx * dt;
	dy = vy * dt;
	dz = vz * dt;

	// Scale
	dx /= 10.0f;
	dy /= 0;
	// dz /= 10.0f;
	dz = 0;

	String msg =
		String(dx, 3) + "," +
		String(dy, 3) + "," +
		String(dz, 3) + "," +
		String(roll, 3) + "," +
		String(pitch, 3) + "," +
		String(yaw, 3);

	SendUDPData(msg.c_str());
	Serial.println(msg);
	delay(20); 
}