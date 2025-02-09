#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float pitch = 0, roll = 0, yaw = 0;
unsigned long lastTime = 0;
const float alpha = 0.98;
const float yawScale = 50;

void setup() {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Time step
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Accelerometer angles
  float accelPitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float accelRoll  = atan2(-a.acceleration.x, a.acceleration.z) * 180 / PI;

  // Gyroscope integration
  float deltaYaw = g.gyro.z * dt * yawScale;
  float deltaPitch = g.gyro.x * dt;
  float deltaRoll = g.gyro.y * dt;

  yaw += deltaYaw;
  pitch += deltaPitch;
  roll += deltaRoll;

  // Apply complementary filter
  pitch = alpha * pitch + (1 - alpha) * accelPitch;
  roll  = alpha * roll + (1 - alpha) * accelRoll;

  // Prevent infinite yaw drift
  if (abs(g.gyro.z) < 0.1) {  // If gyro Z is near zero, slow down yaw
    yaw *= 0.98;
  }

  // Keep yaw within -180 to 180 degrees
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;

  // Send to Unity
  Serial.print(-pitch); Serial.print(",");
  Serial.print(-yaw); Serial.print(",");
  Serial.println(roll);
}
