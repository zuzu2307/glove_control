#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define THUMB_PIN 34
#define INDEX_PIN 35
#define MID_PIN 32
#define RING_PIN 33
#define PINKY_PIN 25

#define THUMB_OUT 26
#define INDEX_OUT 27
#define MID_OUT 14
#define RING_OUT 12
#define PINKY_OUT 13

Adafruit_MPU6050 mpu;  // Create MPU6050 object

int fingerValue[] = { 0, 0, 0, 0, 0 };
int minVal[] = { 4095, 4095, 4095, 4095, 4095 };  // initial high values for calibration
int maxVal[] = { 0, 0, 0, 0, 0 };                 // initial low values for calibration
float smoothedValue[] = { 0, 0, 0, 0, 0 };
float alpha = 0.1;  // Smoothing factor
char spliter[] = ",";

bool inCalibrationMode = true;
unsigned long calibrationStartTime;
const int calibrationDuration = 5000;  // 5 seconds for calibration

float pitch = 0, roll = 0, yaw = 0;
unsigned long lastTime = 0;
const float alphaRotation = 0.98;
const float yawScale = 50;

int hapticFeedback = 0;


void setup() {

  for (int i = 0; i < 5; i++) {
    pinMode(getOutPin(i), OUTPUT);
  }

  Serial.begin(115200);
  Wire.begin();        // Initialize I2C
  if (!mpu.begin()) {  // Initialize the MPU6050 sensor
    Serial.println("Couldn't find the MPU6050 sensor!");
    while (1)
      ;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);  // Set accelerometer range
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);       // Set gyroscope range
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    // Set filter bandwidth

  calibrationStartTime = millis();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (inCalibrationMode) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("CALIBRATION_START");
    if (millis() - calibrationStartTime < calibrationDuration) {
      calibrateSensors();
    } else {
      inCalibrationMode = false;
      Serial.println("CALIBRATION_END");
    }
  } else {
    // Normal mode: read and smooth data, then map to normalized range for VR
    digitalWrite(LED_BUILTIN, LOW);
    readAndSendSensorData();
  }
}

void calibrateSensors() {
  for (int i = 0; i < 5; i++) {
    int sensorValue = analogRead(getPin(i));
    if (sensorValue < minVal[i]) minVal[i] = sensorValue;
    if (sensorValue > maxVal[i]) maxVal[i] = sensorValue;
  }
}

float smooth(float input) {
  static float smoothedValue = 0;
  float alpha = 0.1;  // Smoothing factor (0.1 = smooth, 1 = no smoothing)
  smoothedValue = (alpha * input) + ((1 - alpha) * smoothedValue);
  return smoothedValue;
}

void readAndSendSensorData() {
  // Read flex sensor values
  for (int i = 0; i < 5; i++) {
    int rawValue = analogRead(getPin(i));
    int calibratedValue = map(rawValue, minVal[i], maxVal[i], 0, 100);  // Map to 0-100%
    smoothedValue[i] = alpha * calibratedValue + (1 - alpha) * smoothedValue[i];
  }

  // Read MPU6050 sensor values
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Time step
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Accelerometer angles
  float accelPitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float accelRoll = atan2(-a.acceleration.x, a.acceleration.z) * 180 / PI;

  // Gyroscope integration
  float deltaYaw = g.gyro.z * dt * yawScale;
  float deltaPitch = g.gyro.x * dt;
  float deltaRoll = g.gyro.y * dt;

  yaw += deltaYaw;
  pitch += deltaPitch;
  roll += deltaRoll;

  // Apply complementary filter
  pitch = alphaRotation * pitch + (1 - alphaRotation) * accelPitch;
  roll = alphaRotation * roll + (1 - alphaRotation) * accelRoll;

  // Prevent infinite yaw drift
  if (abs(g.gyro.z) < 0.1) {  // If gyro Z is near zero, slow down yaw
    yaw *= 0.98;
  }

  // Keep yaw within -180 to 180 degrees
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;


  // Send data in CSV format
  Serial.print(smoothedValue[0]);
  Serial.print(spliter);
  Serial.print(smoothedValue[1]);
  Serial.print(spliter);
  Serial.print(smoothedValue[2]);
  Serial.print(spliter);
  Serial.print(smoothedValue[3]);
  Serial.print(spliter);
  Serial.print(smoothedValue[4]);
  Serial.print("/");

  // Serial.print(posX);
  // Serial.print(spliter);
  // Serial.print(posY);
  // Serial.print(spliter);
  // Serial.print(posZ);
  // Serial.print("/");

  Serial.print(-pitch);
  Serial.print(",");
  Serial.print(-yaw);
  Serial.print(",");
  Serial.println(roll);

  if (Serial.available() > 0) {
    hapticFeedback = Serial.parseInt();  // Read the integer from Unity
    // Simulate haptic feedback (Replace with actual motor control)
    for (int i = 0; i < 5; i++) {
      analogWrite(getOutPin(i), hapticFeedback);
    }
  }
}

int getPin(int finger) {
  switch (finger) {
    case 0: return THUMB_PIN;
    case 1: return INDEX_PIN;
    case 2: return MID_PIN;
    case 3: return RING_PIN;
    case 4: return PINKY_PIN;
  }
  return -1;
}

int getOutPin(int finger) {
  switch (finger) {
    case 0: return THUMB_OUT;
    case 1: return INDEX_OUT;
    case 2: return MID_OUT;
    case 3: return RING_OUT;
    case 4: return PINKY_OUT;
  }
  return -1;
}
