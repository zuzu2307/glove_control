#include <Wire.h>
#include <Adafruit_MPU6050.h>

#define THUMB_PIN 34
#define INDEX_PIN 35
#define MID_PIN 32
#define RING_PIN 33
#define PINKY_PIN 25

Adafruit_MPU6050 mpu;  // Create MPU6050 object

int fingerValue[] = {0, 0, 0, 0, 0};
int minVal[] = {4095, 4095, 4095, 4095, 4095}; // initial high values for calibration
int maxVal[] = {0, 0, 0, 0, 0};                // initial low values for calibration
float smoothedValue[] = {0, 0, 0, 0, 0};
float alpha = 0.1;  // Smoothing factor
char spliter[] = ",";

bool inCalibrationMode = true;
unsigned long calibrationStartTime;
const int calibrationDuration = 5000;  // 5 seconds for calibration

float prevAccelX = 0, prevAccelY = 0, prevAccelZ = 0;
float deltaX = 0, deltaY = 0, deltaZ = 0;
const float GRAVITY = 9.8;

// Rotation angles (X, Y, Z in degrees)
float prevAngleX = 0, prevAngleY = 0, prevAngleZ = 0;
float deltaAngleX = 0, deltaAngleY = 0, deltaAngleZ = 0;
float angleX = 0, angleY = 0, angleZ = 0;
const float DEG_PER_S = 180.0 / 3.14159;  // Conversion from radians to degrees
const float alphaR = 0.98;  // Complementary filter coefficient (close to 1)

unsigned long prevMillis = 0;


void setup() {
  Serial.begin(115200);
  Wire.begin();  // Initialize I2C
  if (!mpu.begin()) {  // Initialize the MPU6050 sensor
    Serial.println("Couldn't find the MPU6050 sensor!");
    while (1);
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
  float alpha = 0.1; // Smoothing factor (0.1 = smooth, 1 = no smoothing)
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

  // Store the current accelerometer values
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;

  // Remove the gravitational component (gravity value) from Z-axis
  // accelZ -= GRAVITY;

  // Convert raw accelerometer data to G-force
   // Calculate the delta (change) in accelerometer readings
  deltaX = accelX - prevAccelX;
  deltaY = accelY - prevAccelY;
  deltaZ = accelZ - prevAccelZ;

  // Smooth the delta values to avoid abrupt changes
  deltaX = smooth(deltaX);
  deltaY = smooth(deltaY);
  deltaZ = smooth(deltaZ);

  // Send the delta values for controlling virtual object movement
 

  // Update the previous accelerometer values for next loop
  prevAccelX = accelX;
  prevAccelY = accelY;
  prevAccelZ = accelZ;

  // Gyroscope data (angular velocity in degrees per second)
  float gyroX = g.gyro.x * DEG_PER_S;  // Convert from radians/s to degrees/s
  float gyroY = g.gyro.y * DEG_PER_S;
  float gyroZ = g.gyro.z * DEG_PER_S;

  // Calculate angles from accelerometer (using gravity to estimate orientation)
  float accelAngleX = atan2(accelY, accelZ) * DEG_PER_S;  // Angle around X-axis
  float accelAngleY = atan2(accelX, accelZ) * DEG_PER_S;  // Angle around Y-axis
  float accelAngleZ = atan2(accelY, accelX) * DEG_PER_S;  // Angle around Z-axis 

 // Apply complementary filter for rotation angles
  angleX = alphaR * (angleX + gyroX * (millis() - prevMillis) / 1000.0) + (1 - alphaR) * accelAngleX;
  angleY = alphaR * (angleY + gyroY * (millis() - prevMillis) / 1000.0) + (1 - alphaR) * accelAngleY;
  angleZ = alphaR * (angleZ + gyroZ * (millis() - prevMillis) / 1000.0) + (1 - alphaR) * accelAngleZ;
  
  deltaAngleX = angleX - prevAngleX;
  deltaAngleY = angleY - prevAngleY;
  deltaAngleZ = angleZ - prevAngleZ;

  deltaAngleX = smooth(deltaAngleX);  
  deltaAngleX = smooth(deltaAngleX);  
  deltaAngleX = smooth(deltaAngleX);

  prevAngleX = angleX;
  prevAngleY = angleY;
  prevAngleZ = angleZ;

  // Send data in CSV format
  Serial.print(smoothedValue[0]); Serial.print(spliter);
  Serial.print(smoothedValue[1]); Serial.print(spliter);
  Serial.print(smoothedValue[2]); Serial.print(spliter);
  Serial.print(smoothedValue[3]); Serial.print(spliter);
  Serial.print(smoothedValue[4]); Serial.print("/");
  
   Serial.print(deltaX); Serial.print(spliter);
  Serial.print(deltaY); Serial.print(spliter);
  Serial.print(deltaZ); Serial.print("/");
  Serial.print(angleX); Serial.print(spliter);
  Serial.print(angleY); Serial.print(spliter);
  Serial.println(angleZ);

  prevMillis = millis();
  delay(100); 
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
