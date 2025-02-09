#define THUMB_PIN 34
#define INDEX_PIN 35
#define MID_PIN 32
#define RING_PIN 33
#define PINKY_PIN 25

int fingerValue[] = {0, 0, 0, 0, 0};
int minVal[] = {4095, 4095, 4095, 4095, 4095}; // initial high values for calibration
int maxVal[] = {0, 0, 0, 0, 0};                // initial low values for calibration
float smoothedValue[] = {0, 0, 0, 0, 0};
float alpha = 0.1;  // Smoothing factor
char spliter[] = ",";

bool inCalibrationMode = true;
unsigned long calibrationStartTime;
const int calibrationDuration = 5000;  // 5 seconds for calibration

void setup() {
  Serial.begin(115200);
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
    readAndSendFingerData();
  }
}

void calibrateSensors() {
  // Read each flex sensor and update min/max values for calibration
  for (int i = 0; i < 5; i++) {
    int sensorValue = analogRead(getPin(i));
    if (sensorValue < minVal[i]) minVal[i] = sensorValue;
    if (sensorValue > maxVal[i]) maxVal[i] = sensorValue;
  }
}

void readAndSendFingerData() {
  for (int i = 0; i < 5; i++) {
    int rawValue = analogRead(getPin(i));
    int calibratedValue = map(rawValue, minVal[i], maxVal[i], 0, 100);  // Map to 0-100%
    smoothedValue[i] = alpha * calibratedValue + (1 - alpha) * smoothedValue[i];
  }

  // Send formatted data like LucidGloves
  Serial.print(smoothedValue[0]); Serial.print(spliter);
  Serial.print(smoothedValue[1]); Serial.print(spliter);
  Serial.print(smoothedValue[2]); Serial.print(spliter);
  Serial.print(smoothedValue[3]); Serial.print(spliter);
  Serial.println(smoothedValue[4]);
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
