#include <EEPROM.h>

int sensorPins[5] = {A0, A1, A2, A3, A4};  // 5 IR sensors
int leftMotorPin = 5, rightMotorPin = 6;   // motor pins

int minVal[5], maxVal[5];   // calibration limits
int calibratedValue[5];     // mapped 0–100

// PID constants (to be tuned)
float Kp = 1.0, Ki = 0.0, Kd = 2.0;
int lastError = 0;
long integral = 0;
int baseSpeed = 150;        // base motor speed

void setup() {
  Serial.begin(9600);
  // Load calibration if available, else perform and save
  if (EEPROM.read(0) == 123) {
    loadCalibration();
    Serial.println("Calibration loaded.");
  } else {
    calibrateSensors();
    saveCalibration();
    Serial.println("Calibration saved.");
  }
}

void loop() {
  // Read and calibrate sensors
  for (int i = 0; i < 5; i++) {
    int raw = analogRead(sensorPins[i]);
    calibratedValue[i] = map(raw, minVal[i], maxVal[i], 0, 100);
    calibratedValue[i] = constrain(calibratedValue[i], 0, 100);
  }

  int error = getError();

  // PID calculation
  integral += error;
  int derivative = error - lastError;
  int correction = Kp * error + Ki * integral + Kd * derivative;

  // Adjust motor speeds
  int leftMotorSpeed  = constrain(baseSpeed + correction, 0, 255);
  int rightMotorSpeed = constrain(baseSpeed - correction, 0, 255);

  analogWrite(leftMotorPin, leftMotorSpeed);
  analogWrite(rightMotorPin, rightMotorSpeed);

  lastError = error;

  // Debugging
  Serial.print("Err:"); Serial.print(error);
  Serial.print(" L:"); Serial.print(leftMotorSpeed);
  Serial.print(" R:"); Serial.println(rightMotorSpeed);

  delay(10);
}

void calibrateSensors() {
  for (int i = 0; i < 5; i++) {
    minVal[i] = 1023; maxVal[i] = 0;
  }
  // Sweep over line manually during this loop
  for (int t = 0; t < 400; t++) {
    for (int i = 0; i < 5; i++) {
      int raw = analogRead(sensorPins[i]);
      if (raw < minVal[i]) minVal[i] = raw;
      if (raw > maxVal[i]) maxVal[i] = raw;
    }
    delay(20);
  }
}

void saveCalibration() {
  EEPROM.write(0, 123);
  for (int i = 0; i < 5; i++) {
    EEPROM.put(1 + i * 4, minVal[i]);
    EEPROM.put(21 + i * 4, maxVal[i]);
  }
}

void loadCalibration() {
  for (int i = 0; i < 5; i++) {
    EEPROM.get(1 + i * 4, minVal[i]);
    EEPROM.get(21 + i * 4, maxVal[i]);
  }
}

int getError() {
  long weightedSum = 0, total = 0;
  for (int i = 0; i < 5; i++) {
    int val = calibratedValue[i];
    weightedSum += (long)val * (i * 1000);
    total += val;
  }
  if (total == 0) return 0;
  long position = weightedSum / total;  // 0–4000
  return position - 2000;               // error around center
}