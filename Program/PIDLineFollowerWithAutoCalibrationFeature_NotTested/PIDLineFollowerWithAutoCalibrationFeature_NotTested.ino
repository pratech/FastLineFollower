#include <EEPROM.h>

// Motor pins
const int motorPin1 = 3; // PWM for Motor 1
const int motorPin2 = 5; // PWM for Motor 1
const int motorPin3 = 6; // PWM for Motor 2
const int motorPin4 = 9; // PWM for Motor 2

// Sensor pins
const int sensorPins[5] = {A0, A1, A2, A3, A4}; // Sensor pins

// PID constants
float Kp = 2.0;
float Ki = 0.0;
float Kd = 1.0;

// Other variables
int threshold[5];
int lastError = 0;
int integral = 0;
const int ledPin = 10;
const int switchPin = 12;
const int dipSwitchPin = 11;

void setup() {
  // Motor pins as output
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  pinMode(ledPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(dipSwitchPin, INPUT);

  // Read the calibration threshold values from EEPROM
  for (int i = 0; i < 5; i++) {
    threshold[i] = EEPROM.read(i);
  }
}

void loop() {
  if (digitalRead(switchPin) == LOW) {
    delay(3000); // Wait for 3 seconds to confirm the button press

    if (digitalRead(switchPin) == LOW) {
      calibrateSensors(); // Enter calibration mode
    }
  }

  int position = readSensors();
  int error = position - 2000; // Center position for a 5-sensor array
  integral += error;
  int derivative = error - lastError;

  int correction = Kp * error + Ki * integral + Kd * derivative;

  int motorSpeed1 = constrain(150 - correction, 0, 255);
  int motorSpeed2 = constrain(150 + correction, 0, 255);

  // Control Motor 1
  if (motorSpeed1 > 0) {
    analogWrite(motorPin1, motorSpeed1); // Set PWM for speed
    digitalWrite(motorPin2, LOW);        // Set direction
  } else {
    digitalWrite(motorPin1, LOW);        // Set direction
    analogWrite(motorPin2, -motorSpeed1); // Set PWM for reverse speed
  }

  // Control Motor 2
  if (motorSpeed2 > 0) {
    analogWrite(motorPin3, motorSpeed2); // Set PWM for speed
    digitalWrite(motorPin4, LOW);        // Set direction
  } else {
    digitalWrite(motorPin3, LOW);        // Set direction
    analogWrite(motorPin4, -motorSpeed2); // Set PWM for reverse speed
  }

  lastError = error;
}

int readSensors() {
  bool whiteOnBlack = digitalRead(dipSwitchPin) == HIGH;

  int sensorValues[5];
  int weightedSum = 0;
  int sum = 0;

  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    bool lineDetected = whiteOnBlack ? (sensorValues[i] > threshold[i]) : (sensorValues[i] < threshold[i]);
    if (lineDetected) {
      weightedSum += i * 1000; // Weighted value for position
      sum += 1000;
    }
  }

  return (sum != 0) ? (weightedSum / sum) : 2000; // Return position or center if no line detected
}

void calibrateSensors() {
  digitalWrite(ledPin, HIGH); // Turn on LED to indicate calibration mode

  unsigned long startTime = millis();
  int minValues[5] = {1023, 1023, 1023, 1023, 1023};
  int maxValues[5] = {0, 0, 0, 0, 0};

  while (millis() - startTime < 5000) { // Calibrate for 5 seconds
    for (int i = 0; i < 5; i++) {
      int sensorValue = analogRead(sensorPins[i]);

      if (sensorValue < minValues[i]) {
        minValues[i] = sensorValue;
      }
      if (sensorValue > maxValues[i]) {
        maxValues[i] = sensorValue;
      }
    }
  }

  // Calculate and store threshold values
  for (int i = 0; i < 5; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    EEPROM.write(i, threshold[i]); // Store threshold in EEPROM
  }

  digitalWrite(ledPin, LOW); // Turn off LED after calibration
}

