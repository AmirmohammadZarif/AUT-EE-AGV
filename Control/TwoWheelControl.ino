// Pin definitions for line sensors
const int leftSensorPin = A0;   // Analog pin for left sensor
const int rightSensorPin = A1;  // Analog pin for right sensor

// Pin definitions for motor control
const int leftMotorPWM = 3;    // PWM pin for left motor speed control
const int leftMotorDir = 4;    // Direction pin for left motor
const int rightMotorPWM = 5;   // PWM pin for right motor speed control
const int rightMotorDir = 6;   // Direction pin for right motor

// Constants for motor speed
const int baseSpeed = 150;     // Base speed for both motors
const int maxSpeedDiff = 100;  // Maximum speed difference between the motors

void setup() {
  // Initialize line sensor pins as inputs
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  // Initialize motor control pins as outputs
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDir, OUTPUT);
}

void loop() {
  // Read sensor values
  int leftSensorValue = analogRead(leftSensorPin);
  int rightSensorValue = analogRead(rightSensorPin);

  // Calculate the error
  int error = leftSensorValue - rightSensorValue;

  // Calculate motor speeds based on the error
  int leftMotorSpeed = baseSpeed + error;
  int rightMotorSpeed = baseSpeed - error;

  // Limit the motor speeds to avoid excessive differences
  leftMotorSpeed = constrain(leftMotorSpeed, 0, baseSpeed + maxSpeedDiff);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, baseSpeed + maxSpeedDiff);

  // Set motor directions based on the error sign
  bool leftMotorForward = (error >= 0);
  bool rightMotorForward = (error <= 0);

  // Control the left motor
  analogWrite(leftMotorPWM, leftMotorSpeed);
  digitalWrite(leftMotorDir, leftMotorForward ? HIGH : LOW);

  // Control the right motor
  analogWrite(rightMotorPWM, rightMotorSpeed);
  digitalWrite(rightMotorDir, rightMotorForward ? HIGH : LOW);
}