#include <SoftwareSerial.h>

// Pin definitions for software serial
const int motorDriverRxPin = 2;     // RX pin for motor driver
const int motorDriverTxPin = 3;     // TX pin for motor driver

// Create a software serial object
SoftwareSerial motorDriverSerial(motorDriverRxPin, motorDriverTxPin);

void setup() {
  // Initialize the USB serial communication
  Serial.begin(9600);

  // Initialize the software serial communication for the motor driver
  motorDriverSerial.begin(9600);
}

void loop() {
  // Motor driver communication
  if (motorDriverSerial.available()) {
    // Read data from the motor driver
    char data = motorDriverSerial.read();
    
  }

  // USB serial communication
  if (Serial.available()) {
    // Read data from the USB serial
    char data = Serial.read();

  }
}