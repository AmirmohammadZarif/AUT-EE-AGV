// Pin definitions for hardware serial ports
const int serialPort1RxPin = 2;    // RX pin for Serial Port 1
const int serialPort1TxPin = 3;    // TX pin for Serial Port 1
const int serialPort2RxPin = 10;   // RX pin for Serial Port 2
const int serialPort2TxPin = 11;   // TX pin for Serial Port 2

void setup() {
  // Initialize Serial Port 1
  Serial.begin(9600);                  // Default hardware serial port
  Serial1.begin(9600);                 // Serial Port 1

  // Initialize Serial Port 2
  Serial2.begin(9600);                 // Serial Port 2

  // Set the baud rate and other settings for Serial Port 1
  Serial1.begin(9600);
  Serial1.setTimeout(100);             // Set timeout value if needed
  Serial1.setParity(UART_PARITY_NONE); // Set parity if needed

  // Set the baud rate and other settings for Serial Port 2
  Serial2.begin(9600);
  Serial2.setTimeout(100);             // Set timeout value if needed
  Serial2.setParity(UART_PARITY_NONE); // Set parity if needed
}

void loop() {
  // Serial Port 1 communication
  if (Serial1.available()) {
    // Read data from Serial Port 1
    char data = Serial1.read();

    // Process the received data
    // ...
  }

  // Serial Port 2 communication
  if (Serial2.available()) {
    // Read data from Serial Port 2
    char data = Serial2.read();

    // Process the received data
    // ...
  }

  // Default hardware serial communication
  if (Serial.available()) {
    // Read data from the default hardware serial port
    char data = Serial.read();

    // Process the received data
    // ...
  }
}