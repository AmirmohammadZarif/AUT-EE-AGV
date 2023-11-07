#include <ArduinoJson.h>

const int BUFFER_SIZE = 256;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
}

void loop() {
  if (Serial.available()) {
    // Read the incoming JSON string
    char jsonBuffer[BUFFER_SIZE];
    int bytesRead = Serial.readBytesUntil('\n', jsonBuffer, BUFFER_SIZE - 1);
    jsonBuffer[bytesRead] = '\0';

    // Parse the JSON string
    StaticJsonDocument<BUFFER_SIZE> doc;
    DeserializationError error = deserializeJson(doc, jsonBuffer);

    // Check for parsing errors
    if (error) {
      Serial.print("Parsing failed: ");
      Serial.println(error.c_str());
    } else {
      // Access the JSON data
      const char* name = doc["name"];
      int age = doc["age"];

      // Print the JSON data
      Serial.print("Name: ");
      Serial.println(name);
      Serial.print("Age: ");
      Serial.println(age);
    }
  }
}