#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX for communication with Pixhawk

void setup() {
  Serial.begin(57600); // Initialize the serial communication for debugging purposes
  mySerial.begin(57600); // Initialize the serial communication for communication with Pixhawk

  delay(2000); // Wait for the connection to stabilize

  if (mySerial.available()) {
    Serial.println("Connection established with Pixhawk");
  } else {
    Serial.println("Failed to establish connection with Pixhawk");
  }
}

void loop() {
  if (mySerial.available()) {
    char data = mySerial.read();
    Serial.print("Received data from Pixhawk: ");
    Serial.println(data);
  }
}
