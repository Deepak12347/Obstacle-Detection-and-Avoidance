#include <SoftwareSerial.h>
#include "C:\Users\Deepak Sharma\Documents\Arduino\libraries\PixhawkArduinoMAVLink\src\common\mavlink.h"

#define RIGHT_TRIG_PIN 8
#define RIGHT_ECHO_PIN 9

SoftwareSerial mySerial(10, 11); // RX, TX

const int MAX_DISTANCE = 70; // Maximum obstacle detection distance in cm

int rightDistance = 0; // Variable to store the distance measured by the right sensor
int roll = 1500; // Initial roll value for the quadcopter

unsigned long lastHeartbeatTime = 0;
const unsigned long heartbeatInterval = 1000; // Heartbeat interval in milliseconds

void setup() {
  Serial.begin(57600); // Initialize the serial communication for debugging purposes
  mySerial.begin(57600); // Initialize the serial communication for communication with Pixhawk

  pinMode(RIGHT_TRIG_PIN, OUTPUT); // Set the right sensor trigger pin as OUTPUT
  pinMode(RIGHT_ECHO_PIN, INPUT); // Set the right sensor echo pin as INPUT
}

void loop() {
  // Check if it's time to send a heartbeat
  if (millis() - lastHeartbeatTime >= heartbeatInterval) {
    sendHeartbeat(); // Send MAVLink heartbeat message
    lastHeartbeatTime = millis(); // Update the last heartbeat time
  }

  // Read distance from the right sensor
  rightDistance = measureDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);

  // Print obstacle distance to the serial monitor
  Serial.print("Right Distance: ");
  Serial.print(rightDistance);
  Serial.println(" cm");

  // Perform obstacle avoidance based on sensor reading
  if (rightDistance > 0 && rightDistance < MAX_DISTANCE) {
    // Obstacle detected on the right, adjust roll for avoidance
    roll = 1500 + 30 + ((MAX_DISTANCE - rightDistance) * 6);
  } else {
    // No obstacle detected, maintain neutral roll
    roll = 1500;
  }

  // Print roll value to the serial monitor
  Serial.print("Roll: ");
  Serial.println(roll);

  // Send obstacle distance to Pixhawk
  sendObstacleDistance(rightDistance);

  // Send RC override command to control the quadcopter's roll
  sendRCOverrideCommand(roll);

  // Print informative messages to the serial monitor
  Serial.println("Sending MAVLink heartbeat...");
  Serial.println("Sending obstacle distance to Pixhawk...");
  Serial.println("Sending RC override command...");

  delay(100); // Add a small delay for stability
}

// Measure the distance using an ultrasonic sensor
int measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH);
  return (duration / 2) / 29.1; // Convert the duration to distance in cm
}

// Send a MAVLink heartbeat message
void sendHeartbeat() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 0, 0);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len); // Send the heartbeat message via serial communication
}

// Send obstacle distance to Pixhawk
void sendObstacleDistance(int distance) {
  mavlink_message_t msg;
  mavlink_msg_obstacle_distance_pack(1, 200, &msg, 0, 0, NULL, 0, distance, 0); // Provide the required arguments
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len); // Send the obstacle distance message via serial communication
}

// Send an RC_CHANNELS_OVERRIDE command to control the quadcopter's roll
void sendRCOverrideCommand(int roll) {
  mavlink_message_t msg;
  mavlink_msg_rc_channels_override_pack(1, 200, &msg, 1, 0, 0, 0, roll, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); // Provide the required arguments
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len); // Send the RC override command via serial communication
}
