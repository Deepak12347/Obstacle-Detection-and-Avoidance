#include <SoftwareSerial.h>
#include "C:\Users\Deepak Sharma\Documents\Arduino\libraries\PixhawkArduinoMAVLink\src\common\mavlink.h"
// Pin definitions for the ultrasonic sensors
#define FRONT_TRIG_PIN 2
#define FRONT_ECHO_PIN 3
#define BACK_TRIG_PIN 4
#define BACK_ECHO_PIN 5
#define LEFT_TRIG_PIN 6
#define LEFT_ECHO_PIN 7
#define RIGHT_TRIG_PIN 8
#define RIGHT_ECHO_PIN 9

SoftwareSerial mySerial(10, 11); // RX, TX for communication with pixhawk

const int MAX_DISTANCE = 70; // Maximum obstacle detection distance in cm

int frontDistance = 0; // Variable to store the distance measured by the front sensor
int backDistance = 0; // Variable to store the distance measured by the back sensor
int leftDistance = 0; // Variable to store the distance measured by the left sensor
int rightDistance = 0; // Variable to store the distance measured by the right sensor

int pitch = 1500; // Initial pitch value for the quadcopter
int roll = 1500; // Initial roll value for the quadcopter

unsigned long lastHeartbeatTime = 0;
const unsigned long heartbeatInterval = 1000; // Heartbeat interval in milliseconds

void setup() {
  Serial.begin(57600); // Initialize the serial communication for debugging purposes
  mySerial.begin(57600); // Initialize the serial communication for communication with Pixhawk

  pinMode(FRONT_TRIG_PIN, OUTPUT); // Set the front sensor trigger pin as OUTPUT
  pinMode(FRONT_ECHO_PIN, INPUT); // Set the front sensor echo pin as INPUT
  pinMode(BACK_TRIG_PIN, OUTPUT); // Set the back sensor trigger pin as OUTPUT
  pinMode(BACK_ECHO_PIN, INPUT); // Set the back sensor echo pin as INPUT
  pinMode(LEFT_TRIG_PIN, OUTPUT); // Set the left sensor trigger pin as OUTPUT
  pinMode(LEFT_ECHO_PIN, INPUT); // Set the left sensor echo pin as INPUT
  pinMode(RIGHT_TRIG_PIN, OUTPUT); // Set the right sensor trigger pin as OUTPUT
  pinMode(RIGHT_ECHO_PIN, INPUT); // Set the right sensor echo pin as INPUT
}

void loop() {
  // Check if it's time to send a heartbeat
  if (millis() - lastHeartbeatTime >= heartbeatInterval) {
    sendHeartbeat(); // Send MAVLink heartbeat message
    lastHeartbeatTime = millis(); // Update the last heartbeat time
  }

  // Read distances from the sensors
  frontDistance = measureDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
  backDistance = measureDistance(BACK_TRIG_PIN, BACK_ECHO_PIN);
  leftDistance = measureDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  rightDistance = measureDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);

  // Print obstacle distances to the serial monitor
  Serial.println("Obstacle Distances:");
  Serial.print("Front: ");
  Serial.print(frontDistance);
  Serial.print(" cm\t");
  Serial.print("Back: ");
  Serial.print(backDistance);
  Serial.print(" cm\t");
  Serial.print("Left: ");
  Serial.print(leftDistance);
  Serial.print(" cm\t");
  Serial.print("Right: ");
  Serial.print(rightDistance);
  Serial.println(" cm");

  // Perform obstacle avoidance based on sensor readings
  if (frontDistance > 0 && frontDistance < MAX_DISTANCE) {
    // Obstacle detected in front, adjust pitch for avoidance
    pitch = 1500 - 30 - ((MAX_DISTANCE - frontDistance) * 6);
  } else {
    // No obstacle detected in front, maintain neutral pitch
    pitch = 1500;
  }

  if (backDistance > 0 && backDistance < MAX_DISTANCE) {
    // Obstacle detected at the back, adjust pitch for avoidance
    pitch = 1500 + 30 + ((MAX_DISTANCE - backDistance) * 6);
  }

  if (leftDistance > 0 && leftDistance < MAX_DISTANCE) {
    // Obstacle detected on the left, adjust roll for avoidance
    roll = 1500 - 30 - ((MAX_DISTANCE - leftDistance) * 6);
  } else {
    // No obstacle detected on the left, maintain neutral roll
    roll = 1500;
  }

  if (rightDistance > 0 && rightDistance < MAX_DISTANCE) {
    // Obstacle detected on the right, adjust roll for avoidance
    roll = 1500 + 30 + ((MAX_DISTANCE - rightDistance) * 6);
  }

  // Print pitch and roll values to the serial monitor
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("\tRoll: ");
  Serial.println(roll);

  // Send obstacle distances to Pixhawk
  sendObstacleDistances(frontDistance, backDistance, leftDistance, rightDistance);

  // Send RC override commands to control the quadcopter's pitch and roll
  sendRCOverrideCommands(pitch, roll);

  // Print informative messages to the serial monitor
  Serial.println("Sending MAVLink heartbeat...");
  Serial.println("Sending obstacle distances to Pixhawk...");
  Serial.println("Sending RC override commands...");

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

// Send obstacle distances to Pixhawk
void sendObstacleDistances(int frontDistance, int backDistance, int leftDistance, int rightDistance) {
  mavlink_message_t msg;

  // Send front obstacle distance
  mavlink_msg_obstacle_distance_pack(1, 200, &msg, 0, 0, NULL, 0, frontDistance, 0);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);

  // Send back obstacle distance
  mavlink_msg_obstacle_distance_pack(1, 200, &msg, 0, 180, NULL, 0, backDistance, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);

  // Send left obstacle distance
  mavlink_msg_obstacle_distance_pack(1, 200, &msg, 90, 0, NULL, 0, leftDistance, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);

  // Send right obstacle distance
  mavlink_msg_obstacle_distance_pack(1, 200, &msg, 270, 0, NULL, 0, rightDistance, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
}

// Send RC override commands to control the quadcopter's pitch and roll
void sendRCOverrideCommands(int pitch, int roll) {
  mavlink_message_t msg;

  // Send RC override command for pitch
  mavlink_msg_rc_channels_override_pack(1, 200, &msg, 1, 0, 0, pitch, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);

  // Send RC override command for roll
  mavlink_msg_rc_channels_override_pack(1, 200, &msg, 1, 0, 0, 0, roll, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
}
