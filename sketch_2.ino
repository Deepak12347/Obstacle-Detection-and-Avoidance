#define TRIG_PIN1 2
#define ECHO_PIN1 3
#define TRIG_PIN2 4
#define ECHO_PIN2 5
#define TRIG_PIN3 6
#define ECHO_PIN3 7
#define TRIG_PIN4 8
#define ECHO_PIN4 9

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(TRIG_PIN3, OUTPUT);
  pinMode(ECHO_PIN3, INPUT);
  pinMode(TRIG_PIN4, OUTPUT);
  pinMode(ECHO_PIN4, INPUT);
}

void loop() {
  long duration1, distance1, duration2, distance2, duration3, distance3, duration4, distance4;
  digitalWrite(TRIG_PIN1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN1, LOW);
  duration1 = pulseIn(ECHO_PIN1, HIGH);
  distance1 = (duration1 / 2) / 29.1;

  digitalWrite(TRIG_PIN2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN2, LOW);
  duration2 = pulseIn(ECHO_PIN2, HIGH);
  distance2 = (duration2 / 2) / 29.1;

  digitalWrite(TRIG_PIN3, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN3, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN3, LOW);
  duration3 = pulseIn(ECHO_PIN3, HIGH);
  distance3 = (duration3 / 2) / 29.1;

  digitalWrite(TRIG_PIN4, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN4, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN4, LOW);
  duration4 = pulseIn(ECHO_PIN4, HIGH);
  distance4 = (duration4 / 2) / 29.1;

  if (distance1 < 100) {
    Serial.print("Obstacle Detected - Sensor 1: ");
    Serial.print(distance1);
    Serial.println(" cm");
  }

  if (distance2 < 100) {
    Serial.print("Obstacle Detected - Sensor 2: ");
    Serial.print(distance2);
    Serial.println(" cm");
  }

  if (distance3 < 100) {
    Serial.print("Obstacle Detected - Sensor 3: ");
    Serial.print(distance3);
    Serial.println(" cm");
  }

  if (distance4 < 100) {
    Serial.print("Obstacle Detected - Sensor 4: ");
    Serial.print(distance4);
    Serial.println(" cm");
  }

  if (distance1 >= 100 && distance2 >= 100 && distance3 >= 100 && distance4 >= 100) {
    Serial.println("No Obstacle Detected");
  }

  Serial.println("---------");

  delay(1000);
}
