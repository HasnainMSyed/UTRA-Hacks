#include <Servo.h>

// Motor Initialization
int motor1pin1 = 6;
int motor1pin2 = 5;
int motor1Enable = 10;   // PWM pin

int motor2pin1 = 4;
int motor2pin2 = 3;
int motor2Enable = 9;   // PWM pin

// Colour Initialization
// TCS3200 Color Detection: Red, Green, Blue, Black
#define S0 8
#define S1 12
#define S2 13
#define S3 A2
#define OUT 11

// Ultrasonic Sensor Initialization
int trigPin = A0;
int echoPin = A1;   

// Servo Initialization
Servo servo;

int closed = 90;
int opened = 180;

// Millis Initialization

// Timing variables for non-blocking logic
unsigned long previousMillisDistance = 0;
unsigned long previousMillisPath = 0;

// Time intervals
const unsigned long distanceInterval = 100; // Check distance every 100ms
const unsigned long pathInterval = 50;      // Follow path every 50ms

void setup() {
  Serial.begin(9600);

  // Motor Setup
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor1Enable, OUTPUT);

  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motor2Enable, OUTPUT);

  // Colour Sensor Setup
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  // Frequency scaling 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Ultrasonic Sensor Setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Servo Setup
  servo.attach(10);
  servo.write(closed); 
  delay(200);
}

void loop() {
  section2();
}

void section2() {
  unsigned long currentMillis = millis();

  // Check distance periodically
  if (currentMillis - previousMillisDistance >= distanceInterval) {
    previousMillisDistance = currentMillis;
    float distance = UltrasonicSensor();
    Serial.println(distance);

    // If an obstacle is detected, avoid it
    if (distance < 150) {
      avoidObstacle();
    }
  }

  // Follow path periodically
  if (currentMillis - previousMillisPath >= pathInterval) {
    previousMillisPath = currentMillis;
    followPath();
  }
}

void followPath() {
  String colour = colourInput(); // Collects data from colour sensor; outputs colour to Serial Monitor
  if (colour != "White" && colour != "Blue") {
    forward();
  }
  else if (colour == "White") {
    findPath();
  }
  else if (colour == "Blue"){
    stop();
  }
}

void findPath() {
  Serial.println("Finding Path");
  stop();
  delay(500); // Initial stop delay

  unsigned long previousMillis = millis();
  String colour = "White";
  int count = 0;

  while (colour == "White") {
    unsigned long currentMillis = millis();

    // Check distance periodically
    if (currentMillis - previousMillisDistance >= distanceInterval) {
      previousMillisDistance = currentMillis;
      float distance = UltrasonicSensor();
      Serial.println(distance);
      if (distance < 150) {
        avoidObstacle();
        return; // Exit findPath() if obstacle is detected
      }
    }

    // Perform pathfinding logic
    if (currentMillis - previousMillis >= 500) { // Replace delay(500)
      previousMillis = currentMillis;

      if (count < 8) {
        Serial.println("Looking left");
        motor1(50);
        motor2(100);
      // } else if (count == 8) {
      //   Serial.println("Returning to center");
      //   motor1(200);
      //   motor2(50);
      } else {
        Serial.println("Looking right");
        motor1(100);
        motor2(50);
      }

      count++;
      colour = colourInput();

      // Reset count if it exceeds the turning sequence
      if (count > 16) {
        count = 0;
      }
    }
  }
}


void avoidObstacle() {
  Serial.println("Avoiding Obstacle");

  unsigned long previousMillis = millis();
  String colour = "White";

  // Veer left to avoid obstacle
  while (millis() - previousMillis < 1000) {
    motor1(150);
    motor2(50);
  }

  previousMillis = millis();
  // Move forward
  while (millis() - previousMillis < 1000) {
    forward();
  }

  previousMillis = millis();
  // Veer right to return to path
  while (millis() - previousMillis < 1000) {
    motor1(50);
    motor2(150);
  }

  // Continue moving forward until red is detected
  while (colour != "Red") {
    forward();
    colour = colourInput();

    // Check distance periodically
    if (millis() - previousMillisDistance >= distanceInterval) {
      previousMillisDistance = millis();
      float distance = UltrasonicSensor();
      Serial.println(distance);
      if (distance < 150) {
        avoidObstacle();
        return;
      }
    }
  }

  // Make robot face path
  previousMillis = millis();
  while (millis() - previousMillis < 1000) {
    Serial.println("Facing path after returning to path");
    motor1(50);
    motor2(0);
  }
}

void backward() {
  motor1(150);
  motor2(150);
}

void forward() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  analogWrite(motor1Enable, 100);
  analogWrite(motor2Enable, 100);
}

void turnRight() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(motor1Enable, 255);
  analogWrite(motor2Enable, 59);
}

void turnLeft() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(motor1Enable, 59);
  analogWrite(motor2Enable, 255);
}

void motor1(int speed) {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(motor1Enable, speed);
}

void motor2(int speed) {
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(motor2Enable, speed);
}

void stop() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);

  analogWrite(motor1Enable, 0);
  analogWrite(motor2Enable, 0);
}

String colourInput() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  int redFreq = pulseIn(OUT, LOW);

  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  int greenFreq = pulseIn(OUT, LOW);

  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  int blueFreq = pulseIn(OUT, LOW);

  Serial.print("R: "); Serial.print(redFreq);
  Serial.print(" G: "); Serial.print(greenFreq);
  Serial.print(" B: "); Serial.print(blueFreq);

  String color = detectColor(redFreq, greenFreq, blueFreq);
  Serial.print(" => Detected: "); Serial.println(color);

  return color;
}

String detectColor(int r, int g, int b) {
  if (r < 50 && g < 50 && b < 50) return "White";
  if (r > 120 && g > 120 && b > 120) return "Black";
  if (r < g && r < b) return "Red";
  if (g < r && g < b) return "Green";
  if (b < r && b < g) return "Blue";

  return "Unknown";
}

void openServo() {
  for (int i = closed; i <= opened; i++) {
    servo.write(i);
    delay(1);
  }
}

void closeServo() {
  for (int i = opened; i >= closed; i--) {
    servo.write(i);
    delay(1);
  }
}

float UltrasonicSensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;

  return distance;
}