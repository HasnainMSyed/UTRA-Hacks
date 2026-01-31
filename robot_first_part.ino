#include <Servo.h>

//////////////////// PIN DEFINITIONS ////////////////////

// Motors
const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;
const int ENB = 9;
const int IN3 = 10;
const int IN4 = 11;

// Ultrasonic
const int TRIG = 12;
const int ECHO = 13;

// IR Sensors
const int IR_LEFT  = A0;
const int IR_RIGHT = A1;

// Color Sensor
const int S2 = A2;
const int S3 = A3;
const int COLOR_OUT = A4;

// Servos
Servo claw;
Servo arm;
const int CLAW_PIN = 3;
const int ARM_PIN  = 4;

//////////////////// SERVO POSITIONS ////////////////////

int clawOpen  = 20;
int clawClose = 80;

int armDown = 120;
int armUp   = 70;

//////////////////// MOTOR FUNCTIONS ////////////////////

void moveForward(int speedVal) {
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  analogWrite(ENA, 80);
  analogWrite(ENB, 120);
}

void turnRight() {
  analogWrite(ENA, 120);
  analogWrite(ENB, 80);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

//////////////////// ULTRASONIC ////////////////////

long getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000);
  return duration * 0.034 / 2;  // cm
}

//////////////////// COLOR SENSOR ////////////////////

int readBlue() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  return pulseIn(COLOR_OUT, LOW);
}

int readRed() {
  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  return pulseIn(COLOR_OUT, LOW);
}

bool detectBlue()  { return readBlue() < 120; }
bool detectRed()   { return readRed()  < 120; }
bool detectGreen() { return !detectRed() && !detectBlue(); }

//////////////////// BOX ACTIONS ////////////////////

void pickUpBox() {
  stopMotors();
  claw.write(clawOpen);
  arm.write(armDown);
  delay(500);

  claw.write(clawClose);
  delay(500);

  arm.write(armUp);   // lift fully (important for points)
  delay(500);
}

void dropBox() {
  stopMotors();
  arm.write(armDown);
  delay(500);

  claw.write(clawOpen);
  delay(500);
}

//////////////////// SETUP ////////////////////

void setup() {
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);

  claw.attach(CLAW_PIN);
  arm.attach(ARM_PIN);

  claw.write(clawOpen);
  arm.write(armDown);
}

//////////////////// LOOP (START TASK) ////////////////////

void loop() {

  // 1️⃣ Go straight until ultrasonic detects box
  while (getDistance() > 7) {   // adjust 6–8 cm as needed
    moveForward(100);
  }

  // 2️⃣ Pick up box
  pickUpBox();

  // 3️⃣ Move forward until red or green detected
  while (!detectRed() && !detectGreen()) {
    moveForward(100);
  }

  // 4️⃣ Use IR sensors to align with correct path
  while (true) {
    int left  = digitalRead(IR_LEFT);
    int right = digitalRead(IR_RIGHT);

    if (left == LOW && right == LOW) {
      moveForward(90);
    } else if (left == LOW) {
      turnLeft();
    } else if (right == LOW) {
      turnRight();
    } else {
      break;  // aligned
    }
  }

  // 5️⃣ Move forward until blue detected
  while (!detectBlue()) {
    moveForward(100);
  }

  // 6️⃣ Drop box
  dropBox();

  // Stop after start task
  while (true);
}
