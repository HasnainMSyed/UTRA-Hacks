#include <Arduino.h>
//#include <L298N.h>
#include <Servo.h>

// ************ DEFINES ************

#define LEFT_MOTOR   10   // ENA (PWM) motor 2
#define RIGHT_MOTOR  9   // ENB (PWM) motor 1

// L298N direction pins
#define IN1          6    // Left motor direction 1
#define IN2          5    // Left motor direction 2
#define IN3          4   // Right motor direction 1
#define IN4          3   // Right motor direction 2

#define TRIG_PIN     A0
#define ECHO_PIN     A1

#define LEFT_SERVO_PIN 2
#define RIGHT_SERVO_PIN 7

#define COLOUR_S0    8
#define COLOUR_S1    12
#define COLOUR_S2    13
#define COLOUR_S3    A2
#define COLOUR_OUT   11  // frequency output from colour sensor

// ************ GLOBALS ************

Servo clawServo;

enum State {
  START,
  NAVIGATE_TO_FORK,
  CHOOSE_RED_BRANCH,
  NAVIGATE_RED_TO_SEARCH_BOX,
  PICK_BOX,
  FIND_END,
  FINISH
};

State state = START;

// Tune this after printing readings
long BLACK_THRESHOLD = 6000;

// For a simple time-based red drop detection (tune)
unsigned long redDropStartTime = 0;
bool redDropTimerStarted = false;

// ************ SETUP / LOOP ************

void setup() {
  delay(5000);
  Serial.begin(9600);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(COLOUR_S0, OUTPUT);
  pinMode(COLOUR_S1, OUTPUT);
  pinMode(COLOUR_S2, OUTPUT);
  pinMode(COLOUR_S3, OUTPUT);
  pinMode(COLOUR_OUT, INPUT);

  // Colour sensor frequency scaling
  digitalWrite(COLOUR_S0, HIGH);
  digitalWrite(COLOUR_S1, LOW);

  clawServo.attach(LEFT_SERVO_PIN);
  //clawServo.attach(RIGHT_SERVO_PIN);
  clawServo.write(0);    // open claw

  Serial.println("Robot Ready");
}

void loop() {
  moveMotors(0, 150);
}

// ************ FSM ************

void updateFSM() {
  switch (state) {

    case START:
      stopRobot();
      delay(1000);
      state = NAVIGATE_TO_FORK;
      break;

    case NAVIGATE_TO_FORK:
      followAction();
      if (atFork()) {
        stopRobot();
        state = CHOOSE_RED_BRANCH;
      }
      break;
    
    case CHOOSE_RED_BRANCH:
      if (seeRedLine()) {
        state = NAVIGATE_RED_TO_SEARCH_BOX;
      }
      else {
        turnLeftSoft();
        delay(80);
        stopRobot();
      }
      break;

    case NAVIGATE_RED_TO_SEARCH_BOX:
      followAction();            // colour-based line follow
      if (blueColourDetected()) {
        stopRobot();
        state = PICK_BOX;
      }
      break;

    case PICK_BOX:
      stopRobot();
      turnRight();
      pickup();
      delay(800);
      // reset red drop timer for later
      redDropTimerStarted = false;
      turnLeft();
      state = FIND_END;
      break;

    case FIND_END:
      followAction();
      if (isGrey()) {
        stopRobot();
        state = FINISH;
      }
      break;

    case FINISH:
      stopRobot();
      // Wait here; you will reupload Code 2
      break;
  }
}

// ************ SENSORS ************

long getDistance() { // in CM
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

bool blueColourDetected() {
  int r, g, b;  // tune threshold
  readColour(r, g, b);
  if (isBlue()) {
      return true;
  }
  else {
      return false;
  }
}

bool objectDetected() {
  long d = getDistance();
  return d > 0 && d < 10;   // tune threshold
}

void readColour(int &r, int &g, int &b) {
  // Red
  digitalWrite(COLOUR_S2, LOW);
  digitalWrite(COLOUR_S3, LOW);
  r = pulseIn(COLOUR_OUT, LOW);

  // Green
  digitalWrite(COLOUR_S2, HIGH);
  digitalWrite(COLOUR_S3, HIGH);
  g = pulseIn(COLOUR_OUT, LOW);

  // Blue
  digitalWrite(COLOUR_S2, LOW);
  digitalWrite(COLOUR_S3, HIGH);
  b = pulseIn(COLOUR_OUT, LOW);
}

// Is sensor over black tape?
bool onBlackLine() {
  int r, g, b;
  readColour(r, g, b);
  long total = (long)r + g + b;
  return total < BLACK_THRESHOLD;   // tune with Serial prints
}

// Fork marked by strong red (example)
bool atFork() {
  int r, g, b;
  readColour(r, g, b);
  bool redDominant = (r < g) && (r < b);
  return redDominant;
}

// Placeholder: reach first red blue-zone using time from start of red path.
// Replace with better detection later (e.g. blue colour).
bool atRedDropZone() {
  if (!redDropTimerStarted) {
    redDropStartTime = millis();
    redDropTimerStarted = true;
  }
  // e.g. 4 seconds after entering red path – tune on track
  return (millis() - redDropStartTime) > 4000;
}

bool isRed() {
  int r, g, b;
  readColour(r, g, b);
  return (r < g) && (r < b);
}

bool isGreen() {
  int r, g, b;
  readColour(r, g, b);
  return (g < r) && (g < b);
}

bool isBlue() {
  int r, g, b;
  readColour(r, g, b);
  return (b < r) && (b < g);
}

bool isGrey() {
  int r, g, b;
  readColour(r, g, b);
  long total = (long)r + g + b;

  // You MUST tune these three numbers from your Serial readings:
  long MIN_GREY_TOTAL = 8000;   // greater than black total
  long MAX_GREY_TOTAL = 14000;  // less than white total

  int maxC = max(r, max(g, b));
  int minC = min(r, min(g, b));
  int diff = maxC - minC;

  int MAX_DIFF = 800;           // small difference between r,g,b

  bool totalInRange = (total > MIN_GREY_TOTAL) && (total < MAX_GREY_TOTAL);
  bool balanced     = (diff < MAX_DIFF);

  return totalInRange && balanced;
}

bool seeRedLine() {
  int r, g, b;
  readColour(r, g, b);
  // On red tape/region, red is dominant, but total not super dark like black
  bool redDominant = (r < g) && (r < b);
  long total = (long)r + g + b;
  // Tune these; idea: not on black, not on white floor
  return redDominant && total > BLACK_THRESHOLD;
}


// ************ MOTION ************

void moveMotors(int left, int right) {
  // LEFT motor (Motor A): IN1, IN2, ENA = LEFT_MOTOR
  if (left != 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // RIGHT motor (Motor B): IN3, IN4, ENB = RIGHT_MOTOR
  if (right != 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(LEFT_MOTOR,  abs(left));
  analogWrite(RIGHT_MOTOR, abs(right));
}

void moveForward(int speed) {
  moveMotors(speed, speed);
}

void stopRobot() {
  moveMotors(0, 0);
}

void turnLeft() {
  moveMotors(0, 150);
  delay(400);
}

void turnRight() {
  moveMotors(150, 0);
  delay(400);
}

void turnLeftSoft() {
  moveMotors(0, 120);
}

void turnRightSoft() {
  moveMotors(120, 0);
}

// Colour-based line following (no IR)
void followAction() {
  static bool lastTurnLeft = true;

  if (onBlackLine()) {
    moveForward(120);
  } else {
    // simple wiggle search when off line
    if (lastTurnLeft) {
      turnLeftSoft();
    } else {
      turnRightSoft();
    }
    delay(80);
    lastTurnLeft = !lastTurnLeft;
  }
}

// ************ ACTIONS (CLAW) ************

void pickup() {
  Serial.println("Pickup");
  // Close claw; adjust angle as needed
  clawServo.write(70);
}

void release() {
  Serial.println("Release");
  // Open claw
  clawServo.write(0);
}

