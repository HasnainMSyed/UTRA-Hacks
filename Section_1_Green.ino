/*
  Section 1 Re-upload Code (Target Shooting)
  - Follow BLACK tape up straight ramp
  - Enter target stage, navigate inward using color rings: BLUE -> RED -> GREEN -> BLACK center
  - Shoot ball (launchBall())
  - Find BLACK taped exit path and line-follow down curved ramp
  - Follow BLACK strip to checkpoint (optionally detect PURPLE)

  Hardware assumed from kit:
  - Arduino Uno
  - L298N motor driver + 2 DC motors
  - 2 IR line sensors
  - TCS-style color sensor (frequency output, S2/S3 select)
  - Ultrasonic sensor (HC-SR04 style)
*/

#include <Servo.h>

// ---------------------- PIN MAP (matches your robot_first_part.ino) ----------------------
// Motors (L298N)
const int ENA = 5;   // left PWM
const int IN1 = 6;
const int IN2 = 7;

const int ENB = 9;   // right PWM
const int IN3 = 10;
const int IN4 = 11;

// Ultrasonic
const int TRIG = 12;
const int ECHO = 13;

// IR Sensors (on analog pins; read as analog for robustness)
const int IR_LEFT  = A0;
const int IR_RIGHT = A1;

// Color Sensor (your earlier sketch used only S2/S3 + OUT)
const int S2 = A2;
const int S3 = A3;
const int COLOR_OUT = A4;

// Optional servo/launcher pin (edit if needed)
Servo launcher;
const int LAUNCHER_PIN = 3;

// ---------------------- TUNING: MOTOR BALANCE ----------------------
// Because one motor is weaker, we use separate base speeds and scaling.
int BASE_LEFT  = 120;     // start here (0-255)
int BASE_RIGHT = 150;     // make this higher if right motor is weaker, lower if stronger

float LEFT_SCALE  = 1.00; // fine trim (e.g., 0.95 or 1.05)
float RIGHT_SCALE = 1.00;

// Turn strength for line following
int TURN_DELTA = 55;      // how aggressive the correction is

// ---------------------- TUNING: IR LINE SENSOR ----------------------
// You MUST calibrate: print readings on BLACK tape vs WHITE board.
int IR_THRESHOLD = 500;   // if analogRead < threshold => "black" (common), but depends on your sensors

// ---------------------- TUNING: ULTRASONIC ----------------------
int WALL_CM = 10;         // if closer than this, back away to avoid hitting stage wall

// ---------------------- TUNING: COLOR SENSOR ----------------------
// Your sensor returns a pulse width/frequency-ish value. These thresholds are placeholders.
// You should quickly calibrate by printing raw readings on each ring color.
int RED_DOMINANT_MAX   = 120;   // smaller often means "more of that color" for TCS3200-like sensors
int GREEN_DOMINANT_MAX = 120;
int BLUE_DOMINANT_MAX  = 120;
int BLACK_CLEAR_MIN    = 200;   // "black" often yields low reflection -> higher pulseIn times on some setups

// If you want checkpoint detection (purple re-upload point), you can add a threshold rule.
// Purple is mentioned as reupload point color. :contentReference[oaicite:2]{index=2}
bool ENABLE_PURPLE_CHECKPOINT_STOP = false;

// ---------------------- BEHAVIOR PARAMETERS ----------------------
unsigned long LINE_FOLLOW_TIMEOUT_MS = 12000; // max time to climb ramp before we assume we reached stage
unsigned long CENTER_SEARCH_TIMEOUT_MS = 25000;
unsigned long EXIT_FIND_TIMEOUT_MS = 15000;

// Step motion for “greedy inward” search
int STEP_FWD_MS = 250;
int STEP_TURN_MS = 140;

// ---------------------- LOW-LEVEL MOTOR CONTROL ----------------------
void setMotorLeft(int pwm, bool forward) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(ENA, pwm);
  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW  : HIGH);
}

void setMotorRight(int pwm, bool forward) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(ENB, pwm);
  digitalWrite(IN3, forward ? HIGH : LOW);
  digitalWrite(IN4, forward ? LOW  : HIGH);
}

void drive(int leftPwm, int rightPwm) {
  // forward only here; keep it simple + stable for hackathon
  leftPwm  = int(leftPwm  * LEFT_SCALE);
  rightPwm = int(rightPwm * RIGHT_SCALE);
  setMotorLeft(leftPwm, true);
  setMotorRight(rightPwm, true);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void pivotLeft(int pwm) {
  // left backward, right forward
  pwm = constrain(pwm, 0, 255);
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void pivotRight(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void reverseBoth(int pwm, int ms) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  delay(ms);
  stopMotors();
}

// ---------------------- ULTRASONIC ----------------------
long getDistanceCm() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 25000); // 25ms timeout
  if (duration == 0) return 999;              // no echo
  return long(duration * 0.034 / 2);
}

// ---------------------- IR LINE ----------------------
bool irOnBlackLeft() {
  int v = analogRead(IR_LEFT);
  return v < IR_THRESHOLD;
}

bool irOnBlackRight() {
  int v = analogRead(IR_RIGHT);
  return v < IR_THRESHOLD;
}

// Simple 2-sensor line follow: keep tape between sensors
void lineFollowTick() {
  bool L = irOnBlackLeft();
  bool R = irOnBlackRight();

  int left = BASE_LEFT;
  int right = BASE_RIGHT;

  if (L && R) {
    // centered on tape (or very thick tape)
    drive(left, right);
  } else if (L && !R) {
    // drifted left -> steer left slightly (slow left or speed right)
    drive(left - TURN_DELTA, right + TURN_DELTA);
  } else if (!L && R) {
    // drifted right -> steer right slightly
    drive(left + TURN_DELTA, right - TURN_DELTA);
  } else {
    // lost line: creep forward a bit
    drive(left - 20, right - 20);
  }
}

// ---------------------- COLOR SENSOR (TCS-like with S2/S3 only) ----------------------
int readColorPulse(bool s2, bool s3) {
  digitalWrite(S2, s2);
  digitalWrite(S3, s3);
  delay(3);
  // pulseIn returns time. Depending on your wiring/scaling, “smaller” might mean “more intensity”.
  return pulseIn(COLOR_OUT, LOW, 20000);
}

struct ColorReading {
  int r;
  int g;
  int b;
  int clearVal;
};

ColorReading readColor() {
  ColorReading c;
  c.r = readColorPulse(LOW, LOW);
  c.b = readColorPulse(LOW, HIGH);
  c.clearVal = readColorPulse(HIGH, LOW);
  c.g = readColorPulse(HIGH, HIGH);
  return c;
}

enum DetectedColor { COL_UNKNOWN, COL_BLUE, COL_RED, COL_GREEN, COL_BLACK, COL_PURPLE };

DetectedColor classifyColor(const ColorReading &c) {
  // BLACK: low reflection overall. On many setups this means clearVal is "large".
  if (c.clearVal > BLACK_CLEAR_MIN) return COL_BLACK;

  // Dominant channel test (placeholders; calibrate)
  bool redDom   = (c.r < RED_DOMINANT_MAX   && c.r < c.g && c.r < c.b);
  bool greenDom = (c.g < GREEN_DOMINANT_MAX && c.g < c.r && c.g < c.b);
  bool blueDom  = (c.b < BLUE_DOMINANT_MAX  && c.b < c.r && c.b < c.g);

  if (blueDom)  return COL_BLUE;
  if (redDom)   return COL_RED;
  if (greenDom) return COL_GREEN;

  // Purple (rough): red + blue both strong-ish vs green
  // (This is optional; only used if you want checkpoint stop.)
  if (ENABLE_PURPLE_CHECKPOINT_STOP) {
    bool purpleish = (c.r < RED_DOMINANT_MAX + 30) && (c.b < BLUE_DOMINANT_MAX + 30) && (c.g > c.r) && (c.g > c.b);
    if (purpleish) return COL_PURPLE;
  }

  return COL_UNKNOWN;
}

// ---------------------- HIGH-LEVEL TASKS ----------------------

// 1) Follow black line up ramp (straight)
bool climbRampByLineFollow() {
  unsigned long t0 = millis();
  while (millis() - t0 < LINE_FOLLOW_TIMEOUT_MS) {
    lineFollowTick();
  }
  stopMotors();
  return true;
}

// 2) On target platform: move inward to black center using greedy color transitions
// Official: platform has rings; use color cues to reach center black where ball is. :contentReference[oaicite:3]{index=3}
bool navigateToCenterBlack() {
  unsigned long t0 = millis();

  // We don't know orientation. We do a "greedy hill-climb":
  // - Take a small forward step
  // - If we moved inward (toward BLACK), keep going
  // - Otherwise, back up, rotate a bit, try again
  // Also avoid walls using ultrasonic.

  DetectedColor current = classifyColor(readColor());
  int attempts = 0;

  while (millis() - t0 < CENTER_SEARCH_TIMEOUT_MS) {
    // If we're at center already:
    if (current == COL_BLACK) {
      stopMotors();
      return true;
    }

    // Wall avoidance
    long d = getDistanceCm();
    if (d < WALL_CM) {
      reverseBoth(160, 250);
      pivotLeft(170);
      delay(220);
      stopMotors();
      current = classifyColor(readColor());
      continue;
    }

    // Step forward
    drive(BASE_LEFT, BASE_RIGHT);
    delay(STEP_FWD_MS);
    stopMotors();
    delay(60);

    DetectedColor next = classifyColor(readColor());

    // If we improved (went inward), accept and continue.
    // Desired order inward: BLUE -> RED -> GREEN -> BLACK (based on your description + target concept).
    auto rank = [](DetectedColor c) -> int {
      switch (c) {
        case COL_BLUE:  return 0;
        case COL_RED:   return 1;
        case COL_GREEN: return 2;
        case COL_BLACK: return 3;
        default:        return -1;
      }
    };

    if (rank(next) >= rank(current) && rank(next) != -1) {
      current = next;
      attempts = 0;
      continue;
    }

    // Otherwise: undo + rotate slightly, try a new heading
    reverseBoth(150, 180);

    // alternate turning direction to escape local traps
    if (attempts % 2 == 0) {
      pivotLeft(160);
    } else {
      pivotRight(160);
    }
    delay(STEP_TURN_MS + (attempts * 8)); // slowly increase turn size
    stopMotors();
    delay(80);

    current = classifyColor(readColor());
    attempts++;
  }

  stopMotors();
  return false;
}

// 3) Shoot the ball (you must implement your launcher)
// Official: ball is at center black, must shoot forward; best points if ball ends in blue region. :contentReference[oaicite:4]{index=4}
void launchBall() {
  // TODO: replace with your mechanism
  // Example servo flick (placeholder):
  // launcher.write(0); delay(200);
  // launcher.write(90); delay(250);
  // launcher.write(0); delay(200);

  // For now: just pause so you can confirm position
  delay(500);
}

// 4) Find the black taped exit ramp path by searching for line with IR sensors
bool findExitLine() {
  unsigned long t0 = millis();

  // Spin and look for black under either sensor
  while (millis() - t0 < EXIT_FIND_TIMEOUT_MS) {
    bool L = irOnBlackLeft();
    bool R = irOnBlackRight();
    if (L || R) {
      stopMotors();
      return true;
    }
    pivotLeft(150);
    delay(40);
  }
  stopMotors();
  return false;
}

// 5) Follow the curved black tape down, then follow it to checkpoint
void followLineToCheckpoint(unsigned long maxMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < maxMs) {
    // Optionally stop at purple reupload point
    if (ENABLE_PURPLE_CHECKPOINT_STOP) {
      DetectedColor c = classifyColor(readColor());
      if (c == COL_PURPLE) {
        stopMotors();
        while (true) ; // finished
      }
    }
    lineFollowTick();
  }
  stopMotors();
}

// ---------------------- MAIN ----------------------
void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);

  launcher.attach(LAUNCHER_PIN);

  stopMotors();
  delay(500);
}

void loop() {
  // A) Ramp up (black tape becomes the path after reupload in your plan)
  climbRampByLineFollow();

  // B) On stage: find center black
  bool gotCenter = navigateToCenterBlack();

  // C) Shoot
  if (gotCenter) {
    launchBall();
  }

  // D) Find exit curved ramp tape and go down
  bool foundExit = findExitLine();
  if (foundExit) {
    // follow long enough to get down ramp + reach checkpoint area
    followLineToCheckpoint(20000);
  }

  // End
  while (true) { stopMotors(); }
}
