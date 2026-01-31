// motor pins
int motor1pin1 = 6;
int motor1pin2 = 5;
int motor1Enable = 7;   // PWM pin

int motor2pin1 = 4;
int motor2pin2 = 3;
int motor2Enable = 2;   // PWM pin

// TCS3200 Color Detection: Red, Green, Blue, Black
#define S0 9
#define S1 8
#define S2 12
#define S3 13
#define OUT 11

int redFreq, greenFreq, blueFreq;

void setup() {
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor1Enable, OUTPUT);

  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motor2Enable, OUTPUT);

  // colour code
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  // Frequency scaling 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  Serial.begin(9600);
}

void loop() {
  // MOTOR CODE
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  // these go the same speed
  analogWrite(motor1Enable, 59);
  analogWrite(motor2Enable, 255);

  // COLOR CODE

  int redFreq   = readColor(LOW, LOW);    // Red
  int blueFreq  = readColor(LOW, HIGH);   // Blue
  int clearFreq = readColor(HIGH, LOW);   // Clear
  int greenFreq = readColor(HIGH, HIGH);  // Green

  String color = detectColor(redFreq, greenFreq, blueFreq, clearFreq);

  Serial.print("R: "); Serial.print(redFreq);
  Serial.print(" G: "); Serial.print(greenFreq);
  Serial.print(" B: "); Serial.print(blueFreq);

  Serial.print(" => Detected: "); Serial.println(color);

  delay(500);
}

int readColor(bool s2, bool s3) {
  digitalWrite(S2, s2);
  digitalWrite(S3, s3);
  delay(5); // allow filter to settle
  return pulseIn(OUT, LOW);
}

String detectColor(int r, int g, int b, int clearVal) {
  // BLACK: very low reflection overall
  if (clearVal > 200) return "Black";

  // RED
  if (r < 90 && r < g && r < b) return "Red";

  // GREEN
  if (g < 90 && g < r && g < b) return "Green";

  // BLUE
  if (b < 90 && b < r && b < g) return "Blue";

  return "Unknown";
}
