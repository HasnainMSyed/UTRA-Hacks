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
#define S0 9
#define S1 8
#define S2 12
#define S3 13
#define OUT 11

// int redFreq, greenFreq, blueFreq, clearFreq;

// Ultrasonic Sensor Initialization
int trigPin = A0;
int echoPin = A1;   

// Servo Initialization
Servo servo;

int closed = 90;
int opened = 180;

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
  followPath();
  // pickUpBox2; 
  float distance = UltrasonicSensor();
  if (distance < 10) { //10 cm
    avoidObstacle();
  }

}

void followPath() {
  String colour = colourInput(); // Collects data from colour sensor; outputs colour to Serial Monitor
  if (colour != "white" && colour != "blue") {
    forward();
  }
  else if (colour == "white") {
    findPath();
  }
  else if (colour == "blue"){
    stop();
  }
}

void findPath() {
  // Code for robot to find its way back to the pathline if it encounters white space
  stop();
  delay(1000);

  // turn left to find pathline
  String colour = "white";
  int count = 0;
  while (colour == "white"){
    if (count < 8) {
      // check if pathline is to the left of the robot
      motor1(0);
      motor2(50);
      delay(500);
    } else if (count == 8) {
      // return robot to original orientation
      motor1(100);
      motor2(0);
      delay(500);
    } else {
      // check if pathline is to the right of the robot
      motor1(50);
      motor2(0);
      delay(500);
    }

    stop();
    delay(500);
    count += 1;

    colour = colourInput();
  } 
  
}

void avoidObstacle() {
  // veer left to avoid obstacle
  motor1(200);
  motor2(50);
  delay(1000);

  forward();
  delay(1000);

  // veer right to return to path
  motor1(50);
  motor2(200);
  delay(1000);

  String colour = "white";
  while (colour != "red") {
    forward();
    String colour = colourInput();
  }

  // make robot face path
  motor1(50);
  motor2(0);
  delay(1000);
}

void backward() {
  // these go the same speed
  motor1(150);
  motor2(150);
}

void forward() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  // these go the same speed
  analogWrite(motor1Enable, 100);
  analogWrite(motor2Enable, 100);
}

void turnRight() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  // make turnRight
  analogWrite(motor1Enable, 255);
  analogWrite(motor2Enable, 59);
}

void turnLeft() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  // make turnLeft
  analogWrite(motor1Enable, 59);
  analogWrite(motor2Enable, 255);
}

void motor1(int speed){
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  // make turnRight
  analogWrite(motor1Enable, speed);
}

void motor2(int speed){
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
  // Set the amount of each colour detected by sensor
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

  return color;
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

void openServo(){
  for(int i = closed; i <= opened; i++){
    servo.write(i);
    delay(1);
  }
}

void closeServo(){
  for(int i = opened; i >= closed; i--){
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

  // Read the echo pin and calculate the distance
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2; // Convert to centimeters

  // Print the distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;

  // delay(500); // Wait for 500 milliseconds before the next reading

}