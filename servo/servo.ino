#include <Servo.h>

Servo servo1;
Servo servo2;

const int SERVO_PIN_1 = 3;
const int SERVO_PIN_2 = 5;

void setup() {
  servo1.attach(SERVO_PIN_1);
  servo2.attach(SERVO_PIN_2);
}

void loop() {
  // Sweep 0 -> 90
  for (int pos = 0; pos <= 45; pos += 5) {
    servo1.write(pos);
    servo2.write(45-pos);
    delay(20);
  }

  delay(5000);
  
    // Sweep 90 -> 0
  for (int pos = 45; pos >= 0; pos -= 5) {
    servo1.write(pos);
    servo2.write(45-pos);
    delay(20);
  }


  delay(500);
}
