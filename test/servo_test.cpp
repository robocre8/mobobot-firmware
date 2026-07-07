#include <Arduino.h>
#include "servo_control.h"

#define SERVO_PIN     18

ServoControl servo(SERVO_PIN);


void setup() {
  Serial.begin(115200);
  servo.begin();

  servo.write(0);
  delay(2000);
}

void loop() {
  servo.write(0);
  delay(3000);

  servo.write(90);
  delay(3000);

  servo.write(0);
  delay(3000);

  servo.write(-90);
  delay(3000);

}