#include <Arduino.h>
#include <Led.h>
#include <ESP32Servo.h>

#define LED_PIN      2
#define SERVO_PIN    18
#define SERVO_MIN    500
#define SERVO_MAX    2450

Led led(LED_PIN);
Servo servo;

int pos = 0;

void writeAngle(int angle_deg) {
  int angle = constrain(angle_deg, -90, 90);
  int time_us = map(angle, -90, 90, 500, 2450);
  servo.writeMicroseconds(time_us);
}

int readAngle() {
  // int time_us = servo.readMicroseconds();
  // int angle = map(time_us, 500, 2450, -90, 90);
  // return angle;
  return servo.read();
}

void setup() {
  Serial.begin(115200);
  
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);    // Standard 50 Hz servo frequency
  servo.attach(SERVO_PIN, SERVO_MIN, SERVO_MAX);      // Attach the servo to the pin
  
  led.off();
  delay(1000);
  led.on();
  delay(2000);
  led.off();
  delay(1000);

  // servo.writeMicroseconds(SERVO_MAX);
  writeAngle(0);
  servo.write(90);

}

void loop() {
  int angle;
  // for (pos = -90; pos <= 90; pos += 30) { 
  //   writeAngle(pos);
  //   angle = readAngle();
  //   Serial.println(angle);
  //   delay(500);
  // }
  // for (pos = 90; pos >=  -90; pos -= 30) {
  //   writeAngle(pos);
  //   angle = readAngle();
  //   Serial.println(angle);
  //   delay(500);
  // }

  for (pos = 0; pos <= 180; pos += 30) { 
    servo.write(pos);
    angle = servo.read();
    Serial.println(angle+1);
    delay(500);
  }
  for (pos = 180; pos >=  0; pos -= 30) {
    servo.write(pos);
    angle = servo.read();
    Serial.println(angle+1);
    delay(500);
  }
}
