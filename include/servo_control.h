#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H
#include <Arduino.h>
#include <ESP32Servo.h>

//--------------------------------------------

class ServoControl {
  public:
    ServoControl(int pin);
    void begin();
    void write(int);
    void setMinMaxCommandUS(int, int);

  private:
    int SERVO_MIN_US_COMMAND = 500;
    int SERVO_MAX_US_COMMAND = 2450;

    int servo_pin;
    Servo servo;
    int servoAngle(int);
};

//--------------------------------------------

ServoControl::ServoControl(int pin)
{
  servo_pin = pin;
}

void ServoControl::begin() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);    // Standard 50 Hz servo frequency
  servo.attach(servo_pin, SERVO_MIN_US_COMMAND, SERVO_MAX_US_COMMAND);
}

void ServoControl::setMinMaxCommandUS(int min_us, int max_us)
{
  SERVO_MIN_US_COMMAND = min_us;
  SERVO_MIN_US_COMMAND = max_us;
}

int ServoControl::servoAngle(int angle_deg) {
  int a = constrain(angle_deg, -90, 90);
  int angle = map(a, -90, 90, 0, 180);
  return angle;
}

void ServoControl::write(int angle) {
  servo.write(servoAngle(angle));
}

//---------------------------------------

#endif