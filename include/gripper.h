#ifndef GRIPPER_H
#define GRIPPER_H
#include <Arduino.h>
#include <ESP32Servo.h>

class Gripper {
  public:

    const int SERVO_MIN = 450;
    const int SERVO_MAX = 2450;

    const int GRIPPER_SERVO_ZERO_OFFSET = 15;
    const int GRIPPER_SERVO_MAX_ANGLE = 75;

    const int GRIPPER_MIN_DIST = 10;
    const int GRIPPER_MAX_DIST = 54;
    const int GRIPPER_DIST_OFFSET = 32;
    const int GRIPPER_GRIP_DIST = 31;

    Gripper(int pin);
    void begin();
    void grip();
    void open();
    void open(int opening_dist);
    void close();

  private:
    int servo_pin;
    Servo servo_ctrl;

    int gripperAngle(int);
    int gripperDist(int);

};

#endif