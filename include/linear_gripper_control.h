#ifndef LINEAR_GRIPPER_CONTROL_H
#define LINEAR_GRIPPER_CONTROL_H
#include <Arduino.h>
#include "servo_control.h"

//---------------------------------------

class LinearGripperControl {
  public:
    LinearGripperControl(int pin) {
      gripper_pin = pin;
      servo.attachPin(gripper_pin);
    }

    void begin();

    void gripDist(int grip_dist_mm);
    void gripAngle(int angle_deg);
    void close();

    void setMaxGripAngle(int);
    void setMaxGripDist(int);
    void setMinGripDist(int);
    void setOffsetDist(int);

    int getMaxGripAngle();
    int getMaxGripDist();
    int getMinGripDist();
    int getOffsetDist();

    int convertDistToAngle(int);

  private:
    int GRIPPER_SERVO_MAX_ANGLE = 75;

    int GRIPPER_MIN_DIST = 10;
    int GRIPPER_MAX_DIST = 54;
    int GRIPPER_OFFSET_DIST = 32;

    int gripper_pin;
    ServoControl servo;

};

//------------------------------------------

void LinearGripperControl::begin() {
  servo.begin();
}

void LinearGripperControl::gripAngle(int angle_deg) {
  int angle = constrain(angle_deg, 0, GRIPPER_SERVO_MAX_ANGLE);
  servo.write(angle);
}

void LinearGripperControl::gripDist(int grip_dist_mm) {
  servo.write(convertDistToAngle(grip_dist_mm));
}

void LinearGripperControl::close() {
  servo.write(0);
}

// gripper inverse kinematics
int LinearGripperControl::convertDistToAngle(int opening_dist_mm) {
  int d = constrain(opening_dist_mm, GRIPPER_MIN_DIST, GRIPPER_MAX_DIST);
  int cmd_dist = d + GRIPPER_OFFSET_DIST;

  double dist = (double)cmd_dist/2.0;
  double b = acos((1018.0-(dist*dist))/918.0);
  double b_sin = sin(b);
  double cmd_angle = (PI/2) - (asin((b_sin*27.0/dist)));

  return (int)(cmd_angle*180/PI);
}

void LinearGripperControl::setMaxGripAngle(int grip_angle_deg) {
  GRIPPER_SERVO_MAX_ANGLE = grip_angle_deg;
}

void LinearGripperControl::setMaxGripDist(int max_grip_dist_mm) {
  GRIPPER_MAX_DIST = max_grip_dist_mm;
}

void LinearGripperControl::setMinGripDist(int min_grip_dist_mm) {
  GRIPPER_MIN_DIST = min_grip_dist_mm;
}

void LinearGripperControl::setOffsetDist(int dist_mm) {
  GRIPPER_OFFSET_DIST = dist_mm;
}

int LinearGripperControl::getMaxGripAngle() {
  return GRIPPER_SERVO_MAX_ANGLE;
}

int LinearGripperControl::getMaxGripDist() {
  return GRIPPER_MAX_DIST;
}

int LinearGripperControl::getMinGripDist() {
  return GRIPPER_MIN_DIST;
}

int LinearGripperControl::getOffsetDist() {
  return GRIPPER_OFFSET_DIST;
}

//--------------------------------------------

#endif