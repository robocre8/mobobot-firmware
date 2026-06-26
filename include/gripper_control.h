#ifndef GRIPPER_H
#define GRIPPER_H
#include <Arduino.h>
#include "servo_control.h"

//---------------------------------------

class GripperControl : public ServoControl {
  public:
    GripperControl(int pin) : ServoControl(pin) {}

    void grip();
    void open();
    void open(int opening_dist);
    void close();

    void setMinMaxGripDist(int, int);
    void setGripDist(int);
    void setOffsetDist(int);

    int getGripDist();
    int getMaxGripDist();
    int getMinGripDist();
    int getOffsetDist();

    int convertDistToAngle(int);

  private:
    int GRIPPER_SERVO_ZERO_OFFSET = 15;
    int GRIPPER_SERVO_MAX_ANGLE = 75;

    int GRIPPER_MIN_DIST = 10;
    int GRIPPER_MAX_DIST = 54;
    int GRIPPER_OFFSET_DIST = 32;
    int GRIPPER_GRIP_DIST = 31;

    // int gripperAngle(int);

};

//------------------------------------------


// int GripperControl::gripperAngle(int angle_deg) {
//   static int default_angle = GRIPPER_SERVO_ZERO_OFFSET;
//   int p = constrain(angle_deg, 0, GRIPPER_SERVO_MAX_ANGLE);
//   return constrain(p+default_angle, 0, 180);
// }

// gripper inverse kinematics
int GripperControl::convertDistToAngle(int opening_dist_mm) {
  int d = constrain(opening_dist_mm, GRIPPER_MIN_DIST, GRIPPER_MAX_DIST);
  int cmd_dist = d + GRIPPER_OFFSET_DIST;

  double dist = (double)cmd_dist/2.0;
  double b = acos((1018.0-(dist*dist))/918.0);
  double b_sin = sin(b);
  double cmd_angle = (PI/2) - (asin((b_sin*27.0/dist)));

  return (int)(cmd_angle*180/PI);
}

void GripperControl::setMinMaxGripDist(int min_grip_dist_mm, int max_grip_dist_mm) {
  GRIPPER_MIN_DIST = min_grip_dist_mm;
  GRIPPER_MAX_DIST = max_grip_dist_mm;
}

void GripperControl::setGripDist(int grip_dist_mm) {
  GRIPPER_GRIP_DIST = grip_dist_mm;
}

void GripperControl::setOffsetDist(int dist_mm) {
  GRIPPER_OFFSET_DIST = dist_mm;
}

int GripperControl::getGripDist() {
  return GRIPPER_GRIP_DIST;
}

int GripperControl::getMaxGripDist() {
  return GRIPPER_MAX_DIST;
}

int GripperControl::getMinGripDist() {
  return GRIPPER_MIN_DIST;
}

int GripperControl::getOffsetDist() {
  return GRIPPER_OFFSET_DIST;
}

void GripperControl::grip() {
  write(convertDistToAngle(GRIPPER_GRIP_DIST));
}

void GripperControl::open() {
  write(convertDistToAngle(GRIPPER_MAX_DIST));
}

void GripperControl::open(int opening_dist_mm) {
  write(convertDistToAngle(opening_dist_mm));
}

void GripperControl::close() {
  write(convertDistToAngle(GRIPPER_MIN_DIST));
}

//--------------------------------------------

#endif