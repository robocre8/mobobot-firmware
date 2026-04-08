#include "gripper.h"

Gripper::Gripper(int pin)
{
  servo_pin = pin;
}

void Gripper::begin() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo_ctrl.setPeriodHertz(50);    // Standard 50 Hz servo frequency
  servo_ctrl.attach(servo_pin, SERVO_MIN, SERVO_MAX);      // Attach the servo to the pin
}

int Gripper::gripperAngle(int angle_deg) {
  static int default_angle = GRIPPER_SERVO_ZERO_OFFSET;
  int p = constrain(angle_deg, 0, GRIPPER_SERVO_MAX_ANGLE);
  return constrain(p+default_angle, 0, 180);
}

int Gripper::gripperDist(int dist_mm) {
  int d = constrain(dist_mm, GRIPPER_MIN_DIST, GRIPPER_MAX_DIST);
  int cmd_dist = d + GRIPPER_DIST_OFFSET;

  double dist = (double)cmd_dist/2.0;
  double b = acos((1018.0-(dist*dist))/918.0);
  double b_sin = sin(b);
  double cmd_angle = (PI/2) - (asin((b_sin*27.0/dist)));

  return gripperAngle((int)(cmd_angle*180/PI));
}

void Gripper::grip() {
  servo_ctrl.write(gripperDist(GRIPPER_GRIP_DIST));
}

void Gripper::open() {
  servo_ctrl.write(gripperDist(GRIPPER_MAX_DIST));
}

void Gripper::open(int opening_dist) {
  servo_ctrl.write(gripperDist(opening_dist));
}

void Gripper::close() {
  servo_ctrl.write(gripperDist(GRIPPER_MIN_DIST));
}