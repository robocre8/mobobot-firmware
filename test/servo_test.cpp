#include <Arduino.h>
#include "gripper_control.h"

#define SERVO_PIN     19

GripperControl gripper(SERVO_PIN);

int opening_dist = 0;
int opening_angle = 0;


void setup() {
  Serial.begin(115200);
  gripper.begin();

  gripper.write(0);
  delay(2000);
}

void loop() {

  opening_dist = gripper.getMaxGripDist();
  opening_angle = gripper.convertDistToAngle(opening_dist);
  gripper.open();
  Serial.println(opening_angle);
  delay(2000);

  opening_dist = gripper.getMinGripDist();
  opening_angle = gripper.convertDistToAngle(opening_dist);
  gripper.close();
  Serial.println(opening_angle);
  delay(2000);

}