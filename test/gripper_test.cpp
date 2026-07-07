#include <Arduino.h>
#include "linear_gripper_control.h"

#define GRIPPER_PIN     19

LinearGripperControl gripper(GRIPPER_PIN);

void setup() {
  Serial.begin(115200);
  gripper.begin();

  gripper.close(); //fully close
  delay(5000);
}

void loop() {
  gripper.gripAngle(75); //command angle - close (0) fully open (75)
  delay(5000);

  gripper.gripAngle(0); // fully close
  delay(5000);
}