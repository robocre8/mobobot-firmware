#include <Arduino.h>
#include "buzzer.h"

#define BUZZER_PIN   14

Buzzer buzzer(BUZZER_PIN);

void setup() {
  Serial.begin(115200);
  
  buzzer.off();
  delay(1000);
  buzzer.on();
  delay(3000);
  buzzer.off();
  delay(2000);
  buzzer.on();
  delay(3000);
  buzzer.off();
  delay(1000);

}

void loop() {

}