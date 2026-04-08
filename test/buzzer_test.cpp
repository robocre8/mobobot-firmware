#include <Arduino.h>
#include <Led.h>
#include "ir_sensor.h"

#define LED_PIN      2
#define BUZZER_PIN   14

const int LINE_SENSOR1_PIN = 32;
const int LINE_SENSOR2_PIN = 34;

IRSensor lineSensor1(LINE_SENSOR1_PIN);
IRSensor lineSensor2(LINE_SENSOR2_PIN);

Led led(LED_PIN);
Led buzzer(BUZZER_PIN);

void setup() {
  Serial.begin(115200);

  // buzzer.off();
  // pinMode(LINE_SENSOR1_PIN, INPUT);
  
  led.off();
  delay(500);
  led.on();
  delay(2000);
  led.off();
  delay(500);

}

void loop() {
  // buzzer.blink(1000);

  Serial.println(lineSensor1.read());
  Serial.println(lineSensor2.read());
  Serial.println();
  delay(500);

}