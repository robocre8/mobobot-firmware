#include <Arduino.h>
#include "led.h"
#include "rgb_led.h"

#define LED_PIN      2
#define R_PIN    27
#define B_PIN    26
#define G_PIN    25


Led led(LED_PIN);
RGBLed rgb_led(R_PIN, G_PIN, B_PIN);

void setup() {
  Serial.begin(115200);

  rgb_led.off();
  
  led.off();
  delay(1000);
  led.on();
  delay(2000);
  led.off();
  delay(1000);

}

void loop() {
  delay(2000);
  rgb_led.red();
  delay(2000);
  rgb_led.green();
  delay(2000);
  rgb_led.blue();
  delay(2000);
  rgb_led.white();
  delay(2000);
  rgb_led.off();
}