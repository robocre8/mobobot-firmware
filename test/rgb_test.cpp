#include <Arduino.h>
#include <Led.h>

#define LED_PIN      2
#define R_LED_PIN    27
#define B_LED_PIN    26
#define G_LED_PIN    25


Led led(LED_PIN);

void rgb_init() {
  pinMode(R_LED_PIN, OUTPUT);
  pinMode(G_LED_PIN, OUTPUT);
  pinMode(B_LED_PIN, OUTPUT);

  analogWrite(R_LED_PIN, 255);
  analogWrite(G_LED_PIN, 255);
  analogWrite(B_LED_PIN, 255);
}

void rgb_on(int r_pwm, int g_pwm, int b_pwm) {
  analogWrite(R_LED_PIN, 255-r_pwm);
  analogWrite(G_LED_PIN, 255-g_pwm);
  analogWrite(B_LED_PIN, 255-b_pwm);
}

void rgb_off() {
  analogWrite(R_LED_PIN, 255);
  analogWrite(G_LED_PIN, 255);
  analogWrite(B_LED_PIN, 255);
}

void setup() {
  Serial.begin(115200);
  analogWriteFrequency(1000);

  rgb_init();
  rgb_off();
  
  led.off();
  delay(1000);
  led.on();
  delay(2000);
  led.off();
  delay(1000);

}

void loop() {
  delay(1000);
  rgb_on(0, 255, 0);
  delay(2000);
  rgb_off();
}
