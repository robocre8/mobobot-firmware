#include <Arduino.h>
#include <Ultrasonic.h>
#include <Led.h>

#define TRIGGER_PIN  32
#define ECHO_PIN     34

// #define TRIGGER_PIN  33
// #define ECHO_PIN     35

#define LED_PIN      2

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
Led led(LED_PIN);

void setup() {
  Serial.begin(115200);

  led.off();
  delay(500);
  led.on();
  delay(2000);
  led.off();
  delay(500);
}

void loop() {
  int dist_mm = (int)(ultrasonic.convert(ultrasonic.timing(), Ultrasonic::CM)*10);

  Serial.print("Dist (MM): ");
  Serial.println(dist_mm);
  delay(100);
}