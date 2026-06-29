#include <Arduino.h>
#include <sonar_sensor.h>
#include <led.h>

#define LED_PIN      2

const int TRIGGER_PIN = 33;
const int ECHO_PIN = 35;
int SONAR_TIMEOUT_MS = 15;

SonarSensor sonar(TRIGGER_PIN, ECHO_PIN, SONAR_TIMEOUT_MS);
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
  int dist_mm = sonar.readDist();

  Serial.print("Dist (MM): ");
  Serial.println(dist_mm);
  delay(100);
}