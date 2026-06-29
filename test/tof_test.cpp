#include <Arduino.h>
#include <Wire.h>
#include <tof_sensor.h>
#include <led.h>

#define LED_PIN      2

TOFSensor tof;
Led led(LED_PIN);

void setup() {
  Serial.begin(115200);

  Wire.begin();

  bool is_initialized = tof.initialize();
  String msg;

  is_initialized ? msg = "tof initilized" : msg = "tof not initialized";
  Serial.println(msg);

  if (!is_initialized) while(true) {};

  led.off();
  delay(500);
  led.on();
  delay(2000);
  led.off();
  delay(500);
}

void loop() {
  int dist_mm = tof.readDist();

  Serial.print("Dist (MM): ");
  Serial.println(dist_mm);
  delay(100);
}