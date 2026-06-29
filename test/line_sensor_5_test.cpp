#include <Arduino.h>
#include <line_sensor_5_channel.h>

const int pin0 = 36;
const int pin1 = 39;
const int pin2 = 34;
const int pin3 = 32;
const int pin4 = 13;

LineSensor5Channel line_sensor(pin0, pin1, pin2, pin3, pin4);

int sensor_val[5];

void setup() {
  Serial.begin(115200);

}

void loop() {
  for (int i=0; i<5; i++) {
    sensor_val[i] = line_sensor.readSensorLine(i);
    Serial.print(sensor_val[i]);
  }
  Serial.println();

  byte result = line_sensor.readFullValue();
  Serial.print("Binary value: ");
  Serial.println(result, BIN);
  Serial.println();


  delay(100);
}