#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H
#include <Arduino.h>

#include <Wire.h>
#include <VL53L0X.h>
#include <movingAvg.h>  

VL53L0X tof_sensor;
movingAvg tof_sensor_filter(10);

void tof_sensor_init(){
  tof_sensor.setTimeout(500);
  if (!tof_sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  tof_sensor.setMeasurementTimingBudget(50000);
  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  tof_sensor.startContinuous();
  tof_sensor_filter.begin();
}

int tof_sensor_get_dist_mm(){
  int dist_mm = tof_sensor.readRangeContinuousMillimeters() - 10;
  int dist_filter = tof_sensor_filter.reading(dist_mm);
  if (tof_sensor.timeoutOccurred()) { 
    return 0; 
  }
  return dist_filter;
}

#endif