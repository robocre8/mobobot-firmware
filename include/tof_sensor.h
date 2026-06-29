#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <movingAvg.h>

class TOFSensor : VL53L0X, movingAvg {
  public:
    TOFSensor()
     : VL53L0X(), movingAvg(10) {
      begin(); //from the movingAvg class
     }

    // VL53L0X initialization
    bool initialize() {
      setTimeout(500);
      if(!init()){
        return false;
      }
      setMeasurementTimingBudget(50000);
      // Start continuous back-to-back mode (take readings as
      // fast as possible).  To use continuous timed mode
      // instead, provide a desired inter-measurement period in
      // ms (e.g. sensor.startContinuous(100)).
      startContinuous();

      return true;
    }

  int readDist() {
    int dist_mm = readRangeContinuousMillimeters() - 10;
    int dist_filter = reading(dist_mm);
    if (timeoutOccurred()) { 
      return 0; 
    }
    return dist_filter;
  }
};

#endif



// #ifndef TOF_SENSOR_H
// #define TOF_SENSOR_H
// #include <Arduino.h>

// #include <Wire.h>
// #include <VL53L0X.h>
// #include <movingAvg.h>  

// VL53L0X tof_sensor;
// movingAvg tof_sensor_filter(10);

// void tof_sensor_init(){
//   tof_sensor.setTimeout(500);
//   if (!tof_sensor.init())
//   {
//     Serial.println("Failed to detect and initialize sensor!");
//     while (1) {}
//   }

//   tof_sensor.setMeasurementTimingBudget(50000);
//   // Start continuous back-to-back mode (take readings as
//   // fast as possible).  To use continuous timed mode
//   // instead, provide a desired inter-measurement period in
//   // ms (e.g. sensor.startContinuous(100)).
//   tof_sensor.startContinuous();
//   tof_sensor_filter.begin();
// }

// int tof_sensor_get_dist_mm(){
//   int dist_mm = tof_sensor.readRangeContinuousMillimeters() - 10;
//   int dist_filter = tof_sensor_filter.reading(dist_mm);
//   if (tof_sensor.timeoutOccurred()) { 
//     return 0; 
//   }
//   return dist_filter;
// }

// #endif