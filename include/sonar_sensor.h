#ifndef SONAR_SENSOR_H
#define SONAR_SENSOR_H

#include <Arduino.h>
#include <Ultrasonic.h>
#include <movingAvg.h> 

class SonarSensor : Ultrasonic, movingAvg {
  public:
    SonarSensor(int trig_pin, int echo_pin, int timeout_ms)
     : Ultrasonic(trig_pin, echo_pin, timeout_ms), movingAvg(10) {
      begin(); //from the movingAvg class
     }

  int readDist() {
    int dist_mm = (int)(convert(timing(), Ultrasonic::CM)*10);
    int dist_filter = reading(dist_mm);
    return dist_filter;
  }
};

#endif