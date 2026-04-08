#ifndef IR_SENSOR_H
#define IR_SENSOR_H
#include <Arduino.h>

class IRSensor
{
   public:
    IRSensor(int pin);
    int read();
    
   private:
    int _sensorPin;
    
};

IRSensor::IRSensor(int pin){
    _sensorPin = pin;
    pinMode(_sensorPin, INPUT);
}

int IRSensor::read(){
    if (digitalRead(_sensorPin)) return 0;
    else return 1;
}

#endif