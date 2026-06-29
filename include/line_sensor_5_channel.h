#ifndef LINE_SENSOR_5_CHANNEL_H
#define LINE_SENSOR_5_CHANNEL_H
#include <Arduino.h>

class LineSensor5Channel
{
   public:
    LineSensor5Channel(int pin0, int pin1, int pin2, int pin3, int pin4);
    int readSensorLine(int sensor_num);
    byte readFullValue();
    
   private:
    int _sensorPin[5];
};

LineSensor5Channel::LineSensor5Channel(int pin0, int pin1, int pin2, int pin3, int pin4){
    _sensorPin[0] = pin0;
    _sensorPin[1] = pin1;
    _sensorPin[2] = pin2;
    _sensorPin[3] = pin3;
    _sensorPin[4] = pin4;

    for (int i=0; i<5; i++) {
        pinMode(_sensorPin[i], INPUT);
    }
}

int LineSensor5Channel::readSensorLine(int sensor_num){
    if (digitalRead(_sensorPin[sensor_num])) return 1;
    else return 0;
}

byte LineSensor5Channel::readFullValue(){
    int sensor_val[5];

    for (int i=0; i<5; i++) {
        sensor_val[i] = readSensorLine(i);
    }

    byte full_sensor_val = (sensor_val[0] << 4) | (sensor_val[1] << 3) | (sensor_val[2] << 2) | (sensor_val[3] << 1) | sensor_val[4];
    return full_sensor_val;
}

#endif