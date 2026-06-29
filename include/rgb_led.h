#ifndef RGB_LED_CONTROL_H
#define RGB_LED_CONTROL_H

#include <Arduino.h>


class RGBLed
{
   public:
    RGBLed(int r_pin, int g_pin, int b_pin);
    void on(int r_pwm, int g_pwm, int b_pwm);
    void off();
    void red();
    void green();
    void blue();
    void white();
    
   private:
    int _r_pin, _b_pin, _g_pin;
};

RGBLed::RGBLed(int r_pin, int g_pin, int b_pin) {
    _r_pin = r_pin;
    _g_pin = g_pin;
    _b_pin = b_pin;

    pinMode(_r_pin, OUTPUT);
    pinMode(_g_pin, OUTPUT);
    pinMode(_b_pin, OUTPUT);

    analogWriteFrequency(1000);

    analogWrite(_r_pin, 255);
    analogWrite(_g_pin, 255);
    analogWrite(_b_pin, 255);
}

void RGBLed::on(int r_pwm=255, int g_pwm=255, int b_pwm=255) {
  analogWrite(_r_pin, 255-r_pwm);
  analogWrite(_g_pin, 255-g_pwm);
  analogWrite(_b_pin, 255-b_pwm);
}

void RGBLed::off() {
  analogWrite(_r_pin, 255);
  analogWrite(_g_pin, 255);
  analogWrite(_b_pin, 255);
}

void RGBLed::red() {
  on(255, 0, 0);
}

void RGBLed::green() {
  on(0, 255, 0);
}

void RGBLed::blue() {
  on(0, 0, 255);
}

void RGBLed::white() {
  on(255, 255, 255);
}

#endif