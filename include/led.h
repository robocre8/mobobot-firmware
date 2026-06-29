#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>


class Led
{
   public:
    Led(int pin);
    void on();
    void off();
    bool status();
    void blink(int blinkPeriod);
    
   private:
    int _pin;
    bool _status;
};


Led::Led(int pin)
{
  _pin = pin;
  pinMode(_pin, OUTPUT);
}

void Led::on()
{
  digitalWrite(_pin, HIGH);
  _status = true;
}

void Led::off()
{
  digitalWrite(_pin, LOW);
  _status = false;
}

bool Led::status()
{
  return _status;
}

void Led::blink(int blinkPeriod)
{
  static long lastMillis = 0;
  long currentMillis = millis();
  long elapsed = currentMillis - lastMillis;
  if (elapsed > blinkPeriod) 
  {
      if(_status)
          off();
      else
          on();
      lastMillis = currentMillis;
  }
}

#endif