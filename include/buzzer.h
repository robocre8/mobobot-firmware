#ifndef BUZZER_CONTROL_H
#define BUZZER_CONTROL_H
#include <Arduino.h>


class Buzzer
{
   public:
    Buzzer(int pin) {
      _pin = pin;
      pinMode(_pin, OUTPUT);
      analogWrite(_pin, 0);
    }

    void on(int pwm=50) {
      int PWM = constrain(pwm, 0, 200);
      analogWrite(_pin, PWM);
      _status = true;
    }

    void off() {
      analogWrite(_pin, 0);
      _status = false;
    }

    bool status() {
      return _status;
    }

    void beep(int beepPeriod) {
      static long lastBeepMillis = 0;
      long currentBeepMillis = millis();
      long elapsed = currentBeepMillis - lastBeepMillis;
      if (elapsed > beepPeriod) 
      {
          if(_status)
              off();
          else
              on();
          lastBeepMillis = currentBeepMillis;
      }
    }
    
   private:
    int _pin;
    bool _status;
};

#endif