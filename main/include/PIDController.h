#ifndef __PIDCONTROLLER__

#define __PIDCONTROLLER__

#include <stdint.h>
#include "esp_timer.h"

//#include <Arduino.h>


class PIDCONTROLLER
{
  protected:
    float error;
    // Leitura mais atual do sensor
    float reading;
    // Leitura anterior do sensor
    float previousReading;
    float P;
    float I;
    float D;
    float goal;
    float lastTime;

  public:
    // Constante da proporcional
    float kP;
    // Constante de integral
    float kI;
    // Constante da derivada
    float kD;

    // construtor
    PIDCONTROLLER(float _kI, float _kD, float _kP);
    // Atualiza a leitura
    void updateReading(float _reading);
    // Define o objetivo
    void setGoal(float _goal = 0);
    // Atualiza a ultima vez em que foi executado
    void updateTime(unsigned long int t);
    void set_PID(float,float,float);
    void updateTime()
    {
      updateTime(esp_timer_get_time()*1000);
    }
    // Retorna valor pwm
    float control();
};
#endif
