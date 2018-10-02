#ifndef __PIDCONTROLLER__

#define __PIDCONTROLLER__

#include <stdint.h>
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
    void updateTime()
    {
      updateTime(millis());
    }
    // Retorna valor pwm
    float control();
};
#endif
