#ifndef __PIDCONTROLLER__

#define __PIDCONTROLLER__

#include <stdint.h>
#include "esp_timer.h"
#include "Memory.h"

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
    char  KP_Key[3] = "KP",
          KI_Key[3] = "KI",
          KD_Key[3] = "KD";
    Memory* mem = NULL;

  public:
    // Constante da proporcional
    float kP;
    // Constante de integral
    float kI;
    // Constante da derivada
    float kD;

    // construtor
    PIDCONTROLLER(float _kI, float _kD, float _kP);
    PIDCONTROLLER();

    void  load_params();

    // Atualiza a leitura
    void updateReading(float _reading);

    // Define o objetivo
    void setGoal(float _goal = 0);

    // Retorna o objetivo
    float goal_();

    // Atualiza a ultima vez em que foi executado
    void updateTime(unsigned long int t);

    void set_PID(float,float,float);

    // Retorna valor pwm
    float control();
};
#endif
