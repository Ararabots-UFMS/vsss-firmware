#ifndef __ PIDCONTROLLER__
#include "PIDController.h"
#endif

#include <stdint.h>
#include <Arduino.h>

PIDCONTROLLER::PIDCONTROLLER(float _kP, float _kI, float _kD)
{
  kP = _kP;
  kI = _kI;
  kD = _kD;

  previousReading = 0;
}

void PIDCONTROLLER::updateReading(float _reading)
{
  previousReading = reading;
  reading = _reading;
}

void PIDCONTROLLER::setGoal(float _goal)
{
  goal = _goal;
}

void PIDCONTROLLER::updateTime(unsigned long int t)
{
  lastTime = t;
}

float PIDCONTROLLER::control()
{
  float deltaT;

  // Salva o tempo do ultimo controle
  deltaT = lastTime;
  // Atualiza o tempo de controle para o tempo atual
  updateTime();
  // Normaliza o valor de deltaT, pois a leitura e feita em milisegundos
  deltaT = (lastTime - deltaT) / 1000.0;

  // Calcula o erro
  error = goal - reading;

  // proporcional
  P = kP * error;
  // Integral
  I += kI * error * deltaT;
  // Derivada
  D = kD * (reading - previousReading) / deltaT;

  // returns pwm control
  return P+I+D;
}
