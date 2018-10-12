#ifndef __PIDCONTROLLER__
#include "PIDController.h"
#endif

#include <stdint.h>

#include "Memory.h"
//#include <Arduino.h>

extern controlPackage control_package;

PIDCONTROLLER::PIDCONTROLLER(float _kP, float _kI, float _kD)
{
  kP = _kP;
  kI = _kI;
  kD = _kD;

  previousReading = 0;
}

PIDCONTROLLER::PIDCONTROLLER()
{
  kP = mem.read_memory("KP");
  kI = mem.read_memory("KI");
  kD = mem.read_memory("KD");

  previousReading = 0;
}


void PIDCONTROLLER::set_PID(float _kP, float _kI, float _kD){
  kP = _kP;
  kI = _kI;
  kD = _kD;
  control_package.kP = _kP;
  control_package.kI = _kI;
  control_package.kD = _kD;

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
  updateTime(esp_timer_get_time());
  // Normaliza o valor de deltaT, pois a leitura e feita em microssegundos
  deltaT = (lastTime - deltaT);

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
