#ifndef __PIDCONTROLLER__
#include "PIDController.h"
#endif

#include <stdint.h>
#include "definitions.h"
#include "Memory.h"
#include "esp_log.h"
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
    kP = 0;
    kI = 0;
    kD = 0;
    previousReading = 0;
}


void PIDCONTROLLER::set_PID(float _kP, float _kI, float _kD){
  kP = _kP;
  kI = _kI;
  kD = _kD;

  // TODO: entender esse trecho
  control_package.KP = _kP;
  control_package.KI = _kI;
  control_package.KD = _kD;

  mem->open_handle();
  mem->update_memory(KP_Key, kP);
  mem->update_memory(KI_Key, kI);
  mem->update_memory(KD_Key, kD);
  mem->close_handle();
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

void  PIDCONTROLLER::load_params(){
    mem = new Memory();
    kP = mem->read_memory(KP_Key);
    kI = mem->read_memory(KI_Key);
    kD = mem->read_memory(KD_Key);
    mem->close_handle();
    #ifdef DEBUG
        ESP_LOGI("PID","P:%f I:%f D:%f",kP,kI,kD);
    #endif
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
