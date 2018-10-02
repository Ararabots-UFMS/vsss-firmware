#ifndef __VOLTIMETRO__
#include "Voltimetro.h"
#endif

#define NIVEL_LOGICO 5.0

#include <Arduino.h>

Voltimetro::Voltimetro(uint8_t _analogPin, unsigned long int _r1,
                       unsigned long int _r2)
{
  analogPin = _analogPin;
  r1 = _r1;
  r2 = _r2;
  factor = (float) (r1 + r2) / (float) r2;
}

float Voltimetro::getVoltage()
{
  float Vout;

  Vout = (NIVEL_LOGICO * analogRead(analogPin) * factor) / 1023.0;

  return Vout;

}
