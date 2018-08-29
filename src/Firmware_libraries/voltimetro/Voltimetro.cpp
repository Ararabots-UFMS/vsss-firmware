#ifndef __VOLTIMETRO__
#include "Voltimetro.h"
#endif

#define NIVEL_LOGICO 3.3

#include <Arduino.h>
#include <driver/adc.h>

Voltimetro::Voltimetro(uint8_t _analogPin, unsigned long int _r1,
                       unsigned long int _r2)
{
  adc1_config_width(ADC_WIDTH_10Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_11db);

  analogPin = _analogPin;
  r1 = _r1;
  r2 = _r2;
  factor = (float) (r1 + r2) / (float) (r2);
}

float Voltimetro::getVoltage()
{
  float Vout;
  int read = adc1_get_voltage(ADC1_CHANNEL_5);
  Vout =(NIVEL_LOGICO * read * factor)/1023.0;



  return Vout;

}
