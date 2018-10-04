#ifndef __VOLTIMETRO__
#define __VOLTIMETRO__

#define NIVEL_LOGICO 5.0
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

#define SPEAKER_PIN    18

#define V_MIN 9.0
#define MEASURE_TIME 1000

//#include <Arduino.h>
#include "driver/gpio.h"

class Voltimetro
{
  protected:
    // Pino de leitura
    uint8_t analogPin;
    // Valor resistor 1 divisor de tensão
    unsigned long int r1;
    // Valor resistor 2 divisor de tensão
    unsigned long int r2;
    // Fator de conversão
    float factor;

  public:
    // Método construtor da classe
    Voltimetro(uint8_t, unsigned long int, unsigned long int);
    // Retorna a tensão da bateria
    float getVoltage();
};
#endif
