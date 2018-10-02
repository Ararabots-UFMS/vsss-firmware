#ifndef __VOLTIMETRO__
#define __VOLTIMETRO__

#include <Arduino.h>


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
