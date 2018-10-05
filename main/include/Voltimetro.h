#ifndef __VOLTIMETRO__
#define __VOLTIMETRO__


//#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NIVEL_LOGICO 5.0
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
#define SPEAKER_PIN    GPIO_NUM_18

#define HIGH 1
#define LOW  0

#define V_MIN 9.0
#define MEASURE_TIME 1000


class Voltimetro
{
  protected:
    // Valor resistor 1 divisor de tensão
    unsigned long int r1;
    // Valor resistor 2 divisor de tensão
    unsigned long int r2;
    // Fator de conversão
    float factor;
    uint32_t  adc_reading;
    float read_Voltage;

    float analogRead();

  public:
    // Método construtor da classe
    Voltimetro( float _r1, float _r2 );
    // Retorna a tensão da bateria
    float getVoltage();
    void perform_reading(void*);
};
#endif
