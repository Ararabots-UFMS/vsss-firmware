#ifndef __MOTOR__
#define __MOTOR__

#include "definitions.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"

class Motor
{
  protected:
    // Pinos de controle do motor
    gpio_num_t in1; //input de sentido
    gpio_num_t in2; //input de sentido
    gpio_num_t pwmPin; //input de pwm dos motores
    ledc_channel_t pwmChannel;

    // Salva o sentido de rotação do motor
    // 0 ANTIHORARIO
    // 1 HORARIO
    // -1 INICIALIZACAO
    char sentidoAtual;
    // Inicializa o motor em ponto morto
    void init();

  public:
    

    Motor(gpio_num_t _in1, gpio_num_t _in2, gpio_num_t _pwmPin, ledc_channel_t _pwmChannel);


    void enable(unsigned char, bool);
};

#endif
