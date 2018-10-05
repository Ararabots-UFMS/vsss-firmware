#include "Motors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

Motor::Motor(gpio_num_t _in1, gpio_num_t _in2, int _pwmPin, mcpwm_io_signals_t _pwmChannel)
{
  
  // Atribuicao dos pinos
  in1 = _in1;
  in2 = _in2;
  pwmPin = _pwmPin;
  pwmChannel = _pwmChannel;
  mcpwm_config_t pwm_config;

  // Definicao dos pinos de saida, e do controle de pwm com ledc
  gpio_pad_select_gpio(in1);
  gpio_set_direction(in1, GPIO_MODE_OUTPUT);

  gpio_pad_select_gpio(in2);
  gpio_set_direction(in2, GPIO_MODE_OUTPUT);
  mcpwm_gpio_init(MCPWM_UNIT_0, pwmChannel, pwmPin);

  pwm_config.frequency = 1000;    //frequency = 500Hz,
  pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0;    //duty cycle of PWMxA = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  // Inicialização
  init();
}


  /*
	inicializa os motores
	com valor de pwm zero
	e os motores travados
  */
void Motor::init()
{
  // Escrever 5 para travar o motor
  gpio_set_level(in1, HIGH);
  gpio_set_level(in2, HIGH);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  sentidoAtual = -1;
}


    // Funcao ativa o motor em uma velocidade e um sentido
    // - parametros de entrada:
    //   - pwm: velocidade de roda 0 - 255
    //   - sentido: sentido de rotacao: 1-HORARIO 0-ANTIHORARIO
void Motor::enable(unsigned char pwm, bool sentido)
{
  // Para o motor antes de inverter o sentido
  if(sentidoAtual != -1 && sentido != sentidoAtual)
  {
    init();
  }

  gpio_set_level(in1, sentido);
  gpio_set_level(in2, !sentido);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pwm);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  // Atualiza o sentido
  sentidoAtual = sentido;
}
