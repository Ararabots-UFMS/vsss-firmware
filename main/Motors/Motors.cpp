#include "Motors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

Motor::Motor(gpio_num_t _in1, gpio_num_t _in2, gpio_num_t _pwmPin, ledc_channel_t _pwmChannel)
{
  
  // Atribuicao dos pinos
  in1 = _in1;
  in2 = _in2;
  pwmPin = _pwmPin;
  pwmChannel = _pwmChannel;

  ledc_channel_config_t ledc_channel;
  ledc_channel.gpio_num = pwmPin;
  ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_channel.channel = pwmChannel;
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.timer_sel = MOTOR_PWM_TIMER;
  ledc_channel.duty = 0;

  ledc_timer_config_t ledc_timer;
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_timer.bit_num = MOTOR_PWM_BIT_NUM;
  ledc_timer.timer_num = MOTOR_PWM_TIMER;
  ledc_timer.freq_hz = PWM_FREQ;


   ESP_ERROR_CHECK( ledc_channel_config(&ledc_channel) );
   ESP_ERROR_CHECK( ledc_timer_config(&ledc_timer) );

  // Definicao dos pinos de saida, e do controle de pwm com ledc
  gpio_pad_select_gpio(in1);
  gpio_set_direction(in1, GPIO_MODE_OUTPUT);

  gpio_pad_select_gpio(in2);
  gpio_set_direction(in2, GPIO_MODE_OUTPUT);

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
  ESP_ERROR_CHECK( ledc_set_duty(LEDC_HIGH_SPEED_MODE, pwmChannel, 0) );
  ESP_ERROR_CHECK( ledc_update_duty(LEDC_HIGH_SPEED_MODE, pwmChannel) );
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
  ESP_ERROR_CHECK( ledc_set_duty(LEDC_HIGH_SPEED_MODE, pwmChannel, pwm) );
  ESP_ERROR_CHECK( ledc_update_duty(LEDC_HIGH_SPEED_MODE, pwmChannel) );
  // Atualiza o sentido
  sentidoAtual = sentido;
}
