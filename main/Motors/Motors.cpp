#ifndef __MOTORS__
#include "Motors.h"
#endif


#include <Arduino.h>


Motor::Motor(unsigned char _in1, unsigned char _in2, unsigned char _pwmPin)
{
  // Atribuicao dos pinos
  in1 = _in1;
  in2 = _in2;
  pwmPin = _pwmPin;

  // Define os pinos como saida
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  // Inicialização
  init();
}

void Motor::init()
{
  // Escrever 5 para travar o motor
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  analogWrite(pwmPin, 0);

  sentidoAtual = -1;
}

void Motor::enable(unsigned char pwm, bool sentido)
{
  // Para o motor antes de inverter o sentido
  if(sentidoAtual != -1 && sentido != sentidoAtual)
  {
    init();
  }

  digitalWrite(in1, sentido);
  digitalWrite(in2, !sentido);
  analogWrite(pwmPin, pwm);

  // Atualiza o sentido
  sentidoAtual = sentido;

  // if(direcao == HORARIO)
  // {
  //   pinMode(in1, OUTPUT);
  //   digitalWrite(in1, 1);
  //   analogWrite(in2, 255 - pwm);
  //   sentido = HORARIO;
  // }
  // else
  // {
  //   pinMode(in2, OUTPUT);
  //   digitalWrite(in2, 1);
  //   analogWrite(in1, 255-pwm);
  //   sentido = ANTIHORARIO;
  // }

  // if(direcao)
  //   sentido = HORARIO;
  // else
  //   sentido = ANTIHORARIO;
  //
  //   digitalWrite(in2, direcao);
  //   analogWrite(in1, pwm);

}
