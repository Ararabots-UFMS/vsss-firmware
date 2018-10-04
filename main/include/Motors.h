#ifndef __MOTORS__
#define __MOTORS__

//#include <Arduino.h>

#define ANTIHORARIO 0
#define HORARIO 1

class Motor
{
  protected:
    // Pinos de controle do motor
    unsigned char in1;
    unsigned char in2;
    unsigned char pwmPin;

    // Salva o sentido de rotação do motor
    // 0 ANTIHORARIO
    // 1 HORARIO
    // -1 INICIALIZACAO
    char sentidoAtual;
    // Inicializa o motor em ponto morto
    void init();

  public:
    // Metodo construtor da classe
    Motor(unsigned char, unsigned char, unsigned char);

    // Funcao ativa o motor em uma velocidade e um sentido
    void enable(unsigned char, bool);
};

#endif
