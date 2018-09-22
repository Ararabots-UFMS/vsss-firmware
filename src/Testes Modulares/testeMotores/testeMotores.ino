#include "Defines.h"

#include <Motors.h>

Motor motorA = Motor(INA1, INA2, PWMA, PWM_CHANNEL_A);
Motor motorB = Motor(INB1, INB2, PWMB, PWM_CHANNEL_B);

void setup() {
  pinMode(22, OUTPUT);
  digitalWrite(22, HIGH);
}

void loop() {
 motorB.enable(255,1);
 delay(1500);

 motorB.enable(128,1);
 delay(1500);

 motorB.enable(64,1);
 delay(1500);

 motorB.enable(32,1);
 delay(1500);
 
}
