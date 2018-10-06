#include <math.h>
#include <stdint.h>
#include "driver/ledc.h"
#include "esp_err.h"

// MOTOR CLASS CONSTANTS
#define ANTIHORARIO 0
#define HORARIO 1
#define PWM_FREQ 20000

#define AIN1 GPIO_NUM_26
#define AIN2 GPIO_NUM_25
#define PWMA GPIO_NUM_32
#define MOTOR_PWM_CHANNEL_A LEDC_CHANNEL_1

#define BIN1 GPIO_NUM_27
#define BIN2 GPIO_NUM_14
#define PWMB GPIO_NUM_12
#define MOTOR_PWM_CHANNEL_B LEDC_CHANNEL_2

#define MOTOR_PWM_TIMER LEDC_TIMER_1
#define MOTOR_PWM_BIT_NUM LEDC_TIMER_8_BIT

#define NIVEL_LOGICO 5.0
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
#define SPEAKER_PIN    GPIO_NUM_18

#define HIGH 1
#define LOW  0

#define V_MIN 9.0
#define MEASURE_TIME 1000
#define GPIO_PWM0A_OUT 15

#ifndef __STRUCTS__
#define __STRUCTS__
typedef struct motorPackage{
	uint8_t theta = 0; // Angle
	uint8_t speed_l = 0; // Left speed
	uint8_t direction = 0; 
	uint8_t speed_r = 0; // right speed
	uint8_t control_type = 0; //0 angle and speed correction / 1 right speed and left speed
} motorPackage;

typedef struct controlPackage{
	float KP = 0.0;
	float KI = 0.0;
	float KD = 0.0;
} controlPackage;
#endif