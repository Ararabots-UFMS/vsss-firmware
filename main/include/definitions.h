#include <math.h>
#include "driver/ledc.h"
#include "esp_err.h"

// MOTOR CLASS CONSTANTS
#define ANTIHORARIO 0
#define HORARIO 1
#define PWM_FREQ        1000
#define PWM_RESOLUTION  8

#define MOTOR_PWM_CHANNEL_LEFT LEDC_CHANNEL_1
#define MOTOR_PWM_CHANNEL_RIGHT LEDC_CHANNEL_2
#define MOTOR_PWM_TIMER LEDC_TIMER_1
#define MOTOR_PWM_BIT_NUM LEDC_TIMER_8_BIT

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
#define SPEAKER_PIN    	GPIO_NUM_18

#define HIGH 1
#define LOW  0

#define V_MIN 9.0
#define MEASURE_TIME 1000
#define GPIO_PWM0A_OUT 15

#define R1 100150
#define R2 9910

#define TASK_SIZE 40000