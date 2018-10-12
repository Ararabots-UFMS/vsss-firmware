#include <math.h>
#include "driver/ledc.h"
#include "driver/gpio.h"



// MOTOR CLASS CONSTANTS
#define ANTIHORARIO 	0
#define HORARIO 		1
#define PWM_FREQ        1000
#define PWM_RESOLUTION  8

#define MOTOR_PWM_CHANNEL_LEFT 		LEDC_CHANNEL_1
#define MOTOR_PWM_CHANNEL_RIGHT 	LEDC_CHANNEL_2
#define MOTOR_PWM_TIMER 			LEDC_TIMER_1
#define MOTOR_PWM_BIT_NUM 			LEDC_TIMER_8_BIT

#define NIVEL_LOGICO	3.3
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
#define SPEAKER_PIN    	GPIO_NUM_19
#define LED_PIN			GPIO_NUM_22


#define ADC_R1	100000  // 100K
#define ADC_R2	10000   // 10k

#define HIGH	1
#define LOW		0

#define V_MIN 			9.9
#define MEASURE_TIME 	60000 // 60 seconds in millis, delay measure takes a minute
#define GPIO_PWM0A_OUT 	15

#define SDA					GPIO_NUM_16
#define SCL					GPIO_NUM_5
#define MPU6050_INTERRUPT	4

#define DUTY_CYCLE_10		0.1
#define DUTY_CYCLE_30		0.3
#define DUTY_CYCLE_40		0.4
#define DUTY_CYCLE_50		0.5
#define DUTY_CYCLE_60		0.6
#define DUTY_CYCLE_80		0.8
#define DUTY_CYCLE_100		1.0

#define FREQ_1		1
#define FREQ_2 		0.5
#define FREQ_4		0.25
#define FREQ_6		0.1667
#define FREQ_8		0.125
#define FREQ_10		0.1
#define FREQ_12		0.0833

#define CORE_ZERO	0 
#define CORE_ONE	1 

#define TASK_SIZE 40000

