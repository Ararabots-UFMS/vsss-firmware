#include <math.h>
#include <stdint.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/gpio.h"

// Should debug?
#define DEBUG

// Robot name and tag for debug
#define SPP_TAG "Dumbo"
#define SPP_SERVER_NAME "SPP_SERVER"
#define DEVICE_NAME "Dumbo"

// GYROSCOPE CONSTANTS
#define GYRO_SDA_PIN GPIO_NUM_16
#define GYRO_SCL_PIN GPIO_NUM_5
#define GYRO_CLOCK_SPEED 400000
#define GYRO_INT_PIN 4
#define GYRO_SAMPLE_RATE 250

// MOTOR CLASS CONSTANTS
#define PWM_FREQ 20000
#define HORARIO 1
#define ANTIHORARIO 0

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

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
#define SPEAKER_PIN    	GPIO_NUM_19
#define LED_PIN			GPIO_NUM_22


#define ADC_R1	100000  // 100K
#define ADC_R2	10000   // 10k

#define HIGH	1
#define LOW		0
#define R1 100150
#define R2 9910

#define V_MIN 			9.9
#define MEASURE_TIME 	60000 // 60 seconds in millis, delay measure takes a minute

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

#define PIDERRO 0.5

#ifndef __STRUCTS__
#define __STRUCTS__
typedef struct motorPackage{
	uint8_t theta = 0; // Angle
	uint8_t speed_l = 0; // Left speed
	uint8_t direction = 0; // direcao das rodas
	uint8_t speed_r = 0; // right speed
	uint8_t control_type = 1; //0 angle and speed correction / 1 right speed and left speed
    uint8_t rotation_direction = 0; // 0 horario / 1 anti-horario
} motorPackage;

typedef struct controlPackage{
	float KP = 0.0;
	float KI = 0.0;
	float KD = 0.0;
} controlPackage;
#endif
