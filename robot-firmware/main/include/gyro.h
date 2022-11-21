#ifndef __GYRO__

#define __GYRO__

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"

#include "I2Cbus.hpp"

#include "definitions.h"

class Gyro
{

	MPU_t MPU; // Driver variable
	float roll{0}, pitch{0}, yaw{0}; // Reading variables
	uint32_t notificationValue; // Variable for checking debug rate on terminal
	uint16_t fifocount; // Count variable for checking if sample rate is too high
	
	// Gyro and Acelerometer variables
	float gyroRoll,
		  gyroPitch,
		  gyroYaw,
		  accelRoll,
		  accelPitch;
	// End of reading variables

	TaskHandle_t giroHandle;

	I2C_t& i2c = i2c0; // Port of gyroscope, can be 0 or 1

public:

	Gyro();
	void read(float * , float* , float* ); // Reading function
	void update_yaw(float*);
	//~Gyro();
	
};
#endif