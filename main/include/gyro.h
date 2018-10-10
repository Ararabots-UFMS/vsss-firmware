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

static void mpuISR(void*);
static void mpuTask(void*);
static void printTask(void*);

class Gyro
{

	const MPU_t MPU;
	float roll{0}, pitch{0}, yaw{0};
	uint32_t notificationValue;
	uint16_t fifocount;
	
	float gyroRoll,
		  gyroPitch,
		  gyroYaw,
		  accelRoll,
		  accelPitch;

	I2C_t& i2c = i2c0;

public:

	Gyro();
	void read(float * , float* , float* );
	~Gyro();
	
};