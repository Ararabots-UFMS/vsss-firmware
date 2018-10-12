#ifndef __MEMORYESP__

#define __MEMORYESP__

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"

class Memory{
	protected:
		char * value;
		char * key;
		nvs_handle * handle;
		esp_err_t error;

	public:
		Memory();
		double read_memory(char * key);
		int update_memory(char * key, double value);
		int delete_memory(char * key);
		int close_handle();
		char * str_convert(double number);
		double str_to_double(char * number);
	
};

#endif