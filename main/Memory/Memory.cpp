#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"

#include "Memory.h"
#include <stdlib.h>
#include <string.h>

#define TRUE 1
#define FALSE 0
#define CHARACTER_SIZE 20

Memory::Memory(){
	// Create variable for access to storage of esp
	handle =(nvs_handle *) malloc(sizeof(nvs_handle));
	
	// Init nvs flash
	error = nvs_flash_init();

	// Open function of storage
	error = nvs_open("storage", NVS_READWRITE, handle);

	if (error != ESP_OK)
		printf("Error (%s) opening NVS handle\n", esp_err_to_name(error));
}

double Memory::read_memory(char * key){
	// Create variable of returns
	char * memory = (char *) malloc(sizeof(char) * CHARACTER_SIZE);
	// Set the first position with \0
	memory[0] = 0;
	
	// Set the sizeof double in string 
	size_t size = (sizeof(char) * CHARACTER_SIZE);

	// Get value of memory
	error = nvs_get_str(*handle, key, memory, &size);

	if(error != ESP_OK){
		if (error == ESP_ERR_NVS_NOT_FOUND){
			printf("The value is not initialized yet!\n");
		}else{
            printf("Error (%s) reading!\n", esp_err_to_name(error));
		}
	}

	return str_to_double(memory);
}

int Memory::update_memory(char * key, double value){
	char * memory = (char *) malloc(sizeof(char) * CHARACTER_SIZE);

	strcpy(memory, str_convert(value));

	// Update value in memory
	error = nvs_set_str(*handle, key, memory);
	// IF update is ok
    return ((error != ESP_OK) ? FALSE : TRUE);
}

int Memory::delete_memory(char * key){
	error = nvs_erase_key(*handle, key);

    return ((error != ESP_OK) ? FALSE : TRUE);
}

int Memory::open_handle(){
	error = nvs_open("storage", NVS_READWRITE, handle);

	if (error != ESP_OK)
		printf("Error (%s) opening NVS handle\n", esp_err_to_name(error));
}

int Memory::close_handle(){
	// Commit written value
	error = nvs_commit(*handle);

	// Close handle
	nvs_close(*handle);
    
    return ((error != ESP_OK) ? FALSE : TRUE);
}

char * Memory::str_convert(double number){
	// Create variable of returns
	char * memory = (char *) malloc(sizeof(char) * CHARACTER_SIZE);
	// Set the first position with \0
	memory[0] = 0;
	// Copy float to string
	sprintf(memory, "%lf", number);

	return memory;
}

double Memory::str_to_double(char * number){
	// Convert string to float
	double num = atof(number);
	return num;
}