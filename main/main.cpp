/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#define SPP_TAG "Eymael"

#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "PIDController.h"

#include "bluetooth.h"

#include "Motors.h"
#include "Voltimetro.h"
#include "time.h"
#include "sys/time.h"
#include "driver/mcpwm.h"
#include "definitions.h"
#include "esp_log.h"
struct motorPackage motor_package;
struct controlPackage control_package; 
Motor motor_left = Motor(AIN1, AIN2, PWMA, MOTOR_PWM_CHANNEL_A);
Motor motor_right = Motor(BIN1, BIN2, PWMB, MOTOR_PWM_CHANNEL_B);

PIDCONTROLLER pid_controller = PIDCONTROLLER(0,0,0);

void motor_control_task(void *pvParameter)
{
	while(1)
	{
		if(!motor_package.control_type)
		{
			//pid control
		}
		else
		{	
			motor_left.enable(motor_package.speed_l, motor_package.direction/2);
			motor_right.enable(motor_package.speed_r, motor_package.direction%2);	
		}
	}
}

extern "C" {
void app_main();
}

void app_main()
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    setup_bluetooth();

	    xTaskCreatePinnedToCore(&motor_control_task, "motor_control_task", 75000, NULL, 5, NULL, 1);

}
