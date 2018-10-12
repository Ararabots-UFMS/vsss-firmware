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
#include "driver/gpio.h"
#include "Utils.h"
#include "esp_log.h"
struct motorPackage motor_package;
struct controlPackage control_package; 
Motor motor_left = Motor(AIN1, AIN2, PWMA, MOTOR_PWM_CHANNEL_A);
Motor motor_right = Motor(BIN1, BIN2, PWMB, MOTOR_PWM_CHANNEL_B);

uint8_t lastTheta = -1;
uint8_t lastDirection = -1;

PIDCONTROLLER pid_controller = PIDCONTROLLER(0,0,0);

void motor_control_task(void *pvParameter)
{
	while(1)
	{
		if(!motor_package.control_type)
		{
			//pid control
			// if(motor_package.tetha != lastTheta || motor_package.direction != lastDirection)
			// {
				// lastTheta = motor_package.tetha;
				// lastDirection = motor_package.direction;
				// if(motor_package.direction)
				// pid_controller.setGoal(######GIRO##### + (motor_package.direction - 1))
			// }
				// pid_controller.updateReading(####GIRO#####)
		}
		else
		{	
			motor_left.enable(motor_package.speed_l, motor_package.direction >> 1);
			motor_right.enable(motor_package.speed_r, motor_package.direction & 1);	
		}
	}
}

void voltimetro(void * pvParamters){
    Voltimetro voltimetro(R1,R2);

    /* Select the GPIO to be used */
    gpio_pad_select_gpio(SPEAKER_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(SPEAKER_PIN, GPIO_MODE_OUTPUT);


    while(true){

        // If the read voltage is less than 9 volts it activates the buzzer
        if (voltimetro.getVoltage() < V_MIN) {
            gpio_set_level(SPEAKER_PIN, HIGH);
        }
        else {
            gpio_set_level(SPEAKER_PIN, LOW);
        }
        vTaskDelay(MEASURE_TIME);
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
    // setup_bluetooth();
  
    
    
    while(1){
      

      
    
      vTaskDelay(3000 / portTICK_PERIOD_MS);
    }

	xTaskCreatePinnedToCore(&motor_control_task, "motor_control_task", 75000, NULL, 5, NULL, 1);
    xTaskCreate(voltimetro, "voltimetro", TASK_SIZE, NULL, 0, NULL);

}
