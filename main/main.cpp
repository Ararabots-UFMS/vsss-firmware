/*
This example code is in the Public Domain (or CC0 licensed, at your option.)

Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.
*/

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
#include "esp_timer.h"
#include <gyro.h>

struct motorPackage motor_package;
struct controlPackage control_package;
Motor motor_left = Motor(AIN1, AIN2, PWMA, MOTOR_PWM_CHANNEL_A);
Motor motor_right = Motor(BIN1, BIN2, PWMB, MOTOR_PWM_CHANNEL_B);

uint8_t lastTheta = -1;
uint8_t lastDirection = -1;
uint8_t lastOrigin = 0;

PIDCONTROLLER pid_controller = PIDCONTROLLER();

float ajust_set_point(int rotation_direction, float theta)
{
	if(rotation_direction)
		return theta;
	return -theta;
}

int ajust_direction(int speed)
{
    if(speed < 0){
        return 1;
    }
    return 0;
}
int ajust_speed(int speed)
{
    if(abs(speed) < 255)
        return abs(speed);
    else
        return 255;
}

void motor_control_task(void *pvParameter)
{
    float yaw;
    float pid;
    auto gyro = Gyro();


    #ifdef DEBUG
        long long int last_time = esp_timer_get_time(), newer;
    #endif

	while(1)
	{
		if(!motor_package.control_type)
		{

            gyro.update_yaw(&yaw);

			//pid control
			if(motor_package.theta != lastTheta || motor_package.rotation_direction != lastDirection)
			{
				lastTheta = motor_package.theta;
				lastDirection = motor_package.rotation_direction;
				lastOrigin = yaw;
                pid_controller.setGoal(ajust_set_point(lastDirection, lastTheta));
			}
		    pid_controller.updateReading(yaw - lastOrigin);
            pid = pid_controller.control();

            if(abs(pid) < PIDERRO)
            {
                motor_left.enable(motor_package.speed_l, motor_package.direction);
                motor_right.enable(motor_package.speed_r, motor_package.direction);
            }
            else
            {
                motor_left.enable(ajust_speed(motor_package.speed_l+pid), ajust_direction(motor_package.speed_l+pid));
		       	motor_right.enable(ajust_speed(motor_package.speed_r-pid), ajust_direction(motor_package.speed_r-pid));
            }

                // #ifdef DEBUG
            //     newer = esp_timer_get_time();
            //     if ((newer-last_time) > 50000){
            //         ESP_LOGW("Gyro:", "Yaw: %f" ,yaw);
            //         ESP_LOGI("PID:", " %f %f %f" ,pid, motor_package.speed_l+pid, motor_package.speed_r-pid);
            //         last_time = newer;
            //     }
            // #endif

        }
		else
		{
            //ESP_LOGW("Modo:", " Normal");
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

    float measure;
    //ESP_LOGI("Voltimetro","init");

    while(1){

        // If the read voltage is less than 9 volts it activates the buzzer
        measure = voltimetro.getVoltage();        
        ESP_LOGI("Voltimetro","%f", measure);

        if (measure < V_MIN) {
            // enable led and buzzer indicating low battery measure
            // during a certain time, with a certain frequency
            enable(SPEAKER_PIN, DUTY_CYCLE_30, FREQ_12, BUZZER_TIME);
            enable(LED_PIN, DUTY_CYCLE_50, FREQ_12, BUZZER_TIME);
        }
        else {
            gpio_set_level(SPEAKER_PIN, LOW);
            gpio_set_level(LED_PIN, LOW);
        }
        vTaskDelay(MEASURE_TIME/portTICK_PERIOD_MS);
    }
}

extern "C" {
    void app_main();
}


void app_main(){
    // Init nvs flash

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    pid_controller.load_params();

    setup_bluetooth();

	xTaskCreatePinnedToCore(&motor_control_task, "motor_control_task", 75000, NULL, 0, NULL, 1);
    xTaskCreate(voltimetro, "voltimetro", TASK_SIZE, NULL, 0, NULL);

}
