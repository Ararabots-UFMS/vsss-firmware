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
#include "driver/gpio.h"
#include "definitions.h"
#include "Utils.h"

PIDCONTROLLER pid_controller = PIDCONTROLLER(0,0,0);




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
  
    //enable(LED_PIN, DUTY_CYCLE_10, FREQ_4, 3000);
    //enable(LED_PIN, DUTY_CYCLE_80, FREQ_10, 1000);  
    //enable(LED_PIN, DUTY_CYCLE_50, FREQ_2, 6000);
    while(1){
      enable(SPEAKER_PIN, DUTY_CYCLE_50, FREQ_12, 1000);  
      enable(LED_PIN, DUTY_CYCLE_10, FREQ_4, 3000);
    }
  


    // thing enable = {LED_PIN, DUTY_CYCLE_50, FREQ_2};
    // TaskHandle_t handler;

    // xTaskCreate(thingEnable, "thingEnable", TASK_SIZE,(void*) &enable, 0,&handler);

    // vTaskDelay(pdMS_TO_TICKS(5000));

    // vTaskDelete(handler);


    //Voltimetro v = Voltimetro(1,2);

   // Motor LMotor = Motor(GPIO_NUM_26, GPIO_NUM_33, GPIO_NUM_21, MOTOR_PWM_CHANNEL_LEFT);

}
