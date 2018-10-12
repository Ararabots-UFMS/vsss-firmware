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
  
    
    
    while(1){
      //enable(SPEAKER_PIN, DUTY_CYCLE_40, FREQ_6, 250000); //bt nao conectado
      //enable(LED_PIN, DUTY_CYCLE_60, FREQ_2, 1000000);  //bt nao conectado
      

      //enable(LED_PIN, DUTY_CYCLE_50, FREQ_8, 750000);  //bt conectou
      //enable(SPEAKER_PIN, DUTY_CYCLE_30, FREQ_6, 500000);   //4 bips curtos


      //enable(SPEAKER_PIN, DUTY_CYCLE_80, FREQ_12, 10000000); //bateria fraca
      //enable(LED_PIN, DUTY_CYCLE_50, FREQ_12, 10000000); //estrobo, bateria fraca      


      //enable(LED_PIN, DUTY_CYCLE_100, FREQ_2, 3000000); //aceso tempo todo, gyro setup
      //enable(SPEAKER_PIN, DUTY_CYCLE_50, FREQ_12, 200000); //gyro ok 


      //enable(LED_PIN, DUTY_CYCLE_50, FREQ_1, 500000); //blink medio, ler e gravar kP, Ki, Kd
      //enable(SPEAKER_PIN, DUTY_CYCLE_30, FREQ_6, 500000);   //4 bips curtos


      enable(LED_PIN, DUTY_CYCLE_30, FREQ_2, 250000); //blink curto, funcionamento


      
    
      vTaskDelay(3000 / portTICK_PERIOD_MS);
    }

    //Voltimetro v = Voltimetro(1,2);

    // Motor LMotor = Motor(GPIO_NUM_26, GPIO_NUM_33, GPIO_NUM_21, MOTOR_PWM_CHANNEL_LEFT);

}
