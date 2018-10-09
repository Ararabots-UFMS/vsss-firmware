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

extern "C" {
    void app_main();
}

void app_main(){

    Voltimetro a(R1,R2);
    while(1){
        float f = a.getVoltage();
        printf("Voltage: %f\n", f);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    printf("Restarting now.\n");
    fflush(stdout);
    // esp_restart();
}
