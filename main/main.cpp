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

#include "Memory.h"

extern "C" {
    void app_main();
}



void app_main(){

    Memory mem = Memory();
    
    double valor_grande = 1231123.1231123;
    char a[] = "KP";
    char b[] = "KI";
    char c[] = "KD";
    //printf("%lf\n", a);
    mem.update_memory(c, valor_grande);

    double KP = mem.read_memory(a);
    double KI = mem.read_memory(b);
    double KD = mem.read_memory(c);

    printf("%lf\n%lf\n%lf\n", KP,KI,KD);
    // a = 947.1;

    // mem.update_memory(b, a);
    
    // f = mem.read_memory(b);

    // printf("%lf\n", f);

    mem.close_handle();
}
