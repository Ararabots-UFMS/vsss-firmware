#include "Utils.h"
#include "definitions.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"


void thingEnable(void* pvParameters){

    thing* enable = (thing*) pvParameters;
    //ESP_LOGE("","%d %f %f %lld ", enable->pin, enable->duty, enable->freq, enable->lifetime);
    /* Select the GPIO to be used */
    gpio_pad_select_gpio(enable->pin);
    
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(enable->pin, GPIO_MODE_OUTPUT);

    long long int last_time = esp_timer_get_time();
    while(esp_timer_get_time() - last_time < enable->lifetime) {
        /* Turn ON (output HIGH) */
        gpio_set_level(enable->pin, HIGH);
        vTaskDelay(1000*enable->duty*enable->freq / portTICK_PERIOD_MS);
        /* Turn OFF (output LOW) */
        gpio_set_level(enable->pin, LOW);
        vTaskDelay(1000*(1.0-enable->duty)*enable->freq / portTICK_PERIOD_MS);
    }

      /* turn GPIO low to delete task */
    gpio_set_level(enable->pin, LOW);
    

    free(enable);
    /* delete the task */
    vTaskDelete(NULL);
}


void enable(gpio_num_t pino, float duty, float freq, long long int lifetime){
    
    /* struct to hold information about what will happen */
    thing* enable = (thing* ) malloc(sizeof(thing));

    enable->pin = pino;
    enable->duty = duty;
    enable->freq = freq;
    enable->lifetime = lifetime;


    //ESP_LOGE("","%d %f %f %lld ", enable->pin, enable->duty, enable->freq, enable->lifetime);
    
    /* task handle, so we can finish(delete) a task later*/
    TaskHandle_t handler;

    /* create a task and start to execute */
    xTaskCreate(thingEnable, "thingEnable", TASK_SIZE,(void*) enable, 0,&handler);
  
}