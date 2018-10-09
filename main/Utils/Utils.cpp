#include "Utils.h"
#include "definitions.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"


void thingEnable(void* pvParameters){

    thing* enable = (thing*) pvParameters;

    /* Select the GPIO to be used */
    gpio_pad_select_gpio(enable->pin);
    
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(enable->pin, GPIO_MODE_OUTPUT);

    while(1) {
        /* Turn ON (output HIGH) */
        gpio_set_level(enable->pin, HIGH);
        vTaskDelay(1000*enable->duty*enable->freq / portTICK_PERIOD_MS);
        /* Turn OFF (output LOW) */
        gpio_set_level(enable->pin, LOW);
        vTaskDelay(1000*(1.0-enable->duty)*enable->freq / portTICK_PERIOD_MS);
    }
}


void enable(gpio_num_t pino, float duty, float freq, int lifetime){
    
    /* struct to hold information about what will happen */
    thing enable = {pino, duty, freq, lifetime};
    
    /* task handle, so we can finish(delete) a task later*/
    TaskHandle_t handler;

    /* create a task and start to execute */
    xTaskCreate(thingEnable, "thingEnable", TASK_SIZE,(void*) &enable, 0,&handler);
    
    /* make a delay in ms, task will run for this ms */
    vTaskDelay(pdMS_TO_TICKS(lifetime));

    /* turn GPIO low to delete task */
    gpio_set_level(enable.pin, LOW);
    
    /* delete the task */
    vTaskDelete(handler);

}