#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "time.h"
#include "sys/time.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>

#include "PIDController.h"
#include "bluetooth.h"
#include "Motors.h"
#include "Voltimetro.h"
#include "definitions.h"
#include "Utils.h"
#include <gyro.h>

#define PIDCONTROL 0

struct motorPackage motor_package;
struct controlPackage control_package;
Motor motor_left = Motor(AIN1, AIN2, PWMA, MOTOR_PWM_CHANNEL_A);
Motor motor_right = Motor(BIN1, BIN2, PWMB, MOTOR_PWM_CHANNEL_B);

// Estrutura para a troca de pacote do pid
uint8_t lastTheta = -1;
uint8_t lastDirection = -1;
uint8_t lastSpeed = -1;
// Desculpa
float yaw;
SemaphoreHandle_t yawSemaphore = xSemaphoreCreateBinary();
SemaphoreHandle_t motorPackageSemaphore = xSemaphoreCreateBinary();
TaskHandle_t motorTaskHandler;

PIDCONTROLLER pid_controller = PIDCONTROLLER(3,0,0.5);

int modulus(int a, int b)
{
    // Modulo para manter o mesmo sinal do modulo
    return (a % b + b) % b;
}

int diff_angle(float a, float b)
{
    // Diferença de angulo entre A e B
    int tmp;
    tmp = a - b;
    return modulus((tmp + 180), 360) - 180;
}

float ajust_set_point(float yaw, int rotation_direction, float theta)
{
    // Ajuste de valor do set point
	if(rotation_direction)//Sentido horario
    {
        if(yaw + theta > 180)
            return yaw + theta - 360;
        return yaw + theta;
    }
    else// Sentido anti-horario
    {
        if(abs(yaw - theta) > 180)
            return yaw - theta + 360;
        return yaw - theta;
    }
}

int ajust_direction(int speed)
{
    //Ajuste de direão das rodas
    if(speed < 0){
        return 1;
    }
    return 0;
}

int ajust_speed(int speed)
{
    //Limite de valocidade maxima
    if(abs(speed) < 255)
        return abs(speed);
    else
        return 255;
}

void readYawFromGyro(float *_yaw)
{
  xSemaphoreTake(yawSemaphore, 1.0 /portTICK_PERIOD_MS);
  *_yaw = yaw;
  xSemaphoreGive(yawSemaphore);
}

void writeYawFromGyro(float *_yaw)
{
  xSemaphoreTake(yawSemaphore, 1.0 /portTICK_PERIOD_MS);
  yaw = *_yaw;
  xSemaphoreGive(yawSemaphore);
}

void readMotorPackage(motorPackage *p)
{
  xSemaphoreTake(motorPackageSemaphore, 1.0 /portTICK_PERIOD_MS);
  *p = motor_package;
  xSemaphoreGive(motorPackageSemaphore);
}

void writeMotorPackage(motorPackage *p)
{
  xSemaphoreTake(motorPackageSemaphore, 1.0 /portTICK_PERIOD_MS);
  motor_package = *p;
  xSemaphoreGive(motorPackageSemaphore);
}

void gyro_task(void*)
{
  xSemaphoreGive(yawSemaphore);
  motorPackage myPackage;
  float myYaw;
  auto gyro = Gyro();

  while(1)
  {
    gyro.update_yaw(&myYaw);
    writeYawFromGyro(&myYaw);
    readMotorPackage(&myPackage);
    if(myPackage.control_type == PIDCONTROL) xTaskNotifyGive(motorTaskHandler);
  }
}

void motor_control_task(void *pvParameter)
{
  motorPackage myMotorPackage;
  while(1)
  {
    auto notificationValue = ulTaskNotifyTake(pdFALSE, (TickType_t) portMAX_DELAY);

    if(notificationValue > 0)
    {
      readMotorPackage(&myMotorPackage);
      if(myMotorPackage.control_type != PIDCONTROL)
      {
        #ifdef DEBUG
          ESP_LOGI("MOTOR", "LEFT WHEEL SPEED: %u", myMotorPackage.speed_l);
          ESP_LOGI("MOTOR", "RIGHT WHEEL SPEED: %u", myMotorPackage.speed_r);
        #endif
        motor_left.enable(myMotorPackage.speed_l, myMotorPackage.direction >> 1);
        motor_right.enable(myMotorPackage.speed_r, myMotorPackage.direction & 1);
      }
      else
      {
        // TODO: controle PID

      }
    }

  }
}
/*void motor_control_task(void *pvParameter)
{
    // Task para a ativacao dos motores
    float yaw;
    float pid;
    float set_point = 0;
    auto gyro = Gyro();
    pid_controller.setGoal(0.0);


    #ifdef DEBUG
        long long int last_time = esp_timer_get_time(), newer;
    #endif

    motorPackage myMotorPackage;
    uint32_t notificationValue = 0;


    while(1)
    {
        notificationValue = ulTaskNotifyTake(pdTRUE, (TickType_t) portMAX_DELAY);
        #ifdef DEBUG
          ESP_LOGI("NOTIFY", "Value %u", notificationValue);
        #endif
        myMotorPackage = motor_package;
        if(!myMotorPackage.control_type) //Pacote para a correcao da direcao com pid
        {
            gyro.update_yaw(&yaw);
            #ifdef DEBUG
                newer = esp_timer_get_time();
                if ((newer-last_time) > 50000){
                    ESP_LOGI("Gyro", "Yaw: %f" ,yaw);
                    last_time = newer;
                }
            #endif

            //Pacote diferente
            if(myMotorPackage.theta != lastTheta || myMotorPackage.rotation_direction != lastDirection || myMotorPackage.speed_l != lastSpeed)
            {
                lastTheta = myMotorPackage.theta;
                lastDirection = myMotorPackage.rotation_direction;
                lastSpeed = myMotorPackage.speed_l;
                set_point = ajust_set_point(yaw, lastDirection, lastTheta);
			      }

		        pid_controller.updateReading(diff_angle(yaw, set_point));
            pid = pid_controller.control();

            // Erro aceitavel pelo controlador
            if(abs(pid) < PIDERRO)
            {
                motor_left.enable(myMotorPackage.speed_l, myMotorPackage.direction);
                motor_right.enable(myMotorPackage.speed_r, myMotorPackage.direction);
            }
            else //correcao das velocidades dos motores
            {
                motor_left.enable(ajust_speed(myMotorPackage.speed_l-pid), ajust_direction(myMotorPackage.speed_l-pid));
		       	    motor_right.enable(ajust_speed(myMotorPackage.speed_r+pid), ajust_direction(myMotorPackage.speed_r+pid));
            }
        }
    		else //Pacote para envio das velocidades
    		{
          #ifdef DEBUG
            ESP_LOGI("MOTOR", "LEFT WHEEL SPEED: %u", myMotorPackage.speed_l);
            ESP_LOGI("MOTOR", "RIGHT WHEEL SPEED: %u", myMotorPackage.speed_r);
          #endif
    			motor_left.enable(myMotorPackage.speed_l, myMotorPackage.direction >> 1);
    			motor_right.enable(myMotorPackage.speed_r, myMotorPackage.direction & 1);
    		}
	}
}*/

void voltimetro(void * pvParamters){
    Voltimetro voltimetro(R1,R2);

    /* Select the GPIO to be used */
    gpio_pad_select_gpio(SPEAKER_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(SPEAKER_PIN, GPIO_MODE_OUTPUT);

    #ifdef DEBUG
      ESP_LOGI("VOLTIMETRO","Initilized");
    #endif

    float measure;
    while(1){
        measure = voltimetro.getVoltage();
        // If the read voltage is less than 9 volts it activates the buzzer
        if (measure < V_MIN) {
            // enable led and buzzer indicating low battery measure
            // during a certain time, with a certain frequency
            enable(SPEAKER_PIN, DUTY_CYCLE_30, FREQ_12, BUZZER_TIME);
            //enable(LED_PIN, DUTY_CYCLE_50, FREQ_12, BUZZER_TIME);
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


void app_main()
{

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );

  pid_controller.load_params();

  setup_bluetooth();


  auto x = xTaskCreatePinnedToCore(gyro_task, "gyro_task",
                     4*DEFAULT_TASK_SIZE, NULL, 2, NULL, CORE_ONE);
  if(x != pdPASS)
  {
    ESP_LOGE("GYRO", "Error creating thread, ERR_CODE: #%X", x);
  }

  x = xTaskCreate(voltimetro, "voltimetro", VOLTIMETER_TASK_STACK, NULL, 1, NULL);
  if(x != pdPASS)
  {
    ESP_LOGE("VOLTIMETER", "Error creating thread, ERR_CODE: #%X", x);
  }

  x = xTaskCreatePinnedToCore(motor_control_task, "motor_control_task",
                        MOTOR_TASK_STACK, NULL, 3, &motorTaskHandler, CORE_ONE);
  if(x != pdPASS)
  {
    ESP_LOGE("MOTORS", "Error creating thread, ERR_CODE: #%X", x);
  }

  // vTaskStartScheduler();
}
