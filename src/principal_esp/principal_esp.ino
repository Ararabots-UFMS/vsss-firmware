#include <Voltimetro.h>
#include <Motors.h>
#include <Giro.h>

// MACRO DEFINITIONS #########################################################################

// Motor A
#define PWM_A 13
#define INA_1 15
#define INA_2 2

// Motor B
#define PWM_B 18
#define INB_1 17
#define INB_2 5

// Voltimeter
#define V_PIN 33 // ADC1_CHANNEL_5
//#define V_PIN 34 // ADC1_CHANNEL_6
#define R1 30
#define R2 10
#define V_MIN 9.0
#define MEASURE_TIME 10000

// Leds, Speaker pinouts
#define SPEAKER_LED_PIN 14
//#define SPEAKER_LED_PIN 19           //not implemented
#define WAY_PIN 22
#define WORKING_PIN 32

// Tasks 
#define Task_Stack_Size 10000


// GIROSCOPE PINS
#define SDA 4
#define SCL 16
#define INTERRUPT 26


// OBJECTS #######################################################################

Motor motorA = Motor(INA_1, INA_2, PWM_A);
Motor motorB = Motor(INB_1, INB_2, PWM_B);
Voltimetro Voltimeter = Voltimetro(V_PIN,R1,R2);
// definir o objeto giro

Giro mpu6050 = Giro(SCL, SDA, INTERRUPT);

// GLOBAL VALUES #####################################################################

int Motor_Way = 0;
int PWM;
int Applications_core  = 1;
float Read_Voltage;

// ##############################################################################


// possivelmente task do bluetooth
// terminar documentação e renomear task
void led(void * pvParameters){
  for (;;){
    Serial.println("led");
      
    digitalWrite(WAY_PIN,HIGH);
    // ler valores de pwm e sentido do bluetooth
    Motor_Way = 1;
    //Serial.println(Motor_Way);
    PWM = 255;
    delay(1000);
    digitalWrite(WAY_PIN,LOW);
    Motor_Way = 0;
    //Serial.println(Motor_Way);
    PWM = 255; 
    delay(1000);
  }
}

void voltimeter(void * pvParameters){
  for (;;){
    //Serial.println("voltimeter");

    // gets battery voltage measue using .getVoltage()
    // function from Voltimeter class
    Read_Voltage = Voltimeter.getVoltage();
    
    // Serial.print("TENSAO Bateria: ");
    // Serial.println(Read_Value);
    
    // turns on/off low battery warning led if 
    // battery voltage is less then 9.0 V
    if(Read_Voltage < V_MIN){
      digitalWrite(SPEAKER_LED_PIN,HIGH);
    }
    else {
      digitalWrite(SPEAKER_LED_PIN,LOW);
    }
    Serial.println(Read_Voltage);
    
    // waits 60 seconds for next measurement
    // of battery voltage
    delay(MEASURE_TIME);
  }
}

void giro(void * pvParameters){
  for (;;){
    mpu6050.getYPR();
  }
}


// pinModes, serial, and FreeRTOS tasks 
// creation when robot turns ON
void setup() {

    // pinModes for leds, buzzer
    pinMode(WORKING_PIN,OUTPUT);
    digitalWrite(WORKING_PIN,HIGH);
    
    pinMode(SPEAKER_LED_PIN,OUTPUT);
    pinMode(WAY_PIN,OUTPUT);
   
    Serial.begin(115200);

    //
    // tasks creation pinned to a core
    // with following sequence of parameters
    // (task_function, task_name, task_stack_size, 
    //  task_parameters, task_priority, task_handle, task_core)
    //

    xTaskCreatePinnedToCore(giro,"giro",Task_Stack_Size,NULL,0,NULL,Applications_core);
    xTaskCreatePinnedToCore(led,"led",Task_Stack_Size,NULL,0,NULL,Applications_core);
    xTaskCreatePinnedToCore(voltimeter,"voltimeter",Task_Stack_Size,NULL,0,NULL,Applications_core);
    
    
    delay(3000);
    digitalWrite(WORKING_PIN,LOW);
}


// enable both motors with last pwm and 
// motor_way received values from BT master
// runs on core 1 by default
void loop() {
    //Serial.println("      loop");
    motorA.enable(PWM, Motor_Way); 
    motorB.enable(PWM, Motor_Way); 
    delay(1);
}



