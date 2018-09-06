
#include <PIDController.h>
#include <Voltimetro.h>
#include <Motors.h>

#include "Giro.h"


// MACRO DEFINITIONS #########################################################################

// Motor A pins
#define PWM_A 6
#define INA_1 8
#define INA_2 7

// Motor B pins
#define PWM_B 12
#define INB_1 10
#define INB_2 11

// Voltimeter pins
#define V_PIN 4 // ADC1_CHANNEL_5
//#define V_PIN 34 // ADC1_CHANNEL_6
#define R1 100000
#define R2 10000
#define V_MIN 9.0
#define MEASURE_TIME 10000

// Leds, Speaker pins
#define SPEAKER_PIN 36

#define WORKING_PIN 32

// Tasks  
#define Task_Stack_Size 10000


// GIROSCOPE PINS
#define SDA 27
#define SCL 16
#define INTERRUPT 26


// OBJECTS #######################################################################

Motor motorA = Motor(INA_1, INA_2, PWM_A);
Motor motorB = Motor(INB_1, INB_2, PWM_B);
Voltimetro Voltimeter = Voltimetro(V_PIN,R1,R2);
// definir o objeto giro
Giro mpu6050 = Giro(SCL, SDA, INTERRUPT);
PIDCONTROLLER pid = PIDCONTROLLER(10,10,10);


// GLOBAL VALUES #####################################################################

int Motor_Way = 0;
int PWM;
int Applications_core  = 1;
float Read_Voltage;
float pidGoal = 0.0;

// ##############################################################################


// possivelmente task do bluetooth
// terminar documentação e renomear task
void bluetooth(void * pvParameters){
  for (;;){
    
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
      Serial.println("LOW");
      digitalWrite(SPEAKER_PIN,HIGH);
    }
    else{
      digitalWrite(SPEAKER_PIN,LOW);
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


void PID(void * pvParameters){
  float output;
  for (;;){
    //direcao do giro
    pid.updateReading(3.02);

    //direcao recebida visao
    pid.setGoal(pidGoal);
    
    output = pid.control();

    Serial.println(output);
  }
}

void enableMotors(void * pvParameters){
  for (;;){
    
    PWM = 255;
    
    motorA.enable(PWM, Motor_Way); 
    motorB.enable(PWM, -1*Motor_Way); 
  }
}
   

// pinModes, serial, and FreeRTOS tasks 
// creation when robot turns ON
void setup() {
    Serial.begin(115200);
    Serial.println("running");


    // pinModes for leds, buzzer
    pinMode(WORKING_PIN,OUTPUT);
    digitalWrite(WORKING_PIN,HIGH);
    
    pinMode(SPEAKER_PIN,OUTPUT);
   
    
    //
    // tasks creation pinned to a core
    // with following sequence of parameters
    // (task_function, task_name, task_stack_size, 
    //  task_parameters, task_priority, task_handle, task_core)
    //

    xTaskCreatePinnedToCore(giro,"giro",Task_Stack_Size,NULL,0,NULL,Applications_core);
    xTaskCreatePinnedToCore(bluetooth,"bluetooth",Task_Stack_Size,NULL,0,NULL,Applications_core);
    xTaskCreatePinnedToCore(voltimeter,"voltimeter",Task_Stack_Size,NULL,0,NULL,Applications_core);
    xTaskCreatePinnedToCore(PID,"pid",Task_Stack_Size,NULL,0,NULL,Applications_core);
    xTaskCreatePinnedToCore(enableMotors,"enableMotors",Task_Stack_Size,NULL,0,NULL,Applications_core);

    
    
    delay(200);
    digitalWrite(WORKING_PIN,LOW);
}


// enable both motors with last pwm and 
// motor_way received values from BT master
// runs on core 1 by default
void loop() {

    delay(10000);
}
