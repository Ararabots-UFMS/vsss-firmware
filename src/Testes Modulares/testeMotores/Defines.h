// cada numero do IO no esp32 do robo
// MACRO/PIN-OUT DEFINITIONS #########################################################################


// Motor A PINOUT
#define PWMA 34
#define INA1 32
#define INA2 35
// Motor B PINOUT
#define PWMB 27
#define INB1 25
#define INB2 26
// Voltimeter PINOUT
#define V_PIN 36 // ADC1_CH0
#define R1 100000
#define R2 10000
#define V_MIN 9.0
#define MEASURE_TIME 10000
// Led, Speaker PINOUT
#define SPEAKER_PIN 19
#define WORKING_PIN 22
// Tasks Macro
#define Task_Stack_Size 30000
#define Applications_Core 1 
// GIROSCOPE PINS
#define SDA 16
#define SCL 5
#define INTERRUPT 4
#define OUTPUT_READABLE_YAWPITCHROLL
// GIROSCOPE OFFSETS OBTAINED WITH MPU6050_Calibration
#define XGyOffset -1355
#define YGyOffset 1403
#define ZGyOffset -1256
#define XAccOffset -1393
#define YAccOffset -971
#define ZAccOffset 13830
// PID CONSTANTS
#define kp 1.0
#define ki 0.0
#define kd 0.0

//MOTOR PWM CHANNELS
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1
