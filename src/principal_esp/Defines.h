// MACRO/PIN-OUT DEFINITIONS #########################################################################

// Motor A PINOUT
#define PWM_A 6
#define INA_1 8
#define INA_2 7

// Motor B PINOUT
#define PWM_B 12
#define INB_1 10
#define INB_2 11

// Voltimeter PINOUT
#define V_PIN 4 // ADC1_CHANNEL_5
//#define V_PIN 34 // ADC1_CHANNEL_6
#define R1 100000
#define R2 10000
#define V_MIN 9.0
#define MEASURE_TIME 10000

// Led, Speaker PINOUT
#define SPEAKER_PIN 36
#define WORKING_PIN 32

// Tasks Macro
#define Task_Stack_Size 30000
#define Applications_Core 1 

// GIROSCOPE PINS
#define SDA 32
#define SCL 16
#define INTERRUPT 4
#define OUTPUT_READABLE_YAWPITCHROLL


// GIROSCOPE OFFSETS OBTAINED WITH MPU6050_Calibration
#define XGyOffset 62
#define YGyOffset -7
#define ZGyOffset 11
#define XAccOffset -338
#define YAccOffset -44
#define ZAccOffset 1500

// PID CONSTANTS
#define kP 1.0
#define kI 0.0
#define kD 0.0

