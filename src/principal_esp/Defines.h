// MACRO/PIN-OUT DEFINITIONS #########################################################################

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

// Led, Speaker pins
#define SPEAKER_PIN 36
#define WORKING_PIN 32

// Tasks  
#define Task_Stack_Size 1000000

// GIROSCOPE PINS
#define SDA 32
#define SCL 16
#define INTERRUPT 34

// GIROSCOPE OFFSETS
#define XGyOffset 0
#define YGyOffset 0
#define ZGyOffset 0
#define XAccOffset 0
#define YAccOffset 0
#define ZAccOffset 0

// PID CONSTANTS
#define kP 10
#define kI 10
#define kD 10
