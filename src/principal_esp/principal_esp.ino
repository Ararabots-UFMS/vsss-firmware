

//#######################################################################################################
// INCLUDES SECTION  ####################################################################################
//#######################################################################################################

#include "Defines.h" //include all constant definitions

#include <PIDController.h>
#include <Voltimetro.h>
#include <Motors.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


//#######################################################################################################
// OBJECTS ##############################################################################################
//#######################################################################################################
// Objects definition for motor control, pid control, battery keeper measure, and angular measure 
// and control

Motor motorA = Motor(INA1, INA2, PWMA, PWM_CHANNEL_A);
Motor motorB = Motor(INB1, INB2, PWMB, PWM_CHANNEL_B);

Voltimetro Voltimeter = Voltimetro(V_PIN, R1, R2);

PIDCONTROLLER pid = PIDCONTROLLER(kp,ki,kd);

MPU6050 giroscope;

//#######################################################################################################
// GLOBAL VALUES ########################################################################################
//#######################################################################################################
// Global values needed by more than one task or function in the code

int Motor_Way;                // global value for the way of a motor, received over BT
int PWM;                      // global value for the PWM that will be written to a motor
float Read_Voltage;           // global value for voltimeter measure
float output_correction;      // output value for PID


//#######################################################################################################
// GIRO - MPU6050 - DMP #################################################################################
//#######################################################################################################
// MPU control/status vars
// Do NOT touch this area, Biological Hazard
bool dmpReady = false;      // set true if DMP init was successful
uint8_t giroscopeIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus;          // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;        // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;         // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];     // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ===               INTERRUPT DETECTION ROUTINE                ===
volatile bool giroscopeInterrupt = false; // indicates whether giroscope interrupt pin has gone high

// callback for interrupt call from DMP
void dmpDataReady() {
  giroscopeInterrupt = true;
}

// ===                    GIRO SETUP ROUTINE                    ===
// Setup function that configures MPU6050 for use DMP to get yaw, pitch, row
void giroSetup() {
  Wire.begin(SDA, SCL);
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  giroscope.initialize();
  Serial.println(giroscope.testConnection() ? F("MPU6050 connected") : F("MPU6050 not connected"));

  devStatus = giroscope.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  giroscope.setXGyroOffset(XGyOffset);
  giroscope.setYGyroOffset(YGyOffset);
  giroscope.setZGyroOffset(ZGyOffset);
  giroscope.setZAccelOffset(ZAccOffset);

  giroscope.setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT), dmpDataReady, RISING);
  giroscopeIntStatus = giroscope.getIntStatus();
  dmpReady = true;

  // get expected DMP packet size for later comparison
  packetSize = giroscope.dmpGetFIFOPacketSize();

}

//#######################################################################################################
//FIRMWARE TASKS (THREADS) ##############################################################################
//#######################################################################################################

// performs all BT connection and data handle for the firmware
void bluetooth(void * pvParameters) {
  for (;;) {

  }
}

// performs baterry measure and warning low voltage values
void voltimeter(void * pvParameters) {
  for (;;) {
    Read_Voltage = Voltimeter.getVoltage();

    //debug only
    //Read_Voltage = 8.0;
    //Serial.println(Read_Voltage);

    if (Read_Voltage < V_MIN) {
      digitalWrite(SPEAKER_PIN, HIGH);
    }
    else {
      digitalWrite(SPEAKER_PIN, LOW);
    }
    delay(MEASURE_TIME);
  }
}

// proccess raw MPU6050 values fusion for yaw, pitch, row values
void giro(void * pvParameters) {

  for (;;) {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

        // wait for MPU interrupt or extra packet(s) available
 
    // reset interrupt flag and get INT_STATUS byte
    giroscopeInterrupt = false;
    giroscopeIntStatus = giroscope.getIntStatus();

    // get current FIFO count
    fifoCount = giroscope.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((giroscopeIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        giroscope.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (giroscopeIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = giroscope.getFIFOCount();

        // read a packet from FIFO
        giroscope.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        giroscope.dmpGetQuaternion(&q, fifoBuffer);
        giroscope.dmpGetGravity(&gravity, &q);
        giroscope.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
  }
}


// performs PID correction under yaw values and vision received values
void PID(void * pvParameters) {
  float yaw;              // yaw value for DMP read corrected to be from 0 to 360
  float vision_reference; // angular value received from vision

  pid.setGoal(0.0);

  for (;;) {
  
    yaw = (ypr[0] * 180 / M_PI) + 180.0;  
    vision_reference = 100.0;

    pid.updateReading(yaw-vision_reference);
    output_correction = pid.control();

  }
}

// enable motors with PID correction values and way values 
void enableMotors(void * pvParameters) {
  for (;;) {
      /*
       *  if(sentido == 0){
       *      if(output_correction > 0){
       *        motorA.enable(pwm-correction, sentido);
       *        motorA.enable();
       *      }
       *      else{
       *        motorB.enable(pwm+correction, sentido);
       *        motorA.enable();
       *      }
       *  }
       *  else{
       *      if(output_correction > 0){
       *        motorA.enable(); 
       *        motorA.enable();
       *      }
       *      else{
       *        motorB.enable();
       *        motorA.enable();
       *      }
       *  }
       */
  }
}

//#######################################################################################################
// USEFUL FUNTCIONS  ####################################################################################
//#######################################################################################################

// setup all pins used for the firmware, INPUT and OUTPUT modes
void setPinModes(){
    pinMode(WORKING_PIN, OUTPUT);
    pinMode(SPEAKER_PIN, OUTPUT);
    pinMode(INTERRUPT, INPUT);
    pinMode(V_PIN, INPUT);
}

//#######################################################################################################
// SETUP FUNCTION #######################################################################################
//#######################################################################################################
void setup() {
  Serial.begin(115200);
  
  //giroSetup();
  setPinModes();
  
  digitalWrite(WORKING_PIN, HIGH);
  
  //xTaskCreatePinnedToCore(giro, "giro", Task_Stack_Size, NULL, 0, NULL, Applications_Core);
  
  xTaskCreatePinnedToCore(bluetooth, "bluetooth", Task_Stack_Size, NULL, 0, NULL, Applications_Core);
  
  xTaskCreatePinnedToCore(voltimeter, "voltimeter", Task_Stack_Size, NULL, 0, NULL, Applications_Core);
  
  xTaskCreatePinnedToCore(PID, "pid", Task_Stack_Size, NULL, 0, NULL, Applications_Core);
  
  xTaskCreatePinnedToCore(enableMotors, "enableMotors", Task_Stack_Size, NULL, 0, NULL, Applications_Core);

  delay(1000);
  digitalWrite(WORKING_PIN, LOW);
}


//#######################################################################################################
// LOOP FUNCTION  #######################################################################################
//#######################################################################################################
void loop() {

  delay(1000);
}
