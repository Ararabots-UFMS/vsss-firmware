//#######################################################################################################
// INCLUDES SECTION  ####################################################################################
//#######################################################################################################
#include <PIDController.h>
#include <Voltimetro.h>
#include <Motors.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "Defines.h"

//#######################################################################################################
// OBJECTS ##############################################################################################
//#######################################################################################################

Motor motorA = Motor(INA_1, INA_2, PWM_A);
Motor motorB = Motor(INB_1, INB_2, PWM_B);
Voltimetro Voltimeter = Voltimetro(V_PIN, R1, R2);
PIDCONTROLLER pid = PIDCONTROLLER(kP, kI, kD);
MPU6050 giroscope;

//#######################################################################################################
// GLOBAL VALUES ########################################################################################
//#######################################################################################################

int Motor_Way = 0;
int PWM;
int Applications_core  = 1;
float Read_Voltage;
float pidGoal;
float giroYaw;

//#######################################################################################################
// GIRO - MPU6050 - DMP ##################################################################################
//#######################################################################################################
// giroscope control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t giroscopeIntStatus;   // holds actual interrupt status byte from giroscope
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ===               INTERRUPT DETECTION ROUTINE                ===
volatile bool giroscopeInterrupt = false;     // indicates whether giroscope interrupt pin has gone high
void dmpDataReady() {
  giroscopeInterrupt = true;
}

// ===               GIRO SETUP DETECTION ROUTINE                ===
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
  giroscope.setZAccelOffset(ZAccOffset); // 1688 factory default for my test chip

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
void bluetooth(void * pvParameters) {
  for (;;) {

  }
}

void voltimeter(void * pvParameters) {
  for (;;) {

    Read_Voltage = Voltimeter.getVoltage();

    if (Read_Voltage < V_MIN) {
      digitalWrite(SPEAKER_PIN, HIGH);
    }
    else {
      digitalWrite(SPEAKER_PIN, LOW);
    }

    delay(MEASURE_TIME);
  }
}

void giro(void * pvParameters) {
  for (;;) {
    while (!giroscopeInterrupt && fifoCount < packetSize) {
    }

    // reset interrupt flag and get INT_STATUS byte
    giroscopeInterrupt = false;
    giroscopeIntStatus = giroscope.getIntStatus();

    // get current FIFO count
    fifoCount = giroscope.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((giroscopeIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      giroscope.resetFIFO();
      //Serial.println(F("FIFO overflow!"));

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
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180 / M_PI);
    }
  }
}


void PID(void * pvParameters) {
  float output;
  for (;;) {
    //direcao do giro
    pid.updateReading(giroYaw);

    //direcao recebida visao
    pid.setGoal(pidGoal);

    output = pid.control();

    Serial.println(output);
  }
}

void enableMotors(void * pvParameters) {
  for (;;) {

    PWM = 255;

    motorA.enable(PWM, Motor_Way);
    motorB.enable(PWM, -1 * Motor_Way);
  }
}

//#######################################################################################################
// USEFUL FUNTCIONS  ####################################################################################
//#######################################################################################################
void setPinModes(){
    pinMode(WORKING_PIN, OUTPUT);
    pinMode(SPEAKER_PIN, OUTPUT);
}

//#######################################################################################################
// SETUP FUNCTION #########################################################################################
//#######################################################################################################
void setup() {
  Serial.begin(115200);

  giroSetup();
  setPinModes();
  
  digitalWrite(WORKING_PIN, HIGH);
  
  xTaskCreatePinnedToCore(giro, "giro", Task_Stack_Size, NULL, 0, NULL, Applications_core);
  xTaskCreatePinnedToCore(bluetooth, "bluetooth", Task_Stack_Size, NULL, 0, NULL, Applications_core);
  xTaskCreatePinnedToCore(voltimeter, "voltimeter", Task_Stack_Size, NULL, 0, NULL, Applications_core);
  xTaskCreatePinnedToCore(PID, "pid", Task_Stack_Size, NULL, 0, NULL, Applications_core);
  xTaskCreatePinnedToCore(enableMotors, "enableMotors", Task_Stack_Size, NULL, 0, NULL, Applications_core);

  digitalWrite(WORKING_PIN, LOW);
}


//#######################################################################################################
// LOOP FUNCTION  #######################################################################################
//#######################################################################################################
void loop() {


}
