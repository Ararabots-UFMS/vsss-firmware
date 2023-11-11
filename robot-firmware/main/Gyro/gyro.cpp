// #include "gyro.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include <definitions.h>
// #include <Utils.h>

// /* Bus configuration */

// // This MACROS are defined in "skdconfig.h" and set through 'menuconfig'.
// // Can use to check which protocol has been selected.

// //static I2C_t& i2c                     = i2c0;  // i2c0 or i2c1
// static constexpr gpio_num_t SDA       = GYRO_SDA_PIN;
// static constexpr gpio_num_t SCL       = GYRO_SCL_PIN;
// static constexpr uint32_t CLOCK_SPEED = GYRO_CLOCK_SPEED;  // 400 KHz
// constexpr uint16_t kFIFOPacketSize = 12;
// static void mpuISR(void*);

// /* MPU configuration */
// static constexpr int kInterruptPin         = GYRO_INT_PIN;  // GPIO_NUM
// static constexpr uint16_t kSampleRate      = GYRO_SAMPLE_RATE;  // Hz
// static constexpr mpud::accel_fs_t kAccelFS = mpud::ACCEL_FS_4G;
// static constexpr mpud::gyro_fs_t kGyroFS   = mpud::GYRO_FS_500DPS;
// static constexpr mpud::dlpf_t kDLPF        = mpud::DLPF_98HZ;
// static constexpr mpud::int_config_t kInterruptConfig{
//     .level = mpud::INT_LVL_ACTIVE_HIGH,
//     .drive = mpud::INT_DRV_PUSHPULL,
//     .mode  = mpud::INT_MODE_PULSE50US,
//     .clear = mpud::INT_CLEAR_STATUS_REG  //
// };


// /*-*/

// Gyro::Gyro(){

// 	i2c.begin(SDA, SCL, CLOCK_SPEED);
//     MPU.setBus(i2c);
//     MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);

//     // Verify connection
//     while (esp_err_t err = MPU.testConnection()) {
//         ESP_LOGE(SPP_TAG, "Failed to connect to the MPU, error=%#X", err);
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
//     ESP_LOGI(SPP_TAG, "MPU connection successful!");

//     // Initialize
//     ESP_ERROR_CHECK(MPU.initialize());

//     // Self-Test
//     mpud::selftest_t retSelfTest;
//     while (esp_err_t err = MPU.selfTest(&retSelfTest)) {
//         ESP_LOGE(SPP_TAG, "Failed to perform MPU Self-Test, error=%#X", err);
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
    
//     auto gyro_fail = retSelfTest & mpud::SELF_TEST_GYRO_FAIL;
//     auto accel_fail = retSelfTest & mpud::SELF_TEST_ACCEL_FAIL;

//     ESP_LOGI(SPP_TAG, "MPU Self-Test result: Gyro=%s Accel=%s",  //
//              (gyro_fail ? "FAIL" : "OK"),
//              (accel_fail ? "FAIL" : "OK"));


//     if(!(gyro_fail || accel_fail))
//     {
//         giroHandle = enable(SPEAKER_PIN, DUTY_CYCLE_10, FREQ_3, GIRO_TIME);
//     }

//     // Calibrate
//     mpud::raw_axes_t accelBias, gyroBias;
//     ESP_ERROR_CHECK(MPU.computeOffsets(&accelBias, &gyroBias));
//     ESP_ERROR_CHECK(MPU.setAccelOffset(accelBias));
//     ESP_ERROR_CHECK(MPU.setGyroOffset(gyroBias));

//     // Configure
//     ESP_ERROR_CHECK(MPU.setAccelFullScale(kAccelFS));
//     ESP_ERROR_CHECK(MPU.setGyroFullScale(kGyroFS));
//     ESP_ERROR_CHECK(MPU.setSampleRate(kSampleRate));
//     ESP_ERROR_CHECK(MPU.setDigitalLowPassFilter(kDLPF));

//     // Setup FIFO
//     ESP_ERROR_CHECK(MPU.setFIFOConfig(mpud::FIFO_CFG_ACCEL | mpud::FIFO_CFG_GYRO));
//     ESP_ERROR_CHECK(MPU.setFIFOEnabled(true));

//     // Setup Interrupt
//     constexpr gpio_config_t kGPIOConfig{
//         .pin_bit_mask = (uint64_t) 0x1 << kInterruptPin,
//         .mode         = GPIO_MODE_INPUT,
//         .pull_up_en   = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_ENABLE,
//         .intr_type    = GPIO_INTR_POSEDGE  //
//     };
//     gpio_config(&kGPIOConfig);
//     gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
//     gpio_isr_handler_add((gpio_num_t) kInterruptPin, mpuISR, xTaskGetCurrentTaskHandle());
//     ESP_ERROR_CHECK(MPU.setInterruptConfig(kInterruptConfig));
//     ESP_ERROR_CHECK(MPU.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY));

//     // Ready to start reading
//     ESP_ERROR_CHECK(MPU.resetFIFO());  // start clean


//     }

// void Gyro::update_yaw(float *yaw_param){

// 		// Wait for notification from mpuISR
//     notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     if (notificationValue > 1) {
//         ESP_LOGW(SPP_TAG, "Task Notification higher than 1, value: %d", (int) notificationValue);
//         MPU.resetFIFO();
//         return;
//     }
//     // Check FIFO count
//     fifocount = MPU.getFIFOCount();

//     if (esp_err_t err = MPU.lastError()) {
//         ESP_LOGE(SPP_TAG, "Error reading fifo count, %#X", err);
//         MPU.resetFIFO();
//         return;
//     }

//     if (fifocount > kFIFOPacketSize * 2) {
//         if (!(fifocount % kFIFOPacketSize)) {
//             ESP_LOGE(SPP_TAG, "Sample Rate too high!, not keeping up the pace!, count: %d", (int) fifocount);
//         }
//         else {
//             ESP_LOGE(SPP_TAG, "FIFO Count misaligned! Expected: %d, Actual: %d", (int) kFIFOPacketSize, (int) fifocount);
//         }
//         MPU.resetFIFO();
//         return;
//     }
//     // Burst read data from FIFO
//     uint8_t buffer[kFIFOPacketSize];
//     if (esp_err_t err = MPU.readFIFO(kFIFOPacketSize, buffer)) {
//         ESP_LOGE(SPP_TAG, "Error reading sensor data, %#X", err);
//         MPU.resetFIFO();
//         return;
//     }

// 	// Format
//     mpud::raw_axes_t rawGyro;
//     rawGyro.z  = buffer[10] << 8 | buffer[11];
//     // Calculate tilt angle
//     // range: (roll[-180,180]  pitch[-90,90]  yaw[-180,180])
//     constexpr float kDeltaTime = 1.f / kSampleRate;
//     gyroYaw = yaw + mpud::math::gyroDegPerSec(rawGyro.z, kGyroFS) * kDeltaTime;
//     yaw = gyroYaw;
//     // correct yaw
//     // if (yaw > 180.f)
//         // yaw -= 360.f;
//     // else if (yaw < -180.f)
//         // yaw += 360.f;

//     *yaw_param = yaw;
// }

// void Gyro::read(float * pitch_param, float* yaw_param, float* roll_param){


// 		// Wait for notification from mpuISR
//     notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     if (notificationValue > 1) {
//         ESP_LOGW(SPP_TAG, "Task Notification higher than 1, value: %d", (int) notificationValue);
//         MPU.resetFIFO();
//         return;
//     }
//     // Check FIFO count
//     fifocount = MPU.getFIFOCount();

//     if (esp_err_t err = MPU.lastError()) {
//         ESP_LOGE(SPP_TAG, "Error reading fifo count, %#X", err);
//         MPU.resetFIFO();
//         return;
//     }

//     if (fifocount > kFIFOPacketSize * 2) {
//         if (!(fifocount % kFIFOPacketSize)) {
//             ESP_LOGE(SPP_TAG, "Sample Rate too high!, not keeping up the pace!, count: %d", (int) fifocount);
//         }
//         else {
//             ESP_LOGE(SPP_TAG, "FIFO Count misaligned! Expected: %d, Actual: %d", (int) kFIFOPacketSize, (int) fifocount);
//         }
//         MPU.resetFIFO();
//         return;
//     }
//     // Burst read data from FIFO
//     uint8_t buffer[kFIFOPacketSize];
//     if (esp_err_t err = MPU.readFIFO(kFIFOPacketSize, buffer)) {
//         ESP_LOGE(SPP_TAG, "Error reading sensor data, %#X", err);
//         MPU.resetFIFO();
//         return;
//     }

//     // Format
//     mpud::raw_axes_t rawAccel, rawGyro;
//     rawAccel.x = buffer[0] << 8 | buffer[1];
//     rawAccel.y = buffer[2] << 8 | buffer[3];
//     rawAccel.z = buffer[4] << 8 | buffer[5];
//     rawGyro.x  = buffer[6] << 8 | buffer[7];
//     rawGyro.y  = buffer[8] << 8 | buffer[9];
//     rawGyro.z  = buffer[10] << 8 | buffer[11];
//     // Calculate tilt angle
//     // range: (roll[-180,180]  pitch[-90,90]  yaw[-180,180])
//     constexpr double kRadToDeg = 57.2957795131;
//     constexpr float kDeltaTime = 1.f / kSampleRate;
//     gyroRoll             = roll + mpud::math::gyroDegPerSec(rawGyro.x, kGyroFS) * kDeltaTime;
//     gyroPitch            = pitch + mpud::math::gyroDegPerSec(rawGyro.y, kGyroFS) * kDeltaTime;
//     gyroYaw              = yaw + mpud::math::gyroDegPerSec(rawGyro.z, kGyroFS) * kDeltaTime;
//     accelRoll            = atan2(-rawAccel.x, rawAccel.z) * kRadToDeg;
//     accelPitch = atan2(rawAccel.y, sqrt(rawAccel.x * rawAccel.x + rawAccel.z * rawAccel.z)) * kRadToDeg;
//     // Fusion
//     roll  = gyroRoll * 0.95f + accelRoll * 0.05f;
//     pitch = gyroPitch * 0.95f + accelPitch * 0.05f;
//     yaw   = gyroYaw;
//     // correct yaw
//     if (yaw > 180.f)
//         yaw -= 360.f;
//     else if (yaw < -180.f)
//         yaw += 360.f;

//     *pitch_param = pitch;
//     *yaw_param = yaw;
//     *roll_param =roll;

// }

// static IRAM_ATTR void mpuISR(TaskHandle_t taskHandle)
// {
//     BaseType_t HPTaskWoken = pdFALSE;
//     vTaskNotifyGiveFromISR(taskHandle, &HPTaskWoken);
//     if (HPTaskWoken == pdTRUE) portYIELD_FROM_ISR();
// }
