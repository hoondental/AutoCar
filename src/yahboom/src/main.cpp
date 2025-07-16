#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <task.h>
#include <HardwareTimer.h>
#include <SPI.h>
#include "main.h"
#include "motors.h"
#include "imu.h"
#include "radio_control.h"
#include <sbus.h>
//#include "icm20948.h"

// PWM frequency
#define MOTOR_PWM_FREQUENCY 10000
#define MOTOR_PWM_RESOLUTION 256
#define MPU_I2C_SPEED_KHZ 400
#define MPU_GYRO_RANGE_DPS 500
#define MPU_ACCEL_RANGE_G 2
#define MPU_DATA_RATE_HZ 200
#define MPU_MAG_DATA_RATE_HZ 100 // CONTINUOUS

// filtering
#define FILTER_N_SAMPLES 7

// RC command
#define RC_CHANNEL_MODE 8
#define RC_CHANNEL_LOGNITUDINAL 1
#define RC_CHANNEL_LATERAL 2
#define RC_CHANNEL_YAW 3


HardwareSerial Serial1(PA10, PA9);
EncoderMotors& encoder_motors = EncoderMotors::getInstance();
RC& rc = RC::getInstance(&Serial2, true, false);
MPU9250I2C& mpu = MPU9250I2C::getInstance(GPIOB, GPIO_PIN_15, GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_14, MPU_I2C_SPEED_KHZ);
//MPU9250SPI& mpu = MPU9250SPI::getInstance(GPIOB, GPIO_PIN_15, GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_12);




// global shared variable for motor control
MotorsPWMTarget global_motors_pwm_target;
EncoderReadings global_encoders_readings;
AngularVelocities global_wheel_velocities;
AngularAccelerations global_wheel_accelerations;
bfs::SbusData global_sbus_data;
uint32_t global_lost_time_ms = 0;
MPUData global_mpu_data;


void RCReadTask(void* pvParameters) {
    (void)pvParameters;

    rc.begin();

    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 10ms = 100Hz
    TickType_t xLastWakeTime = xTaskGetTickCount(); 

    for (;;) {
        if (rc.read()) {
            taskENTER_CRITICAL();
            global_sbus_data = rc.data();
            taskEXIT_CRITICAL();
        } 
        global_lost_time_ms = rc.lost_time_ms();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


void MPUReadTask(void* pvParameters) {
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(10000));
    bool status = mpu.begin(MPU_GYRO_RANGE_DPS, MPU_ACCEL_RANGE_G, MPU_DATA_RATE_HZ, MPU_MAG_DATA_RATE_HZ);
    if (status == true) {
        Serial1.println("MPU9250 initialization success!");
    } else {
        Serial1.print("MPU9250 initialization failed!: ");
        Serial1.println(status);
    } 

    MPUData data;

    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 10ms = 100Hz
    TickType_t xLastWakeTime = xTaskGetTickCount(); 

    for (;;) {
        mpu.read();
        taskENTER_CRITICAL();
        global_mpu_data = mpu.data();
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}







void MotorPWMControlTask(void* pvParameters) {
  (void)pvParameters;

  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz
  TickType_t xLastWakeTime = xTaskGetTickCount(); 

  // local copy of the global target
  MotorsPWMTarget target;

  for (;;) {
    taskENTER_CRITICAL();
    target = global_motors_pwm_target;
    taskEXIT_CRITICAL();

    TIM_CCR_MOTOR1_A = target.m1_A;
    TIM_CCR_MOTOR1_B = target.m1_B;
    TIM_CCR_MOTOR2_A = target.m2_A;
    TIM_CCR_MOTOR2_B = target.m2_B;
    TIM_CCR_MOTOR3_A = target.m3_A;
    TIM_CCR_MOTOR3_B = target.m3_B;
    TIM_CCR_MOTOR4_A = target.m4_A;
    TIM_CCR_MOTOR4_B = target.m4_B;
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


void MotorPWMSetTask(void* pvParameters) {
  (void)pvParameters;

  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz
  TickType_t xLastWakeTime = xTaskGetTickCount(); 
  
  // local copy of the global target
  MotorsPWMTarget target;
  int turn_idx = 1;
  
  for (;;) {
    switch (turn_idx){
      case 1:
        target.m1_A += 1;
        if (target.m1_A >= PWM_RESOLUTION) target.m1_A = 0;
        turn_idx += 1;
        break;
      case 2:
        target.m2_A += 1;
        if (target.m2_A >= PWM_RESOLUTION) target.m2_A = 0;
        turn_idx += 1;
        break;
      case 3:
        target.m3_A += 1;
        if (target.m3_A >= PWM_RESOLUTION) target.m3_A = 0;
        turn_idx += 1;
        break;
      case 4:
        target.m4_A += 1;
        if (target.m4_A >= PWM_RESOLUTION) target.m4_A = 0;
        turn_idx = 1;
        break;
    }
    taskENTER_CRITICAL();
    global_motors_pwm_target = target;
    taskEXIT_CRITICAL();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}






void EncoderReadTask(void* pvParameters) {
  (void)pvParameters;

  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz
  TickType_t xLastWakeTime = xTaskGetTickCount(); 

  // 4 filters for each encoder
  float_t dt = 0.01;
  uint32_t n = FILTER_N_SAMPLES;
  uint32_t ppr = ENCODER_PPR;
  SG2Filter filter1(n, dt, ppr), filter2(n, dt, ppr), filter3(n, dt, ppr), filter4(n, dt, ppr);
  float_t a10, a11, a12, a20, a21, a22, a30, a31, a32, a40, a41, a42;
  // local copy of the encoder counters
  EncoderReadings readings;

  for (;;) {
    readings = encoder_motors.readEncoders();
    filter1.fit(readings.enc1, a10, a11, a12);
    filter2.fit(readings.enc2, a20, a21, a22);
    filter3.fit(readings.enc3, a30, a31, a32);
    filter4.fit(readings.enc4, a40, a41, a42);

    global_encoders_readings = readings;
    global_wheel_velocities.vel1 = a11;
    global_wheel_velocities.vel2 = a21;
    global_wheel_velocities.vel3 = a31;
    global_wheel_velocities.vel4 = a41;
    global_wheel_accelerations.acc1 = 2.0 * a12;
    global_wheel_accelerations.acc2 = 2.0 * a22;
    global_wheel_accelerations.acc3 = 2.0 * a32;
    global_wheel_accelerations.acc4 = 2.0 * a42;
    
    //global_encoders_readings = encoder_motors.getReading();
    //global_wheel_velocities = encoder_motors.getAngularVelocity();
    //global_wheel_accelerations = encoder_motors.getAngularAcceleration();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}



void Serial1CastTask(void* pvParameters) {
  (void)pvParameters;

  const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 10ms = 100Hz
  TickType_t xLastWakeTime = xTaskGetTickCount(); 
  
  for (;;) {
    taskENTER_CRITICAL();
    MotorsPWMTarget target = global_motors_pwm_target;
    taskEXIT_CRITICAL();

    Serial1.print(global_wheel_velocities.vel1);
    Serial1.print(" ");
    Serial1.print(global_wheel_velocities.vel2);
    Serial1.print(" ");
    Serial1.print(global_wheel_velocities.vel3);
    Serial1.print(" ");
    Serial1.print(global_wheel_velocities.vel4);
    Serial1.print("  ||   ");
    Serial1.print(global_wheel_accelerations.acc1);
    Serial1.print(" ");
    Serial1.print(global_wheel_accelerations.acc2);
    Serial1.print(" ");
    Serial1.print(global_wheel_accelerations.acc3);
    Serial1.print(" ");
    Serial1.print(global_wheel_accelerations.acc4);
    Serial1.print("  ||   ");
    Serial1.print(target.m1_A);
    Serial1.print(" ");
    Serial1.print(target.m2_A);
    Serial1.print(" ");
    Serial1.print(target.m3_A);
    Serial1.print(" ");
    Serial1.print(target.m4_A);

    Serial1.print("  ||   ");
    for (int i = 0; i < 16; i++) {
       Serial1.print(global_sbus_data.ch[i]);
       Serial1.print(" ");
    }

    Serial1.print(global_sbus_data.failsafe);
    Serial1.print(" ");
    Serial1.print(global_sbus_data.lost_frame);
    Serial1.print(" ");
    Serial1.print(global_lost_time_ms);
    Serial1.print(" ");


    Serial1.print("  ||   ");
    Serial1.print(global_mpu_data.accel.x);
    Serial1.print(" ");
    Serial1.print(global_mpu_data.accel.y);
    Serial1.print(" ");
    Serial1.print(global_mpu_data.accel.z);
    Serial1.print("  ||   ");
    Serial1.print(global_mpu_data.gyro.x);
    Serial1.print(" ");
    Serial1.print(global_mpu_data.gyro.y);
    Serial1.print(" ");
    Serial1.print(global_mpu_data.gyro.z);
    Serial1.print("  ||   ");
    Serial1.print(global_mpu_data.mag.x);
    Serial1.print(" ");
    Serial1.print(global_mpu_data.mag.y);
    Serial1.print(" ");
    Serial1.print(global_mpu_data.mag.z);
    Serial1.print("  ||   ");
    Serial1.print(global_mpu_data.temp);

    Serial1.print("\n");


    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  } 
}


void BlinkTask(void* pvParameters) {
  (void)pvParameters;

  pinMode(PIN_LED_PINK, OUTPUT);
  vTaskDelay(pdMS_TO_TICKS(100));

  const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 10ms = 100Hz
  TickType_t xLastWakeTime = xTaskGetTickCount(); 

  for (;;) {
    digitalWrite(PIN_LED_PINK, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(PIN_LED_PINK, LOW);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


void BuzzTask(void* pvParameters) {
  (void)pvParameters;

  pinMode(PIN_BUZZER, OUTPUT);
  vTaskDelay(pdMS_TO_TICKS(50));

  const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 10ms = 100Hz
  TickType_t xLastWakeTime = xTaskGetTickCount(); 

  for (;;) {
    digitalWrite(PIN_BUZZER, HIGH);
    vTaskDelay(pdMS_TO_TICKS(50));
    digitalWrite(PIN_BUZZER, LOW);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/*
// This function will run very early, before setup()
void early_gpio_setup(void) __attribute__((constructor));
void early_gpio_setup(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);  // Drive CS LOW
}
*/

void setup() {


  Serial1.begin(115200);
  pinMode(PIN_KEY1, INPUT_PULLUP);

  encoder_motors.begin(MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
  delay(10);  


  // Start FreeRTOS task
  xTaskCreate(BlinkTask, "Blink", 128, NULL, 1, NULL);
  //xTaskCreate(BuzzTask, "Buzz", 128, NULL, 1, NULL);
  //xTaskCreate(MotorPWMSetTask, "MotorPWMSet", 128, NULL, 1, NULL);
  //xTaskCreate(MotorPWMControlTask, "MotorPWMControl", 128, NULL, 1, NULL);
  xTaskCreate(EncoderReadTask, "EncoderRead", 128, NULL, 1, NULL); 
  xTaskCreate(Serial1CastTask, "Serial1Cast", 128, NULL, 1, NULL);
  xTaskCreate(MPUReadTask, "MPURead", 128, NULL, 1, NULL);
  xTaskCreate(RCReadTask, "RCRead", 128, NULL, 1, NULL);

  vTaskStartScheduler();
}

void loop() {
  // Nothing here; everything is in tasks
}
