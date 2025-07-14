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

HardwareSerial Serial1(PA10, PA9);
EncoderMotors& encoder_motors = EncoderMotors::getInstance();
RC& rc = RC::getInstance(&Serial2, true, false);
MPU9250I2C& mpu = MPU9250I2C::getInstance(GPIOB, GPIO_PIN_15, GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_14, MPU_I2C_SPEED_KHZ);
//MPU9250SPI& mpu = MPU9250SPI::getInstance(GPIOB, GPIO_PIN_15, GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_12);




// global shared variable for motor control
MotorsPWMTarget global_motors_pwm_target;
EncodersCounter global_encoders_counter;
EncodersCounterDiff global_encoders_counter_diff;
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

  // local copy of the encoder counters
  EncodersCounter encoders_counter;
  EncodersCounterDiff encoders_counter_diff;

  for (;;) {
    encoders_counter.cnt1 = TIM_ENCODER1->CNT;
    encoders_counter.cnt2 = TIM_ENCODER2->CNT;
    encoders_counter.cnt3 = TIM_ENCODER3->CNT;
    encoders_counter.cnt4 = TIM_ENCODER4->CNT;

    encoders_counter_diff.diff1 = encoders_counter.cnt1 - global_encoders_counter.cnt1;
    encoders_counter_diff.diff2 = encoders_counter.cnt2 - global_encoders_counter.cnt2;
    encoders_counter_diff.diff3 = encoders_counter.cnt3 - global_encoders_counter.cnt3;
    encoders_counter_diff.diff4 = encoders_counter.cnt4 - global_encoders_counter.cnt4;
    
    taskENTER_CRITICAL();
    global_encoders_counter = encoders_counter;
    global_encoders_counter_diff = encoders_counter_diff;
    taskEXIT_CRITICAL();

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
    EncodersCounter encoders_counter = global_encoders_counter;
    EncodersCounterDiff encoders_counter_diff = global_encoders_counter_diff;
    taskEXIT_CRITICAL();

    Serial1.print(encoders_counter.cnt1);
    Serial1.print(" ");
    Serial1.print(encoders_counter.cnt2);
    Serial1.print(" ");
    Serial1.print(encoders_counter.cnt3);
    Serial1.print(" ");
    Serial1.print(encoders_counter.cnt4);
    Serial1.print("  ||   ");
    Serial1.print(encoders_counter_diff.diff1);
    Serial1.print(" ");
    Serial1.print(encoders_counter_diff.diff2);
    Serial1.print(" ");
    Serial1.print(encoders_counter_diff.diff3);
    Serial1.print(" ");
    Serial1.print(encoders_counter_diff.diff4);
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
  xTaskCreate(BuzzTask, "Buzz", 128, NULL, 1, NULL);
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
