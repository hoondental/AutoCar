#include <Arduino.h>
#include "motors.h"
#include "imu.h"
#include "radio_control.h"


#define PIN_LED_PINK PC13
#define PIN_BUZZER PC5
#define PIN_KEY1 PD2
#define PIN_USART1_RX PA10
#define PIN_USART1_TX PA9
#define PIN_SBUS_RX PA3
#define PIN_RGB PB5

#define PIN_MPU_SDI PB15
#define PIN_MPU_SDO PB14
#define PIN_MPU_SCLK PB13
#define PIN_MPU_NSS PB12

#define PIN_MOTOR1_A PC6 // TIM8_CH1
#define PIN_MOTOR1_B PC7 // TIM8_CH2
#define PIN_MOTOR2_A PC8 // TIM8_CH3
#define PIN_MOTOR2_B PC9 // TIM8_CH4
#define PIN_MOTOR3_A PA11 // TIM1_CH4
#define PIN_MOTOR3_B PA8 // TIM1_CH1
#define PIN_MOTOR4_A PB0 // TIM1_CH2N
#define PIN_MOTOR4_B PB1 // TIM1_CH3N

#define PIN_ENCODER1_A PA15 // TIM2_CH1
#define PIN_ENCODER1_B PB3 // TIM2_CH2
#define PIN_ENCODER2_A PB6 // TIM4_CH1
#define PIN_ENCODER2_B PB7 // TIM4_CH2
#define PIN_ENCODER3_A PA0 // TIM5_CH1
#define PIN_ENCODER3_B PA1 // TIM5_CH2
#define PIN_ENCODER4_A PA6 // TIM3_CH1
#define PIN_ENCODER4_B PA7 // TIM3_CH2



// timer
#define TIM_MOTOR1 TIM8
#define TIM_MOTOR2 TIM8
#define TIM_MOTOR3 TIM1
#define TIM_MOTOR4 TIM1

#define TIM_ENCODER1 TIM2
#define TIM_ENCODER2 TIM4
#define TIM_ENCODER3 TIM5
#define TIM_ENCODER4 TIM3

#define TIM_CCR_MOTOR1_A TIM8->CCR1
#define TIM_CCR_MOTOR1_B TIM8->CCR2
#define TIM_CCR_MOTOR2_A TIM8->CCR3
#define TIM_CCR_MOTOR2_B TIM8->CCR4
#define TIM_CCR_MOTOR3_A TIM1->CCR4
#define TIM_CCR_MOTOR3_B TIM1->CCR1
#define TIM_CCR_MOTOR4_A TIM1->CCR2
#define TIM_CCR_MOTOR4_B TIM1->CCR3





