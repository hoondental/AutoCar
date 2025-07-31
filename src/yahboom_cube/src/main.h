/* #include <Arduino.h>
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

*/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_PINK_Pin GPIO_PIN_13
#define LED_PINK_GPIO_Port GPIOC
#define S4_Pin GPIO_PIN_0
#define S4_GPIO_Port GPIOC
#define S3_Pin GPIO_PIN_1
#define S3_GPIO_Port GPIOC
#define S2_Pin GPIO_PIN_2
#define S2_GPIO_Port GPIOC
#define S1_Pin GPIO_PIN_3
#define S1_GPIO_Port GPIOC
#define BEEP_Pin GPIO_PIN_5
#define BEEP_GPIO_Port GPIOC
#define MPU_SCL_Pin GPIO_PIN_13
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_AD0_Pin GPIO_PIN_14
#define MPU_AD0_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_15
#define MPU_SDA_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


