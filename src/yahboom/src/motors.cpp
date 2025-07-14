#include <Arduino.h>
#include "motors.h"



void EncoderMotors::enableGPIOs() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // ====== PWM Pins ====== 
    //TIM8: PC6 (CH1), PC7 (CH2), PC8 (CH3), PC9 (CH4)
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // TIM1: PA8 (CH1), PA11 (CH4)
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_11;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // TIM3: PB0 (CH3), PB1 (CH4)
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // ====== Encoder Pins ======
    // TIM2: PA15, PB3
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT; // Or AF_INPUT for some cores
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // TIM4: PB6, PB7
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // TIM5: PA0, PA1
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // TIM3 (Encoder): PA6, PA7
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void EncoderMotors::setupPWMs(uint32_t pwm_frequency, uint32_t pwm_resolution) {
    uint32_t systemClock = HAL_RCC_GetHCLKFreq();  // Usually 72MHz
    uint32_t prescaler = (systemClock / (pwm_frequency * pwm_resolution)) - 1;

    // --- Enable Clocks ---
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    // --- TIM8: Motor 1 and 2 ---
    TIM8->PSC = prescaler;                  // Prescaler
    TIM8->ARR = pwm_resolution - 1;         // Auto-reload

    // Channel 1 (Motor1 A)
    TIM8->CCMR1 |= (6 << 4);                // PWM Mode 1 (OC1M = 110)
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE;         // Preload enable
    TIM8->CCER  |= TIM_CCER_CC1E;           // Enable output

    // Channel 2 (Motor1 B)
    TIM8->CCMR1 |= (6 << 12);               // PWM Mode 1 (OC2M = 110)
    TIM8->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM8->CCER  |= TIM_CCER_CC2E;

    // Channel 3 (Motor2 A)
    TIM8->CCMR2 |= (6 << 4);                // PWM Mode 1 (OC3M = 110)
    TIM8->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM8->CCER  |= TIM_CCER_CC3E;

    // Channel 4 (Motor2 B)
    TIM8->CCMR2 |= (6 << 12);
    TIM8->CCMR2 |= TIM_CCMR2_OC4PE;
    TIM8->CCER  |= TIM_CCER_CC4E;

    TIM8->CR1 |= TIM_CR1_ARPE;              // Auto-reload preload enable
    TIM8->EGR |= TIM_EGR_UG;                // Generate update event to load registers
    TIM8->BDTR |= TIM_BDTR_MOE;             // Main output enable (Advanced timer)
    TIM8->CR1 |= TIM_CR1_CEN;               // Start timer

    // --- TIM1: Motor 3 and 4 ---
    TIM1->PSC = prescaler;
    TIM1->ARR = PWM_RESOLUTION - 1;

    // Channel 4 (Motor3 A)
    TIM1->CCMR2 |= (6 << 12);
    TIM1->CCMR2 |= TIM_CCMR2_OC4PE;
    TIM1->CCER  |= TIM_CCER_CC4E;

    // Channel 1 (Motor3 B)
    TIM1->CCMR1 |= (6 << 4);
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM1->CCER  |= TIM_CCER_CC1E;

    // Channel 2 (Motor4 A) - complementary output
    TIM1->CCMR1 |= (6 << 12);
    TIM1->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM1->CCER  |= TIM_CCER_CC2E | TIM_CCER_CC2NE; // Normal + Complementary

    // Channel 3 (Motor4 B) - complementary output
    TIM1->CCMR2 |= (6 << 4);
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM1->CCER  |= TIM_CCER_CC3E | TIM_CCER_CC3NE;

    TIM1->CR1 |= TIM_CR1_ARPE;
    TIM1->EGR |= TIM_EGR_UG;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;

}



void EncoderMotors::setupEncoders() {
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    __HAL_RCC_AFIO_CLK_ENABLE();
    // TIM2 full remap: PA15 (CH1), PB3 (CH2)
    AFIO->MAPR &= ~(AFIO_MAPR_TIM2_REMAP);  // Clear
    AFIO->MAPR |= 0x3 << 8; // AFIO_MAPR_TIM2_REMAP_FULL;  // Full remap
  
    // TIM3 full remap: PB4 (CH1), PB5 (CH2)
    AFIO->MAPR &= ~(AFIO_MAPR_TIM3_REMAP);
    //AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_FULL;

    // Initialize TIM2 for Encoder 1
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1 = 0;  // &= ~TIM_CR1_DIR; // Clear direction bit
    TIM2->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; // Encoder mode 3
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0; // IC1 and IC2 mapped to TI1 and TI2
    TIM2->CCER = 0; // &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P); // Set polarity
    TIM2->ARR = 0xFFFF; // 32-bit counter
    TIM2->CR1 |= TIM_CR1_CEN;
  
    // Initialize TIM4 for Encoder 2
    __HAL_RCC_TIM4_CLK_ENABLE();
    TIM4->CR1 = 0; //&= ~TIM_CR1_DIR;
    TIM4->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    TIM4->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
    TIM4->CCER = 0; //&= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM4->ARR = 0xFFFF; // 16-bit counter
    TIM4->CR1 |= TIM_CR1_CEN;
  
    // Initialize TIM5 for Encoder 3
    __HAL_RCC_TIM5_CLK_ENABLE();
    TIM5->CR1 = 0; //&= ~TIM_CR1_DIR;
    TIM5->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    TIM5->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
    TIM5->CCER = 0; // &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM5->ARR = 0xFFFF; // 32-bit counter
    TIM5->CR1 |= TIM_CR1_CEN;
  
    // Initialize TIM3 for Encoder 4
    __HAL_RCC_TIM3_CLK_ENABLE();
    TIM3->CR1 = 0; //&= ~TIM_CR1_DIR;
    TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    TIM3->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
    TIM3->CCER = 0; //&= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM3->ARR = 0xFFFF; // 16-bit counter
    TIM3->CR1 |= TIM_CR1_CEN;
}