#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "arm_math.h"

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
    _pwm_frequency = pwm_frequency;
    _pwm_resolution = pwm_resolution;
    
    // Calculate prescaler based on system clock and desired PWM frequency
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



void EncoderMotors::setupEncoders(uint32_t encoder_PPR) {
    _encoder_PPR = encoder_PPR;

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

/*
void EncoderMotors::readEncoders() {
        taskENTER_CRITICAL();
        _reading_2 = _reading_1; // Store previous reading
        _reading_1 = _reading_0;
        _angular_velocity_12 = _angular_velocity_01;
        _reading_0.enc1 = timer_E1->CNT;
        _reading_0.enc2 = timer_E2->CNT;
        _reading_0.enc3 = timer_E3->CNT;
        _reading_0.enc4 = timer_E4->CNT;
        _reading_0.cycles_at_reading = DWT->CYCCNT; // Update last reading time in milliseconds
        taskEXIT_CRITICAL();

        static const float_t _system_clock = SystemCoreClock;
        static const float_t _to_radian = 2.0 * M_PI / _encoder_PPR;

        // calculate angular velocity        
        uint32_t d_cycles_01 = _reading_0.cycles_at_reading - _reading_1.cycles_at_reading;
        if (d_cycles_01 == 0) d_cycles_01 = 1;
        float_t _scale_v = _to_radian / d_cycles_01 * _system_clock;
        _angular_velocity_01.vel1 = _scale_v * int32_t(_reading_0.enc1 - _reading_1.enc1);
        _angular_velocity_01.vel2 = _scale_v * int32_t(_reading_0.enc2 - _reading_1.enc2);
        _angular_velocity_01.vel3 = _scale_v * int32_t(_reading_0.enc3 - _reading_1.enc3);
        _angular_velocity_01.vel4 = _scale_v * int32_t(_reading_0.enc4 - _reading_1.enc4);

        // calculate angular acceleration        
        uint32_t d_cycles_12 = _reading_1.cycles_at_reading - _reading_2.cycles_at_reading;
        if (d_cycles_12 == 0) d_cycles_12 = 1;
        uint32_t d_cycles_02 = d_cycles_01 + d_cycles_12;
        float_t _scale_a = 2.0 / d_cycles_02 * _system_clock;
        _angular_acceleration_012.acc1 = _scale_a * (_angular_velocity_01.vel1 - _angular_velocity_12.vel1);
        _angular_acceleration_012.acc2 = _scale_a * (_angular_velocity_01.vel2 - _angular_velocity_12.vel2);
        _angular_acceleration_012.acc3 = _scale_a * (_angular_velocity_01.vel3 - _angular_velocity_12.vel3);
        _angular_acceleration_012.acc4 = _scale_a * (_angular_velocity_01.vel4 - _angular_velocity_12.vel4);
     }
*/


     // ============================== SGFilter =================================
     bool SG2Filter::generateFilter(uint32_t N_samples, float_t dt) {
        float_t A_data[N_samples * 3];
        float_t AT_data[3 * N_samples];
        float_t ATA_data[3 * 3];
        float_t ATAi_data[3 * 3];
        for (int i=0; i<N_samples; i++) {
            float_t j = -(float_t(N_samples) - 1.0 - i);
            A_data[i * N_samples] = 1.0;
            A_data[i * N_samples + 1] = j * dt;
            A_data[i * N_samples + 2] = pow(j * dt, 2.0);
        }

        arm_matrix_instance_f32 A, AT, ATA, ATAi, ATAiAT;
        arm_mat_init_f32(&A, N_samples, 3, A_data);
        arm_mat_init_f32(&AT, 3, N_samples, AT_data);
        arm_mat_init_f32(&ATA, 3, 3, ATA_data);
        arm_mat_init_f32(&ATAi, 3, 3, ATAi_data);
        arm_mat_init_f32(&_ATAiAT, 3, N_samples, _ATAiAT_data);
        if (arm_mat_trans_f32(&A, &AT) != ARM_MATH_SUCCESS) return false;
        if (arm_mat_mult_f32(&AT, &A, &ATA) != ARM_MATH_SUCCESS) return false;
        if (arm_mat_inverse_f32(&ATA, &ATAi) != ARM_MATH_SUCCESS) return false;
        if (arm_mat_mult_f32(&ATAi, &AT, &_ATAiAT) != ARM_MATH_SUCCESS) return false;
        return true;
     }

void SG2Filter::fit(float_t x, float_t& a0, float_t& a1, float_t& a2) {
    _buf[_idx0] = x;
    _idx0 = (_idx0 + 1) % _N_samples;
    int k = 0;

    a0 = 0;
    for (int i=0; i<_N_samples; i++) {
        k = (i + _idx0) % _N_samples;
        a0 += _buf[k] * _ATAiAT_data[0, i];
    }
    a1 = 0;
    for (int i=0; i<_N_samples; i++) {
        k = (i + _idx0) % _N_samples;
        a1 += _buf[k] * _ATAiAT_data[1, i];
    }
    a2 = 0;
    for (int i=0; i<_N_samples; i++) {
        k = (i + _idx0) % _N_samples;
        a2 += _buf[k] * _ATAiAT_data[2, i];
    }
}