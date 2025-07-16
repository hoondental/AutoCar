#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "arm_math.h"

#define ENCODER_PPR 1320


// in radians / second
// can be used for Encoder readings or for Motor speeds
struct AngularVelocities {
    float vel1;
    float vel2;
    float vel3;
    float vel4;
};

struct AngularAccelerations {
    float acc1;
    float acc2;
    float acc3;
    float acc4;
};


struct EncoderReadings {
    // ARR = 0xFFFF
    uint16_t enc1;
    uint16_t enc2;
    uint16_t enc3;
    uint16_t enc4;
    uint32_t cycles_at_reading; // DWT->CYCCNT
};





struct MotorPWMs {
    uint32_t pwm1_A;
    uint32_t pwm1_B;
    uint32_t pwm2_A;
    uint32_t pwm2_B;
    uint32_t pwm3_A;
    uint32_t pwm3_B;
    uint32_t pwm4_A;
    uint32_t pwm4_B;
};




class EncoderMotors {
private:

    uint32_t _pwm_frequency = 10000;
    uint32_t _pwm_resolution = 256;
    uint32_t _encoder_PPR = 1320;

    //EncoderReadings _reading_0 = {0, 0, 0, 0, 0}; // D-0
    //EncoderReadings _reading_1 = {0, 0, 0, 0, 0}; // D-1
    //EncoderReadings _reading_2 = {0, 0, 0, 0, 0}; // D-2

    //AngularVelocities _angular_velocity_01 = {0, 0, 0, 0};
    //AngularVelocities _angular_velocity_12 = {0, 0, 0, 0};
    //AngularAccelerations _angular_acceleration_012 = {0, 0, 0, 0};


    const TIM_TypeDef* timer_M1 = TIM8;
    const TIM_TypeDef* timer_M2 = TIM8;
    const TIM_TypeDef* timer_M3 = TIM1;
    const TIM_TypeDef* timer_M4 = TIM1;
    const TIM_TypeDef* timer_E1 = TIM2;
    const TIM_TypeDef* timer_E2 = TIM4;
    const TIM_TypeDef* timer_E3 = TIM5;
    const TIM_TypeDef* timer_E4 = TIM3;

    volatile uint32_t& CCR_M1_A = TIM8->CCR1;
    volatile uint32_t& CCR_M1_B = TIM8->CCR2;
    volatile uint32_t& CCR_M2_A = TIM8->CCR3;
    volatile uint32_t& CCR_M2_B = TIM8->CCR4;
    volatile uint32_t& CCR_M3_A = TIM1->CCR4;
    volatile uint32_t& CCR_M3_B = TIM1->CCR1;
    volatile uint32_t& CCR_M4_A = TIM1->CCR2;
    volatile uint32_t& CCR_M4_B = TIM1->CCR3;

    void enableGPIOs();
    void setupPWMs(uint32_t pwm_frequency = 10000, uint32_t pwm_resolution = 256);
    void setupEncoders(uint32_t encoder_PPR = 1320);

    // singleton 
    EncoderMotors() {}
    EncoderMotors(const EncoderMotors&) = delete;
    EncoderMotors& operator=(const EncoderMotors&) = delete;
    
public:
    static EncoderMotors& getInstance() {
        static EncoderMotors instance;
        return instance;
    }

    void begin(uint32_t pwm_frequency = 10000, uint32_t pwm_resolution = 256, uint32_t encoder_PPR = 1320) {
        enableGPIOs();
        setupPWMs(pwm_frequency, pwm_resolution);
        setupEncoders(encoder_PPR);
    }
    
    inline void setPWMs(uint32_t pwm1_A, uint32_t pwm1_B, uint32_t pwm2_A, uint32_t pwm2_B, 
                        uint32_t pwm3_A, uint32_t pwm3_B, uint32_t pwm4_A, uint32_t pwm4_B) {
        CCR_M1_A = pwm1_A;
        CCR_M1_B = pwm1_B;
        CCR_M2_A = pwm2_A;
        CCR_M2_B = pwm2_B;
        CCR_M3_A = pwm3_A;
        CCR_M3_B = pwm3_B;
        CCR_M4_A = pwm4_A;
        CCR_M4_B = pwm4_B;
    }

    inline void setPWMs(const MotorPWMs& pwms) {
        setPWMs(pwms.pwm1_A, pwms.pwm1_B, pwms.pwm2_A, pwms.pwm2_B, pwms.pwm3_A, pwms.pwm3_B, pwms.pwm4_A, pwms.pwm4_B);
    }

    inline EncoderReadings readEncoders() {
        EncoderReadings reading;
        taskENTER_CRITICAL();
        reading.enc1 = timer_E1->CNT;
        reading.enc2 = timer_E2->CNT;
        reading.enc3 = timer_E3->CNT;
        reading.enc4 = timer_E4->CNT;
        reading.cycles_at_reading = DWT->CYCCNT; // Update last reading time in milliseconds
        taskEXIT_CRITICAL();
        return reading;
    }    
};


class SG2Filter {
private:
    uint32_t _encoder_PPR = 1320;
    
    uint16_t _N_samples;
    float_t _dt;
    arm_matrix_instance_f32 _ATAiAT; // 3 x N_samples, a0, a1, a2 order
    float_t *_ATAiAT_data;
    //float_t *_buf;
    uint16_t *_buf_u16;
    size_t _idx0; 
    
    SG2Filter() {}

    bool generateFilter();

public:
    SG2Filter(uint32_t N_samples, float_t dt, uint32_t encoder_PPR = 1320) {
        _encoder_PPR = encoder_PPR;
        _N_samples = N_samples;
        _dt = dt;
        _ATAiAT_data = new float_t[3 * N_samples];
        //_buf = new float_t[N_samples]();  // initialize to zero
        _buf_u16 = new uint16_t[N_samples]();
        _idx0 = N_samples - 1;
        generateFilter();
    }

    //void fit(float_t x, float_t& a0, float_t& a1, float_t& a2);
    void fit(uint16_t x, float_t& a0, float_t& a1, float_t& a2);


};