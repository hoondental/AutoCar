#include <Arduino.h>
#include <STM32FreeRTOS.h>


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
    uint32_t enc1;
    uint32_t enc2;
    uint32_t enc3;
    uint32_t enc4;
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

    EncoderReadings _reading_0 = {0, 0, 0, 0, 0}; // D-0
    EncoderReadings _reading_1 = {0, 0, 0, 0, 0}; // D-1
    EncoderReadings _reading_2 = {0, 0, 0, 0, 0}; // D-2

    AngularVelocities _angular_velocity = {0, 0, 0, 0};
    AngularAccelerations _angular_acceleration = {0, 0, 0, 0};


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

    inline void readEncoders() {
        taskENTER_CRITICAL();
        _reading_2 = _reading_1; // Store previous reading
        _reading_1 = _reading_0;
        _reading_0.enc1 = timer_E1->CNT;
        _reading_0.enc2 = timer_E2->CNT;
        _reading_0.enc3 = timer_E3->CNT;
        _reading_0.enc4 = timer_E4->CNT;
        _reading_0.cycles_at_reading = DWT->CYCCNT / 1000; // Update last reading time in milliseconds
        taskEXIT_CRITICAL();
        // calculate angular velocity

        // calculate angular acceleration
    }

    inline const EncoderReadings getReading0() { return _reading_0; }
    inline const EncoderReadings getReading1() { return _reading_1; }
    inline const EncoderReadings getReading2() { return _reading_2; }

    inline AngularVelocities getAngularVelocities() {
        static const float coeff = 2.0 * M_PI / _encoder_PPR * SystemCoreClock;
        uint32_t d_cycles = _reading_current.cycles_at_reading - _reading_previous.cycles_at_reading;
        float_t _scale = coeff / d_cycles;
        uint32_t d_theta_1 = _reading_current.enc1 - _reading_previous.enc1;
        uint32_t d_theta_2 = _reading_current.enc2 - _reading_previous.enc2;
        uint32_t d_theta_3 = _reading_current.enc3 - _reading_previous.enc3;
        uint32_t d_theta_4 = _reading_current.enc4 - _reading_previous.enc4;
        
        AngularVelocities velocities;
        velocities.vel1 = _scale * d_theta_1;
        velocities.vel2 = _scale * d_theta_2;
        velocities.vel3 = _scale * d_theta_3;
        velocities.vel4 = _scale * d_theta_4;
        return velocities;
    }

    
    
};