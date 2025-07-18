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


// ==================================== SG2Filter =======================================
// encoder 입력을 받아서 SG filter 를 거친 후 angle, angular velocity, angular acceleration 을 출력

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



// ============================== Car Model =================================
// 차의 방향은 오른쪽을 x축, 정면을 y축으로 함. 
// 각은 x축에서부터 반시계방향으로 측정


enum WheelPosition: size_t {
    FRONT_RIGHT,
    FRONT_LEFT,
    REAR_LEFT,
    REAR_RIGHT    
};

class CarModel {
private:
    float_t _wheel_radius;
    float_t _wheel_weight;
    float_t _wheel_moment_of_inertia;
    float_t _wheel_roller_free_angles[4];
    float_t _wheel_center_xs[4];
    float_t _wheel_center_ys[4];
    bool _is_symmetric;

    // 현재 추정 위치 속도 및 가속도
    float_t _car_x, _car_y, _car_theta;
    float_t _car_vx, _car_vy, _car_omega;
    float_t _car_ax, _car_ay, _car_alpha;

    


public:

    void compute_wheel_angular_velocity_from_car_velocity(float_t vx, float_t vy, float_t omega, 
                        float_t& w1, float_t& w2, float_t& w3, float_t& w4) {
        // wheel velocity
        // V_wheel = V_car + omega X r_wheel (as vector)
        float_t vx1 = vx - omega * _wheel_center_ys[FRONT_RIGHT]; 
        float_t vx2 = vx - omega * _wheel_center_ys[FRONT_LEFT];
        float_t vx3 = vx - omega * _wheel_center_ys[REAR_LEFT];
        float_t vx4 = vx - omega * _wheel_center_ys[REAR_RIGHT];
        float_t vy1 = vy + omega * _wheel_center_xs[FRONT_RIGHT];
        float_t vy2 = vy + omega * _wheel_center_xs[FRONT_LEFT];
        float_t vy3 = vy + omega * _wheel_center_xs[REAR_LEFT];
        float_t vy4 = vy + omega * _wheel_center_xs[REAR_RIGHT];

        // angular velocity
        // example
        /*
        vx1 = v1_free; // t = 
        vy1 = _wheel_radius * w1 + v1_free;
        vy1 - vx1 = _wheel_radius * w1;
        w1 = (vy - vx + omega * (_wheel_center_xs[FRONT_RIGHT] + _wheel_center_ys[FRONT_RIGHT])) / _wheel_radius; 
        */
        w1 = (vy - vx + omega * (_wheel_center_xs[FRONT_RIGHT] + _wheel_center_ys[FRONT_RIGHT])) / _wheel_radius; 
        w2 = (vy + vx + omega * (_wheel_center_xs[FRONT_LEFT] - _wheel_center_ys[FRONT_LEFT])) / _wheel_radius;
        w3 = (vy - vx + omega * (_wheel_center_xs[REAR_LEFT] + _wheel_center_ys[REAR_LEFT])) / _wheel_radius;
        w4 = (vy + vx + omega * (_wheel_center_xs[REAR_RIGHT] - _wheel_center_ys[REAR_RIGHT])) / _wheel_radius;               
    }

    void estimate_car_velocity_from_wheel_angular_velocity(float_t w1, float_t w2, float_t w3, float_t w4,
                        float_t& vx, float_t& vy, float_t& omega) {
        // over-determined problem -> no-solution indicates slippage
        /*
        vx1 = v1_free - slippage1;
        vy1 = _wheel_radius * w1 + v1_free + slippage1;
        vx2 = -v2_free + slippage2;
        v2y = _wheel_radius * w2 + v2_free + slippage2;
        v3x = v3_free - slippage3;
        v3y = _wheel_radius * w3 + v3_free + slippage3;
        v4x = -v4_free + slippage4;
        v4y = _wheel_raidus * w4 + v4_free + slippage4;

        */
        if (_is_symmetric) {
            vx = (w1 + w2 + w3 + w4) * _wheel_radius / 4;
            vy = (w1 - w2 - w3 + w4) / 4;
            omega = (w1 - w2 + w3 - w4) / 4;
        vx = (w1 + w2 + w3 + w4) / 4;
        vy = (w1 - w2 - w3 + w4) / 4;
        omega = (w1 - w2 + w3 - w4) / 4;
    }






};