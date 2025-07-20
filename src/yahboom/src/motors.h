#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "arm_math.h"

#define ENCODER_PPR 1320


// in radians / second
// can be used for Encoder readings or for Motor speeds
struct AngularVelocities {
    float_t vel1, vel2, vel3, vel4;
};

struct AngularAccelerations {
    float_t acc1, acc2, acc3, acc4;
};


struct EncoderReadings {
    // ARR = 0xFFFF
    uint16_t enc1, enc2, enc3, enc4;
    uint32_t cycles_at_reading; // DWT->CYCCNT
};


struct CarPosition {
    float_t x, y, theta;
};

struct CarVelocity {
    float_t vx, vy, omega;
};

struct CarAcceleration {
    float_t ax, ay, alpha;
}







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


enum WheelNumber: size_t {
    FRONT_RIGHT,
    FRONT_LEFT,
    REAR_LEFT,
    REAR_RIGHT    
};

struct WheelGeometry {
    float_t radius;
    float_t weight;
    float_t moment_of_inertia;
    float_t free_angle;
    float_t free_cos;
    float_t free_sin;
    float_t free_tan;
    float_t free_cot;
    float_t center_x;
    float_t center_y;
    float_t distance_from_center;
};

class CarModel {
private:
    WheelGeometry _wheels[4];

    // 현재 추정 위치 속도 및 가속도
    CarPosition _current_position;
    CarVelocity _current_velocity;
    CarAcceleration _current_acceleration;


public:

    AngularVelocities compute_wheel_angular_velocity_from_car_velocity(CarVelocity velocity) {
        // wheel velocity
        // V_wheel = V_car + omega X r_wheel (as vector)
        float_t vx1 = velocity.vx - velocity.omega * _wheels[FRONT_RIGHT].center_y; 
        float_t vy1 = velocity.vy + velocity.omega * _wheels[FRONT_RIGHT].center_x;
        float_t vx2 = velocity.vx - velocity.omega * _wheels[FRONT_LEFT].center_y;
        float_t vy2 = velocity.vy + velocity.omega * _wheels[FRONT_LEFT].center_x;
        float_t vx3 = velocity.vx - velocity.omega * _wheels[REAR_LEFT].center_y;
        float_t vy3 = velocity.vy + velocity.omega * _wheels[REAR_LEFT].center_x;
        float_t vx4 = velocity.vx - velocity.omega * _wheels[REAR_RIGHT].center_y;
        float_t vy4 = velocity.vy + velocity.omega * _wheels[REAR_RIGHT].center_x;

        AngularVelocities w;


        // angular velocity
        // example
        /*
        vx1 = v1_free * cos1 + w1 * radius;
        vy1 = v1_free * sin1 ; 
        vx1 = vy1 * cot1 + w1 * radius;
        w1 = (vx1 - vy1 * cot1) /radius;
        */
        AngularVelocities w;
        w.vel1 = (vx1 - vy1 * _wheels[FRONT_RIGHT].free_cot) / _wheels[FRONT_RIGHT].radius;
        w.vel2 = (vx2 - vy2 * _wheels[FRONT_LEFT].free_cot) / _wheels[FRONT_LEFT].radius;
        w.vel3 = (vx3 - vy3 * _wheels[REAR_LEFT].free_cot) / _wheels[REAR_LEFT].radius;
        w.vel4 = (vx4 - vy4 * _wheels[REAR_RIGHT].free_cot) / _wheels[REAR_RIGHT].radius;

        return w;
    }






};