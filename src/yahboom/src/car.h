#include <Arduino.h>
#include "motors.h"
#include "imu.h"
#include "radio_control.h"



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



struct CarPosition {
    float_t x, y, theta;
};

struct CarVelocity {
    float_t vx, vy, omega;
};

struct CarAcceleration {
    float_t ax, ay, alpha;
};

struct CarVelocityRate {
    float_t vx_rate, vy_rate, omega_rate;
};




class CarModel {
private:
    WheelGeometry _wheels[4];
    EncoderMotors &_motors;
    MPU9250I2C &_imu;
    RC &_rc;

    // 현재 추정 위치 속도 및 가속도
    CarPosition _current_position;
    CarVelocity _current_velocity;
    CarAcceleration _current_acceleration;


public:
    CarModel() : 
        _motors(EncoderMotors::getInstance()), 
        _imu(MPU9250I2C::getInstance(GPIOB, GPIO_PIN_15, GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_14, MPU_I2C_SPEED_KHZ)),
        _rc(RC::getInstance(&Serial2, true, false)) 
    {
        // Initialize wheel geometries
        _wheels[FRONT_RIGHT] = {0.1, 1.0, 0.01, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.5, sqrtf(2) * 0.5f};
        _wheels[FRONT_LEFT] = {0.1, 1.0, 0.01, M_PI / 2, 0.0, 1.0, 0.0, 1.0, -0.5, 0.5, sqrtf(2) * 0.5f};
        _wheels[REAR_LEFT] = {0.1, 1.0, 0.01, M_PI, -1.0, 0.0, -1.0, 0.0, -0.5, -0.5, sqrtf(2) * 0.5f};
        _wheels[REAR_RIGHT] = {0.1, 1.0, 0.01, -M_PI / 2, 0.0, -1.0, -1.0, -1.0, 0.5, -0.5, sqrtf(2) * 0.5f};

        // Initialize current state
        _current_position = {0.0f, 0.0f, 0.0f};
        _current_velocity = {0.0f, 0.0f, 0.0f};
        _current_acceleration = {0.0f, 0.0f, 0.0f};

        // Initialize motors
    }

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





class RCController {
private:

    RC_MODE _mode = RC_MODE::RC_RAW_PWM;
    RC& _rc;
    uint8_t _ch_longitudinal = RC_CHANNEL_LOGNITUDINAL;
    uint8_t _ch_lateral = RC_CHANNEL_LATERAL;
    uint8_t _ch_yaw = RC_CHANNEL_YAW;

    RCController() = delete;
    RCController(const RCController&) = delete;
    RCController& operator=(const RCController&) = delete;

public:
    RCController(RC& rc, RC_MODE mode, uint8_t ch_longitudinal, uint8_t ch_lateral, uint8_t ch_yaw) : _rc(rc) {
        _mode = mode;
        _ch_longitudinal = ch_longitudinal;
        _ch_lateral = ch_lateral;
        _ch_yaw = ch_yaw;
    }

    MotorsCommand getCommand() {
        MotorsCommand cmd;

        if (_mode == RC_MODE::RC_RAW_PWM) {
            // Directly use PWM values from SBUS
            cmd.rate1 = _rc.channel_f(_ch_longitudinal);
            cmd.rate2 = _rc.channel_f(_ch_lateral);
            cmd.rate3 = _rc.channel_f(_ch_yaw);
            cmd.rate4 = 0.0; // No fourth channel in this mode
        } else if (_mode == RC_MODE::RC_RAW_SPEED) {
            // Convert SBUS channels to speed values
            cmd.rate1 = _rc.channel_f(_ch_longitudinal);
            cmd.rate2 = _rc.channel_f(_ch_lateral);
            cmd.rate3 = _rc.channel_f(_ch_yaw);
            cmd.rate4 = 0.0; // No fourth channel in this mode
        } else {
            // Default to no command
            cmd.rate1 = 0.0;
            cmd.rate2 = 0.0;
            cmd.rate3 = 0.0;
            cmd.rate4 = 0.0;
        }

        return cmd;
    }
};

