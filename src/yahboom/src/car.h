#ifndef CAR_H
#define CAR_H

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
    WheelGeometry *_wheels;
    EncoderMotors &_motors;
    MPU9250I2C &_imu;
    RC &_rc;

    // 현재 추정 위치 속도 및 가속도
    CarPosition _current_position;
    CarVelocity _current_velocity;
    CarAcceleration _current_acceleration;


public:
    CarModel(EncoderMotors &motors, MPU9250I2C &imu, RC &rc, WheelGeometry *wheels) : 
        _motors(motors), 
        _imu(imu),
        _rc(rc), 
        _wheels(wheels) 
    {
        // Initialize current state
        _current_position = {0.0f, 0.0f, 0.0f};
        _current_velocity = {0.0f, 0.0f, 0.0f};
        _current_acceleration = {0.0f, 0.0f, 0.0f};

        // Initialize motors
    }


    void begin(uint32_t pwm_frequency = 10000, uint32_t pwm_resolution = 256, uint32_t encoder_PPR = 1320, 
               int gyro_range_dps = 250, int accel_range_g = 2, uint16_t mpu_dataRate_Hz = 200, uint16_t mag_dataRate_Hz = 100) {
        _motors.begin(pwm_frequency, pwm_resolution, encoder_PPR);
        _imu.begin(gyro_range_dps, accel_range_g, mpu_dataRate_Hz, mag_dataRate_Hz);
        _rc.begin();
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



/*

class RCController {
private:

    RC_MODE _mode = RC_MODE::RC_RAW_PWM;
    RC& _rc;
    uint8_t _chN_throttle = RC_CHANNEL_LOGNITUDINAL;
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

*/


#endif