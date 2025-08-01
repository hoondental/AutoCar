#include "math.h"
#include "arm_math.h"
#include "stm32f1xx.h"

#include "motors.h"



     // ============================== SGFilter =================================
     bool SG2Filter::generateFilter() {
        float_t A_data[_N_samples * 3];
        float_t AT_data[3 * _N_samples];
        float_t ATA_data[3 * 3];
        float_t ATAi_data[3 * 3];
        for (int i=0; i<_N_samples; i++) {
            float_t t_i = -(float_t(_N_samples) - 1.0 - i) * _dt;
            A_data[i * 3] = 1.0;
            A_data[i * 3 + 1] = t_i;
            A_data[i * 3 + 2] = pow(t_i, 2.0);
        }

        arm_matrix_instance_f32 A, AT, ATA, ATAi;
        arm_mat_init_f32(&A, _N_samples, 3, A_data);
        arm_mat_init_f32(&AT, 3, _N_samples, AT_data);
        arm_mat_init_f32(&ATA, 3, 3, ATA_data);
        arm_mat_init_f32(&ATAi, 3, 3, ATAi_data);
        arm_mat_init_f32(&_ATAiAT, 3, _N_samples, _ATAiAT_data);
        if (arm_mat_trans_f32(&A, &AT) != ARM_MATH_SUCCESS) return false;
        if (arm_mat_mult_f32(&AT, &A, &ATA) != ARM_MATH_SUCCESS) return false;
        if (arm_mat_inverse_f32(&ATA, &ATAi) != ARM_MATH_SUCCESS) return false;
        if (arm_mat_mult_f32(&ATAi, &AT, &_ATAiAT) != ARM_MATH_SUCCESS) return false;
        return true;
     }

     /*
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
*/


void SG2Filter::fit(uint16_t x, float_t& a0, float_t& a1, float_t& a2) {
    static const float_t _to_radian = 2.0 * M_PI / _encoder_PPR;

    _buf_u16[_idx0] = x;
    _idx0 = (_idx0 + 1) % _N_samples;
    int k = 0;

    a0 = 0;
    a1 = 0;
    a2 = 0;
    for (int i=0; i<_N_samples; i++) {
        k = (i + _idx0) % _N_samples;
        float32_t _x = float32_t(int16_t(_buf_u16[k] - x));
        a0 += _x * _ATAiAT_data[i];
        a1 += _x * _ATAiAT_data[_N_samples + i];
        a2 += _x * _ATAiAT_data[2 * _N_samples + i];
    }
    a0 += x;
    a0 *= _to_radian;
    a1 *= _to_radian;
    a2 *= _to_radian;
}    