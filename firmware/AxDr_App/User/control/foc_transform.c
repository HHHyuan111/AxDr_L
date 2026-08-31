/**
 * @file foc_transform.c
 * @brief FOC 角度、Clarke、Park 和逆 Park 的无状态数值实现。
 */

#include "foc_transform.h"

#include <math.h>

#include "compiler.h"

static const float foc_one_by_sqrt3 = 0.57735026919f;

PLATFORM_FAST_CODE void foc_sin_cos(float theta_e_rad,
                                        float *sin_theta,
                                        float *cos_theta)
{
    *sin_theta = sinf(theta_e_rad);
    *cos_theta = cosf(theta_e_rad);
}

PLATFORM_FAST_CODE void foc_clarke(float i_a,
                                       float i_b,
                                       float i_c,
                                       float *i_alpha,
                                       float *i_beta)
{
    *i_alpha = i_a;
    *i_beta = (i_b - i_c) * foc_one_by_sqrt3;
}

PLATFORM_FAST_CODE void foc_park(float i_alpha,
                                     float i_beta,
                                     float sin_theta,
                                     float cos_theta,
                                     float *i_d,
                                     float *i_q)
{
    *i_d = i_alpha * cos_theta + i_beta * sin_theta;
    *i_q = i_beta * cos_theta - i_alpha * sin_theta;
}

PLATFORM_FAST_CODE void foc_inv_park(float v_d,
                                         float v_q,
                                         float sin_theta,
                                         float cos_theta,
                                         float *v_alpha,
                                         float *v_beta)
{
    *v_alpha = v_d * cos_theta - v_q * sin_theta;
    *v_beta = v_d * sin_theta + v_q * cos_theta;
}
