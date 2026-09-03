/**
 * @file foc_transform.c
 * @brief FOC 角度、Clarke、Park 和逆 Park 的无状态数值实现。
 */

#include "foc_transform.h"

#include <math.h>

#include "compiler.h"

static const float foc_one_by_sqrt3 = 0.57735026919f;

PLATFORM_FAST_CODE void foc_sin_cos(float theta,
                                    float *sin,
                                    float *cos)
{
    *sin = sinf(theta);
    *cos = cosf(theta);
}

PLATFORM_FAST_CODE void foc_clarke(float ia,
                                   float ib,
                                   float ic,
                                   float *ialpha,
                                   float *ibeta)
{
    *ialpha = ia;
    *ibeta = (ib - ic) * foc_one_by_sqrt3;
}

PLATFORM_FAST_CODE void foc_park(float ialpha,
                                 float ibeta,
                                 float sin,
                                 float cos,
                                 float *id,
                                 float *iq)
{
    *id = ialpha * cos + ibeta * sin;
    *iq = ibeta * cos - ialpha * sin;
}

PLATFORM_FAST_CODE void foc_inv_park(float vd,
                                     float vq,
                                     float sin,
                                     float cos,
                                     float *valpha,
                                     float *vbeta)
{
    *valpha = vd * cos - vq * sin;
    *vbeta = vd * sin + vq * cos;
}
