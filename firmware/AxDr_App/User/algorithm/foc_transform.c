/**
 * @file foc_transform.c
 * @brief FOC 角度、Clarke、Park 和逆 Park 的无状态数值实现。
 */

#include "foc_transform.h"

#include <math.h>

#include "compiler.h"
#include "fast_trig.h"

static const float foc_one_by_sqrt3 = 0.57735026919f;

/* B 库三闭环验证所用多项式实现（fast_trig.h，14 号政策唯一三角实现），
 * 替代 libm sinf/cosf：精度 1.8e-9 且甩掉 __kernel_rem_pio2f 约 1.6KB 依赖。 */
PLATFORM_FAST_CODE void foc_sin_cos(float theta,
                                    float *sin,
                                    float *cos)
{
    fast_sincos(theta, sin, cos);
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
