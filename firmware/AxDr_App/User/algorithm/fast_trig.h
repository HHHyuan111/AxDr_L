/**
 * @file fast_trig.h
 * @brief 多项式成对 sincos 近似（header-only）。
 * @note  来源：沉沙ABZ固件 axdr_fast_trig.h（B 库三闭环验证所用实现，仅改名）。
 *        FOC 电角已包络到 [0,2π]。先折叠到最近象限使余角在 ±π/4 内；
 *        正弦 9 阶、余弦 10 阶的截断余项分别小于 1.8e-9 和 1.2e-10。
 *        非有限数及区间外输入回退 libm 通用语义，不隐式钳位。
 *        数值政策（14 号）：本函数为全框架唯一三角实现，主机测试对照 libm 容差断言。
 */

#ifndef FAST_TRIG_H
#define FAST_TRIG_H

#include <math.h>

#if defined(_MSC_VER)
#define FAST_TRIG_NOINLINE __declspec(noinline)
#else
#define FAST_TRIG_NOINLINE __attribute__((noinline))
#endif

static FAST_TRIG_NOINLINE void fast_sincos(float angle, float *sine, float *cosine)
{
    if (!(angle >= 0.0f && angle <= 6.2831855f))
    {
        /* 区间外（理论不达路径）回退 libm，保持通用语义 */
        *sine = sinf(angle);
        *cosine = cosf(angle);
        return;
    }

    const unsigned quadrant = (unsigned)(angle * 0.63661977236758134f + 0.5f);
    /* 拆分 π/2，避免接近象限边界时直接减低精度常数引入偏差 */
    const float x = (angle - quadrant * 1.5703125f) - quadrant * 0.00048382679489661923f;
    const float z = x * x;
    const float s = x + x * z * (-1.0f / 6 + z * (1.0f / 120 + z * (-1.0f / 5040 + z / 362880)));
    const float c = 1 + z * (-0.5f + z * (1.0f / 24 + z * (-1.0f / 720 + z * (1.0f / 40320 - z / 3628800))));

    switch (quadrant & 3u)
    {
        case 0:
            *sine = s;
            *cosine = c;
            break;
        case 1:
            *sine = c;
            *cosine = -s;
            break;
        case 2:
            *sine = -s;
            *cosine = -c;
            break;
        default:
            *sine = -c;
            *cosine = s;
            break;
    }
}

#undef FAST_TRIG_NOINLINE

#endif /* FAST_TRIG_H */
