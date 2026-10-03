/**
 * @file test_fast_trig.c
 * @brief 多项式 sincos 数值基准：全圆扫描对照 libm、象限边界、区间外回退。
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "fast_trig.h"

#define SWEEP_N 200001

int main(void)
{
    int fail = 0;
    float max_err = 0.0f;
    float worst = 0.0f;

    /* 场景 1：全圆扫描（0..2π 含端点），最大误差须远小于标称 1.8e-9 的工程余量。 */
    for (int i = 0; i < SWEEP_N; i++)
    {
        float a = (float)i * (6.28318530717959f / (float)(SWEEP_N - 1));
        float s, c;
        fast_sincos(a, &s, &c);
        float es = fabsf(s - sinf(a));
        float ec = fabsf(c - cosf(a));
        if (es > max_err) { max_err = es; worst = a; }
        if (ec > max_err) { max_err = ec; worst = a; }
    }
    if (max_err > 1e-7f)
    {
        (void)printf("FAIL sweep: max_err=%.3e at a=%.6f\n", (double)max_err, (double)worst);
        fail++;
    }

    /* 场景 2：象限边界与特殊角精确性（对称位置上的解析值）。 */
    const float edges[] = { 0.0f, 1.57079632679f, 3.14159265359f, 4.71238898038f, 6.28318530718f };
    for (unsigned i = 0; i < sizeof(edges) / sizeof(edges[0]); i++)
    {
        float s, c;
        fast_sincos(edges[i], &s, &c);
        if (fabsf(s - sinf(edges[i])) > 1e-6f || fabsf(c - cosf(edges[i])) > 1e-6f)
        {
            (void)printf("FAIL edge %u: a=%.8f\n", i, (double)edges[i]);
            fail++;
        }
    }

    /* 场景 3：恒等式 sin²+cos²=1 全圆成立。 */
    for (int i = 0; i < 10001; i++)
    {
        float a = (float)i * 0.00062831853f;
        float s, c;
        fast_sincos(a, &s, &c);
        if (fabsf(s * s + c * c - 1.0f) > 1e-6f)
        {
            (void)printf("FAIL identity at %.4f\n", (double)a);
            fail++;
            break;
        }
    }

    /* 场景 4：区间外回退 libm（逐位一致）。 */
    const float outside[] = { -0.5f, -3.0f, 6.4f, 100.0f };
    for (unsigned i = 0; i < sizeof(outside) / sizeof(outside[0]); i++)
    {
        float s, c;
        fast_sincos(outside[i], &s, &c);
        if (memcmp(&s, &(float){sinf(outside[i])}, sizeof(float)) != 0
            || memcmp(&c, &(float){cosf(outside[i])}, sizeof(float)) != 0)
        {
            (void)printf("FAIL fallback %u: a=%.3f\n", i, (double)outside[i]);
            fail++;
        }
    }

    if (fail == 0)
    {
        (void)printf("test_fast_trig: all pass (max_err=%.2e)\n", (double)max_err);
        return 0;
    }
    return 1;
}
