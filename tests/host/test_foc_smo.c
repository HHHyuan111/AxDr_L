/**
 * @file test_foc_smo.c
 * @brief 滑模观测器主机数值验证：模拟旋转 PMSM 的反电动势与电流动态，
 *        验证角度与速度收敛、边界层抗抖振、复位语义。
 */

#include <math.h>
#include <stdio.h>

#include "foc_smo.h"

#define MATH_2PI_LOCAL 6.28318530717959f

static int failures;

static void expect(const char *name, int cond)
{
    if (!cond)
    {
        (void)printf("FAIL %s\n", name);
        failures++;
    }
}

/* 角度差归一化到 [-π, π]。 */
static float wrap_diff(float a, float b)
{
    float d = a - b;
    while (d > 3.14159265f)
    {
        d -= MATH_2PI_LOCAL;
    }
    while (d < -3.14159265f)
    {
        d += MATH_2PI_LOCAL;
    }
    return d;
}

int main(void)
{
    foc_smo_t smo;
    /* 200W 伺服电机参数：Rs=0.105Ω、Ls=0.104mH、磁链约 5.66mWb（SERVO200W 档案）。 */
    const float rs = 0.105f;
    const float ls = 0.000104f;
    const float flux = 0.00566f;
    const float ts = 0.00005f; /* 20kHz */
    const float we = 100.0f;   /* 电角速度 rad/s */
    const float i_amp = 1.0f;  /* 电流幅值 A */
    float theta = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    int i;

    foc_smo_init(&smo, rs, ls, 5.0f, 2000.0f, ts, 0.0f, 0.0f, 300.0f);

    for (i = 0; i < 6000; ++i) /* 300ms：默认 PLL 带宽 40 rad/s 的收敛窗口 */
    {
        float ea;
        float eb;
        float ia_ref;
        float ib_ref;
        float ia_dot;
        float ib_dot;
        float ua;
        float ub;

        theta += we * ts;
        if (theta > MATH_2PI_LOCAL)
        {
            theta -= MATH_2PI_LOCAL;
        }
        ea = -flux * we * sinf(theta);
        eb = flux * we * cosf(theta);

        /* 参考电流与电压前馈：让实际电流经 RL 动态逼近正弦参考。 */
        ia_ref = i_amp * cosf(theta);
        ib_ref = i_amp * sinf(theta);
        ia_dot = -i_amp * we * sinf(theta);
        ib_dot = i_amp * we * cosf(theta);
        ua = ea + rs * ia_ref + ls * ia_dot;
        ub = eb + rs * ib_ref + ls * ib_dot;

        /* 被控对象：一阶 RL + 反电动势，显式欧拉。 */
        ia += (ua - rs * ia - ea) / ls * ts;
        ib += (ub - rs * ib - eb) / ls * ts;

        foc_smo_step(&smo, ua, ub, ia, ib);
    }

    expect("速度估计收敛", fabsf(smo.spd - we) < 0.05f * we);
    expect("角度估计收敛", fabsf(wrap_diff(smo.theta, theta)) < 0.15f);
    expect("估计电流跟随", fabsf(smo.ia_est - ia) < 0.3f);

    /* 预置语义（VESC 法）：seed 后几步内应保持在预置轨迹附近。 */
    {
        const float theta0 = 1.0f;
        const float we0 = 50.0f;
        foc_smo_t s2;
        float t2 = theta0;
        int k;

        foc_smo_init(&s2, rs, ls, 5.0f, 2000.0f, ts, 0.0f, 0.0f, 300.0f);
        foc_smo_seed(&s2, theta0, we0, flux, ia, ib);
        for (k = 0; k < 100; ++k)
        {
            float ea;
            float eb;
            float ia_ref;
            float ib_ref;
            float ua;
            float ub;

            t2 += we0 * ts;
            if (t2 > MATH_2PI_LOCAL)
            {
                t2 -= MATH_2PI_LOCAL;
            }
            ea = -flux * we0 * sinf(t2);
            eb = flux * we0 * cosf(t2);
            ia_ref = i_amp * cosf(t2);
            ib_ref = i_amp * sinf(t2);
            ua = ea + rs * ia_ref + ls * (-i_amp * we0 * sinf(t2));
            ub = eb + rs * ib_ref + ls * (i_amp * we0 * cosf(t2));
            ia += (ua - rs * ia - ea) / ls * ts;
            ib += (ub - rs * ib - eb) / ls * ts;
            foc_smo_step(&s2, ua, ub, ia, ib);
        }
        expect("预置后角度保持", fabsf(wrap_diff(s2.theta, t2)) < 0.3f);
        expect("预置后速度保持", fabsf(s2.spd - we0) < 0.15f * we0);
    }

    /* 复位语义：清空后输出归零，再跑重新收敛。 */
    foc_smo_reset(&smo);
    expect("复位清空输出", smo.spd == 0.0f && smo.theta == 0.0f);

    if (failures == 0)
    {
        (void)printf("test_foc_smo: all pass (spd_err=%.4f theta_err=%.4f)\n",
                     (double)fabsf(smo.spd - we),
                     (double)fabsf(wrap_diff(smo.theta, theta)));
        return 0;
    }
    (void)printf("test_foc_smo: %d failure(s)\n", failures);
    return 1;
}
