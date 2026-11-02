/**
 * @file foc_smo.c
 * @brief 反电动势滑模观测器实现。主链位置：第 6 步内（旁路阶段只观测，
 *        切换后其角度进入控制链，见 02 规范 §3）。
 */

#include "foc_smo.h"

#include <math.h>

#include "math_const.h"

/* 饱和函数：把电流误差压进 [-limit, limit]，替代硬符号函数抗抖振。 */
static float smo_sat(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void foc_smo_init(foc_smo_t *s, float rs, float ls, float kslide,
                  float lpf_wc, float ts, float pll_kp, float pll_ki,
                  float spd_max)
{
    s->rs = rs;
    s->ls = ls;
    s->kslide = kslide;
    s->lpf_wc = lpf_wc;
    s->ts = ts;
    s->spd_max = spd_max;
    if ((pll_kp <= 0.0f) || (pll_ki <= 0.0f))
    {
        /* 默认 PLL 带宽：自然频率 40 rad/s、阻尼 0.707——真机接管期实测
         * 80 rad/s 时 kp 项放大电流谐波，角度抖动 0.5~1.1 rad。 */
        const float wn = 40.0f;

        s->pll_kp = 2.0f * 0.707f * wn;
        s->pll_ki = wn * wn;
    }
    else
    {
        s->pll_kp = pll_kp;
        s->pll_ki = pll_ki;
    }
    foc_smo_reset(s);
}

void foc_smo_reset(foc_smo_t *s)
{
    s->ia_est = 0.0f;
    s->ib_est = 0.0f;
    s->ea_f = 0.0f;
    s->eb_f = 0.0f;
    s->theta = 0.0f;
    s->spd = 0.0f;
    s->spd_f = 0.0f;
    s->e_mag = 0.0f;
    s->pll_int = 0.0f;
}

void foc_smo_seed(foc_smo_t *s, float theta, float we, float flux,
                  float ia_meas, float ib_meas)
{
    s->theta = theta;
    s->spd = we;
    s->spd_f = we;
    s->pll_int = we / s->pll_ki;
    /* 反电动势按虚拟轴相位的期望值预置（负载角由 PLL 自行收敛）。 */
    s->ea_f = -flux * we * sinf(theta);
    s->eb_f = flux * we * cosf(theta);
    s->e_mag = fabsf(flux * we);
    s->ia_est = ia_meas;
    s->ib_est = ib_meas;
}

void foc_smo_step(foc_smo_t *s, float ua, float ub, float ia, float ib)
{
    float ia_err;
    float ib_err;
    float za;
    float zb;
    float lpf_old;
    float lpf_new;
    float e_mag;
    float err_norm;
    float theta_sin;
    float theta_cos;

    /* 1. 电流误差（估计值减去实测值，滑模面向误差收敛）。 */
    ia_err = s->ia_est - ia;
    ib_err = s->ib_est - ib;

    /* 2. 饱和函数：边界取一拍能产生满幅反电动势的电流误差。 */
    za = smo_sat(ia_err, s->kslide * s->ts / s->ls);
    zb = smo_sat(ib_err, s->kslide * s->ts / s->ls);

    /* 3. 反电动势一阶低通（纯乘法系数，与参考实现同法）。 */
    lpf_old = 1.0f / (1.0f + s->lpf_wc * s->ts);
    lpf_new = 1.0f - lpf_old;
    s->ea_f = s->ea_f * lpf_old + (s->kslide * za) * lpf_new;
    s->eb_f = s->eb_f * lpf_old + (s->kslide * zb) * lpf_new;

    /* 4. 电流模型前推：di/dt = (U - Rs*i - E)/Ls，显式欧拉。 */
    s->ia_est += (ua - s->rs * s->ia_est - s->ea_f) / s->ls * s->ts;
    s->ib_est += (ub - s->rs * s->ib_est - s->eb_f) / s->ls * s->ts;

    /* 5. PLL：误差 = Eβ·sinθ̂ + Eα·cosθ̂ ≈ E·sin(θ̂-θ)，按幅值归一化。 */
    theta_sin = sinf(s->theta);
    theta_cos = cosf(s->theta);
    e_mag = sqrtf(s->ea_f * s->ea_f + s->eb_f * s->eb_f);
    s->e_mag = e_mag;
    if (e_mag < 0.05f)
    {
        return; /* 反电动势太小：保持当前估计，等待转速起来。 */
    }
    err_norm = -(s->eb_f * theta_sin + s->ea_f * theta_cos) / e_mag;

    s->pll_int += err_norm * s->ts;
    if (s->spd_max > 0.0f)
    {
        if (s->pll_int > s->spd_max / s->pll_ki)
        {
            s->pll_int = s->spd_max / s->pll_ki;
        }
        else if (s->pll_int < -s->spd_max / s->pll_ki)
        {
            s->pll_int = -s->spd_max / s->pll_ki;
        }
    }
    s->spd = s->pll_kp * err_norm + s->pll_ki * s->pll_int;
    if ((s->spd_max > 0.0f) && (s->spd > s->spd_max))
    {
        s->spd = s->spd_max;
    }
    else if ((s->spd_max > 0.0f) && (s->spd < -s->spd_max))
    {
        s->spd = -s->spd_max;
    }
    s->theta += s->spd * s->ts;
    if (s->theta >= M_2PI)
    {
        s->theta -= M_2PI;
    }
    else if (s->theta < 0.0f)
    {
        s->theta += M_2PI;
    }

    /* 速度输出低通（截止 100 rad/s）：kp 项噪声 ±30 rad/s 级，
     * 直接进速度环会被放大成振荡；积分项是速度主载波，滤波只去毛刺。 */
    s->spd_f += (s->spd - s->spd_f) * (100.0f * s->ts / (1.0f + 100.0f * s->ts));
}
