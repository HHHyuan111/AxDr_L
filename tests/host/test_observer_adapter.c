/**
 * @file test_observer_adapter.c
 * @brief 验证 FOC 实时量到在线磁链观测器的数据适配。
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "common.h"
#include "observer_adapter.h"

static bool expect_true(bool condition, const char *message)
{
    if (condition)
    {
        return true;
    }

    fprintf(stderr, "%s\n", message);
    return false;
}

int main(void)
{
    foc_t foc = {0};

    foc.motor.Rs = 0.1f;
    foc.motor.pn = 2.0f;
    foc.rate.foc_ts = 0.001f;
    obs_init(&foc);

    foc.pwm_active = true;
    foc.fb.id = 0.0f;
    foc.fb.iq = 1.0f;
    foc.out.vd = 0.0f;
    foc.out.vq = 1.1f;
    foc.fb.spd_r = 50.0f;
    foc.ref.v_lim = 12.0f;
    obs_step(&foc);

    if (!expect_true(g_obs.status == MC_OK,
                     "有效运行点应被磁链观测器接受。") ||
        !expect_true(g_obs.flux.accepted_samples == 1U,
                     "有效运行点应累计一次样本。") ||
        !expect_true(fabsf(g_obs.flux.psi_d_instant_wb - 0.01f) < 1.0e-6f,
                     "适配后的磁链计算结果不正确。"))
    {
        return 1;
    }

    foc.pwm_active = false;
    obs_step(&foc);
    return expect_true(g_obs.flux.accepted_samples == 1U,
                       "PWM 关闭时不应推进在线观测器。") ? 0 : 2;
}
