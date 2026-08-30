/**
 * @file test_control_cascade.c
 * @brief 逐拍比较迁移前后的电流、速度和位置级联控制顺序。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "control_cascade.h"
#include "legacy_control.h"

static pid_para_t make_pid(float kp, float ki)
{
    const pid_para_t pid = {
        .kp = kp,
        .ki = ki,
        .kd = 0.05f,
        .kfp = 0.9f,
        .kf_damp = 0.2f,
        .p_term = -0.1f,
        .i_term = 0.2f,
        .d_term = 0.03f,
        .i_term_max = 8.0f,
        .i_term_min = -8.0f,
        .ts = 0.001f,
        .ref_value = 0.0f,
        .fback_value = 0.0f,
        .error = 0.0f,
        .pre_err = -0.2f,
        .out_min = -10.0f,
        .out_max = 10.0f,
        .out_value = 0.0f,
    };

    return pid;
}

static int same_bytes(const void *actual,
                      const void *expected,
                      size_t size,
                      const char *name,
                      size_t step)
{
    if (memcmp(actual, expected, size) == 0)
    {
        return 1;
    }

    fprintf(stderr, "%s 第 %zu 拍不一致。\n", name, step);
    return 0;
}

static int test_cur_loop(void)
{
    static const float refs[][4] = {
        {0.0f, 1.0f, 0.1f, -0.2f},
        {0.2f, 1.5f, 0.15f, -0.1f},
        {-0.3f, 2.0f, 0.0f, 0.25f},
        {0.1f, -1.0f, -0.2f, 0.4f},
    };
    control_rate_t legacy_rate = {0U, 2U};
    control_rate_t control_rate = legacy_rate;
    pid_para_t legacy_d = make_pid(1.2f, 0.3f);
    pid_para_t control_d = legacy_d;
    pid_para_t legacy_q = make_pid(1.4f, 0.4f);
    pid_para_t control_q = legacy_q;
    float legacy_v_d = 0.75f;
    float legacy_v_q = -0.5f;
    float control_v_d = legacy_v_d;
    float control_v_q = legacy_v_q;
    size_t step;

    for (step = 0U; step < sizeof(refs) / sizeof(refs[0]); step++)
    {
        const bool legacy_updated = legacy_control_cur_step(&legacy_rate,
                                                            &legacy_d,
                                                            &legacy_q,
                                                            refs[step][0],
                                                            refs[step][1],
                                                            refs[step][2],
                                                            refs[step][3],
                                                            &legacy_v_d,
                                                            &legacy_v_q);
        const bool control_updated = control_cur_step(&control_rate,
                                                      &control_d,
                                                      &control_q,
                                                      refs[step][0],
                                                      refs[step][1],
                                                      refs[step][2],
                                                      refs[step][3],
                                                      &control_v_d,
                                                      &control_v_q);

        if ((control_updated != legacy_updated) ||
            !same_bytes(&control_rate, &legacy_rate, sizeof(control_rate), "电流环分频", step) ||
            !same_bytes(&control_d, &legacy_d, sizeof(control_d), "d 轴 PI", step) ||
            !same_bytes(&control_q, &legacy_q, sizeof(control_q), "q 轴 PI", step) ||
            !same_bytes(&control_v_d, &legacy_v_d, sizeof(control_v_d), "d 轴电压", step) ||
            !same_bytes(&control_v_q, &legacy_v_q, sizeof(control_v_q), "q 轴电压", step))
        {
            return 0;
        }
    }

    return 1;
}

static int test_spd_loop(void)
{
    static const float refs[][3] = {
        {1.0f, 0.0f, 0.0f},
        {2.0f, 0.5f, 3.0f},
        {-1.0f, 0.25f, -2.0f},
        {0.5f, -0.5f, 1.5f},
    };
    control_rate_t legacy_rate = {0U, 2U};
    control_rate_t control_rate = legacy_rate;
    pid_para_t legacy_pid = make_pid(0.8f, 0.15f);
    pid_para_t control_pid = legacy_pid;
    float legacy_iq_ref = 0.4f;
    float control_iq_ref = legacy_iq_ref;
    size_t step;

    for (step = 0U; step < sizeof(refs) / sizeof(refs[0]); step++)
    {
        const bool legacy_updated = legacy_control_spd_step(&legacy_rate,
                                                            &legacy_pid,
                                                            refs[step][0],
                                                            refs[step][1],
                                                            refs[step][2],
                                                            &legacy_iq_ref);
        const bool control_updated = control_spd_step(&control_rate,
                                                      &control_pid,
                                                      refs[step][0],
                                                      refs[step][1],
                                                      refs[step][2],
                                                      &control_iq_ref);

        if ((control_updated != legacy_updated) ||
            !same_bytes(&control_rate, &legacy_rate, sizeof(control_rate), "速度环分频", step) ||
            !same_bytes(&control_pid, &legacy_pid, sizeof(control_pid), "速度 PDFF", step) ||
            !same_bytes(&control_iq_ref, &legacy_iq_ref, sizeof(control_iq_ref), "速度环输出", step))
        {
            return 0;
        }
    }

    return 1;
}

static int test_pos_loop(void)
{
    static const float refs[][3] = {
        {0.5f, 0.0f, 0.0f},
        {1.5f, 0.2f, 2.0f},
        {-0.5f, 0.4f, -1.0f},
        {0.0f, -0.2f, 1.25f},
    };
    control_rate_t legacy_rate = {0U, 2U};
    control_rate_t control_rate = legacy_rate;
    pid_para_t legacy_pid = make_pid(1.1f, 0.2f);
    pid_para_t control_pid = legacy_pid;
    float legacy_speed_ref = -0.3f;
    float control_speed_ref = legacy_speed_ref;
    size_t step;

    for (step = 0U; step < sizeof(refs) / sizeof(refs[0]); step++)
    {
        const bool legacy_updated = legacy_control_pos_step(&legacy_rate,
                                                            &legacy_pid,
                                                            refs[step][0],
                                                            refs[step][1],
                                                            refs[step][2],
                                                            &legacy_speed_ref);
        const bool control_updated = control_pos_step(&control_rate,
                                                      &control_pid,
                                                      refs[step][0],
                                                      refs[step][1],
                                                      refs[step][2],
                                                      &control_speed_ref);

        if ((control_updated != legacy_updated) ||
            !same_bytes(&control_rate, &legacy_rate, sizeof(control_rate), "位置环分频", step) ||
            !same_bytes(&control_pid, &legacy_pid, sizeof(control_pid), "位置 PI", step) ||
            !same_bytes(&control_speed_ref, &legacy_speed_ref, sizeof(control_speed_ref), "位置环输出", step))
        {
            return 0;
        }
    }

    return 1;
}

int main(void)
{
    if (!test_cur_loop())
    {
        return 1;
    }

    if (!test_spd_loop())
    {
        return 2;
    }

    if (!test_pos_loop())
    {
        return 3;
    }

    return 0;
}
