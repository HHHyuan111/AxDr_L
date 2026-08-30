/**
 * @file test_control_pid.c
 * @brief 在电脑端逐拍比较新旧 PID/PDFF 实现。
 *
 * 测试直接编译生产目录中的 Legacy 实现和新 Control 实现。同一组初值和输入
 * 分别送给两个实现，每一拍都比较返回值以及 PID 上下文的全部字节。
 */

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "common.h"
#include "control_pid.h"

typedef struct
{
    float ref;
    float feedback;
} pid_test_step_t;

static pid_para_t make_pid(void)
{
    const pid_para_t pid = {
        .kp = 1.75f,
        .ki = 0.35f,
        .kd = 0.08f,
        .kfp = 0.72f,
        .kf_damp = 0.16f,
        .p_term = -0.12f,
        .i_term = 0.21f,
        .d_term = -0.03f,
        .i_term_max = 1.4f,
        .i_term_min = -1.1f,
        .ts = 0.001f,
        .ref_value = 0.4f,
        .fback_value = -0.2f,
        .error = 0.6f,
        .pre_err = -0.15f,
        .out_min = -2.0f,
        .out_max = 2.5f,
        .out_value = 0.3f
    };

    return pid;
}

static int expect_same_float_bits(const char *name,
                                  float actual,
                                  float expected)
{
    if (memcmp(&actual, &expected, sizeof(actual)) == 0)
    {
        return 1;
    }

    fprintf(stderr,
            "%s 返回值不一致：Control %.9g，Legacy %.9g\n",
            name,
            (double)actual,
            (double)expected);
    return 0;
}

static int expect_same_context(const char *name,
                               size_t step,
                               const pid_para_t *actual,
                               const pid_para_t *expected)
{
    if (memcmp(actual, expected, sizeof(*actual)) == 0)
    {
        return 1;
    }

    fprintf(stderr, "%s 第 %zu 拍的 PID 上下文未保持逐位一致。\n", name, step);
    return 0;
}

static int test_limits_and_clear(void)
{
    pid_para_t legacy = make_pid();
    pid_para_t control = legacy;

    pid_limit_init(&legacy, 0.9f, -0.8f, 1.6f, -1.3f);
    control_pid_set_limits(&control, 0.9f, -0.8f, 1.6f, -1.3f);

    if (!expect_same_context("限值设置", 0U, &control, &legacy))
    {
        return 0;
    }

    pid_clear(&legacy);
    control_pid_clear(&control);

    if (!expect_same_context("历史清零", 0U, &control, &legacy))
    {
        return 0;
    }

    return (control.kp == 1.75f) &&
           (control.ki == 0.35f) &&
           (control.kd == 0.08f) &&
           (control.kfp == 0.72f) &&
           (control.kf_damp == 0.16f) &&
           (control.ts == 0.001f) &&
           (control.i_term_max == 0.9f) &&
           (control.i_term_min == -0.8f) &&
           (control.out_max == 1.6f) &&
           (control.out_min == -1.3f);
}

static int test_parallel_equivalence(void)
{
    static const pid_test_step_t steps[] = {
        {0.8f, 0.1f},
        {3.0f, -0.5f},
        {-2.8f, 0.9f},
        {-0.3f, -0.1f},
        {0.0f, 0.0f}
    };
    pid_para_t legacy = make_pid();
    pid_para_t control = legacy;
    size_t index;

    for (index = 0U; index < sizeof(steps) / sizeof(steps[0]); index++)
    {
        const float legacy_output = parallel_pid_ctrl(&legacy,
                                                       steps[index].ref,
                                                       steps[index].feedback);
        const float control_output = control_pid_parallel_step(
            &control,
            steps[index].ref,
            steps[index].feedback);

        if (!expect_same_float_bits("并联 PID", control_output, legacy_output) ||
            !expect_same_context("并联 PID", index, &control, &legacy))
        {
            return 0;
        }
    }

    return 1;
}

static int test_serial_equivalence(void)
{
    static const pid_test_step_t steps[] = {
        {0.2f, -0.4f},
        {1.8f, 0.1f},
        {-1.4f, 0.6f},
        {0.3f, 0.35f}
    };
    pid_para_t legacy = make_pid();
    pid_para_t control = legacy;
    size_t index;

    for (index = 0U; index < sizeof(steps) / sizeof(steps[0]); index++)
    {
        const float legacy_output = serial_pid_ctrl(&legacy,
                                                     steps[index].ref,
                                                     steps[index].feedback);
        const float control_output = control_pid_serial_step(
            &control,
            steps[index].ref,
            steps[index].feedback);

        if (!expect_same_float_bits("串联 PID", control_output, legacy_output) ||
            !expect_same_context("串联 PID", index, &control, &legacy))
        {
            return 0;
        }
    }

    return 1;
}

static int test_pdff_equivalence(void)
{
    static const pid_test_step_t steps[] = {
        {0.7f, -0.2f},
        {2.6f, 0.4f},
        {-2.2f, 0.8f},
        {-0.1f, -0.3f}
    };
    pid_para_t legacy = make_pid();
    pid_para_t control = legacy;
    size_t index;

    for (index = 0U; index < sizeof(steps) / sizeof(steps[0]); index++)
    {
        const float legacy_output = pdff_ctrl(&legacy,
                                               steps[index].ref,
                                               steps[index].feedback);
        const float control_output = control_pid_pdff_step(
            &control,
            steps[index].ref,
            steps[index].feedback);

        if (!expect_same_float_bits("PDFF", control_output, legacy_output) ||
            !expect_same_context("PDFF", index, &control, &legacy))
        {
            return 0;
        }
    }

    return 1;
}

int main(void)
{
    _Static_assert(sizeof(pid_para_t) == (18U * sizeof(float)),
                   "pid_para_t 的字段数量或布局发生了变化");

    if (!test_limits_and_clear())
    {
        return 1;
    }

    if (!test_parallel_equivalence())
    {
        return 2;
    }

    if (!test_serial_equivalence())
    {
        return 3;
    }

    if (!test_pdff_equivalence())
    {
        return 4;
    }

    return 0;
}
