/**
 * @file test_foc_math.c
 * @brief 在电脑端直接验证现有 FOC 基础数学函数。
 *
 * 测试用 tests/legacy 中保留的迁移前公式，对照生产目录中的新 Control
 * 实现。旧公式不再参与 MCU 固件编译。
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "common.h"
#include "control_filter.h"
#include "control_limit.h"
#include "foc_core.h"
#include "foc_svm.h"
#include "foc_transform.h"
#include "legacy_control.h"

#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 201112L)
#error "需要支持 C11 的电脑端编译器"
#endif

#define TEST_TOLERANCE (1.0e-5f)

static int expect_close(const char *name, float actual, float expected)
{
    if (fabsf(actual - expected) <= TEST_TOLERANCE) {
        return 1;
    }

    fprintf(stderr, "%s 不符合预期：实际 %.8f，期望 %.8f\n", name, actual, expected);
    return 0;
}

static int expect_same_float_bits(const char *name, float actual, float expected)
{
    if (memcmp(&actual, &expected, sizeof(actual)) == 0) {
        return 1;
    }

    fprintf(stderr,
            "%s 未保持逐位一致：新值 %.9g，Legacy 值 %.9g\n",
            name,
            (double)actual,
            (double)expected);
    return 0;
}

static float float_from_bits(uint32_t bits)
{
    float value;

    memcpy(&value, &bits, sizeof(value));
    return value;
}

static int test_limit_migration_equivalence(void)
{
    const float quiet_nan = float_from_bits(UINT32_C(0x7fc12345));
    static const float finite_cases[] = {
        -1.5f,
        -1.0f,
        -0.0f,
        0.0f,
        0.75f,
        2.0f,
        2.5f
    };
    size_t index;

    for (index = 0U; index < sizeof(finite_cases) / sizeof(finite_cases[0]); index++)
    {
        const float legacy = sat1_datf(finite_cases[index], 2.0f, -1.0f);
        const float control = control_limit(finite_cases[index], 2.0f, -1.0f);

        if (!expect_same_float_bits("有限值限幅", control, legacy))
        {
            return 0;
        }
    }

    if (!expect_same_float_bits("负无穷限幅",
                                control_limit(-INFINITY, 2.0f, -1.0f),
                                sat1_datf(-INFINITY, 2.0f, -1.0f)) ||
        !expect_same_float_bits("正无穷限幅",
                                control_limit(INFINITY, 2.0f, -1.0f),
                                sat1_datf(INFINITY, 2.0f, -1.0f)) ||
        !expect_same_float_bits("NaN 限幅",
                                control_limit(quiet_nan, 2.0f, -1.0f),
                                sat1_datf(quiet_nan, 2.0f, -1.0f)))
    {
        return 0;
    }

    return 1;
}

static int test_lpf_migration_equivalence(void)
{
    static const float input_cases[] = {
        0.0f,
        1.0f,
        2.5f,
        -1.25f,
        0.0f,
        4.0f
    };
    lpf_t legacy = {
        .val = 3.0f,
        .val_f = -2.0f,
        .fs = 20000.0f,
        .fc = 200.0f,
        .filt_a = 0.25f,
        .filt_b = 0.75f
    };
    lpf_t control = legacy;
    size_t index;

    low_pf_init(&legacy);
    control_lpf_init(&control);

    if (memcmp(&control, &legacy, sizeof(control)) != 0)
    {
        fprintf(stderr, "低通滤波初始化结果未保持逐位一致。\n");
        return 0;
    }

    for (index = 0U; index < sizeof(input_cases) / sizeof(input_cases[0]); index++)
    {
        const float legacy_output = low_pf(&legacy, input_cases[index]);
        const float control_output = control_lpf_step(&control, input_cases[index]);

        if (!expect_same_float_bits("低通滤波输出", control_output, legacy_output) ||
            (memcmp(&control, &legacy, sizeof(control)) != 0))
        {
            fprintf(stderr, "低通滤波等价用例 %zu 失败。\n", index);
            return 0;
        }
    }

    return 1;
}

static int test_pure_transform_interface(void)
{
    float sin_theta = 0.0f;
    float cos_theta = 0.0f;
    float i_alpha = 0.0f;
    float i_beta = 0.0f;
    float i_d = 0.0f;
    float i_q = 0.0f;
    float v_alpha = 0.0f;
    float v_beta = 0.0f;

    foc_sin_cos(0.0f, &sin_theta, &cos_theta);
    foc_clarke(1.25f, -0.25f, -1.0f, &i_alpha, &i_beta);
    foc_park(i_alpha, i_beta, 0.6f, 0.8f, &i_d, &i_q);
    foc_inv_park(0.4f, -0.2f, 0.6f, 0.8f, &v_alpha, &v_beta);

    return expect_close("纯 sin", sin_theta, 0.0f) &&
           expect_close("纯 cos", cos_theta, 1.0f) &&
           expect_close("纯 Clarke i_alpha", i_alpha, 1.25f) &&
           expect_close("纯 Clarke i_beta", i_beta, 0.43301270f) &&
           expect_close("纯 Park i_d", i_d, 1.25980762f) &&
           expect_close("纯 Park i_q", i_q, -0.40358984f) &&
           expect_close("纯逆 Park v_alpha", v_alpha, 0.44f) &&
           expect_close("纯逆 Park v_beta", v_beta, 0.08f);
}

static int test_foc_core_migration_equivalence(void)
{
    static const foc_sample_t samples[] = {
        {1.25f, -0.25f, -1.0f, 0.0f},
        {-2.0f, 0.75f, 1.25f, -0.2f},
        {3.0f, -1.0f, -2.0f, 6.5f},
        {0.0f, -0.0f, 0.0f, 6.28318530716f}
    };
    static const foc_voltage_t voltages[] = {
        {0.4f, -0.2f, 0.04f},
        {-1.5f, 2.0f, 0.03f},
        {20.0f, 20.0f, 0.10f},
        {0.0f, -0.0f, 0.05f}
    };
    size_t index;

    for (index = 0U; index < sizeof(samples) / sizeof(samples[0]); index++)
    {
        foc_frame_t legacy_frame;
        foc_frame_t control_frame;
        foc_duty_t legacy_duty;
        foc_duty_t control_duty;
        bool legacy_valid;
        bool control_valid;

        legacy_foc_core_prepare(&samples[index], &legacy_frame);
        foc_core_prepare(&samples[index], &control_frame);

        if (memcmp(&control_frame, &legacy_frame, sizeof(control_frame)) != 0)
        {
            fprintf(stderr, "FOC 输入阶段等价用例 %zu 失败。\n", index);
            return 0;
        }

        legacy_valid = legacy_foc_core_modulate(&legacy_frame,
                                                &voltages[index],
                                                &legacy_duty);
        control_valid = foc_core_modulate(&control_frame,
                                          &voltages[index],
                                          &control_duty);

        if ((control_valid != legacy_valid) ||
            (memcmp(&control_duty, &legacy_duty, sizeof(control_duty)) != 0))
        {
            fprintf(stderr, "FOC 输出阶段等价用例 %zu 失败。\n", index);
            return 0;
        }
    }

    return 1;
}

static int test_sin_cos_migration_equivalence(void)
{
    static const float theta_cases[] = {
        0.0f,
        -0.0f,
        0.6f,
        -1.2f,
        6.28318530f
    };
    size_t index;

    for (index = 0U; index < sizeof(theta_cases) / sizeof(theta_cases[0]); index++)
    {
        legacy_foc_state_t legacy = {0};
        float sin_theta = 0.0f;
        float cos_theta = 0.0f;

        legacy.theta = theta_cases[index];
        sin_cos_val(&legacy);
        foc_sin_cos(theta_cases[index], &sin_theta, &cos_theta);

        /* P4 起三角实现换为 B 库多项式（fast_trig.h），Legacy 为 512 点查表插值，
         * 数学同源但量化不同：逐位比较改为容差比较（查表量化误差量级 5e-4）。 */
        if (fabsf(sin_theta - legacy.sin_val) > 5.0e-4f ||
            fabsf(cos_theta - legacy.cos_val) > 5.0e-4f)
        {
            fprintf(stderr,
                    "正余弦等价用例 %zu 超差：sin 差 %.3e，cos 差 %.3e。\n",
                    index,
                    (double)fabsf(sin_theta - legacy.sin_val),
                    (double)fabsf(cos_theta - legacy.cos_val));
            return 0;
        }
    }

    return 1;
}

static int test_transform_migration_equivalence(void)
{
    static const struct {
        float i_a;
        float i_b;
        float i_c;
        float sin_theta;
        float cos_theta;
        float v_d;
        float v_q;
    } cases[] = {
        {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
        {1.25f, -0.25f, -1.0f, 0.6f, 0.8f, 0.4f, -0.2f},
        {-2.0f, 1.25f, 0.75f, -0.8f, 0.6f, -0.3f, 0.9f},
        {-0.0f, 0.0f, -0.0f, 1.0f, 0.0f, -0.0f, 0.0f}
    };
    size_t index;

    for (index = 0U; index < sizeof(cases) / sizeof(cases[0]); index++) {
        legacy_foc_state_t legacy = {0};
        float i_alpha = 0.0f;
        float i_beta = 0.0f;
        float i_d = 0.0f;
        float i_q = 0.0f;
        float v_alpha = 0.0f;
        float v_beta = 0.0f;

        legacy.ia = cases[index].i_a;
        legacy.ib = cases[index].i_b;
        legacy.ic = cases[index].i_c;
        legacy.sin_val = cases[index].sin_theta;
        legacy.cos_val = cases[index].cos_theta;
        legacy.vd = cases[index].v_d;
        legacy.vq = cases[index].v_q;

        clarke_transform(&legacy);
        park_transform(&legacy);
        inverse_park(&legacy);

        foc_clarke(cases[index].i_a,
                   cases[index].i_b,
                   cases[index].i_c,
                   &i_alpha,
                   &i_beta);
        foc_park(i_alpha,
                 i_beta,
                 cases[index].sin_theta,
                 cases[index].cos_theta,
                 &i_d,
                 &i_q);
        foc_inv_park(cases[index].v_d,
                     cases[index].v_q,
                     cases[index].sin_theta,
                     cases[index].cos_theta,
                     &v_alpha,
                     &v_beta);

        if (!expect_same_float_bits("Clarke i_alpha", i_alpha, legacy.ialpha) ||
            !expect_same_float_bits("Clarke i_beta", i_beta, legacy.ibeta) ||
            !expect_same_float_bits("Park i_d", i_d, legacy.id) ||
            !expect_same_float_bits("Park i_q", i_q, legacy.iq) ||
            !expect_same_float_bits("逆 Park v_alpha", v_alpha, legacy.valpha) ||
            !expect_same_float_bits("逆 Park v_beta", v_beta, legacy.vbeta)) {
            fprintf(stderr, "坐标变换等价用例 %zu 失败。\n", index);
            return 0;
        }
    }

    return 1;
}

static int test_coordinate_transforms(void)
{
    legacy_foc_state_t foc = {0};

    foc.ia = 1.0f;
    foc.ib = -0.5f;
    foc.ic = -0.5f;
    clarke_transform(&foc);

    if (!expect_close("Clarke i_alpha", foc.ialpha, 1.0f) ||
        !expect_close("Clarke i_beta", foc.ibeta, 0.0f)) {
        return 0;
    }

    foc.sin_val = 0.0f;
    foc.cos_val = 1.0f;
    park_transform(&foc);

    if (!expect_close("Park i_d", foc.id, 1.0f) ||
        !expect_close("Park i_q", foc.iq, 0.0f)) {
        return 0;
    }

    foc.vd = 0.25f;
    foc.vq = -0.5f;
    inverse_park(&foc);

    if (!expect_close("逆 Park v_alpha", foc.valpha, 0.25f) ||
        !expect_close("逆 Park v_beta", foc.vbeta, -0.5f)) {
        return 0;
    }

    inverse_clarke(&foc);
    return expect_close("逆 Clarke v_a", foc.va, 0.25f) &&
           expect_close("逆 Clarke v_b", foc.vb, -0.55801270f) &&
           expect_close("逆 Clarke v_c", foc.vc, 0.30801270f);
}

static int test_svm_migration_equivalence(void)
{
    static const struct {
        float alpha;
        float beta;
    } cases[] = {
        {0.0f, 0.0f},
        {0.2f, 0.1f},
        {0.05f, 0.2f},
        {-0.2f, 0.1f},
        {-0.2f, -0.1f},
        {-0.05f, -0.2f},
        {0.2f, -0.1f},
        {2.0f, 2.0f}
    };
    size_t index;

    for (index = 0U; index < sizeof(cases) / sizeof(cases[0]); index++) {
        float legacy_a = 0.0f;
        float legacy_b = 0.0f;
        float legacy_c = 0.0f;
        float duty_a = 0.0f;
        float duty_b = 0.0f;
        float duty_c = 0.0f;
        const int legacy_result = svm(cases[index].alpha,
                                      cases[index].beta,
                                      &legacy_a,
                                      &legacy_b,
                                      &legacy_c);
        const int result = foc_svm(cases[index].alpha,
                                   cases[index].beta,
                                   &duty_a,
                                   &duty_b,
                                   &duty_c);

        if ((result != legacy_result) ||
            !expect_same_float_bits("SVM duty_a", duty_a, legacy_a) ||
            !expect_same_float_bits("SVM duty_b", duty_b, legacy_b) ||
            !expect_same_float_bits("SVM duty_c", duty_c, legacy_c)) {
            fprintf(stderr, "SVM 等价用例 %zu 失败。\n", index);
            return 0;
        }
    }

    return 1;
}

static int test_svm_vectors(void)
{
    float duty_a = 0.0f;
    float duty_b = 0.0f;
    float duty_c = 0.0f;

    if (foc_svm(0.0f, 0.0f, &duty_a, &duty_b, &duty_c) != 0) {
        fprintf(stderr, "SVM 零矢量被错误判定为无效。\n");
        return 0;
    }

    if (!expect_close("SVM 零矢量 duty_a", duty_a, 0.5f) ||
        !expect_close("SVM 零矢量 duty_b", duty_b, 0.5f) ||
        !expect_close("SVM 零矢量 duty_c", duty_c, 0.5f)) {
        return 0;
    }

    if (foc_svm(0.2f, 0.1f, &duty_a, &duty_b, &duty_c) != 0) {
        fprintf(stderr, "SVM 非零测试矢量被错误判定为无效。\n");
        return 0;
    }

    return expect_close("SVM 非零矢量 duty_a", duty_a, 0.37113249f) &&
           expect_close("SVM 非零矢量 duty_b", duty_b, 0.51339746f) &&
           expect_close("SVM 非零矢量 duty_c", duty_c, 0.62886751f);
}

static int test_foc_zero_angle_pipeline(void)
{
    legacy_foc_state_t foc = {0};

    foc.ia = 1.0f;
    foc.ib = -0.5f;
    foc.ic = -0.5f;
    foc.theta = 0.0f;
    foc.inv_vbus = 1.0f;

    foc_calc(&foc);

    return expect_close("FOC sin", foc.sin_val, 0.0f) &&
           expect_close("FOC cos", foc.cos_val, 1.0f) &&
           expect_close("FOC i_d", foc.id, 1.0f) &&
           expect_close("FOC i_q", foc.iq, 0.0f) &&
           expect_close("FOC duty_a", foc.duty_a, 0.5f) &&
           expect_close("FOC duty_b", foc.duty_b, 0.5f) &&
           expect_close("FOC duty_c", foc.duty_c, 0.5f);
}

int main(void)
{
    _Static_assert(sizeof(uint32_t) == 4U, "uint32_t 必须为 32 位");
    _Static_assert(sizeof(float) == sizeof(uint32_t), "测试要求 float 为 32 位");
    _Static_assert(sizeof(lpf_t) == (6U * sizeof(float)), "lpf_t 布局发生了变化");
    _Static_assert(sizeof(foc_frame_t) == (7U * sizeof(float)), "foc_frame_t 布局发生了变化");
    _Static_assert(sizeof(foc_duty_t) == (5U * sizeof(float)), "foc_duty_t 布局发生了变化");

    if (!test_limit_migration_equivalence()) {
        return 1;
    }

    if (!test_lpf_migration_equivalence()) {
        return 2;
    }

    if (!test_foc_core_migration_equivalence()) {
        return 5;
    }

    if (!test_pure_transform_interface()) {
        return 6;
    }

    if (!test_sin_cos_migration_equivalence()) {
        return 7;
    }

    if (!test_transform_migration_equivalence()) {
        return 8;
    }

    if (!test_coordinate_transforms()) {
        return 9;
    }

    if (!test_svm_migration_equivalence()) {
        return 10;
    }

    if (!test_svm_vectors()) {
        return 11;
    }

    if (!test_foc_zero_angle_pipeline()) {
        return 12;
    }

    return 0;
}
