/**
 * @file test_foc_math.c
 * @brief 在电脑端直接验证现有 FOC 基础数学函数。
 *
 * 测试直接编译生产目录中的 foc_calc.c 和 util.c，不复制控制公式。
 * fakes 目录只替代当前测试不需要的 STM32/BSP 头文件。
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "common.h"

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

static int test_coordinate_transforms(void)
{
    pmsm_foc_t foc = {0};

    foc.i_a = 1.0f;
    foc.i_b = -0.5f;
    foc.i_c = -0.5f;
    clarke_transform(&foc);

    if (!expect_close("Clarke i_alpha", foc.i_alph, 1.0f) ||
        !expect_close("Clarke i_beta", foc.i_beta, 0.0f)) {
        return 0;
    }

    foc.sin_val = 0.0f;
    foc.cos_val = 1.0f;
    park_transform(&foc);

    if (!expect_close("Park i_d", foc.i_d, 1.0f) ||
        !expect_close("Park i_q", foc.i_q, 0.0f)) {
        return 0;
    }

    foc.v_d = 0.25f;
    foc.v_q = -0.5f;
    inverse_park(&foc);

    if (!expect_close("逆 Park v_alpha", foc.v_alph, 0.25f) ||
        !expect_close("逆 Park v_beta", foc.v_beta, -0.5f)) {
        return 0;
    }

    inverse_clarke(&foc);
    return expect_close("逆 Clarke v_a", foc.v_a, 0.25f) &&
           expect_close("逆 Clarke v_b", foc.v_b, -0.55801270f) &&
           expect_close("逆 Clarke v_c", foc.v_c, 0.30801270f);
}

static int test_svm_vectors(void)
{
    float duty_a = 0.0f;
    float duty_b = 0.0f;
    float duty_c = 0.0f;

    if (svm(0.0f, 0.0f, &duty_a, &duty_b, &duty_c) != 0) {
        fprintf(stderr, "SVM 零矢量被错误判定为无效。\n");
        return 0;
    }

    if (!expect_close("SVM 零矢量 duty_a", duty_a, 0.5f) ||
        !expect_close("SVM 零矢量 duty_b", duty_b, 0.5f) ||
        !expect_close("SVM 零矢量 duty_c", duty_c, 0.5f)) {
        return 0;
    }

    if (svm(0.2f, 0.1f, &duty_a, &duty_b, &duty_c) != 0) {
        fprintf(stderr, "SVM 非零测试矢量被错误判定为无效。\n");
        return 0;
    }

    return expect_close("SVM 非零矢量 duty_a", duty_a, 0.37113249f) &&
           expect_close("SVM 非零矢量 duty_b", duty_b, 0.51339746f) &&
           expect_close("SVM 非零矢量 duty_c", duty_c, 0.62886751f);
}

static int test_foc_zero_angle_pipeline(void)
{
    pmsm_foc_t foc = {0};

    foc.i_a = 1.0f;
    foc.i_b = -0.5f;
    foc.i_c = -0.5f;
    foc.theta = 0.0f;
    foc.inv_vbus = 1.0f;

    foc_calc(&foc);

    return expect_close("FOC sin", foc.sin_val, 0.0f) &&
           expect_close("FOC cos", foc.cos_val, 1.0f) &&
           expect_close("FOC i_d", foc.i_d, 1.0f) &&
           expect_close("FOC i_q", foc.i_q, 0.0f) &&
           expect_close("FOC duty_a", foc.dtc_a, 0.5f) &&
           expect_close("FOC duty_b", foc.dtc_b, 0.5f) &&
           expect_close("FOC duty_c", foc.dtc_c, 0.5f);
}

int main(void)
{
    _Static_assert(sizeof(uint32_t) == 4U, "uint32_t 必须为 32 位");

    if (!test_coordinate_transforms()) {
        return 1;
    }

    if (!test_svm_vectors()) {
        return 2;
    }

    if (!test_foc_zero_angle_pipeline()) {
        return 3;
    }

    return 0;
}
