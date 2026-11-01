/**
 * @file test_board_adapter.c
 * @brief 验证板卡 ADC 相序、零偏和物理量换算。
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "board_adapter.h"
#include "common.h"

#define TEST_TOLERANCE (1.0e-6f)

static bool expect_true(bool condition, const char *message)
{
    if (condition)
    {
        return true;
    }

    fprintf(stderr, "%s\n", message);
    return false;
}

static bool expect_close(float actual, float expected, const char *message)
{
    return expect_true(fabsf(actual - expected) <= TEST_TOLERANCE, message);
}

static target_adc_raw_t make_raw(void)
{
    return (target_adc_raw_t){
        .i = {.a = 2100U, .b = 2200U, .c = 2300U},
        .v = {.a = 100U, .b = 200U, .c = 300U},
        .vbus = 1000U,
    };
}

/* 只替代硬件读取；生产采样与换算函数参与测试。 */
void target_adc_read_raw(target_adc_raw_t *raw)
{
    *raw = make_raw();
}

static bool test_sample_result_and_failure(void)
{
    foc_t motor = {0};
    board_sample_t sample = {0};
    motor.motor.phase_order = PHASE_ORDER_ACB;
    motor.board.i_ratio = 0.01f;
    motor.board.v_ratio = 0.02f;
    motor.adc.ia_off = 2000.0f;
    motor.adc.ib_off = 2000.0f;
    motor.adc.ic_off = 2000.0f;
    motor.fb.ia = 9.0f;
    motor.fb.ib = 8.0f;
    motor.fb.ic = 7.0f;
    motor.fb.vbus = 6.0f;
    const foc_fb_state_t fb_before = motor.fb;

    if (!expect_true(foc_adc_sample(&motor, &sample), "有效相序应返回新样本。") ||
        !expect_close(sample.ia, 1.0f, "采样 A 相换算错误。") ||
        !expect_close(sample.ib, 3.0f, "采样 B 相换算错误。") ||
        !expect_close(sample.ic, 2.0f, "采样 C 相换算错误。") ||
        !expect_close(sample.vbus, 20.0f, "采样母线换算错误。") ||
        !expect_true(motor.adc.raw.ia == 2100U && motor.adc.raw.ib == 2300U &&
                     motor.adc.raw.ic == 2200U && motor.adc.raw.va == 100U &&
                     motor.adc.raw.vb == 300U && motor.adc.raw.vc == 200U &&
                     motor.adc.raw.vbus == 1000U, "原始计数仍须按逻辑相序保存。") ||
        !expect_true(memcmp(&motor.fb, &fb_before, sizeof(motor.fb)) == 0,
                     "ADC 采样不能提前修改控制反馈。"))
    {
        return false;
    }

    const board_sample_t sample_before = sample;
    motor.motor.phase_order = (phase_order_t)99;
    const foc_t motor_before = motor;
    return expect_true(!foc_adc_sample(&motor, &sample), "非法相序应返回失败。") &&
           expect_true(memcmp(&sample, &sample_before, sizeof(sample)) == 0 &&
                       memcmp(&motor, &motor_before, sizeof(motor)) == 0,
                       "采样失败须保留输出、原始计数和控制状态。");
}

static board_adc_cfg_t make_cfg(phase_order_t order)
{
    return (board_adc_cfg_t){
        .phase_order = order,
        .i_scale = 0.01f,
        .v_scale = 0.02f,
        .i_offset_a = 2000.0f,
        .i_offset_b = 2000.0f,
        .i_offset_c = 2000.0f,
    };
}

static bool test_abc_conversion(void)
{
    const target_adc_raw_t raw = make_raw();
    const board_adc_cfg_t cfg = make_cfg(PHASE_ORDER_ABC);
    board_sample_t sample;

    return expect_true(board_adc_convert(&cfg, &raw, &sample),
                       "ABC 相序应完成换算。")
        && expect_true((sample.i_raw.a == 2100U)
                       && (sample.i_raw.b == 2200U)
                       && (sample.i_raw.c == 2300U),
                       "ABC 电流原始值映射错误。")
        && expect_close(sample.ia, 1.0f, "A 相电流换算错误。")
        && expect_close(sample.ib, 2.0f, "B 相电流换算错误。")
        && expect_close(sample.ic, 3.0f, "C 相电流换算错误。")
        && expect_close(sample.vbus, 20.0f, "母线电压换算错误。");
}

static bool test_acb_conversion(void)
{
    const target_adc_raw_t raw = make_raw();
    const board_adc_cfg_t cfg = make_cfg(PHASE_ORDER_ACB);
    board_sample_t sample;

    return expect_true(board_adc_convert(&cfg, &raw, &sample),
                       "ACB 相序应完成换算。")
        && expect_true((sample.i_raw.a == 2100U)
                       && (sample.i_raw.b == 2300U)
                       && (sample.i_raw.c == 2200U),
                       "ACB 电流原始值应交换 B、C 两相。")
        && expect_true((sample.v_raw.a == 100U)
                       && (sample.v_raw.b == 300U)
                       && (sample.v_raw.c == 200U),
                       "ACB 电压原始值应交换 B、C 两相。")
        && expect_close(sample.ib, 3.0f, "ACB 的 B 相电流换算错误。")
        && expect_close(sample.ic, 2.0f, "ACB 的 C 相电流换算错误。");
}

static bool test_invalid_phase_order(void)
{
    const target_adc_raw_t raw = make_raw();
    const board_adc_cfg_t cfg = make_cfg((phase_order_t)99);
    board_sample_t sample = {0};

    return expect_true(!board_adc_convert(&cfg, &raw, &sample),
                       "非法相序不得生成物理反馈。");
}

int main(void)
{
    if (!test_abc_conversion()
        || !test_acb_conversion()
        || !test_invalid_phase_order()
        || !test_sample_result_and_failure())
    {
        return 1;
    }

    return 0;
}
