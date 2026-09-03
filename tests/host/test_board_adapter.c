/**
 * @file test_board_adapter.c
 * @brief 验证板卡 ADC 相序、零偏和物理量换算。
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "board_adapter.h"

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
        || !test_invalid_phase_order())
    {
        return 1;
    }

    return 0;
}
