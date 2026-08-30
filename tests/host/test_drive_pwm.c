/**
 * @file test_drive_pwm.c
 * @brief 使用 Fake Target 验证生产 Drive PWM 提交边界。
 */

#include <stdbool.h>
#include <stdio.h>

#include "common.h"
#include "drive_pwm.h"

static unsigned int start_count;
static unsigned int stop_count;
static unsigned int commit_count;
static float channel_1_duty;
static float channel_2_duty;
static float channel_3_duty;

static void fake_target_reset(void)
{
    start_count = 0U;
    stop_count = 0U;
    commit_count = 0U;
    channel_1_duty = 0.0f;
    channel_2_duty = 0.0f;
    channel_3_duty = 0.0f;
}

void target_pwm_start_phase_outputs(void)
{
    start_count++;
}

void target_pwm_stop_phase_outputs(void)
{
    stop_count++;
}

void target_pwm_set_duty_ratios(float channel_1,
                                float channel_2,
                                float channel_3)
{
    commit_count++;
    channel_1_duty = channel_1;
    channel_2_duty = channel_2;
    channel_3_duty = channel_3;
}

static bool expect_true(bool condition, const char *message)
{
    if (condition)
    {
        return true;
    }

    fprintf(stderr, "%s\n", message);
    return false;
}

static bool expect_logical_record(uint32_t actual_seq,
                                  bool actual_valid,
                                  float actual_duty_a,
                                  float actual_duty_b,
                                  float actual_duty_c,
                                  uint32_t expected_seq,
                                  float expected_duty_a,
                                  float expected_duty_b,
                                  float expected_duty_c)
{
    return expect_true(actual_valid, "PWM 记录应标记为有效。") &&
           expect_true(actual_seq == expected_seq, "PWM 记录周期编号不正确。") &&
           expect_true(actual_duty_a == expected_duty_a,
                       "逻辑 A 相占空比不正确。") &&
           expect_true(actual_duty_b == expected_duty_b,
                       "逻辑 B 相占空比不正确。") &&
           expect_true(actual_duty_c == expected_duty_c,
                       "逻辑 C 相占空比不正确。");
}

static bool test_start_and_stop(void)
{
    fake_target_reset();

    drive_pwm_start();
    drive_pwm_stop();

    return expect_true(start_count == 1U, "PWM 启动应调用一次 Target。") &&
           expect_true(stop_count == 1U, "PWM 停止应调用一次 Target。");
}

static bool test_abc_commit(void)
{
    pmsm_t pm = {
        .fast_seq = 10U,
        .para = {.phase_order = ABC_PHASE},
        .foc = {.dtc_a = 0.1f, .dtc_b = 0.2f, .dtc_c = 0.3f}
    };

    fake_target_reset();
    drive_pwm_commit(&pm);

    return expect_true(commit_count == 1U, "ABC 相序应提交一次 PWM。") &&
           expect_true(channel_1_duty == 0.1f, "ABC 通道 1 应对应 A 相。") &&
           expect_true(channel_2_duty == 0.2f, "ABC 通道 2 应对应 B 相。") &&
           expect_true(channel_3_duty == 0.3f, "ABC 通道 3 应对应 C 相。") &&
           expect_logical_record(pm.pwm_cmd.seq,
                                 pm.pwm_cmd.valid,
                                 pm.pwm_cmd.duty_a,
                                 pm.pwm_cmd.duty_b,
                                 pm.pwm_cmd.duty_c,
                                 10U,
                                 0.1f,
                                 0.2f,
                                 0.3f) &&
           expect_logical_record(pm.pwm_commit.seq,
                                 pm.pwm_commit.valid,
                                 pm.pwm_commit.duty_a,
                                 pm.pwm_commit.duty_b,
                                 pm.pwm_commit.duty_c,
                                 10U,
                                 0.1f,
                                 0.2f,
                                 0.3f);
}

static bool test_acb_commit(void)
{
    pmsm_t pm = {
        .fast_seq = 11U,
        .para = {.phase_order = ACB_PHASE},
        .foc = {.dtc_a = 0.4f, .dtc_b = 0.5f, .dtc_c = 0.6f}
    };

    fake_target_reset();
    drive_pwm_commit(&pm);

    return expect_true(commit_count == 1U, "ACB 相序应提交一次 PWM。") &&
           expect_true(channel_1_duty == 0.4f, "ACB 通道 1 应对应 A 相。") &&
           expect_true(channel_2_duty == 0.6f, "ACB 通道 2 应对应 C 相。") &&
           expect_true(channel_3_duty == 0.5f, "ACB 通道 3 应对应 B 相。") &&
           expect_logical_record(pm.pwm_cmd.seq,
                                 pm.pwm_cmd.valid,
                                 pm.pwm_cmd.duty_a,
                                 pm.pwm_cmd.duty_b,
                                 pm.pwm_cmd.duty_c,
                                 11U,
                                 0.4f,
                                 0.5f,
                                 0.6f) &&
           expect_logical_record(pm.pwm_commit.seq,
                                 pm.pwm_commit.valid,
                                 pm.pwm_commit.duty_a,
                                 pm.pwm_commit.duty_b,
                                 pm.pwm_commit.duty_c,
                                 11U,
                                 0.4f,
                                 0.5f,
                                 0.6f);
}

static bool test_invalid_phase_does_not_commit(void)
{
    pmsm_t pm = {
        .fast_seq = 12U,
        .para = {.phase_order = (phase_order_e)99},
        .foc = {.dtc_a = 0.7f, .dtc_b = 0.8f, .dtc_c = 0.9f},
        .pwm_commit = {.seq = 5U, .valid = true}
    };

    fake_target_reset();
    drive_pwm_commit(&pm);

    return expect_true(commit_count == 0U,
                       "无效相序不能写入 Target PWM。") &&
           expect_logical_record(pm.pwm_cmd.seq,
                                 pm.pwm_cmd.valid,
                                 pm.pwm_cmd.duty_a,
                                 pm.pwm_cmd.duty_b,
                                 pm.pwm_cmd.duty_c,
                                 12U,
                                 0.7f,
                                 0.8f,
                                 0.9f) &&
           expect_true(pm.pwm_commit.seq == 5U,
                       "无效相序不能伪造新的提交周期。") &&
           expect_true(pm.pwm_commit.valid,
                       "无效相序应保留最近一次有效提交记录。");
}

static bool test_neutral_commit(void)
{
    pmsm_t pm = {
        .fast_seq = 13U,
        .para = {.phase_order = (phase_order_e)99}
    };

    fake_target_reset();
    drive_pwm_set_neutral(&pm);

    return expect_true(commit_count == 1U, "START 中性值应提交一次 PWM。") &&
           expect_true(channel_1_duty == 0.5f, "中性值通道 1 应为 50%。") &&
           expect_true(channel_2_duty == 0.5f, "中性值通道 2 应为 50%。") &&
           expect_true(channel_3_duty == 0.5f, "中性值通道 3 应为 50%。") &&
           expect_logical_record(pm.pwm_cmd.seq,
                                 pm.pwm_cmd.valid,
                                 pm.pwm_cmd.duty_a,
                                 pm.pwm_cmd.duty_b,
                                 pm.pwm_cmd.duty_c,
                                 13U,
                                 0.5f,
                                 0.5f,
                                 0.5f) &&
           expect_logical_record(pm.pwm_commit.seq,
                                 pm.pwm_commit.valid,
                                 pm.pwm_commit.duty_a,
                                 pm.pwm_commit.duty_b,
                                 pm.pwm_commit.duty_c,
                                 13U,
                                 0.5f,
                                 0.5f,
                                 0.5f);
}

int main(void)
{
    if (!test_start_and_stop())
    {
        return 1;
    }

    if (!test_abc_commit())
    {
        return 2;
    }

    if (!test_acb_commit())
    {
        return 3;
    }

    if (!test_invalid_phase_does_not_commit())
    {
        return 4;
    }

    if (!test_neutral_commit())
    {
        return 5;
    }

    return 0;
}
