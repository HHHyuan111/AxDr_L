/**
 * @file test_drive_pwm.c
 * @brief 使用 Fake Target 验证生产 Drive PWM 提交边界。
 */

#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#include "common.h"
#include "drive_pwm.h"

static unsigned int start_count;
static unsigned int stop_count;
static unsigned int commit_count;
static bool start_result;
static bool stop_result;
static float channel_1_duty;
static float channel_2_duty;
static float channel_3_duty;

static void fake_target_reset(void)
{
    start_count = 0U;
    stop_count = 0U;
    commit_count = 0U;
    start_result = true;
    stop_result = true;
    channel_1_duty = 0.0f;
    channel_2_duty = 0.0f;
    channel_3_duty = 0.0f;
}

bool target_pwm_start_phase_outputs(void)
{
    start_count++;
    return start_result;
}

bool target_pwm_stop_phase_outputs(void)
{
    stop_count++;
    return stop_result;
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

    const bool started = drive_pwm_start();
    const bool stopped = drive_pwm_stop();

    return expect_true(started, "Target 成功时 Drive 应报告 PWM 启动成功。") &&
           expect_true(stopped, "Target 成功时 Drive 应报告 PWM 停止成功。") &&
           expect_true(start_count == 1U, "PWM 启动应调用一次 Target。") &&
           expect_true(stop_count == 1U, "PWM 停止应调用一次 Target。");
}

static bool test_abc_commit(void)
{
    pmsm_t pm = {
        .fast_seq = 10U,
        .para = {.phase_order = PHASE_ORDER_ABC},
        .foc = {.dtc_a = 0.1f, .dtc_b = 0.2f, .dtc_c = 0.3f}
    };

    fake_target_reset();
    const bool committed = drive_pwm_commit(&pm);

    return expect_true(committed, "ABC 相序和有效占空比应提交成功。") &&
           expect_true(commit_count == 1U, "ABC 相序应提交一次 PWM。") &&
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
        .para = {.phase_order = PHASE_ORDER_ACB},
        .foc = {.dtc_a = 0.4f, .dtc_b = 0.5f, .dtc_c = 0.6f}
    };

    fake_target_reset();
    const bool committed = drive_pwm_commit(&pm);

    return expect_true(committed, "ACB 相序和有效占空比应提交成功。") &&
           expect_true(commit_count == 1U, "ACB 相序应提交一次 PWM。") &&
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
        .para = {.phase_order = (phase_order_t)99},
        .foc = {.dtc_a = 0.7f, .dtc_b = 0.8f, .dtc_c = 0.9f},
        .pwm_active = true,
        .pwm_commit = {.seq = 5U, .valid = true}
    };

    fake_target_reset();
    const bool committed = drive_pwm_commit(&pm);

    return expect_true(!committed,
                       "无效相序必须报告提交失败。") &&
           expect_true(commit_count == 0U,
                       "无效相序不能写入 Target PWM。") &&
           expect_true(stop_count == 1U,
                       "无效相序必须立即关闭 Target PWM。") &&
           expect_true(!pm.pwm_active,
                       "无效相序后 PWM 软件状态必须为关闭。") &&
           expect_true(!pm.pwm_cmd.valid,
                       "无效相序不能生成有效逻辑命令。") &&
           expect_true(pm.pwm_cmd.seq == 12U,
                       "失败命令仍应记录本周期编号。") &&
           expect_true(pm.pwm_commit.seq == 5U,
                       "无效相序不能伪造新的提交周期。") &&
           expect_true(pm.pwm_commit.valid,
                       "无效相序应保留最近一次有效提交记录。");
}

static bool test_invalid_duty_stops_output(void)
{
    pmsm_t pm = {
        .fast_seq = 14U,
        .para = {.phase_order = PHASE_ORDER_ABC},
        .foc = {.dtc_a = NAN, .dtc_b = 0.5f, .dtc_c = 0.5f},
        .pwm_active = true,
        .pwm_commit = {.seq = 6U, .valid = true}
    };

    fake_target_reset();
    const bool nan_committed = drive_pwm_commit(&pm);

    if (!expect_true(!nan_committed, "NaN 占空比必须报告提交失败。") ||
        !expect_true(commit_count == 0U, "NaN 占空比不能写比较寄存器。") ||
        !expect_true(stop_count == 1U, "NaN 占空比必须关闭 Target PWM。") ||
        !expect_true(!pm.pwm_active, "NaN 占空比后 PWM 状态必须为关闭。") ||
        !expect_true(!pm.pwm_cmd.valid, "NaN 占空比不能生成有效命令。") ||
        !expect_true(pm.pwm_commit.seq == 6U,
                     "无效占空比不能覆盖最近一次有效提交。"))
    {
        return false;
    }

    pm.foc.dtc_a = 0.5f;
    pm.foc.dtc_b = 1.01f;
    pm.pwm_active = true;
    fake_target_reset();

    return expect_true(!drive_pwm_commit(&pm),
                       "超出 0～1 的占空比必须报告失败。") &&
           expect_true(commit_count == 0U,
                       "越界占空比不能写比较寄存器。") &&
           expect_true(stop_count == 1U,
                       "越界占空比必须关闭 Target PWM。");
}

static bool test_failed_stop_keeps_active_state(void)
{
    pmsm_t pm = {
        .para = {.phase_order = PHASE_ORDER_ABC},
        .foc = {.dtc_a = NAN, .dtc_b = 0.5f, .dtc_c = 0.5f},
        .pwm_active = true
    };

    fake_target_reset();
    stop_result = false;

    return expect_true(!drive_pwm_commit(&pm),
                       "无效占空比仍必须报告提交失败。") &&
           expect_true(pm.pwm_active,
                       "Target 停止失败时必须保留活动状态以便上层重试。");
}

static bool test_neutral_commit(void)
{
    pmsm_t pm = {
        .fast_seq = 13U,
        .para = {.phase_order = (phase_order_t)99}
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

    if (!test_invalid_duty_stops_output())
    {
        return 6;
    }

    if (!test_failed_stop_keeps_active_state())
    {
        return 7;
    }

    return 0;
}
