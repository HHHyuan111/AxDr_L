/**
 * @file test_drive_state.c
 * @brief 在电脑端验证生产 Drive 状态闭环和动作顺序。
 *
 * 测试直接编译生产目录中的 drive.c。这里仅替代 PWM、控制器复位和模式执行
 * 函数，以记录调用顺序；不会连接 STM32 HAL，也不会操作真实硬件。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "common.h"
#include "drive.h"

#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 201112L)
#error "需要支持 C11 的电脑端编译器"
#endif

typedef enum
{
    TEST_EVENT_PWM_START = 0,
    TEST_EVENT_DUTY_NEUTRAL,
    TEST_EVENT_RESET,
    TEST_EVENT_RUN_MODE,
    TEST_EVENT_PWM_STOP
} test_event_e;

#define TEST_EVENT_CAPACITY (8U)

static test_event_e test_events[TEST_EVENT_CAPACITY];
static size_t test_event_count;
static bool test_mode_sets_fault;
static bool test_mode_requests_stop;

static void test_log_event(test_event_e event)
{
    test_events[test_event_count] = event;
    test_event_count++;
}

static void test_reset_fakes(void)
{
    test_event_count = 0U;
    test_mode_sets_fault = false;
    test_mode_requests_stop = false;
}

static bool test_expect(bool condition, const char *message)
{
    if (condition)
    {
        return true;
    }

    fprintf(stderr, "%s\n", message);
    return false;
}

static bool test_expect_events(const test_event_e *expected, size_t expected_count)
{
    size_t index;

    if (test_event_count != expected_count)
    {
        fprintf(stderr,
                "动作数量不符合预期：实际 %zu，期望 %zu。\n",
                test_event_count,
                expected_count);
        return false;
    }

    for (index = 0U; index < expected_count; index++)
    {
        if (test_events[index] != expected[index])
        {
            fprintf(stderr,
                    "第 %zu 个动作不符合预期：实际 %d，期望 %d。\n",
                    index,
                    (int)test_events[index],
                    (int)expected[index]);
            return false;
        }
    }

    return true;
}

void drive_pwm_start(void)
{
    test_log_event(TEST_EVENT_PWM_START);
}

void drive_pwm_stop(void)
{
    test_log_event(TEST_EVENT_PWM_STOP);
}

void drive_pwm_set_neutral(pmsm_t *pm)
{
    (void)pm;
    test_log_event(TEST_EVENT_DUTY_NEUTRAL);
}

void pmsm_reset(pmsm_t *pm)
{
    (void)pm;
    test_log_event(TEST_EVENT_RESET);
}

void drive_mode_step(pmsm_t *pm)
{
    test_log_event(TEST_EVENT_RUN_MODE);

    if (test_mode_sets_fault)
    {
        pm->fault.all = 1U;
    }

    if (test_mode_requests_stop)
    {
        pm->req = DRIVE_REQ_STOP;
    }
}

static bool test_power_on_stays_stopped(void)
{
    pmsm_t pm = {
        .fast_seq = 17U,
        .pwm_cmd = {
            .seq = 3U,
            .valid = true,
            .duty_a = 0.1f,
            .duty_b = 0.2f,
            .duty_c = 0.3f
        }
    };

    test_reset_fakes();
    drive_fast_step(&pm);

    return test_expect(test_event_count == 0U,
                       "上电 STOP 周期不应调用 PWM 接口。") &&
           test_expect(pm.req == DRIVE_REQ_STOP,
                       "上电请求应保持 STOP。") &&
           test_expect(pm.state == DRIVE_STATE_STOP,
                       "上电状态应保持 STOP。") &&
           test_expect(!pm.pwm_active,
                       "上电时三相 PWM 软件状态应为关闭。") &&
           test_expect(pm.pwm_cmd.seq == 17U,
                       "本周期占空比请求应使用当前快速周期序号。") &&
           test_expect(!pm.pwm_cmd.valid,
                       "没有运行控制模式时不得沿用上一周期占空比请求。");
}

static bool test_start_then_run(void)
{
    static const test_event_e start_events[] = {
        TEST_EVENT_PWM_START,
        TEST_EVENT_DUTY_NEUTRAL,
        TEST_EVENT_RESET
    };
    static const test_event_e run_events[] = {
        TEST_EVENT_RUN_MODE
    };
    pmsm_t pm = {0};

    pm.req = DRIVE_REQ_START;
    test_reset_fakes();
    drive_fast_step(&pm);

    if (!test_expect_events(start_events,
                            sizeof(start_events) / sizeof(start_events[0])) ||
        !test_expect(pm.req == DRIVE_REQ_RUN,
                     "START 成功后请求应自动转为 RUN。") ||
        !test_expect(pm.state == DRIVE_STATE_STARTING,
                     "START 周期结束时状态应为 STARTING。") ||
        !test_expect(pm.pwm_active,
                     "START 周期结束时三相 PWM 应标记为已启动。"))
    {
        return false;
    }

    test_reset_fakes();
    drive_fast_step(&pm);

    if (!test_expect_events(run_events,
                            sizeof(run_events) / sizeof(run_events[0])) ||
        !test_expect(pm.state == DRIVE_STATE_RUN,
                     "START 后的下一周期应进入 RUN。"))
    {
        return false;
    }

    test_reset_fakes();
    drive_fast_step(&pm);

    return test_expect_events(run_events,
                              sizeof(run_events) / sizeof(run_events[0]));
}

static bool test_stop_runs_once(void)
{
    static const test_event_e stop_events[] = {
        TEST_EVENT_PWM_STOP,
        TEST_EVENT_RESET
    };
    pmsm_t pm = {
        .req = DRIVE_REQ_STOP,
        .state = DRIVE_STATE_RUN,
        .pwm_active = true
    };

    test_reset_fakes();
    drive_fast_step(&pm);

    if (!test_expect_events(stop_events,
                            sizeof(stop_events) / sizeof(stop_events[0])) ||
        !test_expect(pm.state == DRIVE_STATE_STOP,
                     "STOP 后状态应为 STOP。") ||
        !test_expect(!pm.pwm_active,
                     "STOP 后三相 PWM 应标记为关闭。"))
    {
        return false;
    }

    test_reset_fakes();
    drive_fast_step(&pm);

    return test_expect(test_event_count == 0U,
                       "持续 STOP 时不应重复停止 PWM。") &&
           test_expect(pm.state == DRIVE_STATE_STOP,
                       "持续 STOP 时状态应保持 STOP。");
}

static bool test_fault_stops_same_cycle(void)
{
    static const test_event_e fault_events[] = {
        TEST_EVENT_RUN_MODE,
        TEST_EVENT_PWM_STOP,
        TEST_EVENT_RESET
    };
    pmsm_t pm = {
        .req = DRIVE_REQ_RUN,
        .state = DRIVE_STATE_RUN,
        .pwm_active = true
    };

    test_reset_fakes();
    test_mode_sets_fault = true;
    drive_fast_step(&pm);

    return test_expect_events(fault_events,
                              sizeof(fault_events) / sizeof(fault_events[0])) &&
           test_expect(pm.req == DRIVE_REQ_STOP,
                       "故障后请求应退回 STOP。") &&
           test_expect(pm.state == DRIVE_STATE_FAULT,
                       "故障后状态应为 FAULT。") &&
           test_expect(!pm.pwm_active,
                       "故障周期末三相 PWM 应标记为关闭。");
}

static bool test_existing_fault_keeps_original_order(void)
{
    static const test_event_e fault_start_events[] = {
        TEST_EVENT_PWM_START,
        TEST_EVENT_DUTY_NEUTRAL,
        TEST_EVENT_RESET,
        TEST_EVENT_PWM_STOP,
        TEST_EVENT_RESET
    };
    pmsm_t pm = {
        .req = DRIVE_REQ_START,
        .state = DRIVE_STATE_STOP,
        .fault = {.all = 1U}
    };

    test_reset_fakes();
    drive_fast_step(&pm);

    return test_expect_events(
               fault_start_events,
               sizeof(fault_start_events) / sizeof(fault_start_events[0])) &&
           test_expect(pm.req == DRIVE_REQ_STOP,
                       "已有故障的 START 周期应退回 STOP 请求。") &&
           test_expect(pm.state == DRIVE_STATE_FAULT,
                       "已有故障的 START 周期应进入 FAULT。") &&
           test_expect(!pm.pwm_active,
                       "已有故障的 START 周期末 PWM 应关闭。");
}

static bool test_stop_request_during_mode_is_preserved(void)
{
    static const test_event_e run_events[] = {
        TEST_EVENT_RUN_MODE
    };
    static const test_event_e stop_events[] = {
        TEST_EVENT_PWM_STOP,
        TEST_EVENT_RESET
    };
    pmsm_t pm = {
        .req = DRIVE_REQ_RUN,
        .state = DRIVE_STATE_RUN,
        .pwm_active = true
    };

    test_reset_fakes();
    test_mode_requests_stop = true;
    drive_fast_step(&pm);

    if (!test_expect_events(run_events,
                            sizeof(run_events) / sizeof(run_events[0])) ||
        !test_expect(pm.req == DRIVE_REQ_STOP,
                     "模式内写入的 STOP 请求不能被状态更新覆盖。") ||
        !test_expect(pm.state == DRIVE_STATE_RUN,
                     "STOP 请求所在周期仍应完成当前 RUN 动作。"))
    {
        return false;
    }

    test_reset_fakes();
    drive_fast_step(&pm);

    return test_expect_events(stop_events,
                              sizeof(stop_events) / sizeof(stop_events[0])) &&
           test_expect(pm.state == DRIVE_STATE_STOP,
                       "下一周期应执行 STOP。");
}

static bool test_invalid_requests_do_not_run(void)
{
    static const test_event_e stop_events[] = {
        TEST_EVENT_PWM_STOP,
        TEST_EVENT_RESET
    };
    pmsm_t inactive_pm = {
        .req = DRIVE_REQ_RUN,
        .state = DRIVE_STATE_STOP,
        .pwm_active = false
    };
    pmsm_t invalid_pm = {
        .req = (drive_req_e)99,
        .state = DRIVE_STATE_RUN,
        .pwm_active = true
    };

    test_reset_fakes();
    drive_fast_step(&inactive_pm);

    if (!test_expect(test_event_count == 0U,
                     "PWM 未启动时，RUN 请求不能执行控制模式。") ||
        !test_expect(inactive_pm.req == DRIVE_REQ_STOP,
                     "无效 RUN 应退回 STOP 请求。") ||
        !test_expect(inactive_pm.state == DRIVE_STATE_STOP,
                     "无效 RUN 应保持 STOP 状态。"))
    {
        return false;
    }

    test_reset_fakes();
    drive_fast_step(&invalid_pm);

    return test_expect_events(stop_events,
                              sizeof(stop_events) / sizeof(stop_events[0])) &&
           test_expect(invalid_pm.req == DRIVE_REQ_STOP,
                       "非法请求应退回 STOP。") &&
           test_expect(invalid_pm.state == DRIVE_STATE_STOP,
                       "非法请求应进入 STOP。") &&
           test_expect(!invalid_pm.pwm_active,
                       "非法请求不应保持 PWM 输出。");
}

int main(void)
{
    if (!test_power_on_stays_stopped())
    {
        return 1;
    }

    if (!test_start_then_run())
    {
        return 2;
    }

    if (!test_stop_runs_once())
    {
        return 3;
    }

    if (!test_fault_stops_same_cycle())
    {
        return 4;
    }

    if (!test_existing_fault_keeps_original_order())
    {
        return 5;
    }

    if (!test_stop_request_during_mode_is_preserved())
    {
        return 6;
    }

    if (!test_invalid_requests_do_not_run())
    {
        return 7;
    }

    return 0;
}
