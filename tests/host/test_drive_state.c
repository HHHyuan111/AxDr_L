/**
 * @file test_drive_state.c
 * @brief 在电脑端验证生产 Drive 状态闭环和动作顺序。
 *
 * 测试直接编译生产目录中的 drive.c。这里仅替代 PWM、控制器复位和模式执行
 * 函数，以记录调用顺序；不会连接 STM32 HAL，也不会操作真实硬件。
 */

#include <stdbool.h>
#include <math.h>
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
static bool test_mode_valid;
static bool test_mode_writes_pwm;
static bool test_mode_supported;
static bool test_pwm_start_result;
static bool test_pwm_stop_result;

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
    test_mode_valid = true;
    test_mode_writes_pwm = true;
    test_mode_supported = true;
    test_pwm_start_result = true;
    test_pwm_stop_result = true;
}

void drive_diag_poll_request(pmsm_t *pm)
{
    (void)pm;
}

void drive_diag_on_stopped(pmsm_t *pm)
{
    (void)pm;
}

void drive_diag_on_fault(pmsm_t *pm)
{
    (void)pm;
}

bool drive_mode_prepare(pmsm_t *pm)
{
    (void)pm;
    return test_mode_supported;
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

bool drive_pwm_start(void)
{
    test_log_event(TEST_EVENT_PWM_START);
    return test_pwm_start_result;
}

bool drive_pwm_stop(void)
{
    test_log_event(TEST_EVENT_PWM_STOP);
    return test_pwm_stop_result;
}

void drive_pwm_set_neutral(pmsm_t *pm)
{
    (void)pm;
    test_log_event(TEST_EVENT_DUTY_NEUTRAL);
}

void drive_control_reset(pmsm_t *pm)
{
    (void)pm;
    test_log_event(TEST_EVENT_RESET);
}

bool drive_mode_step(pmsm_t *pm)
{
    test_log_event(TEST_EVENT_RUN_MODE);

    if (test_mode_writes_pwm)
    {
        pm->pwm_cmd.valid = true;
    }

    if (test_mode_sets_fault)
    {
        pm->fault.all = 1U;
    }

    if (test_mode_requests_stop)
    {
        pm->req = DRIVE_REQ_STOP;
    }

    return test_mode_valid;
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
        TEST_EVENT_DUTY_NEUTRAL,
        TEST_EVENT_PWM_START,
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

static bool test_unsupported_mode_never_starts_pwm(void)
{
    pmsm_t pm = {
        .req = DRIVE_REQ_START,
        .state = DRIVE_STATE_STOP
    };

    test_reset_fakes();
    test_mode_supported = false;
    drive_fast_step(&pm);

    return test_expect(test_event_count == 0U,
                       "未验证模式不能产生任何功率级启动动作。") &&
           test_expect(pm.req == DRIVE_REQ_STOP,
                       "未验证模式的 START 请求应退回 STOP。") &&
           test_expect(pm.state == DRIVE_STATE_STOP,
                       "未验证模式应保持 STOP 状态。") &&
           test_expect(!pm.pwm_active,
                       "未验证模式不能把 PWM 标记为已启动。");
}

static bool test_pwm_action_failure_is_reported(void)
{
    static const test_event_e start_failure_events[] = {
        TEST_EVENT_DUTY_NEUTRAL,
        TEST_EVENT_PWM_START
    };
    static const test_event_e stop_failure_events[] = {
        TEST_EVENT_PWM_STOP,
        TEST_EVENT_RESET,
        TEST_EVENT_PWM_STOP,
        TEST_EVENT_RESET
    };
    pmsm_t start_pm = {.req = DRIVE_REQ_START};
    pmsm_t stop_pm = {
        .req = DRIVE_REQ_STOP,
        .state = DRIVE_STATE_RUN,
        .pwm_active = true
    };

    test_reset_fakes();
    test_pwm_start_result = false;
    drive_fast_step(&start_pm);
    if (!test_expect_events(
            start_failure_events,
            sizeof(start_failure_events) / sizeof(start_failure_events[0])) ||
        !test_expect(start_pm.fault.bit.pwm_err == 1U,
                     "PWM 启动失败必须锁存执行故障。") ||
        !test_expect(start_pm.state == DRIVE_STATE_FAULT,
                     "PWM 启动失败必须进入 FAULT。") ||
        !test_expect(!start_pm.pwm_active,
                     "PWM 启动失败不能标记为已启动。"))
    {
        return false;
    }

    test_reset_fakes();
    test_pwm_stop_result = false;
    drive_fast_step(&stop_pm);

    return test_expect_events(
               stop_failure_events,
               sizeof(stop_failure_events) / sizeof(stop_failure_events[0])) &&
           test_expect(stop_pm.fault.bit.pwm_err == 1U,
                       "PWM 停止失败必须锁存执行故障。") &&
           test_expect(stop_pm.state == DRIVE_STATE_FAULT,
                       "PWM 停止失败必须进入 FAULT。") &&
           test_expect(stop_pm.pwm_active,
                       "PWM 停止失败必须保留活动状态供下一周期重试。");
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

static bool test_existing_fault_blocks_start_and_run(void)
{
    static const test_event_e fault_run_events[] = {
        TEST_EVENT_PWM_STOP,
        TEST_EVENT_RESET
    };
    pmsm_t start_pm = {
        .req = DRIVE_REQ_START,
        .state = DRIVE_STATE_STOP,
        .fault = {.all = 1U}
    };
    pmsm_t run_pm = {
        .req = DRIVE_REQ_RUN,
        .state = DRIVE_STATE_RUN,
        .pwm_active = true,
        .fault = {.all = 1U}
    };

    test_reset_fakes();
    drive_fast_step(&start_pm);

    if (!test_expect(test_event_count == 0U,
                     "已有故障时不能执行 START 的任何功率动作。") ||
        !test_expect(start_pm.req == DRIVE_REQ_STOP,
                     "已有故障的 START 请求应退回 STOP。") ||
        !test_expect(start_pm.state == DRIVE_STATE_FAULT,
                     "已有故障的 START 请求应进入 FAULT。") ||
        !test_expect(!start_pm.pwm_active,
                     "已有故障时 PWM 必须保持关闭。"))
    {
        return false;
    }

    test_reset_fakes();
    drive_fast_step(&run_pm);

    return test_expect_events(
               fault_run_events,
               sizeof(fault_run_events) / sizeof(fault_run_events[0])) &&
           test_expect(run_pm.req == DRIVE_REQ_STOP,
                       "已有故障的 RUN 请求应退回 STOP。") &&
           test_expect(run_pm.state == DRIVE_STATE_FAULT,
                       "已有故障的 RUN 请求应进入 FAULT。") &&
           test_expect(!run_pm.pwm_active,
                       "已有故障的 RUN 周期必须关闭 PWM。");
}

static bool test_invalid_mode_or_missing_pwm_stops(void)
{
    static const test_event_e stop_events[] = {
        TEST_EVENT_RUN_MODE,
        TEST_EVENT_PWM_STOP,
        TEST_EVENT_RESET
    };
    pmsm_t invalid_mode_pm = {
        .req = DRIVE_REQ_RUN,
        .state = DRIVE_STATE_RUN,
        .pwm_active = true
    };
    pmsm_t missing_pwm_pm = invalid_mode_pm;

    test_reset_fakes();
    test_mode_valid = false;
    drive_fast_step(&invalid_mode_pm);

    if (!test_expect_events(stop_events,
                            sizeof(stop_events) / sizeof(stop_events[0])) ||
        !test_expect(invalid_mode_pm.req == DRIVE_REQ_STOP,
                     "无效模式应退回 STOP 请求。") ||
        !test_expect(invalid_mode_pm.state == DRIVE_STATE_STOP,
                     "无效模式应进入 STOP 状态。") ||
        !test_expect(!invalid_mode_pm.pwm_active,
                     "无效模式必须关闭 PWM。"))
    {
        return false;
    }

    test_reset_fakes();
    test_mode_writes_pwm = false;
    drive_fast_step(&missing_pwm_pm);

    return test_expect_events(stop_events,
                              sizeof(stop_events) / sizeof(stop_events[0])) &&
           test_expect(missing_pwm_pm.req == DRIVE_REQ_STOP,
                       "本周期没有新 PWM 命令时应退回 STOP。") &&
           test_expect(missing_pwm_pm.state == DRIVE_STATE_STOP,
                       "本周期没有新 PWM 命令时应进入 STOP。") &&
           test_expect(!missing_pwm_pm.pwm_active,
                       "本周期没有新 PWM 命令时必须关闭输出。");
}

static bool test_protection_blocks_power_actions(void)
{
    static const test_event_e stop_events[] = {
        TEST_EVENT_PWM_STOP,
        TEST_EVENT_RESET
    };
    pmsm_t under_voltage_pm = {
        .req = DRIVE_REQ_START,
        .foc = {.vbus = 12.0f},
        .fb_status = {.i_valid = true, .vbus_valid = true, .pos_valid = true},
        .prot_cfg = {
            .under_voltage_v = 15.0f,
            .under_voltage_samples = 1U
        }
    };
    pmsm_t over_current_pm = {
        .req = DRIVE_REQ_RUN,
        .state = DRIVE_STATE_RUN,
        .pwm_active = true,
        .foc = {.i_a = 81.0f, .vbus = 24.0f},
        .fb_status = {.i_valid = true, .vbus_valid = true, .pos_valid = true},
        .prot_cfg = {
            .over_current_a = 80.0f,
            .over_current_samples = 1U
        }
    };

    test_reset_fakes();
    drive_fast_step(&under_voltage_pm);

    if (!test_expect(test_event_count == 0U,
                     "母线欠压时不能执行 START 动作。") ||
        !test_expect(under_voltage_pm.fault.bit.un_volt == 1U,
                     "母线欠压应写入对应故障位。") ||
        !test_expect(under_voltage_pm.state == DRIVE_STATE_FAULT,
                     "母线欠压应进入 FAULT。"))
    {
        return false;
    }

    test_reset_fakes();
    drive_fast_step(&over_current_pm);

    return test_expect_events(stop_events,
                              sizeof(stop_events) / sizeof(stop_events[0])) &&
           test_expect(over_current_pm.fault.bit.ov_curr == 1U,
                       "三相过流应写入对应故障位。") &&
           test_expect(over_current_pm.state == DRIVE_STATE_FAULT,
                       "三相过流应进入 FAULT。") &&
           test_expect(!over_current_pm.pwm_active,
                       "三相过流必须在本周期关闭 PWM。");
}

static bool test_invalid_feedback_blocks_start(void)
{
    pmsm_t invalid_current_pm = {
        .req = DRIVE_REQ_START,
        .foc = {.i_a = NAN, .vbus = 24.0f},
        .fb_status = {.i_valid = true, .vbus_valid = true, .pos_valid = true},
        .prot_cfg = {.invalid_current_samples = 1U}
    };
    pmsm_t invalid_position_pm = {
        .req = DRIVE_REQ_START,
        .foc = {.vbus = 24.0f, .p_e = NAN},
        .fb_status = {.i_valid = true, .vbus_valid = true, .pos_valid = true},
        .prot_cfg = {.invalid_position_samples = 1U}
    };

    test_reset_fakes();
    drive_fast_step(&invalid_current_pm);
    if (!test_expect(test_event_count == 0U,
                     "电流反馈非法时不能执行 START。") ||
        !test_expect(invalid_current_pm.fault.bit.ioff_err == 1U,
                     "非法电流反馈应锁存采样故障。"))
    {
        return false;
    }

    test_reset_fakes();
    drive_fast_step(&invalid_position_pm);

    return test_expect(test_event_count == 0U,
                       "位置反馈非法时不能执行 START。") &&
           test_expect(invalid_position_pm.fault.bit.enc_err == 1U,
                       "非法位置反馈应锁存编码器故障。");
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

static bool test_fault_clear_requires_stopped_pwm(void)
{
    static const test_event_e reset_events[] = {TEST_EVENT_RESET};
    pmsm_t active_pm = {
        .req = DRIVE_REQ_STOP,
        .state = DRIVE_STATE_FAULT,
        .pwm_active = true,
        .fault = {.all = 1U}
    };
    pmsm_t stopped_pm = {
        .req = DRIVE_REQ_STOP,
        .state = DRIVE_STATE_FAULT,
        .pwm_active = false,
        .fault = {.all = 1U},
        .prot_state = {
            .invalid_position_count = 2U,
            .latched_faults = DRIVE_PROTECTION_FAULT_POSITION_FEEDBACK
        }
    };

    test_reset_fakes();
    if (!test_expect(!drive_fault_clear(&active_pm),
                     "PWM 仍活动时不能清除故障。") ||
        !test_expect(active_pm.fault.all == 1U,
                     "拒绝清除时必须保留原故障。") ||
        !test_expect(test_event_count == 0U,
                     "拒绝清除时不能修改控制器状态。"))
    {
        return false;
    }

    test_reset_fakes();
    return test_expect(drive_fault_clear(&stopped_pm),
                       "PWM 关闭后应允许显式清除故障。") &&
           test_expect_events(reset_events,
                              sizeof(reset_events) / sizeof(reset_events[0])) &&
           test_expect(stopped_pm.fault.all == 0U,
                       "故障清除后故障位必须归零。") &&
           test_expect(stopped_pm.prot_state.latched_faults == 0U,
                       "故障清除后保护锁存必须归零。") &&
           test_expect(stopped_pm.state == DRIVE_STATE_STOP,
                       "故障清除后 Drive 必须回到 STOP。");
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

    if (!test_unsupported_mode_never_starts_pwm())
    {
        return 4;
    }

    if (!test_pwm_action_failure_is_reported())
    {
        return 5;
    }

    if (!test_fault_stops_same_cycle())
    {
        return 6;
    }

    if (!test_existing_fault_blocks_start_and_run())
    {
        return 7;
    }

    if (!test_stop_request_during_mode_is_preserved())
    {
        return 8;
    }

    if (!test_invalid_requests_do_not_run())
    {
        return 9;
    }

    if (!test_invalid_mode_or_missing_pwm_stops())
    {
        return 10;
    }

    if (!test_protection_blocks_power_actions())
    {
        return 11;
    }

    if (!test_invalid_feedback_blocks_start())
    {
        return 12;
    }

    if (!test_fault_clear_requires_stopped_pwm())
    {
        return 13;
    }

    return 0;
}
