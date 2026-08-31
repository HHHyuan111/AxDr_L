/**
 * @file test_control_loop.c
 * @brief 验证可移植 FOC 模式主链和 MIT 给定计算。
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "control_loop.h"
#include "control_mit.h"
#include "legacy_control.h"

#define TEST_TOLERANCE (1.0e-6f)

typedef struct
{
    pid_para_t current_d_pid;
    pid_para_t current_q_pid;
    pid_para_t speed_pid;
    pid_para_t position_pid;
    control_loop_runtime_t runtime;
} test_loop_t;

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

static void test_pid_init(pid_para_t *pid, float kp)
{
    *pid = (pid_para_t){
        .kp = kp,
        .ki = 0.0f,
        .kfp = 1.0f,
        .kf_damp = 1.0f,
        .i_term_max = 10.0f,
        .i_term_min = -10.0f,
        .ts = 0.001f,
        .out_max = 10.0f,
        .out_min = -10.0f,
    };
}

static void test_loop_init(test_loop_t *loop)
{
    test_pid_init(&loop->current_d_pid, 0.2f);
    test_pid_init(&loop->current_q_pid, 0.2f);
    test_pid_init(&loop->speed_pid, 0.1f);
    test_pid_init(&loop->position_pid, 0.1f);

    loop->runtime = (control_loop_runtime_t){
        .current_rate = {.count = 0U, .divider = 1U},
        .speed_rate = {.count = 0U, .divider = 1U},
        .position_rate = {.count = 0U, .divider = 1U},
        .current_d_pid = &loop->current_d_pid,
        .current_q_pid = &loop->current_q_pid,
        .speed_pid = &loop->speed_pid,
        .position_pid = &loop->position_pid,
    };
}

static control_loop_feedback_t test_feedback(void)
{
    return (control_loop_feedback_t){
        .foc_sample = {
            .i_a = 0.2f,
            .i_b = -0.1f,
            .i_c = -0.1f,
            .theta = 0.25f,
        },
        .inv_bus_voltage = 0.05f,
        .rotor_speed_rad_s = 0.4f,
        .rotor_position_rad = 0.3f,
    };
}

static bool test_voltage_mode(void)
{
    test_loop_t loop;
    const control_loop_feedback_t feedback = test_feedback();
    const control_loop_request_t request = {
        .mode = CONTROL_LOOP_MODE_VOLTAGE,
        .voltage_d_v = 0.5f,
        .voltage_q_v = -0.25f,
    };
    control_loop_output_t output;

    test_loop_init(&loop);

    return expect_true(control_loop_step(&loop.runtime,
                                         &feedback,
                                         &request,
                                         &output),
                       "电压模式应生成有效占空比。") &&
           expect_close(output.voltage_d_v, 0.5f,
                        "电压模式 d 轴给定错误。") &&
           expect_close(output.voltage_q_v, -0.25f,
                        "电压模式 q 轴给定错误。") &&
           expect_true(loop.runtime.current_rate.count == 0U,
                       "电压模式不应运行电流环。");
}

static bool test_current_mode(void)
{
    test_loop_t loop;
    const control_loop_feedback_t feedback = test_feedback();
    const control_loop_request_t request = {
        .mode = CONTROL_LOOP_MODE_CURRENT,
        .current_d_ref_a = 0.1f,
        .current_q_ref_a = 0.2f,
    };
    control_loop_output_t output;

    test_loop_init(&loop);

    return expect_true(control_loop_step(&loop.runtime,
                                         &feedback,
                                         &request,
                                         &output),
                       "电流模式应生成有效占空比。") &&
           expect_close(loop.current_d_pid.ref_value, 0.1f,
                        "电流模式 d 轴参考没有进入 PI。") &&
           expect_close(loop.current_q_pid.ref_value, 0.2f,
                        "电流模式 q 轴参考没有进入 PI。") &&
           expect_true(loop.runtime.current_rate.count == 1U,
                       "电流模式应更新电流环分频状态。") &&
           expect_true(loop.runtime.speed_rate.count == 0U,
                       "电流模式不应运行速度环。");
}

static bool test_speed_mode(void)
{
    test_loop_t loop;
    const control_loop_feedback_t feedback = test_feedback();
    const control_loop_request_t request = {
        .mode = CONTROL_LOOP_MODE_SPEED,
        .current_d_ref_a = 0.0f,
        .speed_ref_rad_s = 1.0f,
        .current_limit_a = 2.0f,
    };
    control_loop_output_t output;

    test_loop_init(&loop);

    return expect_true(control_loop_step(&loop.runtime,
                                         &feedback,
                                         &request,
                                         &output),
                       "速度模式应生成有效占空比。") &&
           expect_close(loop.speed_pid.ref_value, 1.0f,
                        "速度参考没有进入速度环。") &&
           expect_close(loop.current_q_pid.ref_value,
                        loop.runtime.current_q_ref_a,
                        "速度环输出没有进入 q 轴电流环。") &&
           expect_true((loop.runtime.speed_rate.count == 0U) &&
                       (loop.runtime.current_rate.count == 1U),
                       "速度模式应依次运行速度环和电流环。");
}

static bool test_position_mode(void)
{
    test_loop_t loop;
    const control_loop_feedback_t feedback = test_feedback();
    const control_loop_request_t request = {
        .mode = CONTROL_LOOP_MODE_POSITION,
        .current_d_ref_a = 0.0f,
        .position_ref_rad = 0.8f,
        .current_limit_a = 2.0f,
        .speed_limit_rad_s = 3.0f,
    };
    control_loop_output_t output;

    test_loop_init(&loop);

    return expect_true(control_loop_step(&loop.runtime,
                                         &feedback,
                                         &request,
                                         &output),
                       "位置模式应生成有效占空比。") &&
           expect_close(loop.position_pid.ref_value, 0.8f,
                        "位置参考没有进入位置环。") &&
           expect_close(loop.speed_pid.ref_value,
                        loop.runtime.speed_ref_rad_s,
                        "位置环输出没有进入速度环。") &&
           expect_close(loop.current_q_pid.ref_value,
                        loop.runtime.current_q_ref_a,
                        "速度环输出没有进入 q 轴电流环。") &&
           expect_true((loop.runtime.position_rate.count == 0U) &&
                       (loop.runtime.speed_rate.count == 0U) &&
                       (loop.runtime.current_rate.count == 1U),
                       "位置模式应依次运行位置、速度和电流环。");
}

static bool test_invalid_mode(void)
{
    test_loop_t loop;
    const control_loop_feedback_t feedback = test_feedback();
    const control_loop_request_t request = {
        .mode = (control_loop_mode_t)99,
    };
    control_loop_output_t output;

    test_loop_init(&loop);

    return expect_true(!control_loop_step(&loop.runtime,
                                          &feedback,
                                          &request,
                                          &output),
                       "非法控制模式不应生成占空比。") &&
           expect_true(!output.duty_valid,
                       "非法控制模式输出必须标记为无效。");
}

static bool test_legacy_equivalence(control_loop_mode_t mode)
{
    test_loop_t migrated;
    test_loop_t legacy;
    const control_loop_feedback_t feedback = test_feedback();
    const control_loop_request_t request = {
        .mode = mode,
        .voltage_d_v = 0.5f,
        .voltage_q_v = -0.25f,
        .current_d_ref_a = 0.1f,
        .current_q_ref_a = 0.2f,
        .speed_ref_rad_s = 1.0f,
        .position_ref_rad = 0.8f,
        .current_limit_a = 2.0f,
        .speed_limit_rad_s = 3.0f,
    };
    control_loop_output_t migrated_output;
    control_loop_output_t legacy_output;
    bool migrated_valid;
    bool legacy_valid;

    test_loop_init(&migrated);
    test_loop_init(&legacy);

    migrated_valid = control_loop_step(&migrated.runtime,
                                       &feedback,
                                       &request,
                                       &migrated_output);
    legacy_valid = legacy_control_loop_step(&legacy.runtime,
                                            &feedback,
                                            &request,
                                            &legacy_output);

    return expect_true(migrated_valid == legacy_valid,
                       "新旧主链占空比有效性不一致。") &&
           expect_true(memcmp(&migrated_output.frame,
                              &legacy_output.frame,
                              sizeof(migrated_output.frame)) == 0,
                       "新旧主链 FOC 坐标结果未保持逐位一致。") &&
           expect_true(memcmp(&migrated_output.duty,
                              &legacy_output.duty,
                              sizeof(migrated_output.duty)) == 0,
                       "新旧主链占空比结果未保持逐位一致。") &&
           expect_true(memcmp(&migrated.current_d_pid,
                              &legacy.current_d_pid,
                              sizeof(migrated.current_d_pid)) == 0,
                       "新旧主链 d 轴 PI 状态不一致。") &&
           expect_true(memcmp(&migrated.current_q_pid,
                              &legacy.current_q_pid,
                              sizeof(migrated.current_q_pid)) == 0,
                       "新旧主链 q 轴 PI 状态不一致。") &&
           expect_true(memcmp(&migrated.speed_pid,
                              &legacy.speed_pid,
                              sizeof(migrated.speed_pid)) == 0,
                       "新旧主链速度控制器状态不一致。") &&
           expect_true(memcmp(&migrated.position_pid,
                              &legacy.position_pid,
                              sizeof(migrated.position_pid)) == 0,
                       "新旧主链位置控制器状态不一致。") &&
           expect_true((migrated.runtime.current_rate.count ==
                        legacy.runtime.current_rate.count) &&
                       (migrated.runtime.speed_rate.count ==
                        legacy.runtime.speed_rate.count) &&
                       (migrated.runtime.position_rate.count ==
                        legacy.runtime.position_rate.count),
                       "新旧主链分频状态不一致。");
}

static bool test_mit_command(void)
{
    const control_mit_input_t input = {
        .position_ref_rad = 1.0f,
        .position_feedback_rad = 0.25f,
        .speed_ref_rad_s = 2.0f,
        .speed_feedback_rad_s = 1.5f,
        .torque_feedforward_nm = 0.2f,
        .position_gain_nm_per_rad = 2.0f,
        .speed_gain_nm_s_per_rad = 3.0f,
    };
    control_mit_output_t output;

    control_mit_step(&input, &output);

    return expect_close(output.torque_cmd_nm, 3.2f,
                        "MIT 位置、速度和前馈转矩合成错误。");
}

int main(void)
{
    if (!test_voltage_mode() ||
        !test_current_mode() ||
        !test_speed_mode() ||
        !test_position_mode() ||
        !test_invalid_mode() ||
        !test_mit_command() ||
        !test_legacy_equivalence(CONTROL_LOOP_MODE_VOLTAGE) ||
        !test_legacy_equivalence(CONTROL_LOOP_MODE_CURRENT) ||
        !test_legacy_equivalence(CONTROL_LOOP_MODE_SPEED) ||
        !test_legacy_equivalence(CONTROL_LOOP_MODE_POSITION))
    {
        return 1;
    }

    return 0;
}
