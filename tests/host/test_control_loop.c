/**
 * @file test_control_loop.c
 * @brief 验证可移植 FOC 模式主链和 MIT 给定计算。
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "control_mit.h"
#include "foc_control.h"
#include "legacy_control.h"

#define TEST_TOLERANCE (1.0e-6f)

typedef struct
{
    pid_para_t id_pi;
    pid_para_t iq_pi;
    pid_para_t spd_pi;
    pid_para_t pos_pi;
    foc_ctrl_t ctrl;
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
    test_pid_init(&loop->id_pi, 0.2f);
    test_pid_init(&loop->iq_pi, 0.2f);
    test_pid_init(&loop->spd_pi, 0.1f);
    test_pid_init(&loop->pos_pi, 0.1f);

    loop->ctrl = (foc_ctrl_t){
        .cur_rate = {.count = 0U, .divider = 1U},
        .spd_rate = {.count = 0U, .divider = 1U},
        .pos_rate = {.count = 0U, .divider = 1U},
        .id_pi = &loop->id_pi,
        .iq_pi = &loop->iq_pi,
        .spd_pi = &loop->spd_pi,
        .pos_pi = &loop->pos_pi,
    };
}

static foc_fb_t test_feedback(void)
{
    return (foc_fb_t){
        .sample = {
            .i_a = 0.2f,
            .i_b = -0.1f,
            .i_c = -0.1f,
            .theta = 0.25f,
        },
        .inv_v_bus = 0.05f,
        .spd = 0.4f,
        .pos = 0.3f,
    };
}

static bool test_voltage_mode(void)
{
    test_loop_t loop;
    const foc_fb_t fb = test_feedback();
    const foc_ref_t ref = {
        .mode = FOC_CTRL_MODE_VOLT,
        .v_d = 0.5f,
        .v_q = -0.25f,
    };
    foc_out_t out;

    test_loop_init(&loop);

    return expect_true(foc_ctrl_step(&loop.ctrl, &fb, &ref, &out),
                       "电压模式应生成有效占空比。") &&
           expect_close(out.v_d, 0.5f,
                        "电压模式 d 轴给定错误。") &&
           expect_close(out.v_q, -0.25f,
                        "电压模式 q 轴给定错误。") &&
           expect_true(loop.ctrl.cur_rate.count == 0U,
                       "电压模式不应运行电流环。");
}

static bool test_current_mode(void)
{
    test_loop_t loop;
    const foc_fb_t fb = test_feedback();
    const foc_ref_t ref = {
        .mode = FOC_CTRL_MODE_CUR,
        .i_d_ref = 0.1f,
        .i_q_ref = 0.2f,
    };
    foc_out_t out;

    test_loop_init(&loop);

    return expect_true(foc_ctrl_step(&loop.ctrl, &fb, &ref, &out),
                       "电流模式应生成有效占空比。") &&
           expect_close(loop.id_pi.ref_value, 0.1f,
                        "电流模式 d 轴参考没有进入 PI。") &&
           expect_close(loop.iq_pi.ref_value, 0.2f,
                        "电流模式 q 轴参考没有进入 PI。") &&
           expect_true(loop.ctrl.cur_rate.count == 1U,
                       "电流模式应更新电流环分频状态。") &&
           expect_true(loop.ctrl.spd_rate.count == 0U,
                       "电流模式不应运行速度环。");
}

static bool test_speed_mode(void)
{
    test_loop_t loop;
    const foc_fb_t fb = test_feedback();
    const foc_ref_t ref = {
        .mode = FOC_CTRL_MODE_SPD,
        .i_d_ref = 0.0f,
        .spd_ref = 1.0f,
        .cur_lim = 2.0f,
    };
    foc_out_t out;

    test_loop_init(&loop);

    return expect_true(foc_ctrl_step(&loop.ctrl, &fb, &ref, &out),
                       "速度模式应生成有效占空比。") &&
           expect_close(loop.spd_pi.ref_value, 1.0f,
                        "速度参考没有进入速度环。") &&
           expect_close(loop.iq_pi.ref_value,
                        loop.ctrl.i_q_ref,
                        "速度环输出没有进入 q 轴电流环。") &&
           expect_true((loop.ctrl.spd_rate.count == 0U) &&
                       (loop.ctrl.cur_rate.count == 1U),
                       "速度模式应依次运行速度环和电流环。");
}

static bool test_position_mode(void)
{
    test_loop_t loop;
    const foc_fb_t fb = test_feedback();
    const foc_ref_t ref = {
        .mode = FOC_CTRL_MODE_POS,
        .i_d_ref = 0.0f,
        .pos_ref = 0.8f,
        .cur_lim = 2.0f,
        .spd_lim = 3.0f,
    };
    foc_out_t out;

    test_loop_init(&loop);

    return expect_true(foc_ctrl_step(&loop.ctrl, &fb, &ref, &out),
                       "位置模式应生成有效占空比。") &&
           expect_close(loop.pos_pi.ref_value, 0.8f,
                        "位置参考没有进入位置环。") &&
           expect_close(loop.spd_pi.ref_value,
                        loop.ctrl.spd_ref,
                        "位置环输出没有进入速度环。") &&
           expect_close(loop.iq_pi.ref_value,
                        loop.ctrl.i_q_ref,
                        "速度环输出没有进入 q 轴电流环。") &&
           expect_true((loop.ctrl.pos_rate.count == 0U) &&
                       (loop.ctrl.spd_rate.count == 0U) &&
                       (loop.ctrl.cur_rate.count == 1U),
                       "位置模式应依次运行位置、速度和电流环。");
}

static bool test_invalid_mode(void)
{
    test_loop_t loop;
    const foc_fb_t fb = test_feedback();
    const foc_ref_t ref = {
        .mode = (foc_ctrl_mode_t)99,
    };
    foc_out_t out;

    test_loop_init(&loop);

    return expect_true(!foc_ctrl_step(&loop.ctrl, &fb, &ref, &out),
                       "非法控制模式不应生成占空比。") &&
           expect_true(!out.valid,
                       "非法控制模式输出必须标记为无效。");
}

static bool test_legacy_equivalence(foc_ctrl_mode_t mode)
{
    test_loop_t migrated;
    test_loop_t legacy;
    const foc_fb_t fb = test_feedback();
    const foc_ref_t ref = {
        .mode = mode,
        .v_d = 0.5f,
        .v_q = -0.25f,
        .i_d_ref = 0.1f,
        .i_q_ref = 0.2f,
        .spd_ref = 1.0f,
        .pos_ref = 0.8f,
        .cur_lim = 2.0f,
        .spd_lim = 3.0f,
    };
    foc_out_t migrated_out;
    foc_out_t legacy_out;
    bool migrated_valid;
    bool legacy_valid;

    test_loop_init(&migrated);
    test_loop_init(&legacy);

    migrated_valid = foc_ctrl_step(&migrated.ctrl,
                                   &fb,
                                   &ref,
                                   &migrated_out);
    legacy_valid = legacy_foc_ctrl_step(&legacy.ctrl,
                                        &fb,
                                        &ref,
                                        &legacy_out);

    return expect_true(migrated_valid == legacy_valid,
                       "新旧主链占空比有效性不一致。") &&
           expect_true(memcmp(&migrated_out.frame,
                              &legacy_out.frame,
                              sizeof(migrated_out.frame)) == 0,
                       "新旧主链 FOC 坐标结果未保持逐位一致。") &&
           expect_true(memcmp(&migrated_out.pwm,
                              &legacy_out.pwm,
                              sizeof(migrated_out.pwm)) == 0,
                       "新旧主链占空比结果未保持逐位一致。") &&
           expect_true(memcmp(&migrated.id_pi,
                              &legacy.id_pi,
                              sizeof(migrated.id_pi)) == 0,
                       "新旧主链 d 轴 PI 状态不一致。") &&
           expect_true(memcmp(&migrated.iq_pi,
                              &legacy.iq_pi,
                              sizeof(migrated.iq_pi)) == 0,
                       "新旧主链 q 轴 PI 状态不一致。") &&
           expect_true(memcmp(&migrated.spd_pi,
                              &legacy.spd_pi,
                              sizeof(migrated.spd_pi)) == 0,
                       "新旧主链速度控制器状态不一致。") &&
           expect_true(memcmp(&migrated.pos_pi,
                              &legacy.pos_pi,
                              sizeof(migrated.pos_pi)) == 0,
                       "新旧主链位置控制器状态不一致。") &&
           expect_true((migrated.ctrl.cur_rate.count ==
                        legacy.ctrl.cur_rate.count) &&
                       (migrated.ctrl.spd_rate.count ==
                        legacy.ctrl.spd_rate.count) &&
                       (migrated.ctrl.pos_rate.count ==
                        legacy.ctrl.pos_rate.count),
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
        !test_legacy_equivalence(FOC_CTRL_MODE_VOLT) ||
        !test_legacy_equivalence(FOC_CTRL_MODE_CUR) ||
        !test_legacy_equivalence(FOC_CTRL_MODE_SPD) ||
        !test_legacy_equivalence(FOC_CTRL_MODE_POS))
    {
        return 1;
    }

    return 0;
}
