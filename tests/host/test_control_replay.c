/**
 * @file test_control_replay.c
 * @brief 用连续离线样本验证控制周期、Drive 和 PWM 边界的端到端时序。
 */

#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#include "common.h"
#include "control_cycle.h"
#include "speed_adapter.h"

static unsigned int pwm_start_count;
static unsigned int pwm_stop_count;
static unsigned int pwm_write_count;
static float target_duty_a;
static float target_duty_b;
static float target_duty_c;

bool target_pwm_start_phase_outputs(void)
{
    pwm_start_count++;
    return true;
}

bool target_pwm_stop_phase_outputs(void)
{
    pwm_stop_count++;
    return true;
}

void target_pwm_set_duty_ratios(float duty_a, float duty_b, float duty_c)
{
    pwm_write_count++;
    target_duty_a = duty_a;
    target_duty_b = duty_b;
    target_duty_c = duty_c;
}

void ctrl_fb_update(foc_t *motor, float vbus)
{
    motor->fb.vbus = vbus;
    motor->fb.spd_r = 0.0f;
}

void drive_control_reset(foc_t *motor)
{
    motor->ref.vd = 0.0f;
    motor->ref.vq = 0.0f;
}

void drive_control_reset_for_start(foc_t *motor)
{
    motor->ref.vd = 0.0f;
    motor->ref.vq = 0.0f;
}

void drive_diag_poll_request(foc_t *motor)
{
    (void)motor;
}

bool drive_diag_is_supported(const foc_t *motor)
{
    (void)motor;
    return false;
}

bool drive_diag_prepare(foc_t *motor)
{
    (void)motor;
    return false;
}

bool drive_diag_step(foc_t *motor)
{
    (void)motor;
    return false;
}

void drive_diag_on_stopped(void)
{
}

void drive_diag_on_fault(void)
{
}

void obs_step(const foc_t *motor)
{
    (void)motor;
}

void open_volt_step(foc_t *motor)
{
    (void)motor;
}

void open_cur_step(foc_t *motor)
{
    (void)motor;
}

void mit_step(foc_t *motor)
{
    (void)motor;
}

void pv_step(foc_t *motor)
{
    (void)motor;
}

void pp_step(foc_t *motor)
{
    (void)motor;
}

void cst_step(foc_t *motor)
{
    (void)motor;
}

void csv_step(foc_t *motor)
{
    (void)motor;
}

void csp_step(foc_t *motor)
{
    (void)motor;
}

void quick_stop_step(foc_t *motor)
{
    (void)motor;
}

bool foc_volt_step(foc_t *motor, float voltage_d_v, float voltage_q_v, float angle_rad)
{
    (void)motor;
    (void)voltage_d_v;
    (void)voltage_q_v;
    (void)angle_rad;
    return false;
}

bool foc_cur_step(foc_t *motor, float current_d_a, float current_q_a, float angle_rad)
{
    (void)current_d_a;
    (void)current_q_a;
    (void)angle_rad;

    motor->out.duty_a = 0.2f;
    motor->out.duty_b = 0.4f;
    motor->out.duty_c = 0.6f;
    return true;
}

bool foc_spd_step(foc_t *motor, float speed_rad_s, float current_limit_a, float angle_rad)
{
    (void)motor;
    (void)speed_rad_s;
    (void)current_limit_a;
    (void)angle_rad;
    return false;
}

bool foc_pos_step(foc_t *motor,
             float position_rad,
             float speed_limit_rad_s,
             float current_limit_a,
             float angle_rad)
{
    (void)motor;
    (void)position_rad;
    (void)speed_limit_rad_s;
    (void)current_limit_a;
    (void)angle_rad;
    return false;
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

static control_cycle_input_t replay_sample(uint32_t seq)
{
    return (control_cycle_input_t){
        .seq = seq,
        .i_valid = true,
        .vbus_valid = true,
        .pos_valid = true,
        .ia = 0.1f,
        .ib = -0.1f,
        .ic = 0.0f,
        .vbus = 24.0f,
        .theta_e = 0.3f,
        .pos_r = 1.0f,
        .pos_m = 1.0f,
    };
}

/*
 * 速度回放缝场景（架构审查修复的验收）：仅凭 control_cycle_input_t 驱动，
 * RUN 段 pos_r 每拍 +0.001 rad → 窗口 2、fs=20000 → spd_r_raw 应为 20 rad/s；
 * 随后一拍 pos_valid=false → 速度冻结（丢样本策略）。
 */
static int replay_speed_scenario(void)
{
    foc_t motor = {
        .mode = {.sys = debug_mode, .debug = curr_cl},
        .req = DRIVE_REQ_STOP,
        .state = DRIVE_STATE_STOP,
        .motor = {.phase_order = PHASE_ORDER_ABC},
        .prot_cfg = {.invalid_position_samples = 1U},
        .rate = {.foc_fs = 20000.0f}
    };
    control_cycle_input_t input;
    control_cycle_output_t output = {0};
    float pos = 1.0f;
    int tick;

    pwm_start_count = 0U;
    pwm_stop_count = 0U;
    pwm_write_count = 0U;

    /* STOP → START → RUN 握手 */
    input = replay_sample(100U);
    control_cycle_step(&motor, &input, &output);
    motor.req = DRIVE_REQ_START;
    input = replay_sample(101U);
    control_cycle_step(&motor, &input, &output);
    input = replay_sample(102U);
    control_cycle_step(&motor, &input, &output);
    if (output.state != (uint32_t)DRIVE_STATE_RUN)
    {
        (void)fprintf(stderr, "速度场景启动握手失败。\n");
        return 1;
    }

    /* RUN 段：每拍 pos_r +0.001 */
    for (tick = 0; tick < 10; tick++)
    {
        input = replay_sample((uint32_t)(103U + tick));
        pos += 0.001f;
        input.pos_r = pos;
        control_cycle_step(&motor, &input, &output);
    }
    if (!(fabsf(speed_est_get() - 20.0f) <= 5.0e-3f))
    {
        (void)fprintf(stderr,
                      "回放速度断言失败：spd_r_raw=%.6f，期望 20。\n",
                      (double)speed_est_get());
        return 2;
    }

    /* 丢样本拍：速度必须冻结（保持上一拍值） */
    {
        const float frozen = speed_est_get();
        input = replay_sample(113U);
        input.pos_valid = false;
        control_cycle_step(&motor, &input, &output);
        if (speed_est_get() != frozen)
        {
            (void)fprintf(stderr,
                          "丢样本拍速度未冻结：%.6f -> %.6f\n",
                          (double)frozen,
                          (double)motor.fb.spd_r_raw);
            return 3;
        }
    }
    return 0;
}

int main(void)
{
    foc_t motor = {
        .mode = {.sys = debug_mode, .debug = curr_cl},
        .req = DRIVE_REQ_STOP,
        .state = DRIVE_STATE_STOP,
        .motor = {.phase_order = PHASE_ORDER_ABC},
        .prot_cfg = {.invalid_position_samples = 1U}
    };
    control_cycle_input_t input = replay_sample(1U);
    control_cycle_output_t output = {0};

    control_cycle_step(&motor, &input, &output);
    if (!expect_true(output.state == (uint32_t)DRIVE_STATE_STOP,
                     "第 1 拍 STOP 样本必须保持功率输出关闭。") ||
        !expect_true(pwm_start_count == 0U,
                     "STOP 样本不能启动 PWM。"))
    {
        return 1;
    }

    motor.req = DRIVE_REQ_START;
    input = replay_sample(2U);
    control_cycle_step(&motor, &input, &output);
    if (!expect_true(output.state == (uint32_t)DRIVE_STATE_STARTING,
                     "第 2 拍 START 样本必须进入 STARTING。") ||
        !expect_true(output.pwm_on,
                     "START 成功后必须报告 PWM 已启用。") ||
        !expect_true((target_duty_a == 0.5f) &&
                     (target_duty_b == 0.5f) &&
                     (target_duty_c == 0.5f),
                     "启动前必须先写三相 50% 中性占空比。") ||
        !expect_true(pwm_start_count == 1U,
                     "START 只能启动一次 PWM。"))
    {
        return 2;
    }

    input = replay_sample(3U);
    control_cycle_step(&motor, &input, &output);
    if (!expect_true(output.state == (uint32_t)DRIVE_STATE_RUN,
                     "第 3 拍必须进入 RUN。") ||
        !expect_true(output.duty_ok,
                     "RUN 拍必须产生新的有效占空比。") ||
        !expect_true((target_duty_a == 0.2f) &&
                     (target_duty_b == 0.4f) &&
                     (target_duty_c == 0.6f),
                     "RUN 拍必须把本周期逻辑占空比提交到 Target。") ||
        !expect_true(pwm_write_count == 2U,
                     "回放到 RUN 时应包含中性值和控制值两次写入。"))
    {
        return 3;
    }

    input = replay_sample(4U);
    input.pos_valid = false;
    control_cycle_step(&motor, &input, &output);

    if (!expect_true(output.state == (uint32_t)DRIVE_STATE_FAULT,
                     "位置反馈失效必须在同一回放拍进入 FAULT。") ||
        !expect_true(!output.pwm_on,
                     "位置反馈故障后必须报告 PWM 已关闭。") ||
        !expect_true((output.fault != 0U) &&
                     (motor.fault.bit.enc_err == 1U),
                     "位置反馈失效必须锁存编码器故障。") ||
        !expect_true(pwm_stop_count == 1U,
                     "故障拍必须且只需执行一次 PWM 停止。"))
    {
        return 4;
    }

    {
        const int speed_result = replay_speed_scenario();
        if (speed_result != 0)
        {
            return 10 + speed_result;
        }
    }
    return 0;
}
