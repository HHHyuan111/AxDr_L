/**
 * @file test_control_replay.c
 * @brief 用连续离线样本验证控制周期、Drive 和 PWM 边界的端到端时序。
 */

#include <stdbool.h>
#include <stdio.h>

#include "common.h"
#include "control_cycle.h"

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

void foc_feedback_update(pmsm_t *motor, float bus_voltage_v)
{
    motor->foc.vbus = bus_voltage_v;
    motor->foc.wr_f = 0.0f;
}

void drive_control_reset(pmsm_t *motor)
{
    motor->ctrl.vd_set = 0.0f;
    motor->ctrl.vq_set = 0.0f;
}

void drive_diag_poll_request(pmsm_t *motor)
{
    (void)motor;
}

bool drive_diag_is_supported(const pmsm_t *motor)
{
    (void)motor;
    return false;
}

bool drive_diag_prepare(pmsm_t *motor)
{
    (void)motor;
    return false;
}

bool drive_diag_step(pmsm_t *motor)
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

void force_volt_mode(pmsm_t *motor)
{
    (void)motor;
}

void force_curr_mode(pmsm_t *motor)
{
    (void)motor;
}

void cst_tor_mode(pmsm_t *motor)
{
    (void)motor;
}

void csv_vel_mode(pmsm_t *motor)
{
    (void)motor;
}

void csp_pos_mode(pmsm_t *motor)
{
    (void)motor;
}

void pmsm_quick_stop_mode(pmsm_t *motor)
{
    (void)motor;
}

bool foc_volt(pmsm_t *motor, float voltage_d_v, float voltage_q_v, float angle_rad)
{
    (void)motor;
    (void)voltage_d_v;
    (void)voltage_q_v;
    (void)angle_rad;
    return false;
}

bool foc_curr(pmsm_t *motor, float current_d_a, float current_q_a, float angle_rad)
{
    (void)current_d_a;
    (void)current_q_a;
    (void)angle_rad;

    motor->foc.dtc_a = 0.2f;
    motor->foc.dtc_b = 0.4f;
    motor->foc.dtc_c = 0.6f;
    return true;
}

bool foc_vel(pmsm_t *motor, float speed_rad_s, float current_limit_a, float angle_rad)
{
    (void)motor;
    (void)speed_rad_s;
    (void)current_limit_a;
    (void)angle_rad;
    return false;
}

bool foc_pos(pmsm_t *motor,
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
        .current_a_a = 0.1f,
        .current_b_a = -0.1f,
        .current_c_a = 0.0f,
        .bus_voltage_v = 24.0f,
        .electrical_angle_rad = 0.3f,
        .rotor_position_rad = 1.0f,
        .output_position_rad = 1.0f,
    };
}

int main(void)
{
    pmsm_t motor = {
        .mode = {.sys = debug_mode, .debug = curr_cl},
        .req = DRIVE_REQ_STOP,
        .state = DRIVE_STATE_STOP,
        .para = {.phase_order = ABC_PHASE},
        .prot_cfg = {.invalid_position_samples = 1U}
    };
    control_cycle_input_t input = replay_sample(1U);
    control_cycle_output_t output = {0};

    control_cycle_step(&motor, &input, &output);
    if (!expect_true(output.drive_state == (uint32_t)DRIVE_STATE_STOP,
                     "第 1 拍 STOP 样本必须保持功率输出关闭。") ||
        !expect_true(pwm_start_count == 0U,
                     "STOP 样本不能启动 PWM。"))
    {
        return 1;
    }

    motor.req = DRIVE_REQ_START;
    input = replay_sample(2U);
    control_cycle_step(&motor, &input, &output);
    if (!expect_true(output.drive_state == (uint32_t)DRIVE_STATE_STARTING,
                     "第 2 拍 START 样本必须进入 STARTING。") ||
        !expect_true(output.pwm_enabled,
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
    if (!expect_true(output.drive_state == (uint32_t)DRIVE_STATE_RUN,
                     "第 3 拍必须进入 RUN。") ||
        !expect_true(output.duty_valid,
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

    return expect_true(output.drive_state == (uint32_t)DRIVE_STATE_FAULT,
                       "位置反馈失效必须在同一回放拍进入 FAULT。") &&
           expect_true(!output.pwm_enabled,
                       "位置反馈故障后必须报告 PWM 已关闭。") &&
           expect_true((output.fault_bits != 0U) &&
                       (motor.fault.bit.enc_err == 1U),
                       "位置反馈失效必须锁存编码器故障。") &&
           expect_true(pwm_stop_count == 1U,
                       "故障拍必须且只需执行一次 PWM 停止。")
        ? 0
        : 4;
}
