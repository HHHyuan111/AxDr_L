/**
 * @file test_control_cycle.c
 * @brief 验证单周期显式输入、执行顺序和输出数据合同。
 */

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

#include <stdlib.h>
#include "board_adapter.h"
#include "position_adapter.h"
#include "common.h"
#include "drive.h"
#include "drive_cycle.h"
#include "control_cycle.h"

static unsigned stage;
static bool feedback_called;
static bool drive_called;
static bool observer_called;
static bool input_visible;
static float received_bus_voltage_v;

void ctrl_fb_update(foc_t *motor, float vbus)
{
    assert(stage == 0);
    stage = 1;
    feedback_called = true;
    received_bus_voltage_v = vbus;
    input_visible = (motor->fast_seq == 42U) &&
                    motor->fb.i_valid &&
                    motor->fb.vbus_valid &&
                    motor->fb.pos_valid &&
                    (motor->fb.ia == 1.0f) &&
                    (motor->fb.ib == -2.0f) &&
                    (motor->fb.ic == 3.0f) &&
                    (motor->fb.theta_e == 0.25f) &&
                    (motor->fb.pos_r == 4.0f) &&
                    (motor->fb.pos_m == 5.0f);
    motor->fb.vbus = vbus;
}

void fake_drive_result(foc_t *motor)
{
    drive_called = feedback_called;
    motor->state = DRIVE_STATE_RUN;
    motor->fault.all = 0x12U;
    motor->pwm_active = true;
    motor->pwm_cmd = (drive_pwm_cmd_t){
        .seq = motor->fast_seq,
        .valid = true,
        .duty_a = 0.2f,
        .duty_b = 0.4f,
        .duty_c = 0.6f,
    };
}

void obs_step(const foc_t *motor)
{
    assert(stage == 7);
    stage = 8;
    observer_called = drive_called && (motor->state == DRIVE_STATE_RUN);
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

int main(void)
{
    foc_t motor = {0};
    const control_cycle_input_t input = {
        .seq = 42U,
        .i_valid = true,
        .vbus_valid = true,
        .pos_valid = true,
        .ia = 1.0f,
        .ib = -2.0f,
        .ic = 3.0f,
        .vbus = 24.0f,
        .theta_e = 0.25f,
        .pos_r = 4.0f,
        .pos_m = 5.0f,
    };
    control_cycle_output_t output = {0};

    control_cycle_step(&motor, &input, &output);

    if (!expect_true(input_visible,
                     "反馈更新前必须能看到本周期全部显式输入。") ||
        !expect_true(received_bus_voltage_v == 24.0f,
                     "母线电压必须通过显式输入传入反馈更新。") ||
        !expect_true(drive_called,
                     "Drive 必须在反馈更新之后运行。") ||
        !expect_true(observer_called,
                     "观测器必须在 Drive 本周期计算完成后运行。") ||
        !expect_true(output.seq == 42U,
                     "输出周期序号必须与输入一致。") ||
        !expect_true(output.state == (uint32_t)DRIVE_STATE_RUN,
                     "输出必须带出本周期 Drive 状态。") ||
        !expect_true(output.fault == 0x12U,
                     "输出必须带出本周期故障位。") ||
        !expect_true(output.pwm_on && output.duty_ok,
                     "输出必须区分 PWM 状态和占空比有效性。") ||
        !expect_true((output.duty_a == 0.2f) &&
                     (output.duty_b == 0.4f) &&
                     (output.duty_c == 0.6f),
                     "输出必须带出逻辑三相占空比。"))
    {
        return 1;
    }

    return 0;
}

void drive_check_protection(foc_t *foc) { (void)foc; assert(stage == 1); stage = 2; }
void drive_update_state(foc_t *foc)
{ (void)foc; assert(stage == 2); stage = 3; }
void drive_handle_state(foc_t *foc, drive_cycle_t *cycle)
{ (void)foc; assert(stage == 3); stage = 4; cycle->ready = true; }
void drive_output(foc_t *foc, drive_cycle_t *cycle)
{ assert(stage == 6 && cycle->valid); stage = 7; fake_drive_result(foc); }

void foc_update_current(foc_t *foc, float theta, foc_frame_t *frame)
{ (void)foc; assert(stage == 4); stage = 5; *frame = (foc_frame_t){.theta = theta}; }

bool foc_execute(foc_t *foc, const foc_plan_t *cmd, const foc_frame_t *frame)
{ (void)foc; (void)cmd; (void)frame; assert(stage == 5); stage = 6; return true; }

/* 显式输入测试禁止访问硬件采样分支。 */
bool encoder_sample(encoder_state_t *enc) { (void)enc; abort(); }
bool position_update(foc_t *foc, position_sample_t *s) { (void)foc; (void)s; abort(); }
bool foc_adc_sample(foc_t *foc, board_sample_t *s) { (void)foc; (void)s; abort(); }
