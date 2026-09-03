/**
 * @file test_control_cycle.c
 * @brief 验证单周期显式输入、执行顺序和输出数据合同。
 */

#include <stdbool.h>
#include <stdio.h>

#include "common.h"
#include "control_cycle.h"

static bool feedback_called;
static bool drive_called;
static bool observer_called;
static bool input_visible;
static float received_bus_voltage_v;

void ctrl_fb_update(foc_t *motor, float bus_voltage_v)
{
    feedback_called = true;
    received_bus_voltage_v = bus_voltage_v;
    input_visible = (motor->fast_seq == 42U) &&
                    motor->fb_status.i_valid &&
                    motor->fb_status.vbus_valid &&
                    motor->fb_status.pos_valid &&
                    (motor->sig.i_a == 1.0f) &&
                    (motor->sig.i_b == -2.0f) &&
                    (motor->sig.i_c == 3.0f) &&
                    (motor->sig.p_e == 0.25f) &&
                    (motor->sig.mp_r == 4.0f) &&
                    (motor->sig.mp_m == 5.0f);
    motor->sig.vbus = bus_voltage_v;
}

void drive_fast_step(foc_t *motor)
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
        .current_a_a = 1.0f,
        .current_b_a = -2.0f,
        .current_c_a = 3.0f,
        .bus_voltage_v = 24.0f,
        .electrical_angle_rad = 0.25f,
        .rotor_position_rad = 4.0f,
        .output_position_rad = 5.0f,
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
        !expect_true(output.drive_state == (uint32_t)DRIVE_STATE_RUN,
                     "输出必须带出本周期 Drive 状态。") ||
        !expect_true(output.fault_bits == 0x12U,
                     "输出必须带出本周期故障位。") ||
        !expect_true(output.pwm_enabled && output.duty_valid,
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
