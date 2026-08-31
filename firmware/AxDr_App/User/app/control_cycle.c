/**
 * @file control_cycle.c
 * @brief 单个快速控制周期输入输出接口的实现。
 */

#include "control_cycle.h"

#include "common.h"
#include "compiler.h"
#include "drive.h"

PLATFORM_FAST_CODE void control_cycle_step(
    pmsm_t *motor,
    const control_cycle_input_t *input,
    control_cycle_output_t *output)
{
    /* 将硬件采样或回放数据写入现有控制器上下文。 */
    motor->fast_seq = input->seq;
    motor->foc.i_a = input->current_a_a;
    motor->foc.i_b = input->current_b_a;
    motor->foc.i_c = input->current_c_a;
    motor->foc.p_e = input->electrical_angle_rad;
    motor->foc.mp_r = input->rotor_position_rad;
    motor->foc.mp_m = input->output_position_rad;

    /* 控制器只消费物理量，不关心这些数据来自真实 Target、文件还是仿真模型。 */
    foc_feedback_update(motor, input->bus_voltage_v);
    drive_fast_step(motor);

    output->seq = motor->fast_seq;
    output->drive_state = (uint32_t)motor->state;
    output->fault_bits = motor->fault.all;
    output->pwm_enabled = motor->pwm_active;
    output->duty_valid = motor->pwm_cmd.valid;
    output->duty_a = motor->pwm_cmd.duty_a;
    output->duty_b = motor->pwm_cmd.duty_b;
    output->duty_c = motor->pwm_cmd.duty_c;
}
