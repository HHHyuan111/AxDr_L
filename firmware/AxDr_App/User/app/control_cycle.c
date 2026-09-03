/**
 * @file control_cycle.c
 * @brief 单个快速控制周期输入输出接口的实现。
 */

#include "control_cycle.h"

#include "common.h"
#include "compiler.h"
#include "drive.h"

PLATFORM_FAST_CODE void control_cycle_step(
    foc_t *foc,
    const control_cycle_input_t *input,
    control_cycle_output_t *output)
{
    /* 将硬件采样或回放数据写入现有控制器上下文。 */
    foc->fast_seq = input->seq;
    foc->fb_status.i_valid = input->i_valid;
    foc->fb_status.vbus_valid = input->vbus_valid;
    foc->fb_status.pos_valid = input->pos_valid;
    foc->sig.i_a = input->current_a_a;
    foc->sig.i_b = input->current_b_a;
    foc->sig.i_c = input->current_c_a;
    foc->sig.p_e = input->electrical_angle_rad;
    foc->sig.mp_r = input->rotor_position_rad;
    foc->sig.mp_m = input->output_position_rad;

    /* 控制器只消费物理量，不关心这些数据来自真实 Target、文件还是仿真模型。 */
    ctrl_fb_update(foc, input->bus_voltage_v);
    drive_fast_step(foc);

    output->seq = foc->fast_seq;
    output->drive_state = (uint32_t)foc->state;
    output->fault_bits = foc->fault.all;
    output->pwm_enabled = foc->pwm_active;
    output->duty_valid = foc->pwm_cmd.valid;
    output->duty_a = foc->pwm_cmd.duty_a;
    output->duty_b = foc->pwm_cmd.duty_b;
    output->duty_c = foc->pwm_cmd.duty_c;
}
