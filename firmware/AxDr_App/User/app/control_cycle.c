/**
 * @file control_cycle.c
 * @brief 单个快速控制周期输入输出接口的实现。
 */

#include "control_cycle.h"

#include "common.h"
#include "compiler.h"
#include "drive.h"
#include "observer_adapter.h"

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
    foc->sig.ia = input->ia;
    foc->sig.ib = input->ib;
    foc->sig.ic = input->ic;
    foc->sig.theta_e = input->theta_e;
    foc->sig.pos_r = input->pos_r;
    foc->sig.pos_m = input->pos_m;

    /* 控制器只消费物理量，不关心这些数据来自真实 Target、文件还是仿真模型。 */
    ctrl_fb_update(foc, input->vbus);
    drive_fast_step(foc);
    obs_step(foc);

    output->seq = foc->fast_seq;
    output->state = (uint32_t)foc->state;
    output->fault = foc->fault.all;
    output->pwm_on = foc->pwm_active;
    output->duty_ok = foc->pwm_cmd.valid;
    output->duty_a = foc->pwm_cmd.duty_a;
    output->duty_b = foc->pwm_cmd.duty_b;
    output->duty_c = foc->pwm_cmd.duty_c;
}
