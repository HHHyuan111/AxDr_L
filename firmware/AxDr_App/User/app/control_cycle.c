/**
 * @file control_cycle.c
 * @brief FOC 七步主流程。硬件运行与离线输入共用同一条执行路径。
 *
 * 输入可来自真实采样或离线回放；Drive 会执行启停并提交 PWM。
 * 离线测试需要替代硬件输出接口，不能把本函数当作纯数学计算。
 */

#include "control_cycle.h"
#include <stddef.h>

#include "common.h"
#include "board_adapter.h"
#include "position_adapter.h"
#include "compiler.h"
#include "drive.h"
#include "drive_cycle.h"
#include "observer_adapter.h"
#include "speed_adapter.h"

static void foc_read_feedback(foc_t *foc, const control_cycle_input_t *sample);

/* FOC 单拍主流程：按阅读顺序执行，保护和状态先决定能否进入控制。 */
PLATFORM_FAST_CODE void control_cycle_step(foc_t *foc,
                                          const control_cycle_input_t *input,
                                          control_cycle_output_t *output)
{
    drive_cycle_t cycle = {0};
    foc_frame_t frame;

    /* 1. 读取反馈：采样/离线输入、有效性、速度和母线限幅。 */
    foc_read_feedback(foc, input);

    /* 2. 检查保护：电流、电压、编码器有效性和启动方向。 */
    drive_check_protection(foc);

    /* 3. 更新运行状态：空闲、启动、对齐、运行、诊断或故障。 */
    drive_update_state(foc);

    /* 4. 处理特殊状态并生成给定：确定本拍用哪个角度、哪个控制器。 */
    drive_handle_state(foc, &cycle);

    if (cycle.ready && foc->fault.all == 0U)
    {
        /* 5. 更新电流与角度：Clarke/Park，得到本拍 id/iq。 */
        foc_update_current(foc, cycle.cmd.theta, &frame);

        /* 6. 执行控制：电压/电流/速度/位置控制，再计算三相占空比。 */
        cycle.valid = foc_execute(foc, &cycle.cmd, &frame);
    }

    /* 7. 输出 PWM：校验后提交；STOP、故障、无效计算在这里关断。 */
    drive_output(foc, &cycle);

    /* 观测放在控制之后；当前磁链观测不接管控制角度。 */
    obs_step(foc);

    if (output != NULL)
    {
        /* 返回本拍最终状态与占空比，供调用方或离线测试读取。 */
        output->seq     = foc->fast_seq;
        output->state   = (uint32_t)foc->state;
        output->fault   = foc->fault.all;
        output->pwm_on  = foc->pwm_active;
        output->duty_ok = foc->pwm_cmd.valid;
        output->duty_a  = foc->pwm_cmd.duty_a;
        output->duty_b  = foc->pwm_cmd.duty_b;
        output->duty_c  = foc->pwm_cmd.duty_c;
    }
}

PLATFORM_FAST_CODE void foc_run(foc_t *foc)
{
    control_cycle_step(foc, NULL, NULL);
}

/* 生产调用读取硬件；离线调用传入同单位、同有效性定义的 sample。
 * 采样失败时保留上拍数值，但有效性为 false，交给保护判断。 */
static PLATFORM_FAST_CODE void foc_read_feedback(foc_t *foc, const control_cycle_input_t *sample)
{
    control_cycle_input_t measured;
    const control_cycle_input_t *input = sample;
    if (input == NULL)
    {
        position_sample_t pos = {
            .theta_e = foc->fb.theta_e, .pos_r = foc->fb.pos_r, .pos_m = foc->fb.pos_m,
        };
        board_sample_t adc = {
            .ia = foc->fb.ia, .ib = foc->fb.ib, .ic = foc->fb.ic, .vbus = foc->fb.vbus,
        };
        const bool encoder_valid = encoder_sample(&foc->enc);
        const bool pos_valid = encoder_valid && position_update(foc, &pos);
        const bool i_valid = foc_adc_sample(foc, &adc);
        measured = (control_cycle_input_t){
            .seq = foc->fast_seq, .i_valid = i_valid,
            .vbus_valid = true, /* 沿用当前母线采样有效性规则。 */
            .pos_valid = pos_valid,
            .ia = adc.ia, .ib = adc.ib, .ic = adc.ic, .vbus = adc.vbus,
            .theta_e = pos.theta_e, .pos_r = pos.pos_r, .pos_m = pos.pos_m,
        };
        input = &measured;
    }
    /* 1. 装入本周期采样。位置为 rad，电流为 A，母线电压为 V。 */
    foc->fast_seq      = input->seq;
    foc->fb.i_valid    = input->i_valid;
    foc->fb.vbus_valid = input->vbus_valid;
    foc->fb.pos_valid  = input->pos_valid;
    foc->fb.ia         = input->ia;
    foc->fb.ib         = input->ib;
    foc->fb.ic         = input->ic;
    foc->fb.theta_e    = input->theta_e;
    foc->fb.pos_r      = input->pos_r;
    foc->fb.pos_m      = input->pos_m;

    /* 2. 更新测速：无效位置也传入，由测速器重建连续窗口。 */
    speed_est_step(&foc->speed, input->pos_r, foc->rate.foc_fs,
                   input->seq, input->pos_valid);

    ctrl_fb_update(foc, input->vbus);

}
