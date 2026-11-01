/**
 * @file foc_mapping.c
 * @brief FOC 总对象与可移植控制器之间的每拍映射。主链位置：第 5、6 步。
 *
 * 把 foc_t 的宽字段组装成控制器的窄输入，执行控制后把结果写回 foc_t。
 * 只做映射与派生量计算，不读取硬件，不决定运行状态。
 */

#include "common.h"
#include "foc_run.h"

#include "algorithm_config.h"
#include "board_config.h"
#include "control_filter.h"
#include "foc_control.h"
#include "speed_adapter.h"

/**
 * @brief 把现有电机对象映射到可移植控制主链，并将本周期结果写回。
 */
_RAM_FUNC bool foc_execute(
    foc_t *foc,
    const foc_plan_t *cmd,
    const foc_frame_t *frame)
{
    foc_ctrl_t ctrl = {
        .cur_rate = {
            .count = foc->rate.cur_pid_cnt,
            .divider = foc->rate.cur_pid_cnt_val,
        },
        .spd_rate = {
            .count = foc->rate.spd_pid_cnt,
            .divider = foc->rate.spd_pid_cnt_val,
        },
        .pos_rate = {
            .count = foc->rate.pos_pid_cnt,
            .divider = foc->rate.pos_pid_cnt_val,
        },
        .id_pi = &foc->id_pi,
        .iq_pi = &foc->iq_pi,
        .spd_pi = &foc->spd_pi,
        .pos_pi = &foc->pos_pi,
        .vd = foc->out.vd,
        .vq = foc->out.vq,
        .iq_ref = foc->ref.iq_lim,
        .spd_ref = foc->ref.spd_r_lim,
        .deadtime = {
            .enabled = (DRIVE_DEADTIME_COMPENSATION_ENABLE != 0),
            .deadtime_s = foc->board.dead_time * 1.0e-6f,
            .pwm_period_s = foc->rate.foc_ts,
            .current_blend_a = DRIVE_DEADTIME_CURRENT_BLEND_A,
            .polarity = DRIVE_DEADTIME_COMPENSATION_POLARITY,
            .max_duty_delta = DRIVE_DEADTIME_MAX_DUTY_DELTA,
        },
    };
    const foc_fb_t fb = {
        .inv_vbus = foc->fb.inv_vbus,
        .spd = foc->fb.spd_r,
        .pos = foc->fb.pos_r,
    };
    foc_out_t out;
    const bool valid = foc_ctrl_execute(&ctrl, &fb, &cmd->ref, frame, &out);

    foc->out.mode = (foc_mode_e)cmd->ref.mode;

    foc->rate.cur_pid_cnt = ctrl.cur_rate.count;
    foc->rate.spd_pid_cnt = ctrl.spd_rate.count;
    foc->rate.pos_pid_cnt = ctrl.pos_rate.count;
    foc->ref.iq_lim = ctrl.iq_ref;
    foc->ref.spd_r_lim = ctrl.spd_ref;

    foc->out.vd = out.vd;
    foc->out.vq = out.vq;
    foc->out.valpha = out.pwm.valpha;
    foc->out.vbeta = out.pwm.vbeta;
    foc->out.duty_a = out.pwm.duty_a;
    foc->out.duty_b = out.pwm.duty_b;
    foc->out.duty_c = out.pwm.duty_c;

    return valid;
}

/* 电流和角度转换只有这一处；角度由状态/模式在前一步选好。 */
_RAM_FUNC void foc_update_current(foc_t *foc, float theta, foc_frame_t *frame)
{
    const foc_sample_t sample = {
        .ia = foc->fb.ia, .ib = foc->fb.ib, .ic = foc->fb.ic, .theta = theta,
    };
    foc_core_prepare(&sample, frame);
    foc->out.theta = frame->theta;
    foc->out.sin = frame->sin;
    foc->out.cos = frame->cos;
    foc->fb.ialpha = frame->ialpha;
    foc->fb.ibeta = frame->ibeta;
    foc->fb.id = frame->id;
    foc->fb.iq = frame->iq;
}

/**
 * @brief 更新当前控制周期使用的 FOC 运行反馈。
 *
 * @param[in,out] foc 电机控制对象。
 *
 * 使用本拍已更新的 ADC 与角度结果，依次计算母线电压、控制器限幅、转矩和速度。
 */
_RAM_FUNC void ctrl_fb_update(foc_t *foc, float vbus)
{
    /* 第 1 步：保存本周期母线电压，并计算调制所需的电压系数和余量。 */
    foc->fb.vbus = vbus;

    if (foc->fb.vbus > 0.0f)
    {
        foc->fb.inv_vbus = 1.5f / foc->fb.vbus;
    }
    else
    {
        foc->fb.inv_vbus = 0.0f;
    }

    foc->ref.v_lim = foc->fb.vbus * 0.5f * CTRL_VOLTAGE_UTILIZATION_RATIO;

    /* 第 2 步：把本周期允许的电压、电流和速度范围交给各级 PI 控制器。 */
    foc->id_pi.out_max    =  foc->ref.v_lim;
    foc->id_pi.out_min    = -foc->ref.v_lim;
    foc->iq_pi.out_max    =  foc->ref.v_lim;
    foc->iq_pi.out_min    = -foc->ref.v_lim;

    foc->spd_pi.out_max   =  foc->ref.iq_max;
    foc->spd_pi.out_min   =  foc->ref.iq_min;

    foc->pos_pi.out_max   =  foc->ref.spd_max;
    foc->pos_pi.out_min   =  foc->ref.spd_min;

    /* 第 3 步：滤波 q 轴电流，并换算转子侧和减速器输出侧转矩。 */
    foc->fb.iq_f = control_lpf_step(&foc->iq_lpf, foc->fb.iq);
    foc->fb.torq_r  = foc->fb.iq     * foc->motor.Kt;
    foc->fb.torq_r_f = foc->fb.iq_f   * foc->motor.Kt;
    foc->fb.torq_m  = foc->fb.torq_r  * foc->motor.Gr;
    foc->fb.torq_m_f = foc->fb.torq_r_f * foc->motor.Gr;

    /* 第 4 步：转子侧窗口差分测速（关口①实机验证通过后转正，B 库同构实现）。
     * 旧链为电角度单拍差分（foc->elec_speed_diff），已由 speed_adapter 取代；
     * fb.spd_e 无其他消费者，保留字段由新测速推导以维持可观测性。 */
    {
        float spd_r = speed_est_get(&foc->speed); /* rad/s，转子机械角速度 */
        foc->fb.spd_e = spd_r * foc->motor.pn;
        foc->fb.spd_r_raw = spd_r; /* rad/s */
    }

    foc->fb.spd_r = control_lpf_step(&foc->wr_lpf, foc->fb.spd_r_raw);
    foc->fb.spd_m = foc->fb.spd_r * foc->motor.div_Gr; // rad/s

}

