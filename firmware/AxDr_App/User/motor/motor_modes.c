/**
 * @file motor_modes.c
 * @brief 当前正式 Drive 模式使用的电机控制适配函数。
 */

#include "common.h"

#include <math.h>
#include "algorithm_config.h"
#include "control_limit.h"
#include "control_mit.h"
#include "control_traj.h"
#include "drive_pwm.h"

/**
 * @brief 按给定机械速度积分出电角度，执行开环电压控制并提交 PWM。
 */
_RAM_FUNC void open_volt_step(foc_t *foc)
{
    foc->ctrl.we_set = foc->ctrl.wr_set * foc->motor.pn;
    foc->ctrl.pos_acc = foc->ctrl.we_set * foc->rate.foc_ts;
    foc->ctrl.drag_pe += foc->ctrl.pos_acc;
    wrap_0_2pi(foc->ctrl.drag_pe);

    if (foc_volt_step(foc,
                 foc->ctrl.vd_set,
                 foc->ctrl.vq_set,
                 foc->ctrl.drag_pe))
    {
        (void)drive_pwm_commit(foc);
    }
}

/**
 * @brief 按给定机械速度积分出电角度，执行开环电流控制并提交 PWM。
 */
_RAM_FUNC void open_cur_step(foc_t *foc)
{
    foc->ctrl.we_set = foc->ctrl.wr_set * foc->motor.pn;
    foc->ctrl.pos_acc = foc->ctrl.we_set * foc->rate.foc_ts;
    foc->ctrl.drag_pe += foc->ctrl.pos_acc;
    wrap_0_2pi(foc->ctrl.drag_pe);

    if (foc_cur_step(foc,
                 foc->ctrl.id_set,
                 foc->ctrl.iq_set,
                 foc->ctrl.drag_pe))
    {
        (void)drive_pwm_commit(foc);
    }
}

/**
 * @brief 执行 MIT 位置、速度和前馈转矩控制，并进入 FOC 电流环。
 */
_RAM_FUNC void mit_step(foc_t *foc)
{
    const control_mit_input_t input = {
        .position_ref_rad = foc->ctrl.posm_set,
        .position_feedback_rad = foc->sig.mp_m,
        .speed_ref_rad_s = foc->ctrl.wm_set,
        .speed_feedback_rad_s = foc->sig.wm,
        .torque_feedforward_nm = foc->ctrl.mit_tor_set,
        .position_gain_nm_per_rad = foc->ctrl.kp,
        .speed_gain_nm_s_per_rad = foc->ctrl.kd,
    };
    control_mit_output_t output;

    control_mit_step(&input, &output);
    foc->ctrl.mit_tor_out = control_limit(output.torque_cmd_nm,
                                          foc->app.pmax_torm,
                                          foc->app.nmax_torm);
    foc->ctrl.tor_set = foc->ctrl.mit_tor_out * foc->motor.div_Gr;
    foc->ctrl.iq_set = foc->ctrl.tor_set * foc->motor.div_Kt;

    if (foc_cur_step(foc,
                     foc->ctrl.id_set,
                     foc->ctrl.iq_set,
                     foc->sig.p_e))
    {
        (void)drive_pwm_commit(foc);
    }
}

/**
 * @brief 执行轮廓速度模式：先生成平滑速度参考，再运行速度—电流级联。
 */
_RAM_FUNC void pv_step(foc_t *foc)
{
    foc->ctrl.wm_ref = traj_spd_step(&foc->spd_traj,
                                     foc->ctrl.wm_set,
                                     foc->ctrl.wm_acc,
                                     foc->ctrl.wm_dec,
                                     foc->rate.foc_ts);
    foc->ctrl.wr_set = foc->ctrl.wm_ref * foc->motor.Gr;
    foc->ctrl.tor_set = foc->ctrl.torm_set * foc->motor.div_Gr;
    foc->ctrl.iq_set = foc->ctrl.tor_set * foc->motor.div_Kt;
    foc->app.vel_reached = (foc->ctrl.wm_ref == foc->ctrl.wm_set);

    if (foc_spd_step(foc,
                     foc->ctrl.wr_set,
                     foc->ctrl.iq_set,
                     foc->sig.p_e))
    {
        (void)drive_pwm_commit(foc);
    }
}

/**
 * @brief 执行轮廓位置模式：在线生成位置参考，再运行位置—速度—电流级联。
 */
_RAM_FUNC void pp_step(foc_t *foc)
{
    float target;
    float spd_lim;

    if (foc->app.pos_pause)
    {
        stop_ramp_step(foc, foc->app.pause_dec);
        foc->app.last_pos_pause = true;
        return;
    }

    if (foc->app.pos_ctrl_mode == abs_pos_mode)
    {
        foc->app.abs_pos_ref = foc->ctrl.posm_set;
        target = foc->app.abs_pos_ref;
    }
    else if (foc->app.pos_ctrl_mode == rel_pos_mode)
    {
        const bool new_command = (foc->ctrl.posm_set != foc->ctrl.posm_lst)
            || foc->app.pos_set_by_flag
            || foc->app.last_pos_pause;

        if (new_command)
        {
            foc->app.rel_pos_ref = foc->sig.mp_m + foc->ctrl.posm_set;
            foc->ctrl.posm_lst = foc->ctrl.posm_set;
            foc->app.pos_set_by_flag = false;
        }
        target = foc->app.rel_pos_ref;
    }
    else
    {
        return;
    }

    spd_lim = fabsf(foc->ctrl.wm_set);
    if (spd_lim == 0.0f)
    {
        spd_lim = foc->app.pmax_velm;
    }

    foc->ctrl.posm_ref = traj_pos_step(&foc->pos_traj,
                                       target,
                                       spd_lim,
                                       foc->ctrl.wm_acc,
                                       foc->ctrl.wm_dec,
                                       foc->rate.foc_ts);
    foc->ctrl.posr_set = foc->ctrl.posm_ref * foc->motor.Gr;
    foc->ctrl.wr_set = spd_lim * foc->motor.Gr;
    foc->ctrl.tor_set = foc->ctrl.torm_set * foc->motor.div_Gr;
    foc->ctrl.iq_set = foc->ctrl.tor_set * foc->motor.div_Kt;
    foc->app.pos_reached = foc->pos_traj.done;
    foc->app.last_pos_pause = false;

    if (foc_pos_step(foc,
                     foc->ctrl.posr_set,
                     foc->ctrl.wr_set,
                     foc->ctrl.iq_set,
                     foc->sig.p_e))
    {
        (void)drive_pwm_commit(foc);
    }
}

/**
 * @brief 把输出轴转矩指令换算为 q 轴电流，执行 CST 电流环。
 */
_RAM_FUNC void cst_step(foc_t *foc)
{
    foc->ctrl.tor_set = foc->ctrl.torm_set * foc->motor.div_Gr;
    foc->ctrl.iq_set = foc->ctrl.tor_set * foc->motor.div_Kt;

    if (foc_cur_step(foc,
                 foc->ctrl.id_set,
                 foc->ctrl.iq_set,
                 foc->sig.p_e))
    {
        (void)drive_pwm_commit(foc);
    }
}

/**
 * @brief 把输出轴速度指令换算到转子侧，执行 CSV 速度—电流级联。
 */
_RAM_FUNC void csv_step(foc_t *foc)
{
    foc->ctrl.wm_ref = foc->ctrl.wm_set;
    foc->ctrl.wr_set = foc->ctrl.wm_ref * foc->motor.Gr;
    foc->ctrl.tor_set = foc->ctrl.torm_set * foc->motor.div_Gr;
    foc->ctrl.iq_set = foc->ctrl.tor_set * foc->motor.div_Kt;

    if (foc_spd_step(foc,
                foc->ctrl.wr_set,
                foc->ctrl.iq_set,
                foc->sig.p_e))
    {
        (void)drive_pwm_commit(foc);
    }
}

/**
 * @brief 把输出轴位置指令换算到转子侧，执行 CSP 位置—速度—电流级联。
 */
_RAM_FUNC void csp_step(foc_t *foc)
{
    foc->ctrl.posm_ref = foc->ctrl.posm_set;
    foc->ctrl.posr_set = foc->ctrl.posm_ref * foc->motor.Gr;
    foc->ctrl.wr_set = foc->ctrl.wm_set * foc->motor.Gr;
    foc->ctrl.tor_set = foc->ctrl.torm_set * foc->motor.div_Gr;
    foc->ctrl.iq_set = foc->ctrl.tor_set * foc->motor.div_Kt;

    if (foc_pos_step(foc,
                foc->ctrl.posr_set,
                foc->ctrl.wr_set,
                foc->ctrl.iq_set,
                foc->sig.p_e))
    {
        (void)drive_pwm_commit(foc);
    }
}

/**
 * @brief 按给定减速度逐周期降低速度指令，并继续执行闭环速度控制。
 *
 * @param[in] deceleration_rad_s2 输出轴减速度绝对值，单位为 rad/s²。
 */
_RAM_FUNC void stop_ramp_step(foc_t *foc, float deceleration_rad_s2)
{
    if (foc->ctrl.wm_set > 0.0f)
    {
        foc->ctrl.wr_set -= deceleration_rad_s2 * foc->motor.Gr * foc->rate.foc_ts;
        if (foc->ctrl.wr_set < 0.0f)
        {
            foc->ctrl.wr_set = 0.0f;
        }
    }
    if (foc->ctrl.wr_set < 0.0f)
    {
        foc->ctrl.wr_set += deceleration_rad_s2 * foc->motor.Gr * foc->rate.foc_ts;
        if (foc->ctrl.wr_set > 0.0f)
        {
            foc->ctrl.wr_set = 0.0f;
        }
    }

    if (foc_spd_step(foc,
                foc->ctrl.wr_set,
                foc->ctrl.iq_set,
                foc->sig.p_e))
    {
        (void)drive_pwm_commit(foc);
    }
}

/**
 * @brief 执行快速停机减速；转子速度足够低后切回 STOP 请求。
 */
_RAM_FUNC void quick_stop_step(foc_t *foc)
{
    stop_ramp_step(foc, foc->app.quick_stop_dec);
    if (fabsf(foc->sig.wr_f) < CTRL_QUICK_STOP_SPEED_THRESHOLD_RAD_S)
    {
        foc->req = DRIVE_REQ_STOP;
        foc->mode.sys = release_mode;
    }
}
