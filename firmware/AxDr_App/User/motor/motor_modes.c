/**
 * @file motor_modes.c
 * @brief 控制模式给定生成：轨迹、单位换算和控制角度。主链位置：第 4 步内。
 */

#include "common.h"
#include "foc_run.h"

#include <math.h>
#include "algorithm_config.h"
#include "control_limit.h"
#include "control_mit.h"
#include "control_traj.h"


/**
 * @brief 按给定机械速度积分出电角度，生成开环电压给定。
 */
_RAM_FUNC bool open_volt_step(foc_t *foc, foc_plan_t *cmd)
{
    foc->ref.spd_e = foc->ref.spd_r * foc->motor.pn;
    foc->ref.theta_step = foc->ref.spd_e * foc->rate.foc_ts;
    foc->ref.theta_e += foc->ref.theta_step;
    wrap_0_2pi(foc->ref.theta_e);

    *cmd = (foc_plan_t){
        .ref = {
            .mode = FOC_CTRL_MODE_VOLT,
            .vd = foc->ref.vd,
            .vq = foc->ref.vq,
        },
        .theta = foc->ref.theta_e,
    };
    return true;
}

/**
 * @brief 按给定机械速度积分出电角度，生成开环电流给定。
 */
_RAM_FUNC bool open_cur_step(foc_t *foc, foc_plan_t *cmd)
{
    foc->ref.spd_e = foc->ref.spd_r * foc->motor.pn;
    foc->ref.theta_step = foc->ref.spd_e * foc->rate.foc_ts;
    foc->ref.theta_e += foc->ref.theta_step;
    wrap_0_2pi(foc->ref.theta_e);

    *cmd = (foc_plan_t){
        .ref = {
            .mode = FOC_CTRL_MODE_CUR,
            .id_ref = foc->ref.id,
            .iq_ref = foc->ref.iq,
        },
        .theta = foc->ref.theta_e,
    };
    return true;
}

/**
 * @brief 把 MIT 位置、速度和前馈转矩换算为电流环给定。
 */
_RAM_FUNC bool mit_step(foc_t *foc, foc_plan_t *cmd)
{
    const control_mit_input_t input = {
        .position_ref_rad = foc->ref.pos_m,
        .position_feedback_rad = foc->fb.pos_m,
        .speed_ref_rad_s = foc->ref.spd_m,
        .speed_feedback_rad_s = foc->fb.spd_m,
        .torque_feedforward_nm = foc->ref.torq_ff,
        .position_gain_nm_per_rad = foc->ref.kp,
        .speed_gain_nm_s_per_rad = foc->ref.kd,
    };
    control_mit_output_t output;

    control_mit_step(&input, &output);
    foc->ref.torq_mit = control_limit(output.torque_cmd_nm,
                                          foc->app.pmax_torm,
                                          foc->app.nmax_torm);
    foc->ref.torq_r = foc->ref.torq_mit * foc->motor.div_Gr;
    foc->ref.iq = foc->ref.torq_r * foc->motor.div_Kt;

    *cmd = (foc_plan_t){
        .ref = {
            .mode = FOC_CTRL_MODE_CUR,
            .id_ref = foc->ref.id,
            .iq_ref = foc->ref.iq,
        },
        .theta = foc->fb.theta_e,
    };
    return true;
}

/**
 * @brief 执行轮廓速度模式：生成平滑速度参考，选择速度—电流级联。
 */
_RAM_FUNC bool pv_step(foc_t *foc, foc_plan_t *cmd)
{
    foc->ref.spd_m_ramp = traj_spd_step(&foc->spd_traj,
                                     foc->ref.spd_m,
                                     foc->ref.acc_m,
                                     foc->ref.dec_m,
                                     foc->rate.foc_ts);
    foc->ref.spd_r = foc->ref.spd_m_ramp * foc->motor.Gr;
    foc->ref.torq_r = foc->ref.torq_m * foc->motor.div_Gr;
    foc->ref.iq = foc->ref.torq_r * foc->motor.div_Kt;
    foc->app.vel_reached = (foc->ref.spd_m_ramp == foc->ref.spd_m);

    *cmd = (foc_plan_t){
        .ref = {
            .mode = FOC_CTRL_MODE_SPD,
            .spd_ref = foc->ref.spd_r,
            .cur_lim = foc->ref.iq,
            .id_ref = foc->ref.id,
        },
        .theta = foc->fb.theta_e,
    };
    return true;
}

/**
 * @brief 执行轮廓位置模式：在线生成位置参考，选择位置—速度—电流级联。
 */
_RAM_FUNC bool pp_step(foc_t *foc, foc_plan_t *cmd)
{
    float target;
    float spd_lim;

    if (foc->app.pos_pause)
    {
        const bool ready = stop_ramp_step(foc, foc->app.pause_dec, cmd);
        foc->app.last_pos_pause = true;
        return ready;
    }

    if (foc->app.pos_ctrl_mode == abs_pos_mode)
    {
        foc->app.abs_pos_ref = foc->ref.pos_m;
        target = foc->app.abs_pos_ref;
    }
    else if (foc->app.pos_ctrl_mode == rel_pos_mode)
    {
        const bool new_command = (foc->ref.pos_m != foc->ref.pos_m_last)
            || foc->app.pos_set_by_flag
            || foc->app.last_pos_pause;

        if (new_command)
        {
            foc->app.rel_pos_ref = foc->fb.pos_m + foc->ref.pos_m;
            foc->ref.pos_m_last = foc->ref.pos_m;
            foc->app.pos_set_by_flag = false;
        }
        target = foc->app.rel_pos_ref;
    }
    else
    {
        return false;
    }

    spd_lim = fabsf(foc->ref.spd_m);
    if (spd_lim == 0.0f)
    {
        spd_lim = foc->app.pmax_velm;
    }

    foc->ref.pos_m_ramp = traj_pos_step(&foc->pos_traj,
                                       target,
                                       spd_lim,
                                       foc->ref.acc_m,
                                       foc->ref.dec_m,
                                       foc->rate.foc_ts);
    foc->ref.pos_r = foc->ref.pos_m_ramp * foc->motor.Gr;
    foc->ref.spd_r = spd_lim * foc->motor.Gr;
    foc->ref.torq_r = foc->ref.torq_m * foc->motor.div_Gr;
    foc->ref.iq = foc->ref.torq_r * foc->motor.div_Kt;
    foc->app.pos_reached = foc->pos_traj.done;
    foc->app.last_pos_pause = false;

    *cmd = (foc_plan_t){
        .ref = {
            .mode = FOC_CTRL_MODE_POS,
            .pos_ref = foc->ref.pos_r,
            .spd_lim = foc->ref.spd_r,
            .cur_lim = foc->ref.iq,
            .id_ref = foc->ref.id,
        },
        .theta = foc->fb.theta_e,
    };
    return true;
}

/**
 * @brief 把输出轴转矩指令换算为 q 轴电流，生成 CST 电流环给定。
 */
_RAM_FUNC bool cst_step(foc_t *foc, foc_plan_t *cmd)
{
    foc->ref.torq_r = foc->ref.torq_m * foc->motor.div_Gr;
    foc->ref.iq = foc->ref.torq_r * foc->motor.div_Kt;

    *cmd = (foc_plan_t){
        .ref = {
            .mode = FOC_CTRL_MODE_CUR,
            .id_ref = foc->ref.id,
            .iq_ref = foc->ref.iq,
        },
        .theta = foc->fb.theta_e,
    };
    return true;
}

/**
 * @brief 把输出轴速度指令换算到转子侧，生成 CSV 速度—电流级联给定。
 */
_RAM_FUNC bool csv_step(foc_t *foc, foc_plan_t *cmd)
{
    foc->ref.spd_m_ramp = foc->ref.spd_m;
    foc->ref.spd_r = foc->ref.spd_m_ramp * foc->motor.Gr;
    foc->ref.torq_r = foc->ref.torq_m * foc->motor.div_Gr;
    foc->ref.iq = foc->ref.torq_r * foc->motor.div_Kt;

    *cmd = (foc_plan_t){
        .ref = {
            .mode = FOC_CTRL_MODE_SPD,
            .spd_ref = foc->ref.spd_r,
            .cur_lim = foc->ref.iq,
            .id_ref = foc->ref.id,
        },
        .theta = foc->fb.theta_e,
    };
    return true;
}

/**
 * @brief 把输出轴位置指令换算到转子侧，生成 CSP 位置—速度—电流级联给定。
 */
_RAM_FUNC bool csp_step(foc_t *foc, foc_plan_t *cmd)
{
    foc->ref.pos_m_ramp = foc->ref.pos_m;
    foc->ref.pos_r = foc->ref.pos_m_ramp * foc->motor.Gr;
    foc->ref.spd_r = foc->ref.spd_m * foc->motor.Gr;
    foc->ref.torq_r = foc->ref.torq_m * foc->motor.div_Gr;
    foc->ref.iq = foc->ref.torq_r * foc->motor.div_Kt;

    *cmd = (foc_plan_t){
        .ref = {
            .mode = FOC_CTRL_MODE_POS,
            .pos_ref = foc->ref.pos_r,
            .spd_lim = foc->ref.spd_r,
            .cur_lim = foc->ref.iq,
            .id_ref = foc->ref.id,
        },
        .theta = foc->fb.theta_e,
    };
    return true;
}

/**
 * @brief 按给定减速度逐周期降低速度指令，生成减速期间的速度环给定。
 *
 * @param[in] deceleration_rad_s2 输出轴减速度绝对值，单位为 rad/s²。
 */
_RAM_FUNC bool stop_ramp_step(foc_t *foc, float deceleration_rad_s2, foc_plan_t *cmd)
{
    if (foc->ref.spd_m > 0.0f)
    {
        foc->ref.spd_r -= deceleration_rad_s2 * foc->motor.Gr * foc->rate.foc_ts;
        if (foc->ref.spd_r < 0.0f)
        {
            foc->ref.spd_r = 0.0f;
        }
    }
    if (foc->ref.spd_r < 0.0f)
    {
        foc->ref.spd_r += deceleration_rad_s2 * foc->motor.Gr * foc->rate.foc_ts;
        if (foc->ref.spd_r > 0.0f)
        {
            foc->ref.spd_r = 0.0f;
        }
    }

    *cmd = (foc_plan_t){
        .ref = {
            .mode = FOC_CTRL_MODE_SPD,
            .spd_ref = foc->ref.spd_r,
            .cur_lim = foc->ref.iq,
            .id_ref = foc->ref.id,
        },
        .theta = foc->fb.theta_e,
    };
    return true;
}

/**
 * @brief 执行快速停机减速；转子速度足够低后切回 STOP 请求。
 */
_RAM_FUNC bool quick_stop_step(foc_t *foc, foc_plan_t *cmd)
{
    const bool ready = stop_ramp_step(foc, foc->app.quick_stop_dec, cmd);
    if (fabsf(foc->fb.spd_r) < CTRL_QUICK_STOP_SPEED_THRESHOLD_RAD_S)
    {
        foc->req = DRIVE_REQ_STOP;
        foc->mode.sys = release_mode;
    }
    return ready;
}
