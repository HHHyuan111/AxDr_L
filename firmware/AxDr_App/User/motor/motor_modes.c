/**
 * @file motor_modes.c
 * @brief 当前正式 Drive 模式使用的电机控制适配函数。
 */

#include "common.h"
#include "algorithm_config.h"
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
