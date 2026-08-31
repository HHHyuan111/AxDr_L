/**
 * @file motor_modes.c
 * @brief 当前正式 Drive 模式使用的电机控制适配函数。
 */

#include "common.h"
#include "drive_pwm.h"

/**
 * @brief 按给定机械速度积分出电角度，执行开环电压控制并提交 PWM。
 */
_RAM_FUNC void force_volt_mode(pmsm_t *pm)
{
    pm->ctrl.we_set = pm->ctrl.wr_set * pm->para.pn;
    pm->ctrl.pos_acc = pm->ctrl.we_set * pm->period.foc_ts;
    pm->ctrl.drag_pe += pm->ctrl.pos_acc;
    wrap_0_2pi(pm->ctrl.drag_pe);

    if (foc_volt(pm,
                 pm->ctrl.vd_set,
                 pm->ctrl.vq_set,
                 pm->ctrl.drag_pe))
    {
        (void)drive_pwm_commit(pm);
    }
}

/**
 * @brief 按给定机械速度积分出电角度，执行开环电流控制并提交 PWM。
 */
_RAM_FUNC void force_curr_mode(pmsm_t *pm)
{
    pm->ctrl.we_set = pm->ctrl.wr_set * pm->para.pn;
    pm->ctrl.pos_acc = pm->ctrl.we_set * pm->period.foc_ts;
    pm->ctrl.drag_pe += pm->ctrl.pos_acc;
    wrap_0_2pi(pm->ctrl.drag_pe);

    if (foc_curr(pm,
                 pm->ctrl.id_set,
                 pm->ctrl.iq_set,
                 pm->ctrl.drag_pe))
    {
        (void)drive_pwm_commit(pm);
    }
}

/**
 * @brief 把输出轴转矩指令换算为 q 轴电流，执行 CST 电流环。
 */
_RAM_FUNC void cst_tor_mode(pmsm_t *pm)
{
    pm->ctrl.tor_set = pm->ctrl.torm_set * pm->para.div_Gr;
    pm->ctrl.iq_set = pm->ctrl.tor_set * pm->para.div_Kt;

    if (foc_curr(pm,
                 pm->ctrl.id_set,
                 pm->ctrl.iq_set,
                 pm->foc.p_e))
    {
        (void)drive_pwm_commit(pm);
    }
}

/**
 * @brief 把输出轴速度指令换算到转子侧，执行 CSV 速度—电流级联。
 */
_RAM_FUNC void csv_vel_mode(pmsm_t *pm)
{
    pm->ctrl.wm_ref = pm->ctrl.wm_set;
    pm->ctrl.wr_set = pm->ctrl.wm_ref * pm->para.Gr;
    pm->ctrl.tor_set = pm->ctrl.torm_set * pm->para.div_Gr;
    pm->ctrl.iq_set = pm->ctrl.tor_set * pm->para.div_Kt;

    if (foc_vel(pm,
                pm->ctrl.wr_set,
                pm->ctrl.iq_set,
                pm->foc.p_e))
    {
        (void)drive_pwm_commit(pm);
    }
}

/**
 * @brief 把输出轴位置指令换算到转子侧，执行 CSP 位置—速度—电流级联。
 */
_RAM_FUNC void csp_pos_mode(pmsm_t *pm)
{
    pm->ctrl.posm_ref = pm->ctrl.posm_set;
    pm->ctrl.posr_set = pm->ctrl.posm_ref * pm->para.Gr;
    pm->ctrl.wr_set = pm->ctrl.wm_set * pm->para.Gr;
    pm->ctrl.tor_set = pm->ctrl.torm_set * pm->para.div_Gr;
    pm->ctrl.iq_set = pm->ctrl.tor_set * pm->para.div_Kt;

    if (foc_pos(pm,
                pm->ctrl.posr_set,
                pm->ctrl.wr_set,
                pm->ctrl.iq_set,
                pm->foc.p_e))
    {
        (void)drive_pwm_commit(pm);
    }
}

/**
 * @brief 按给定减速度逐周期降低速度指令，并继续执行闭环速度控制。
 *
 * @param[in] deceleration_rad_s2 输出轴减速度绝对值，单位为 rad/s²。
 */
_RAM_FUNC void pmsm_slow_down(pmsm_t *pm, float deceleration_rad_s2)
{
    if (pm->ctrl.wm_set > 0.0f)
    {
        pm->ctrl.wr_set -= deceleration_rad_s2 * pm->para.Gr * pm->period.foc_ts;
        if (pm->ctrl.wr_set < 0.0f)
        {
            pm->ctrl.wr_set = 0.0f;
        }
    }
    if (pm->ctrl.wr_set < 0.0f)
    {
        pm->ctrl.wr_set += deceleration_rad_s2 * pm->para.Gr * pm->period.foc_ts;
        if (pm->ctrl.wr_set > 0.0f)
        {
            pm->ctrl.wr_set = 0.0f;
        }
    }

    if (foc_vel(pm,
                pm->ctrl.wr_set,
                pm->ctrl.iq_set,
                pm->foc.p_e))
    {
        (void)drive_pwm_commit(pm);
    }
}

/**
 * @brief 执行快速停机减速；转子速度足够低后切回 STOP 请求。
 */
_RAM_FUNC void pmsm_quick_stop_mode(pmsm_t *pm)
{
    pmsm_slow_down(pm, pm->app_ctrl.quick_stop_dec);
    if (fabsf(pm->foc.wr_f) < 0.5f)
    {
        pm->req = DRIVE_REQ_STOP;
        pm->mode.sys = release_mode;
    }
}
