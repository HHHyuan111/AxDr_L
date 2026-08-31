/**
 * @file drive_reset.c
 * @brief Drive 控制运行状态复位实现。
 */

#include "drive_reset.h"

#include <math.h>

#include "common.h"
#include "compiler.h"
#include "control_pid.h"
#include "control_speed.h"

PLATFORM_FAST_CODE void drive_control_reset(pmsm_t *pm)
{
    const float angle_rad = isfinite(pm->foc.p_e) ? pm->foc.p_e : 0.0f;

    control_pid_clear(&pm->id_pi);
    control_pid_clear(&pm->iq_pi);
    control_pid_clear(&pm->spd_pi);
    control_pid_clear(&pm->pos_pi);
    control_angle_speed_reset(&pm->elec_speed_diff, angle_rad);

    pm->period.cur_pid_cnt = 0U;
    pm->period.spd_pid_cnt = 0U;
    pm->period.pos_pid_cnt = 0U;

    pm->ctrl.drag_pe = 0.0f;
    pm->ctrl.pos_acc = 0.0f;
    pm->ctrl.vd_set = 0.0f;
    pm->ctrl.vq_set = 0.0f;
    pm->ctrl.id_set = 0.0f;
    pm->ctrl.iq_set = 0.0f;
    pm->ctrl.iq_lim = 0.0f;
    pm->ctrl.torm_set = 0.0f;
    pm->ctrl.tor_set = 0.0f;
    pm->ctrl.we_set = 0.0f;
    pm->ctrl.wr_set = 0.0f;
    pm->ctrl.wr_lim = 0.0f;
    pm->ctrl.wm_set = 0.0f;
    pm->ctrl.wm_ref = 0.0f;
    pm->ctrl.wm_lim = 0.0f;
    pm->ctrl.wm_diff = 0.0f;
    pm->ctrl.posm_set = 0.0f;
    pm->ctrl.posm_ref = 0.0f;
    pm->ctrl.posr_set = 0.0f;
    pm->ctrl.wm_lst = 0.0f;
    pm->ctrl.posm_lst = 0.0f;
    pm->ctrl.mit_tor_set = 0.0f;
    pm->ctrl.kp = 0.0f;
    pm->ctrl.kd = 0.0f;

    pm->foc.i_d = 0.0f;
    pm->foc.i_q = 0.0f;
    pm->foc.v_d = 0.0f;
    pm->foc.v_q = 0.0f;
    pm->foc.i_alph = 0.0f;
    pm->foc.i_beta = 0.0f;
    pm->foc.v_alph = 0.0f;
    pm->foc.v_beta = 0.0f;
    pm->foc.dtc_a = 0.5f;
    pm->foc.dtc_b = 0.5f;
    pm->foc.dtc_c = 0.5f;
}
