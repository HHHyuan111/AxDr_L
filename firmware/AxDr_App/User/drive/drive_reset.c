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
#include "control_traj.h"

PLATFORM_FAST_CODE void drive_control_reset(foc_t *foc)
{
    const float angle_rad = isfinite(foc->sig.p_e) ? foc->sig.p_e : 0.0f;

    control_pid_clear(&foc->id_pi);
    control_pid_clear(&foc->iq_pi);
    control_pid_clear(&foc->spd_pi);
    control_pid_clear(&foc->pos_pi);
    control_angle_speed_reset(&foc->elec_speed_diff, angle_rad);

    foc->rate.cur_pid_cnt = 0U;
    foc->rate.spd_pid_cnt = 0U;
    foc->rate.pos_pid_cnt = 0U;

    foc->ctrl.drag_pe = 0.0f;
    foc->ctrl.pos_acc = 0.0f;
    foc->ctrl.vd_set = 0.0f;
    foc->ctrl.vq_set = 0.0f;
    foc->ctrl.id_set = 0.0f;
    foc->ctrl.iq_set = 0.0f;
    foc->ctrl.iq_lim = 0.0f;
    foc->ctrl.torm_set = 0.0f;
    foc->ctrl.tor_set = 0.0f;
    foc->ctrl.we_set = 0.0f;
    foc->ctrl.wr_set = 0.0f;
    foc->ctrl.wr_lim = 0.0f;
    foc->ctrl.wm_set = 0.0f;
    foc->ctrl.wm_ref = 0.0f;
    foc->ctrl.posm_set = 0.0f;
    foc->ctrl.posm_ref = 0.0f;
    foc->ctrl.posr_set = 0.0f;
    foc->ctrl.posm_lst = 0.0f;
    foc->ctrl.mit_tor_set = 0.0f;
    foc->ctrl.mit_tor_out = 0.0f;
    foc->ctrl.kp = 0.0f;
    foc->ctrl.kd = 0.0f;

    traj_spd_reset(&foc->spd_traj, foc->sig.wm);
    traj_pos_reset(&foc->pos_traj, foc->sig.mp_m, foc->sig.wm);
    foc->app.rel_pos_ref = foc->sig.mp_m;
    foc->app.pos_reached = false;
    foc->app.vel_reached = false;
    foc->app.last_pos_pause = false;

    foc->sig.i_d = 0.0f;
    foc->sig.i_q = 0.0f;
    foc->sig.v_d = 0.0f;
    foc->sig.v_q = 0.0f;
    foc->sig.i_alph = 0.0f;
    foc->sig.i_beta = 0.0f;
    foc->sig.v_alph = 0.0f;
    foc->sig.v_beta = 0.0f;
    foc->sig.dtc_a = 0.5f;
    foc->sig.dtc_b = 0.5f;
    foc->sig.dtc_c = 0.5f;
}
