/**
 * @file drive_reset.c
 * @brief Drive 控制运行状态复位实现。
 */

#include "drive_reset.h"

#include <math.h>

#include "common.h"
#include "compiler.h"
#include "control_pid.h"
#include "control_traj.h"

PLATFORM_FAST_CODE void drive_control_reset(foc_t *foc)
{
    control_pid_clear(&foc->id_pi);
    control_pid_clear(&foc->iq_pi);
    control_pid_clear(&foc->spd_pi);
    control_pid_clear(&foc->pos_pi);
    foc->rate.cur_pid_cnt = 0U;
    foc->rate.spd_pid_cnt = 0U;
    foc->rate.pos_pid_cnt = 0U;

    foc->ref.theta_e = 0.0f;
    foc->ref.theta_step = 0.0f;
    foc->ref.vd = 0.0f;
    foc->ref.vq = 0.0f;
    foc->ref.id = 0.0f;
    foc->ref.iq = 0.0f;
    foc->ref.iq_lim = 0.0f;
    foc->ref.torq_m = 0.0f;
    foc->ref.torq_r = 0.0f;
    foc->ref.spd_e = 0.0f;
    foc->ref.spd_r = 0.0f;
    foc->ref.spd_r_lim = 0.0f;
    foc->ref.spd_m = 0.0f;
    foc->ref.spd_m_ramp = 0.0f;
    foc->ref.pos_m = 0.0f;
    foc->ref.pos_m_ramp = 0.0f;
    foc->ref.pos_r = 0.0f;
    foc->ref.pos_m_last = 0.0f;
    foc->ref.torq_ff = 0.0f;
    foc->ref.torq_mit = 0.0f;
    foc->ref.kp = 0.0f;
    foc->ref.kd = 0.0f;

    traj_spd_reset(&foc->spd_traj, foc->fb.spd_m);
    traj_pos_reset(&foc->pos_traj, foc->fb.pos_m, foc->fb.spd_m);
    foc->app.rel_pos_ref = foc->fb.pos_m;
    foc->app.pos_reached = false;
    foc->app.vel_reached = false;
    foc->app.last_pos_pause = false;

    foc->fb.id = 0.0f;
    foc->fb.iq = 0.0f;
    foc->out.vd = 0.0f;
    foc->out.vq = 0.0f;
    foc->fb.ialpha = 0.0f;
    foc->fb.ibeta = 0.0f;
    foc->out.valpha = 0.0f;
    foc->out.vbeta = 0.0f;
    foc->out.duty_a = 0.5f;
    foc->out.duty_b = 0.5f;
    foc->out.duty_c = 0.5f;
}
