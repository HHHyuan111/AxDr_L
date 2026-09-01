/**
 * @file debug_snapshot.c
 * @brief 快速控制周期的只读调试快照实现。
 */

#include "debug_snapshot.h"

#include "common.h"

volatile debug_snapshot_t g_debug_snapshot;

_RAM_FUNC void debug_snapshot_publish(const pmsm_t *pm)
{
    uint32_t op_mode;

    switch (pm->mode.sys)
    {
        case debug_mode:
            op_mode = (uint32_t)pm->mode.debug;
            break;

        case release_mode:
            op_mode = (uint32_t)pm->mode.release;
            break;

        case calibrat_mode:
            op_mode = (uint32_t)pm->mode.calibrat;
            break;

        case halt_mode:
            op_mode = (uint32_t)pm->mode.halt;
            break;

        default:
            op_mode = UINT32_MAX;
            break;
    }

    g_debug_snapshot.req = (uint32_t)pm->req;
    g_debug_snapshot.state = (uint32_t)pm->state;
    g_debug_snapshot.pwm_on = (uint32_t)pm->pwm_active;
    g_debug_snapshot.fault = pm->fault.all;
    g_debug_snapshot.sys_mode = (uint32_t)pm->mode.sys;
    g_debug_snapshot.op_mode = op_mode;
    g_debug_snapshot.pwm_cmd_seq = pm->pwm_cmd.seq;
    g_debug_snapshot.pwm_commit_seq = pm->pwm_commit.seq;
    g_debug_snapshot.pwm_cmd_valid = (uint32_t)pm->pwm_cmd.valid;
    g_debug_snapshot.pwm_committed =
        (uint32_t)(pm->pwm_commit.valid &&
                   (pm->pwm_commit.seq == pm->fast_seq));
    g_debug_snapshot.diag_req = pm->diag.request;
    g_debug_snapshot.diag_job = (uint32_t)pm->diag.active_job;
    g_debug_snapshot.diag_state = (uint32_t)pm->diag.manager.state;
    g_debug_snapshot.diag_status = (uint32_t)pm->diag.last_status;
    g_debug_snapshot.diag_active = (uint32_t)pm->diag.active;
    g_debug_snapshot.diag_v_sat = (uint32_t)pm->diag.voltage_saturated;

    g_debug_snapshot.v_bus = pm->foc.vbus;
    g_debug_snapshot.i_a = pm->foc.i_a;
    g_debug_snapshot.i_b = pm->foc.i_b;
    g_debug_snapshot.i_c = pm->foc.i_c;
    g_debug_snapshot.theta_e = pm->foc.p_e;

    g_debug_snapshot.pos_r_ref = pm->ctrl.posr_set;
    g_debug_snapshot.pos_r_fbk = pm->foc.mp_r;
    g_debug_snapshot.vel_r_ref = pm->ctrl.wr_set;
    g_debug_snapshot.vel_r_fbk = pm->foc.wr_f;

    g_debug_snapshot.i_d_ref = pm->ctrl.id_set;
    g_debug_snapshot.i_d_fbk = pm->foc.i_d;
    g_debug_snapshot.i_q_ref = pm->ctrl.iq_set;
    g_debug_snapshot.i_q_lim = pm->ctrl.iq_lim;
    g_debug_snapshot.i_q_fbk = pm->foc.i_q;

    g_debug_snapshot.v_d_cmd = pm->foc.v_d;
    g_debug_snapshot.v_q_cmd = pm->foc.v_q;
    g_debug_snapshot.diag_id_ref = pm->diag.command.id_ref_a;
    g_debug_snapshot.diag_iq_ref = pm->diag.command.iq_ref_a;
    g_debug_snapshot.diag_vd_ref = pm->diag.command.vd_ref_v;
    g_debug_snapshot.diag_vq_ref = pm->diag.command.vq_ref_v;
    g_debug_snapshot.diag_freq = pm->diag.sweep.active_frequency_hz;
    g_debug_snapshot.duty_a = pm->foc.dtc_a;
    g_debug_snapshot.duty_b = pm->foc.dtc_b;
    g_debug_snapshot.duty_c = pm->foc.dtc_c;

    g_debug_snapshot.duty_cmd_a = pm->pwm_cmd.duty_a;
    g_debug_snapshot.duty_cmd_b = pm->pwm_cmd.duty_b;
    g_debug_snapshot.duty_cmd_c = pm->pwm_cmd.duty_c;
    g_debug_snapshot.duty_commit_a = pm->pwm_commit.duty_a;
    g_debug_snapshot.duty_commit_b = pm->pwm_commit.duty_b;
    g_debug_snapshot.duty_commit_c = pm->pwm_commit.duty_c;

    /* 最后更新序号，表示上面各字段已经完成本周期发布。 */
    g_debug_snapshot.seq = pm->fast_seq;
}
