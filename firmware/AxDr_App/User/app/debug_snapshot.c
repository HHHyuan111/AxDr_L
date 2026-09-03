/**
 * @file debug_snapshot.c
 * @brief 快速控制周期的只读调试快照实现。
 */

#include "debug_snapshot.h"

#include "common.h"
#include "diag_runtime.h"
#include "drive_diag.h"
#include "observer_adapter.h"

volatile debug_snapshot_t g_debug_snapshot;

_RAM_FUNC void debug_snapshot_publish(const foc_t *foc)
{
    uint32_t op_mode;

    switch (foc->mode.sys)
    {
        case debug_mode:
            op_mode = (uint32_t)foc->mode.debug;
            break;

        case release_mode:
            op_mode = (uint32_t)foc->mode.release;
            break;

        case calibrat_mode:
            op_mode = (uint32_t)foc->mode.calibrat;
            break;

        case halt_mode:
            op_mode = (uint32_t)foc->mode.halt;
            break;

        default:
            op_mode = UINT32_MAX;
            break;
    }

    g_debug_snapshot.req = (uint32_t)foc->req;
    g_debug_snapshot.state = (uint32_t)foc->state;
    g_debug_snapshot.pwm_on = (uint32_t)foc->pwm_active;
    g_debug_snapshot.fault = foc->fault.all;
    g_debug_snapshot.sys_mode = (uint32_t)foc->mode.sys;
    g_debug_snapshot.op_mode = op_mode;
    g_debug_snapshot.pwm_cmd_seq = foc->pwm_cmd.seq;
    g_debug_snapshot.pwm_commit_seq = foc->pwm_commit.seq;
    g_debug_snapshot.pwm_cmd_valid = (uint32_t)foc->pwm_cmd.valid;
    g_debug_snapshot.pwm_committed =
        (uint32_t)(foc->pwm_commit.valid &&
                   (foc->pwm_commit.seq == foc->fast_seq));
    g_debug_snapshot.diag_req = g_diag.request;
    g_debug_snapshot.diag_job = (uint32_t)g_diag.active_job;
    g_debug_snapshot.diag_state = (uint32_t)g_diag.manager.state;
    g_debug_snapshot.diag_status = (uint32_t)g_diag.last_status;
    g_debug_snapshot.diag_active = (uint32_t)g_diag.active;
    g_debug_snapshot.diag_v_sat = (uint32_t)g_diag.voltage_saturated;
    g_debug_snapshot.obs_status = (uint32_t)g_obs.status;
    g_debug_snapshot.obs_samples = g_obs.flux.accepted_samples;

    g_debug_snapshot.v_bus = foc->sig.vbus;
    g_debug_snapshot.i_a = foc->sig.i_a;
    g_debug_snapshot.i_b = foc->sig.i_b;
    g_debug_snapshot.i_c = foc->sig.i_c;
    g_debug_snapshot.theta_e = foc->sig.p_e;

    g_debug_snapshot.pos_r_ref = foc->ctrl.posr_set;
    g_debug_snapshot.pos_r_fbk = foc->sig.mp_r;
    g_debug_snapshot.vel_r_ref = foc->ctrl.wr_set;
    g_debug_snapshot.vel_r_fbk = foc->sig.wr_f;

    g_debug_snapshot.i_d_ref = foc->ctrl.id_set;
    g_debug_snapshot.i_d_fbk = foc->sig.i_d;
    g_debug_snapshot.i_q_ref = foc->ctrl.iq_set;
    g_debug_snapshot.i_q_lim = foc->ctrl.iq_lim;
    g_debug_snapshot.i_q_fbk = foc->sig.i_q;

    g_debug_snapshot.v_d_cmd = foc->sig.v_d;
    g_debug_snapshot.v_q_cmd = foc->sig.v_q;
    g_debug_snapshot.diag_id_ref = g_diag.command.id_ref_a;
    g_debug_snapshot.diag_iq_ref = g_diag.command.iq_ref_a;
    g_debug_snapshot.diag_vd_ref = g_diag.command.vd_ref_v;
    g_debug_snapshot.diag_vq_ref = g_diag.command.vq_ref_v;
    g_debug_snapshot.diag_freq = g_diag.sweep.active_frequency_hz;
    g_debug_snapshot.flux_wb = g_obs.flux.psi_magnitude_filtered_wb;
    g_debug_snapshot.duty_a = foc->sig.dtc_a;
    g_debug_snapshot.duty_b = foc->sig.dtc_b;
    g_debug_snapshot.duty_c = foc->sig.dtc_c;

    g_debug_snapshot.duty_cmd_a = foc->pwm_cmd.duty_a;
    g_debug_snapshot.duty_cmd_b = foc->pwm_cmd.duty_b;
    g_debug_snapshot.duty_cmd_c = foc->pwm_cmd.duty_c;
    g_debug_snapshot.duty_commit_a = foc->pwm_commit.duty_a;
    g_debug_snapshot.duty_commit_b = foc->pwm_commit.duty_b;
    g_debug_snapshot.duty_commit_c = foc->pwm_commit.duty_c;

    /* 最后更新序号，表示上面各字段已经完成本周期发布。 */
    g_debug_snapshot.seq = foc->fast_seq;
}
