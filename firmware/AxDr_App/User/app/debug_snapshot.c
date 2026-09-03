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

    g_debug_snapshot.vbus = foc->sig.vbus;
    g_debug_snapshot.ia = foc->sig.ia;
    g_debug_snapshot.ib = foc->sig.ib;
    g_debug_snapshot.ic = foc->sig.ic;
    g_debug_snapshot.theta_e = foc->sig.theta_e;

    g_debug_snapshot.pos_r_ref = foc->ctrl.posr_set;
    g_debug_snapshot.pos_r_fbk = foc->sig.pos_r;
    g_debug_snapshot.spd_r_ref = foc->ctrl.wr_set;
    g_debug_snapshot.spd_r_fbk = foc->sig.spd_r;

    g_debug_snapshot.id_ref = foc->ctrl.id_set;
    g_debug_snapshot.id_fbk = foc->sig.id;
    g_debug_snapshot.iq_ref = foc->ctrl.iq_set;
    g_debug_snapshot.iq_lim = foc->ctrl.iq_lim;
    g_debug_snapshot.iq_fbk = foc->sig.iq;

    g_debug_snapshot.vd = foc->sig.vd;
    g_debug_snapshot.vq = foc->sig.vq;
    g_debug_snapshot.diag_id_ref = g_diag.command.id_ref_a;
    g_debug_snapshot.diag_iq_ref = g_diag.command.iq_ref_a;
    g_debug_snapshot.diag_vd_ref = g_diag.command.vd_ref_v;
    g_debug_snapshot.diag_vq_ref = g_diag.command.vq_ref_v;
    g_debug_snapshot.diag_freq = g_diag.sweep.active_frequency_hz;
    g_debug_snapshot.flux_wb = g_obs.flux.psi_magnitude_filtered_wb;
    g_debug_snapshot.duty_a = foc->sig.duty_a;
    g_debug_snapshot.duty_b = foc->sig.duty_b;
    g_debug_snapshot.duty_c = foc->sig.duty_c;

    g_debug_snapshot.duty_cmd_a = foc->pwm_cmd.duty_a;
    g_debug_snapshot.duty_cmd_b = foc->pwm_cmd.duty_b;
    g_debug_snapshot.duty_cmd_c = foc->pwm_cmd.duty_c;
    g_debug_snapshot.duty_commit_a = foc->pwm_commit.duty_a;
    g_debug_snapshot.duty_commit_b = foc->pwm_commit.duty_b;
    g_debug_snapshot.duty_commit_c = foc->pwm_commit.duty_c;

    /* 最后更新序号，表示上面各字段已经完成本周期发布。 */
    g_debug_snapshot.seq = foc->fast_seq;
}
