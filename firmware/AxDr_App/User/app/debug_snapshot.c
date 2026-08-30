/**
 * @file debug_snapshot.c
 * @brief 快速控制周期的只读调试快照实现。
 */

#include "debug_snapshot.h"

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
    g_debug_snapshot.duty_a = pm->foc.dtc_a;
    g_debug_snapshot.duty_b = pm->foc.dtc_b;
    g_debug_snapshot.duty_c = pm->foc.dtc_c;

    /* 最后更新序号，表示上面各字段已经完成本次发布。 */
    g_debug_snapshot.seq++;
}
