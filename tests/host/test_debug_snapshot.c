/**
 * @file test_debug_snapshot.c
 * @brief 在电脑端验证生产调试快照的字段映射和单向数据关系。
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "common.h"
#include "debug_snapshot.h"

static int expect_u32(const char *name, uint32_t actual, uint32_t expected)
{
    if (actual == expected)
    {
        return 1;
    }

    fprintf(stderr, "%s 不符合预期：实际 %lu，期望 %lu\n",
            name,
            (unsigned long)actual,
            (unsigned long)expected);
    return 0;
}

static int expect_float_bits(const char *name, float actual, float expected)
{
    if (memcmp(&actual, &expected, sizeof(actual)) == 0)
    {
        return 1;
    }

    fprintf(stderr,
            "%s 位模式不符合预期：实际 %.9g，期望 %.9g\n",
            name,
            (double)actual,
            (double)expected);
    return 0;
}

static void fill_source(pmsm_t *motor)
{
    memset(motor, 0, sizeof(*motor));

    motor->fast_seq = 41U;
    motor->req = DRIVE_REQ_START;
    motor->state = DRIVE_STATE_RUN;
    motor->pwm_active = true;
    motor->fault.all = UINT32_C(0x00000125);
    motor->mode.sys = debug_mode;
    motor->mode.debug = pos_spd_curr_cl;
    motor->diag.request = DIAG_REQUEST_START;
    motor->diag.active_job = DIAG_JOB_CURRENT_SWEEP;
    motor->diag.manager.state = MC_DIAG_RUNNING;
    motor->diag.last_status = MC_BUSY;
    motor->diag.active = true;
    motor->diag.voltage_saturated = true;
    motor->diag.command.id_ref_a = 0.4f;
    motor->diag.command.iq_ref_a = 0.5f;
    motor->diag.command.vd_ref_v = 0.6f;
    motor->diag.command.vq_ref_v = 0.7f;
    motor->diag.sweep.active_frequency_hz = 100.0f;

    motor->pwm_cmd.seq = motor->fast_seq;
    motor->pwm_cmd.valid = true;
    motor->pwm_cmd.duty_a = 0.12f;
    motor->pwm_cmd.duty_b = 0.23f;
    motor->pwm_cmd.duty_c = 0.34f;

    motor->pwm_commit.seq = motor->fast_seq;
    motor->pwm_commit.valid = true;
    motor->pwm_commit.duty_a = 0.10f;
    motor->pwm_commit.duty_b = 0.20f;
    motor->pwm_commit.duty_c = 0.30f;

    motor->foc.vbus = 24.25f;
    motor->foc.i_a = 1.1f;
    motor->foc.i_b = -2.2f;
    motor->foc.i_c = 3.3f;
    motor->foc.p_e = 0.45f;

    motor->ctrl.posr_set = 5.1f;
    motor->foc.mp_r = 5.2f;
    motor->ctrl.wr_set = 6.1f;
    motor->foc.wr_f = 6.2f;

    motor->ctrl.id_set = 7.1f;
    motor->foc.i_d = 7.2f;
    motor->ctrl.iq_set = 8.1f;
    motor->ctrl.iq_lim = 8.2f;
    motor->foc.i_q = 8.3f;

    motor->foc.v_d = 9.1f;
    motor->foc.v_q = 9.2f;
    motor->foc.dtc_a = 0.11f;
    motor->foc.dtc_b = 0.22f;
    motor->foc.dtc_c = 0.33f;
}

static int expect_snapshot_fields(const pmsm_t *motor,
                                  uint32_t expected_seq,
                                  uint32_t expected_op_mode)
{
    return expect_u32("seq", g_debug_snapshot.seq, expected_seq) &&
           expect_u32("req", g_debug_snapshot.req, (uint32_t)motor->req) &&
           expect_u32("state", g_debug_snapshot.state, (uint32_t)motor->state) &&
           expect_u32("pwm_on",
                      g_debug_snapshot.pwm_on,
                      (uint32_t)motor->pwm_active) &&
           expect_u32("fault", g_debug_snapshot.fault, motor->fault.all) &&
           expect_u32("sys_mode", g_debug_snapshot.sys_mode, (uint32_t)motor->mode.sys) &&
           expect_u32("op_mode", g_debug_snapshot.op_mode, expected_op_mode) &&
           expect_u32("pwm_cmd_seq", g_debug_snapshot.pwm_cmd_seq, motor->pwm_cmd.seq) &&
           expect_u32("pwm_commit_seq",
                      g_debug_snapshot.pwm_commit_seq,
                      motor->pwm_commit.seq) &&
           expect_u32("pwm_cmd_valid",
                      g_debug_snapshot.pwm_cmd_valid,
                      (uint32_t)motor->pwm_cmd.valid) &&
           expect_u32("pwm_committed",
                      g_debug_snapshot.pwm_committed,
                      (uint32_t)(motor->pwm_commit.valid &&
                                 (motor->pwm_commit.seq == motor->fast_seq))) &&
           expect_u32("diag_req", g_debug_snapshot.diag_req, motor->diag.request) &&
           expect_u32("diag_job",
                      g_debug_snapshot.diag_job,
                      (uint32_t)motor->diag.active_job) &&
           expect_u32("diag_state",
                      g_debug_snapshot.diag_state,
                      (uint32_t)motor->diag.manager.state) &&
           expect_u32("diag_status",
                      g_debug_snapshot.diag_status,
                      (uint32_t)motor->diag.last_status) &&
           expect_u32("diag_active",
                      g_debug_snapshot.diag_active,
                      (uint32_t)motor->diag.active) &&
           expect_u32("diag_v_sat",
                      g_debug_snapshot.diag_v_sat,
                      (uint32_t)motor->diag.voltage_saturated) &&
           expect_float_bits("v_bus", g_debug_snapshot.v_bus, motor->foc.vbus) &&
           expect_float_bits("i_a", g_debug_snapshot.i_a, motor->foc.i_a) &&
           expect_float_bits("i_b", g_debug_snapshot.i_b, motor->foc.i_b) &&
           expect_float_bits("i_c", g_debug_snapshot.i_c, motor->foc.i_c) &&
           expect_float_bits("theta_e", g_debug_snapshot.theta_e, motor->foc.p_e) &&
           expect_float_bits("pos_r_ref", g_debug_snapshot.pos_r_ref, motor->ctrl.posr_set) &&
           expect_float_bits("pos_r_fbk", g_debug_snapshot.pos_r_fbk, motor->foc.mp_r) &&
           expect_float_bits("vel_r_ref", g_debug_snapshot.vel_r_ref, motor->ctrl.wr_set) &&
           expect_float_bits("vel_r_fbk", g_debug_snapshot.vel_r_fbk, motor->foc.wr_f) &&
           expect_float_bits("i_d_ref", g_debug_snapshot.i_d_ref, motor->ctrl.id_set) &&
           expect_float_bits("i_d_fbk", g_debug_snapshot.i_d_fbk, motor->foc.i_d) &&
           expect_float_bits("i_q_ref", g_debug_snapshot.i_q_ref, motor->ctrl.iq_set) &&
           expect_float_bits("i_q_lim", g_debug_snapshot.i_q_lim, motor->ctrl.iq_lim) &&
           expect_float_bits("i_q_fbk", g_debug_snapshot.i_q_fbk, motor->foc.i_q) &&
           expect_float_bits("v_d_cmd", g_debug_snapshot.v_d_cmd, motor->foc.v_d) &&
           expect_float_bits("v_q_cmd", g_debug_snapshot.v_q_cmd, motor->foc.v_q) &&
           expect_float_bits("diag_id_ref",
                             g_debug_snapshot.diag_id_ref,
                             motor->diag.command.id_ref_a) &&
           expect_float_bits("diag_iq_ref",
                             g_debug_snapshot.diag_iq_ref,
                             motor->diag.command.iq_ref_a) &&
           expect_float_bits("diag_vd_ref",
                             g_debug_snapshot.diag_vd_ref,
                             motor->diag.command.vd_ref_v) &&
           expect_float_bits("diag_vq_ref",
                             g_debug_snapshot.diag_vq_ref,
                             motor->diag.command.vq_ref_v) &&
           expect_float_bits("diag_freq",
                             g_debug_snapshot.diag_freq,
                             motor->diag.sweep.active_frequency_hz) &&
           expect_float_bits("duty_a", g_debug_snapshot.duty_a, motor->foc.dtc_a) &&
           expect_float_bits("duty_b", g_debug_snapshot.duty_b, motor->foc.dtc_b) &&
           expect_float_bits("duty_c", g_debug_snapshot.duty_c, motor->foc.dtc_c) &&
           expect_float_bits("duty_cmd_a", g_debug_snapshot.duty_cmd_a, motor->pwm_cmd.duty_a) &&
           expect_float_bits("duty_cmd_b", g_debug_snapshot.duty_cmd_b, motor->pwm_cmd.duty_b) &&
           expect_float_bits("duty_cmd_c", g_debug_snapshot.duty_cmd_c, motor->pwm_cmd.duty_c) &&
           expect_float_bits("duty_commit_a",
                             g_debug_snapshot.duty_commit_a,
                             motor->pwm_commit.duty_a) &&
           expect_float_bits("duty_commit_b",
                             g_debug_snapshot.duty_commit_b,
                             motor->pwm_commit.duty_b) &&
           expect_float_bits("duty_commit_c",
                             g_debug_snapshot.duty_commit_c,
                             motor->pwm_commit.duty_c);
}

static int test_mode_mapping(pmsm_t *motor, uint32_t *expected_seq)
{
    motor->mode.sys = release_mode;
    motor->mode.release = csp_mode;
    motor->fast_seq++;
    motor->pwm_cmd.seq = motor->fast_seq;
    debug_snapshot_publish(motor);
    *expected_seq = motor->fast_seq;
    if (!expect_snapshot_fields(motor, *expected_seq, (uint32_t)csp_mode))
    {
        return 0;
    }

    motor->mode.sys = calibrat_mode;
    motor->mode.calibrat = anticogging_pm;
    motor->fast_seq++;
    motor->pwm_cmd.seq = motor->fast_seq;
    debug_snapshot_publish(motor);
    *expected_seq = motor->fast_seq;
    if (!expect_snapshot_fields(motor, *expected_seq, (uint32_t)anticogging_pm))
    {
        return 0;
    }

    motor->mode.sys = halt_mode;
    motor->mode.halt = fault_mode;
    motor->fast_seq++;
    motor->pwm_cmd.seq = motor->fast_seq;
    debug_snapshot_publish(motor);
    *expected_seq = motor->fast_seq;
    return expect_snapshot_fields(motor, *expected_seq, (uint32_t)fault_mode);
}

static int test_snapshot_cannot_change_source(pmsm_t *motor)
{
    const pmsm_t before = *motor;

    g_debug_snapshot.req = UINT32_MAX;
    g_debug_snapshot.state = UINT32_MAX;
    g_debug_snapshot.i_a = -99.0f;
    g_debug_snapshot.duty_a = 0.99f;

    if (memcmp(motor, &before, sizeof(*motor)) == 0)
    {
        return 1;
    }

    fprintf(stderr, "修改调试快照反向改变了电机控制对象。\n");
    return 0;
}

int main(void)
{
    pmsm_t motor;
    uint32_t expected_seq;

    _Static_assert(sizeof(debug_snapshot_t) == 188U, "调试快照布局发生了变化");

    fill_source(&motor);
    debug_snapshot_publish(&motor);
    expected_seq = motor.fast_seq;

    if (!expect_snapshot_fields(&motor,
                                expected_seq,
                                (uint32_t)pos_spd_curr_cl))
    {
        return 1;
    }

    if (!test_mode_mapping(&motor, &expected_seq))
    {
        return 2;
    }

    motor.foc.i_a = 12.5f;
    motor.foc.dtc_c = 0.77f;
    motor.req = DRIVE_REQ_STOP;
    motor.state = DRIVE_STATE_STOP;
    motor.pwm_active = false;
    motor.fast_seq++;
    motor.pwm_cmd.seq = motor.fast_seq;
    motor.pwm_cmd.valid = false;
    debug_snapshot_publish(&motor);
    expected_seq = motor.fast_seq;
    if (!expect_snapshot_fields(&motor, expected_seq, (uint32_t)fault_mode))
    {
        return 3;
    }

    if (!test_snapshot_cannot_change_source(&motor))
    {
        return 4;
    }

    return 0;
}
