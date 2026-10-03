/**
 * @file test_debug_snapshot.c
 * @brief 在电脑端验证生产调试快照的字段映射和单向数据关系。
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "common.h"
#include "diag_runtime.h"
#include "debug_snapshot.h"
#include "drive_diag.h"
#include "observer_adapter.h"

diag_runtime_t g_diag;

/* target_qenc 桩：debug_snapshot 发布 ABZ Z 计数，主机测试返回固定值。 */
uint32_t target_qenc_z_count(void)
{
    return 7U;
}
obs_t g_obs;

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

static void fill_source(foc_t *motor)
{
    memset(motor, 0, sizeof(*motor));
    memset(&g_diag, 0, sizeof(g_diag));
    memset(&g_obs, 0, sizeof(g_obs));

    motor->fast_seq = 41U;
    motor->req = DRIVE_REQ_START;
    motor->state = DRIVE_STATE_RUN;
    motor->pwm_active = true;
    motor->fault.all = UINT32_C(0x00000125);
    motor->mode.sys = debug_mode;
    motor->mode.debug = pos_spd_curr_cl;
    g_diag.request = DIAG_REQUEST_START;
    g_diag.active_job = DIAG_JOB_CURRENT_SWEEP;
    g_diag.manager.state = MC_DIAG_RUNNING;
    g_diag.last_status = MC_BUSY;
    g_diag.active = true;
    g_diag.voltage_saturated = true;
    g_diag.command.id_ref_a = 0.4f;
    g_diag.command.iq_ref_a = 0.5f;
    g_diag.command.vd_ref_v = 0.6f;
    g_diag.command.vq_ref_v = 0.7f;
    g_diag.sweep.active_frequency_hz = 100.0f;
    g_obs.status = MC_OK;
    g_obs.flux.accepted_samples = 7U;
    g_obs.flux.psi_magnitude_filtered_wb = 0.0049f;

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

    motor->fb.vbus = 24.25f;
    motor->fb.ia = 1.1f;
    motor->fb.ib = -2.2f;
    motor->fb.ic = 3.3f;
    motor->fb.theta_e = 0.45f;

    motor->ref.pos_r = 5.1f;
    motor->fb.pos_r = 5.2f;
    motor->ref.spd_r = 6.1f;
    motor->fb.spd_r = 6.2f;

    motor->ref.id = 7.1f;
    motor->fb.id = 7.2f;
    motor->ref.iq = 8.1f;
    motor->ref.iq_lim = 8.2f;
    motor->fb.iq = 8.3f;

    motor->out.vd = 9.1f;
    motor->out.vq = 9.2f;
    motor->out.duty_a = 0.11f;
    motor->out.duty_b = 0.22f;
    motor->out.duty_c = 0.33f;
}

static int expect_snapshot_fields(const foc_t *motor,
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
           expect_u32("diag_req", g_debug_snapshot.diag_req, g_diag.request) &&
           expect_u32("diag_job",
                      g_debug_snapshot.diag_job,
                      (uint32_t)g_diag.active_job) &&
           expect_u32("diag_state",
                      g_debug_snapshot.diag_state,
                      (uint32_t)g_diag.manager.state) &&
           expect_u32("diag_status",
                      g_debug_snapshot.diag_status,
                      (uint32_t)g_diag.last_status) &&
           expect_u32("diag_active",
                      g_debug_snapshot.diag_active,
                      (uint32_t)g_diag.active) &&
           expect_u32("diag_v_sat",
                      g_debug_snapshot.diag_v_sat,
                      (uint32_t)g_diag.voltage_saturated) &&
           expect_u32("obs_status",
                      g_debug_snapshot.obs_status,
                      (uint32_t)g_obs.status) &&
           expect_u32("obs_samples",
                      g_debug_snapshot.obs_samples,
                      g_obs.flux.accepted_samples) &&
           expect_float_bits("vbus", g_debug_snapshot.vbus, motor->fb.vbus) &&
           expect_float_bits("ia", g_debug_snapshot.ia, motor->fb.ia) &&
           expect_float_bits("ib", g_debug_snapshot.ib, motor->fb.ib) &&
           expect_float_bits("ic", g_debug_snapshot.ic, motor->fb.ic) &&
           expect_float_bits("theta_e", g_debug_snapshot.theta_e, motor->fb.theta_e) &&
           expect_float_bits("pos_r_ref", g_debug_snapshot.pos_r_ref, motor->ref.pos_r) &&
           expect_float_bits("pos_r_fbk", g_debug_snapshot.pos_r_fbk, motor->fb.pos_r) &&
           expect_float_bits("spd_r_ref", g_debug_snapshot.spd_r_ref, motor->ref.spd_r) &&
           expect_float_bits("spd_r_fbk", g_debug_snapshot.spd_r_fbk, motor->fb.spd_r) &&
           expect_float_bits("id_ref", g_debug_snapshot.id_ref, motor->ref.id) &&
           expect_float_bits("id_fbk", g_debug_snapshot.id_fbk, motor->fb.id) &&
           expect_float_bits("iq_ref", g_debug_snapshot.iq_ref, motor->ref.iq) &&
           expect_float_bits("iq_lim", g_debug_snapshot.iq_lim, motor->ref.iq_lim) &&
           expect_float_bits("iq_fbk", g_debug_snapshot.iq_fbk, motor->fb.iq) &&
           expect_float_bits("vd", g_debug_snapshot.vd, motor->out.vd) &&
           expect_float_bits("vq", g_debug_snapshot.vq, motor->out.vq) &&
           expect_float_bits("diag_id_ref",
                             g_debug_snapshot.diag_id_ref,
                             g_diag.command.id_ref_a) &&
           expect_float_bits("diag_iq_ref",
                             g_debug_snapshot.diag_iq_ref,
                             g_diag.command.iq_ref_a) &&
           expect_float_bits("diag_vd_ref",
                             g_debug_snapshot.diag_vd_ref,
                             g_diag.command.vd_ref_v) &&
           expect_float_bits("diag_vq_ref",
                             g_debug_snapshot.diag_vq_ref,
                             g_diag.command.vq_ref_v) &&
           expect_float_bits("diag_freq",
                             g_debug_snapshot.diag_freq,
                             g_diag.sweep.active_frequency_hz) &&
           expect_float_bits("flux_wb",
                             g_debug_snapshot.flux_wb,
                             g_obs.flux.psi_magnitude_filtered_wb) &&
           expect_float_bits("duty_a", g_debug_snapshot.duty_a, motor->out.duty_a) &&
           expect_float_bits("duty_b", g_debug_snapshot.duty_b, motor->out.duty_b) &&
           expect_float_bits("duty_c", g_debug_snapshot.duty_c, motor->out.duty_c) &&
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

static int test_mode_mapping(foc_t *motor, uint32_t *expected_seq)
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

static int test_snapshot_cannot_change_source(foc_t *motor)
{
    const foc_t before = *motor;

    g_debug_snapshot.req = UINT32_MAX;
    g_debug_snapshot.state = UINT32_MAX;
    g_debug_snapshot.ia = -99.0f;
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
    foc_t motor;
    uint32_t expected_seq;

    /* P3 布局变更：尾部追加 4 个关口①影子字段（abz_raw/abz_pos/abz_spd_raw/abz_z，16B，
     4 字节对齐无填充），200 -> 216。快照尚无协议消费者（P7 才冻结帧格式）。 */
    _Static_assert(sizeof(debug_snapshot_t) == 216U, "调试快照布局发生了变化");

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

    motor.fb.ia = 12.5f;
    motor.out.duty_c = 0.77f;
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
