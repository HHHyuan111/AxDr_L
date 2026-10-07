/**
 * @file test_drive_align.c
 * @brief ABZ 上电自动对齐验证：状态流转、固定角电流注入、e_off 圆均值数学、
 *        自动晋升 RUN、二次启动不重复对齐、绝对编码器跳过对齐。
 *
 * 用小 fs（10Hz）把 1.5s 对齐压缩为 15 拍：settle=12 拍，平均=3 拍。
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "common.h"
#include "drive.h"

foc_t g_foc;

/* ---- 生产符号桩（对齐 foc_cur_step/驱动路径，记录调用参数） ---- */
static float last_align_id;
static float last_align_iq;
static float last_align_theta;
static int foc_cur_calls;

bool foc_cur_step(foc_t *foc, float id_ref, float iq_ref, float angle)
{
    (void)foc;
    last_align_id = id_ref;
    last_align_iq = iq_ref;
    last_align_theta = angle;
    foc_cur_calls++;
    return true;
}

static int commit_calls;
bool drive_pwm_commit(foc_t *foc)
{
    (void)foc;
    commit_calls++;
    return true;
}

static bool pwm_start_ok = true;
static int pwm_start_calls;
bool drive_pwm_start(void)
{
    pwm_start_calls++;
    return pwm_start_ok;
}

static int pwm_stop_calls;
bool drive_pwm_stop(void)
{
    pwm_stop_calls++;
    return true;
}

void drive_pwm_set_neutral(foc_t *foc)
{
    (void)foc;
}

void drive_control_reset(foc_t *foc)
{
    (void)foc;
}

bool drive_mode_step(foc_t *foc)
{
    (void)foc;
    return true;
}

void drive_diag_poll_request(foc_t *foc)
{
    (void)foc;
}

void drive_diag_on_stopped(void)
{
}

void drive_diag_on_fault(void)
{
}

bool drive_mode_prepare(foc_t *foc)
{
    (void)foc;
    return true;
}

static int fail_count;
static void expect(const char *name, bool cond)
{
    if (!cond)
    {
        (void)printf("FAIL %s\n", name);
        fail_count++;
    }
}

static foc_t make_align_motor(void)
{
    foc_t foc = {0};
    foc.mode.sys = debug_mode;
    foc.mode.debug = curr_cl;
    foc.req = DRIVE_REQ_STOP;
    foc.state = DRIVE_STATE_STOP;
    foc.motor.phase_order = PHASE_ORDER_ABC;
    foc.motor.align_current_a = 2.0f;
    foc.rate.foc_fs = 10.0f; /* settle=12 拍 / avg=3 拍 */
    foc.enc.primary = ENCODER_TYPE_ABZ;
    foc.prot_cfg.invalid_position_samples = 100U;
    return foc;
}

int main(void)
{
    /* 场景 1：START → STARTING，对齐期间固定角 0 注入档案电流 */
    foc_t m = make_align_motor();
    m.fb.theta_e = 1.0f;
    m.fb.id = 2.0f;
    m.fb.pos_valid = true;
    m.fb.i_valid = true;
    m.fb.vbus_valid = true;
    m.fb.vbus = 24.0f;
    m.req = DRIVE_REQ_START;
    drive_fast_step(&m);
    expect("对齐首拍进入 STARTING", m.state == DRIVE_STATE_STARTING);
    expect("注入电流=档案对齐电流", last_align_id == 2.0f);
    expect("注入角度恒 0（开环）", last_align_theta == 0.0f);
    expect("iq 给定 0", last_align_iq == 0.0f);

    /* 场景 2：平均期 fb.theta_e=1.0 → e_off = 2pi-1.0，完成后自动晋升 RUN。
     * 时序：settle=12 + avg=3 → 第 15 拍计数满，第 16 拍收尾（含首拍共 16 次调用）。 */
    for (int i = 0; i < 15; i++)
    {
        m.fb.theta_e = 1.0f;
        m.fb.id = 2.0f; /* 对齐电流已建立（判据②） */
        m.fb.pos_valid = true;
        m.fb.i_valid = true;
        m.fb.vbus_valid = true;
        m.fb.vbus = 24.0f;
        drive_fast_step(&m);
    }
    expect("15 拍后对齐完成", m.enc_aligned);
    expect("自动晋升 RUN", m.req == DRIVE_REQ_RUN);
    expect("e_off=2pi-1.0", fabsf(m.motor.e_off - (6.28318530718f - 1.0f)) < 1e-4f);

    /* 场景 3：已对齐 → STOP 后再 START 直接晋升，不再注入对齐电流 */
    m.req = DRIVE_REQ_STOP;
    drive_fast_step(&m);
    expect("STOP 后回到 STOP 态", m.state == DRIVE_STATE_STOP);
    const int calls_before = foc_cur_calls;
    m.req = DRIVE_REQ_START;
    drive_fast_step(&m);
    expect("二次 START 直接晋升 RUN", m.req == DRIVE_REQ_RUN);
    expect("不再注入对齐电流", foc_cur_calls == calls_before);

    /* 场景 4：绝对编码器（MT6816）跳过对齐直接晋升 */
    foc_t abs = make_align_motor();
    abs.enc.primary = ENCODER_TYPE_MT6816;
    abs.req = DRIVE_REQ_START;
    drive_fast_step(&abs);
    expect("绝对编码器直接 RUN", abs.req == DRIVE_REQ_RUN && !abs.enc_aligned);

    /* 场景 5：对齐中断（STOP）进度作废，重启从第 0 拍计时 */
    foc_t m2 = make_align_motor();
    m2.req = DRIVE_REQ_START;
    drive_fast_step(&m2);
    drive_fast_step(&m2);
    drive_fast_step(&m2); /* 3 拍进度 */
    expect("中断前未完成", !m2.enc_aligned);
    m2.req = DRIVE_REQ_STOP;
    drive_fast_step(&m2);
    expect("STOP 清对齐进度", m2.align_ticks == 0U);

    /* 场景 6：角度未收敛（样本 0/π 交替 → R̄≈0）→ 拒绝对齐，置 enc_err */
    {
        foc_t m3 = make_align_motor();
        m3.fb.pos_valid = true;
        m3.fb.i_valid = true;
        m3.fb.vbus_valid = true;
        m3.fb.vbus = 24.0f;
        m3.fb.id = 2.0f;
        m3.req = DRIVE_REQ_START;
        for (int i = 0; i < 16; i++)
        {
            m3.fb.theta_e = (i % 2 == 0) ? 0.0f : 3.14159265f;
            drive_fast_step(&m3);
        }
        expect("角度未收敛拒绝对齐", !m3.enc_aligned);
        expect("置编码器故障", m3.fault.bit.enc_err == 1U);
        expect("不晋升 RUN", m3.req == DRIVE_REQ_STOP || m3.req == DRIVE_REQ_START);
    }

    /* 场景 7：电流未建立（fb.id=0.5 < 0.8x2）→ 拒绝对齐 */
    {
        foc_t m4 = make_align_motor();
        m4.fb.pos_valid = true;
        m4.fb.i_valid = true;
        m4.fb.vbus_valid = true;
        m4.fb.vbus = 24.0f;
        m4.fb.id = 0.5f;
        m4.req = DRIVE_REQ_START;
        for (int i = 0; i < 16; i++)
        {
            m4.fb.theta_e = 1.0f;
            drive_fast_step(&m4);
        }
        expect("电流未建立拒绝对齐", !m4.enc_aligned);
        expect("同样置编码器故障", m4.fault.bit.enc_err == 1U);
    }

    /* 场景 8：重对齐（现行 e_off=0.478 非零）→ 增量收敛而非绝对替换。
     * 旧代码产 e_off=2pi-1.0（多扣旧值 0.478，真机 RS_NO_SAMPLE 根因）；
     * 增量语义产 wrap(0.478-1.0)=2pi-0.522，使锁定位 fb 归零。 */
    {
        foc_t m5 = make_align_motor();
        m5.motor.e_off = 0.478f;
        m5.fb.pos_valid = true;
        m5.fb.i_valid = true;
        m5.fb.vbus_valid = true;
        m5.fb.vbus = 24.0f;
        m5.fb.id = 2.0f;
        m5.req = DRIVE_REQ_START;
        for (int i = 0; i < 16; i++)
        {
            m5.fb.theta_e = 1.0f;
            drive_fast_step(&m5);
        }
        expect("重对齐完成", m5.enc_aligned);
        expect("e_off=增量收敛 2pi-0.522",
               fabsf(m5.motor.e_off - (6.28318530718f - 0.522f)) < 1e-4f);
    }

    if (fail_count == 0)
    {
        (void)printf("test_drive_align: all pass\n");
        return 0;
    }
    (void)printf("test_drive_align: %d failure(s)\n", fail_count);
    return 1;
}
