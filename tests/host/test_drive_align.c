/**
 * @file test_drive_align.c
 * @brief ABZ 上电自动对齐验证（参考预定位+整圈扫场）：状态流转、电流
 *        渐变+扫场注入、e_off 圆均值定零（=参考清 TIM3 计数器的等价
 *        实现）、对齐后保持 STOP、二次启动不重复对齐、绝对编码器跳过
 *        对齐、样本分散拒绝、重对齐增量语义、卡死转子参考语义。
 *
 * 用小 fs（10Hz）把 3.2s 对齐压缩为 32 拍：渐变=[0,10)、扫场=[10,25)、
 * 保持=[25,32)、采样=[29,32)、第 33 拍收尾（n=32 时判收）。
 * 注：扫场=本台架联轴静摩擦下保证转子到场 0（静场拉不动，2026-10-18
 * 三轮实测位置全平）；参考无到位验收，场景 8 文档化卡死转子以停位
 * 定零的参考同款语义。
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

void drive_control_reset_for_start(foc_t *foc)
{
    (void)foc;
}

bool drive_mode_step(foc_t *foc)
{
    /* 真实模式步进会生成并提交有效 PWM 命令；桩同步置 valid，
     * 否则 RUN 晋升断言会被 fast_step 的无效命令守卫误伤。 */
    foc->pwm_cmd.valid = true;
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
    foc.rate.foc_fs = 10.0f; /* 渐变=10 拍 / 扫场=15 拍 / 保持=7 拍 / 第 33 拍收尾 */
    foc.enc.primary = ENCODER_TYPE_ABZ;
    foc.prot_cfg.invalid_position_samples = 100U;
    return foc;
}

int main(void)
{
    /* 场景 1：START → STARTING，首拍电流=0（渐变起点），场角 0 */
    foc_t m = make_align_motor();
    m.fb.theta_e = 1.0f;
    m.fb.pos_valid = true;
    m.fb.i_valid = true;
    m.fb.vbus_valid = true;
    m.fb.vbus = 24.0f;
    m.req = DRIVE_REQ_START;
    drive_fast_step(&m);
    expect("对齐首拍进入 STARTING", m.state == DRIVE_STATE_STARTING);
    expect("首拍电流=渐变起点 0", last_align_id == 0.0f);
    expect("首拍场角 0", last_align_theta == 0.0f);
    expect("iq 给定 0", last_align_iq == 0.0f);

    /* 场景 2：渐变段静场转子不动（停 1.0），扫场段被移动场拖到 0，
     * 保持段稳在 0 → 采样窗均值 0 → e_off=0，完成后关 PWM 回 STOP
     * （对齐≠运行授权）。渐变中点电流=半幅；扫场中点场角=2pi*7/15。 */
    for (int i = 0; i < 45; i++)
    {
        const float n = (float)m.align_ticks;

        if (n < 10.0f)
        {
            m.fb.theta_e = 1.0f;
        }
        else if (n < 25.0f)
        {
            m.fb.theta_e = 1.0f * (25.0f - n) / 15.0f;
        }
        else
        {
            m.fb.theta_e = 0.0f;
        }
        m.fb.pos_valid = true;
        m.fb.i_valid = true;
        m.fb.vbus_valid = true;
        m.fb.vbus = 24.0f;
        drive_fast_step(&m);
        if (n == 5.0f)
        {
            expect("渐变中点电流=半幅", last_align_id == 1.0f);
        }
        if (n == 17.0f)
        {
            expect("扫场中点电流=全幅", last_align_id == 2.0f);
            expect("扫场中点场角=2pi*7/15",
                   fabsf(last_align_theta - 6.28318530718f * 7.0f / 15.0f) <
                       1e-4f);
        }
        if (m.enc_aligned)
        {
            break;
        }
    }
    expect("33 拍后对齐完成", m.enc_aligned);
    expect("对齐完成保持 STOP", m.req == DRIVE_REQ_STOP);
    expect("对齐完成关闭 PWM", !m.pwm_active && pwm_stop_calls > 0);
    expect("e_off=0（吸合停位=零位）", m.motor.e_off == 0.0f);
    expect("末拍电流=档案全幅", last_align_id == 2.0f);
    expect("末拍场角收尾回 0", last_align_theta == 0.0f);

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

    /* 场景 6：角度未收敛（样本 0/π 交替 → 圆均值矢量长 1/3 < 0.9）
     * → 拒绝对齐，置 enc_err（转子在转/编码器异常的诚实报错） */
    {
        foc_t m3 = make_align_motor();
        m3.fb.pos_valid = true;
        m3.fb.i_valid = true;
        m3.fb.vbus_valid = true;
        m3.fb.vbus = 24.0f;
        m3.req = DRIVE_REQ_START;
        for (int i = 0; i < 45; i++)
        {
            m3.fb.theta_e = (m3.align_ticks % 2U == 0U) ? 0.0f : 3.14159265f;
            drive_fast_step(&m3);
            if (m3.fault.all != 0U)
            {
                break;
            }
        }
        expect("角度未收敛拒绝对齐", !m3.enc_aligned);
        expect("置编码器故障", m3.fault.bit.enc_err == 1U);
        expect("不晋升 RUN", m3.req == DRIVE_REQ_STOP || m3.req == DRIVE_REQ_START);
    }

    /* 场景 7：重对齐（现行 e_off=0.478 非零）→ 增量收敛而非绝对替换。
     * 绝对替换产 e_off=2pi-0.5（多扣旧值 0.478，真机 RS_NO_SAMPLE 根因）；
     * 增量语义产 wrap(0.478-0.5)=2pi-0.022，使锁定位 fb 归零。 */
    {
        foc_t m5 = make_align_motor();
        m5.motor.e_off = 0.478f;
        m5.fb.pos_valid = true;
        m5.fb.i_valid = true;
        m5.fb.vbus_valid = true;
        m5.fb.vbus = 24.0f;
        m5.req = DRIVE_REQ_START;
        for (int i = 0; i < 45; i++)
        {
            const float n = (float)m5.align_ticks;

            if (n < 10.0f)
            {
                m5.fb.theta_e = 1.0f;
            }
            else if (n < 25.0f)
            {
                m5.fb.theta_e = 0.5f + 0.5f * (25.0f - n) / 15.0f;
            }
            else
            {
                m5.fb.theta_e = 0.5f;
            }
            drive_fast_step(&m5);
            if (m5.enc_aligned)
            {
                break;
            }
        }
        expect("重对齐完成", m5.enc_aligned);
        expect("e_off=增量收敛 2pi-0.022",
               fabsf(m5.motor.e_off - (6.28318530718f - 0.022f)) < 1e-4f);
    }

    /* 场景 8：卡死转子（θ_e 恒 1.5，静场拉不动）→ 参考同款语义：
     * 无到位验收，以停位定零"成功"（e_off=2pi-1.5）。参考实现
     * （清计数器）行为一致——参考靠 5A 渐变保证拉入，不靠验收。 */
    {
        foc_t m6 = make_align_motor();
        m6.fb.theta_e = 1.5f;
        m6.fb.pos_valid = true;
        m6.fb.i_valid = true;
        m6.fb.vbus_valid = true;
        m6.fb.vbus = 24.0f;
        m6.req = DRIVE_REQ_START;
        for (int i = 0; i < 45; i++)
        {
            drive_fast_step(&m6);
            if (m6.enc_aligned)
            {
                break;
            }
        }
        expect("卡死转子按参考语义定零成功", m6.enc_aligned);
        expect("e_off=2pi-1.5（停位即零位）",
               fabsf(m6.motor.e_off - (6.28318530718f - 1.5f)) < 1e-4f);
    }

    if (fail_count == 0)
    {
        (void)printf("test_drive_align: all pass\n");
        return 0;
    }
    (void)printf("test_drive_align: %d failure(s)\n", fail_count);
    return 1;
}
