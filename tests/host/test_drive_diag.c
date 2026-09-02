/**
 * @file test_drive_diag.c
 * @brief 验证诊断算法与 Drive/FOC 之间的数据适配。
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "common.h"
#include "diag_runtime.h"
#include "drive_diag.h"
#include "motor_drive_config.h"

static mc_status_t fake_start_status;
static mc_status_t fake_step_status;
static mc_command_t fake_command;
static mc_sample_t captured_sample;
static float captured_d;
static float captured_q;
static float captured_angle;
static unsigned int current_call_count;
static unsigned int voltage_call_count;
static unsigned int pwm_commit_count;
static unsigned int abort_count;
static unsigned int stopped_count;

static void reset_fakes(void)
{
    memset(&g_diag, 0, sizeof(g_diag));
    g_diag.profile.current_limit_a = 2.0f;
    g_diag.profile.voltage_limit_v = 5.0f;
    g_diag.profile.encoder_full_scale = 16384U;
    fake_start_status = MC_OK;
    fake_step_status = MC_BUSY;
    fake_command = (mc_command_t){0};
    captured_sample = (mc_sample_t){0};
    captured_d = 0.0f;
    captured_q = 0.0f;
    captured_angle = 0.0f;
    current_call_count = 0U;
    voltage_call_count = 0U;
    pwm_commit_count = 0U;
    abort_count = 0U;
    stopped_count = 0U;
}

void diag_runtime_init(diag_runtime_t *runtime, const diag_seed_t *seed)
{
    memset(runtime, 0, sizeof(*runtime));
    runtime->profile.control_period_s = seed->control_period_s;
    runtime->profile.phase_resistance_ohm = seed->phase_resistance_ohm;
    runtime->profile.pole_pairs = seed->pole_pairs;
    runtime->profile.encoder_full_scale = seed->encoder_full_scale;
    runtime->profile.encoder_direction = seed->encoder_direction;
}

mc_status_t diag_runtime_start(diag_runtime_t *runtime,
                               float initial_vbus_v,
                               bool platform_ready_and_disabled)
{
    runtime->last_sample.vbus_v = initial_vbus_v;
    runtime->active = (fake_start_status == MC_OK)
        && platform_ready_and_disabled;
    return fake_start_status;
}

mc_status_t diag_runtime_step(diag_runtime_t *runtime,
                              const mc_sample_t *sample,
                              mc_command_t *command)
{
    (void)runtime;
    captured_sample = *sample;
    *command = fake_command;
    return fake_step_status;
}

mc_status_t diag_runtime_abort(diag_runtime_t *runtime,
                               mc_command_t *command)
{
    abort_count++;
    runtime->last_status = MC_ABORTED;
    *command = (mc_command_t){.disable_request = true};
    return MC_ABORTED;
}

void diag_runtime_confirm_stopped(diag_runtime_t *runtime,
                                  bool platform_ready_and_disabled)
{
    if (runtime->active && platform_ready_and_disabled)
    {
        stopped_count++;
        runtime->active = false;
    }
}

bool foc_curr(pmsm_t *pm, float d_ref, float q_ref, float angle)
{
    current_call_count++;
    captured_d = d_ref;
    captured_q = q_ref;
    captured_angle = angle;
    pm->foc.v_d = 1.0f;
    pm->foc.v_q = 0.5f;
    pm->foc.vs = 12.0f;
    return true;
}

bool foc_volt(pmsm_t *pm, float d_ref, float q_ref, float angle)
{
    voltage_call_count++;
    captured_d = d_ref;
    captured_q = q_ref;
    captured_angle = angle;
    pm->foc.v_d = d_ref;
    pm->foc.v_q = q_ref;
    pm->foc.vs = 12.0f;
    return true;
}

bool drive_pwm_commit(pmsm_t *pm)
{
    (void)pm;
    pwm_commit_count++;
    return true;
}

static bool expect_true(bool condition, const char *message)
{
    if (condition)
    {
        return true;
    }

    fprintf(stderr, "%s\n", message);
    return false;
}

static bool nearly_equal(float actual, float expected)
{
    return fabsf(actual - expected) < 1.0e-5f;
}

static pmsm_t make_motor(void)
{
    pmsm_t pm = {0};

    pm.state = DRIVE_STATE_STOP;
    pm.foc.i_a = 1.0f;
    pm.foc.i_b = -0.5f;
    pm.foc.i_c = -0.5f;
    pm.foc.p_e = 0.0f;
    pm.foc.mp_r = 2.0f;
    pm.foc.wr_f = 3.0f;
    pm.foc.vbus = 24.0f;
    pm.period.foc_ts = 0.00005f;
    pm.pos_box.raw_1 = 1234;
    return pm;
}

static bool test_init_builds_independent_profile(void)
{
    pmsm_t pm = make_motor();

    reset_fakes();
    pm.para.Rs = 0.16f;
    pm.para.pn = 10;
    pm.pos_box.ma732.cpr = 16384U;
    pm.pos_box.ma732.dir = 1;
    pm.prot_cfg.under_voltage_v = 15.0f;
    drive_diag_init(&pm);

    return expect_true(nearly_equal(g_diag.profile.control_period_s,
                                    pm.period.foc_ts),
                       "诊断对象应取得当前快速控制周期。")
        && expect_true(nearly_equal(g_diag.profile.phase_resistance_ohm,
                                    pm.para.Rs),
                       "诊断对象应取得当前电机相电阻。")
        && expect_true(g_diag.profile.pole_pairs == 10U,
                       "诊断对象应取得当前电机极对数。")
        && expect_true(g_diag.profile.encoder_full_scale == 16384U,
                       "诊断对象应取得 MA732 满量程。")
        && expect_true(nearly_equal(g_diag.profile.current_limit_a,
                                    DRIVE_DIAG_CURRENT_LIMIT_A),
                       "诊断电流硬上限应来自电机驱动配置。")
        && expect_true(nearly_equal(g_diag.profile.voltage_limit_v,
                                    DRIVE_DIAG_VOLTAGE_LIMIT_V),
                       "诊断电压硬上限应来自电机驱动配置。")
        && expect_true(nearly_equal(g_diag.profile.minimum_vbus_v, 15.0f),
                       "诊断对象应复用 Drive 欠压门槛。");
}

static bool test_request_enters_diagnostic_mode(void)
{
    pmsm_t pm = make_motor();

    g_diag.request = DIAG_REQUEST_START;
    g_diag.requested_job = DIAG_JOB_CURRENT_SWEEP;
    drive_diag_poll_request(&pm);

    return expect_true(g_diag.request == DIAG_REQUEST_NONE,
                       "诊断请求应在读取后清零。")
        && expect_true(pm.mode.sys == calibrat_mode,
                       "START 请求应选择标定系统模式。")
        && expect_true(pm.mode.calibrat == iden_pm,
                       "START 请求应选择统一辨识入口。")
        && expect_true(pm.req == DRIVE_REQ_START,
                       "START 请求应交给普通 Drive 状态机启动。");
}

static bool test_prepare_uses_stopped_platform(void)
{
    pmsm_t pm = make_motor();

    reset_fakes();
    if (!expect_true(drive_diag_prepare(&pm),
                     "STOP 状态下应允许启动有效诊断任务。")
        || !expect_true(g_diag.active,
                        "算法启动成功后应记录为活动任务。")
        || !expect_true(nearly_equal(g_diag.last_sample.vbus_v, 24.0f),
                        "启动算法时应传入当前母线电压。"))
    {
        return false;
    }

    g_diag.active = false;
    fake_start_status = MC_INVALID_ARGUMENT;
    return expect_true(!drive_diag_prepare(&pm),
                       "算法参数无效时不得启动 PWM。")
        && expect_true(g_diag.last_status == MC_INVALID_ARGUMENT,
                       "启动失败原因应保留给调试器查看。");
}

static bool test_current_command_uses_fresh_feedback(void)
{
    pmsm_t pm = make_motor();

    reset_fakes();
    g_diag.active = true;
    fake_command = (mc_command_t){
        .mode = MC_CONTROL_CURRENT,
        .id_ref_a = 0.2f,
        .iq_ref_a = -0.3f,
        .enable_request = true,
    };

    if (!expect_true(drive_diag_step(&pm),
                     "有效电流命令应完成一次 FOC 和 PWM 提交。"))
    {
        return false;
    }

    return expect_true(nearly_equal(captured_sample.i_alpha_a, 1.0f),
                       "适配层应使用本周期三相电流重新做 Clarke 变换。")
        && expect_true(nearly_equal(captured_sample.i_beta_a, 0.0f),
                       "对称三相测试电流的 beta 分量应为零。")
        && expect_true(nearly_equal(captured_sample.id_a, 1.0f),
                       "适配层应使用本周期电角度重新计算 d 轴电流。")
        && expect_true(captured_sample.encoder_raw == 1234U,
                       "应把 MA732 当前原始角度传给辨识算法。")
        && expect_true(current_call_count == 1U,
                       "电流命令只能调用一次现有电流环。")
        && expect_true(voltage_call_count == 0U,
                       "电流命令不能误入电压控制。")
        && expect_true(nearly_equal(captured_d, 0.2f)
                       && nearly_equal(captured_q, -0.3f)
                       && nearly_equal(captured_angle, 0.0f),
                       "算法电流给定和闭环角度应原样传给 FOC。")
        && expect_true(pwm_commit_count == 1U,
                       "有效 FOC 结果应统一提交一次 PWM。");
}

static bool test_openloop_voltage_uses_command_angle(void)
{
    pmsm_t pm = make_motor();

    reset_fakes();
    g_diag.active = true;
    fake_command = (mc_command_t){
        .mode = MC_CONTROL_VOLTAGE,
        .vd_ref_v = 0.8f,
        .vq_ref_v = 0.1f,
        .openloop_theta_e_rad = 1.2f,
        .openloop_enable = true,
        .enable_request = true,
    };

    return expect_true(drive_diag_step(&pm),
                       "有效开环电压命令应完成一次 PWM 提交。")
        && expect_true(voltage_call_count == 1U,
                       "开环电压命令应调用现有电压控制。")
        && expect_true(nearly_equal(captured_angle, 1.2f),
                       "极对数等开环任务必须使用算法生成的电角度。")
        && expect_true(pwm_commit_count == 1U,
                       "开环电压命令也必须走统一 PWM 提交层。");
}

static bool test_finish_and_limit_return_to_stop(void)
{
    pmsm_t done_pm = make_motor();
    pmsm_t limited_pm = make_motor();

    reset_fakes();
    g_diag.active = true;
    fake_step_status = MC_DONE;
    if (!expect_true(!drive_diag_step(&done_pm),
                     "任务完成后本周期应停止继续输出。")
        || !expect_true(done_pm.req == DRIVE_REQ_STOP,
                        "任务完成后应请求 Drive 停止。")
        || !expect_true(pwm_commit_count == 0U,
                        "任务完成命令不得再提交 PWM。"))
    {
        return false;
    }

    reset_fakes();
    g_diag.active = true;
    fake_command = (mc_command_t){
        .mode = MC_CONTROL_CURRENT,
        .id_ref_a = 0.2f,
        .enable_request = true,
    };
    g_diag.profile.voltage_limit_v = 0.5f;
    return expect_true(!drive_diag_step(&limited_pm),
                       "FOC 输出超过诊断电压上限时必须停止。")
        && expect_true(abort_count == 1U,
                       "越过已确认电压上限时应中止当前任务。")
        && expect_true(pwm_commit_count == 0U,
                       "越限结果不得提交到 PWM。");
}

static bool test_stop_and_fault_close_runtime(void)
{
    pmsm_t pm = make_motor();

    reset_fakes();
    g_diag.active = true;
    g_diag.request = DIAG_REQUEST_STOP;
    drive_diag_poll_request(&pm);
    if (!expect_true(abort_count == 1U,
                     "显式 STOP 应中止活动算法。")
        || !expect_true(pm.req == DRIVE_REQ_STOP,
                        "显式 STOP 应交给 Drive 关闭 PWM。"))
    {
        return false;
    }

    drive_diag_on_stopped();
    if (!expect_true(stopped_count == 1U,
                     "PWM 关闭后应释放诊断任务。"))
    {
        return false;
    }

    g_diag.active = true;
    drive_diag_on_fault();
    return expect_true(abort_count == 2U,
                       "Drive 故障应中止活动算法。")
        && expect_true(pm.req == DRIVE_REQ_STOP,
                       "故障处理不能让诊断任务继续运行。");
}

int main(void)
{
    reset_fakes();
    if (!test_init_builds_independent_profile()) return 1;
    if (!test_request_enters_diagnostic_mode()) return 2;
    if (!test_prepare_uses_stopped_platform()) return 3;
    if (!test_current_command_uses_fresh_feedback()) return 4;
    if (!test_openloop_voltage_uses_command_angle()) return 5;
    if (!test_finish_and_limit_return_to_stop()) return 6;
    if (!test_stop_and_fault_close_runtime()) return 7;
    return 0;
}
