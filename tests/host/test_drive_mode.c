/**
 * @file test_drive_mode.c
 * @brief 在 Host 上验证生产 Drive 模式分派表。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "common.h"
#include "drive_mode.h"

typedef enum
{
    EVENT_MIT = 0,
    EVENT_TORQUE,
    EVENT_VELOCITY,
    EVENT_POSITION,
    EVENT_CST,
    EVENT_CSV,
    EVENT_CSP,
    EVENT_QUICK_STOP,
    EVENT_FAULT_STOP,
    EVENT_ENCODER_CALIBRATION,
    EVENT_IDENTIFICATION,
    EVENT_ANTICOGGING,
    EVENT_FORCE_VOLTAGE,
    EVENT_FORCE_CURRENT,
    EVENT_FOC_VOLTAGE,
    EVENT_FOC_CURRENT,
    EVENT_FOC_VELOCITY,
    EVENT_FOC_POSITION
} mode_event_e;

typedef struct
{
    int mode;
    mode_event_e event;
    bool commits_pwm;
} mode_case_t;

static mode_event_e last_event;
static unsigned int event_count;
static unsigned int commit_count;
static unsigned int command_apply_count;
static bool foc_result_valid;
static bool command_valid;
static bool arguments_ok;

static void log_event(mode_event_e event);

static void reset_fakes(void)
{
    last_event = EVENT_MIT;
    event_count = 0U;
    commit_count = 0U;
    command_apply_count = 0U;
    foc_result_valid = true;
    command_valid = true;
    arguments_ok = true;
}

bool drive_cmd_apply(foc_t *foc)
{
    (void)foc;
    command_apply_count++;
    return command_valid;
}

bool drive_diag_is_supported(const foc_t *foc)
{
    return (foc->mode.sys == calibrat_mode)
        && (foc->mode.calibrat == iden_pm);
}

bool drive_diag_prepare(foc_t *foc)
{
    (void)foc;
    return true;
}

bool drive_diag_step(foc_t *foc)
{
    (void)foc;
    log_event(EVENT_IDENTIFICATION);
    return true;
}

static void log_event(mode_event_e event)
{
    last_event = event;
    event_count++;
}

void mit_step(foc_t *foc)
{
    (void)foc;
    log_event(EVENT_MIT);
}

void pt_tor_mode(foc_t *foc)
{
    (void)foc;
    log_event(EVENT_TORQUE);
}

void pv_step(foc_t *foc)
{
    (void)foc;
    log_event(EVENT_VELOCITY);
}

void pp_step(foc_t *foc)
{
    (void)foc;
    log_event(EVENT_POSITION);
}

void cst_step(foc_t *foc)
{
    (void)foc;
    log_event(EVENT_CST);
}

void csv_step(foc_t *foc)
{
    (void)foc;
    log_event(EVENT_CSV);
}

void csp_step(foc_t *foc)
{
    (void)foc;
    log_event(EVENT_CSP);
}

void quick_stop_step(foc_t *foc)
{
    (void)foc;
    log_event(EVENT_QUICK_STOP);
}

void fault_stop_step(foc_t *foc)
{
    (void)foc;
    log_event(EVENT_FAULT_STOP);
}

void cali_mag_encoder(foc_t *foc)
{
    (void)foc;
    log_event(EVENT_ENCODER_CALIBRATION);
}

void anticogging_calibration(foc_t *foc)
{
    (void)foc;
    log_event(EVENT_ANTICOGGING);
}

void open_volt_step(foc_t *foc)
{
    (void)foc;
    log_event(EVENT_FORCE_VOLTAGE);
}

void open_cur_step(foc_t *foc)
{
    (void)foc;
    log_event(EVENT_FORCE_CURRENT);
}

bool foc_volt_step(foc_t *foc, float vd_ref, float vq_ref, float pos)
{
    arguments_ok = arguments_ok &&
                   (vd_ref == foc->ref.vd) &&
                   (vq_ref == foc->ref.vq) &&
                   (pos == foc->fb.theta_e);
    log_event(EVENT_FOC_VOLTAGE);
    return foc_result_valid;
}

bool foc_cur_step(foc_t *foc, float id_ref, float iq_ref, float pos)
{
    arguments_ok = arguments_ok &&
                   (id_ref == foc->ref.id) &&
                   (iq_ref == foc->ref.iq) &&
                   (pos == foc->fb.theta_e);
    log_event(EVENT_FOC_CURRENT);
    return foc_result_valid;
}

bool foc_spd_step(foc_t *foc, float vel_ref, float iq_ref, float pos)
{
    arguments_ok = arguments_ok &&
                   (vel_ref == foc->ref.spd_r) &&
                   (iq_ref == foc->ref.iq) &&
                   (pos == foc->fb.theta_e);
    log_event(EVENT_FOC_VELOCITY);
    return foc_result_valid;
}

bool foc_pos_step(foc_t *foc,
             float pos_ref,
             float vel_ref,
             float iq_ref,
             float pos)
{
    arguments_ok = arguments_ok &&
                   (pos_ref == foc->ref.pos_r) &&
                   (vel_ref == foc->ref.spd_r) &&
                   (iq_ref == foc->ref.iq) &&
                   (pos == foc->fb.theta_e);
    log_event(EVENT_FOC_POSITION);
    return foc_result_valid;
}

bool drive_pwm_commit(foc_t *foc)
{
    (void)foc;
    commit_count++;
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

static bool expect_dispatch(foc_t *foc,
                            mode_event_e expected_event,
                            bool expected_commit)
{
    bool mode_valid;

    reset_fakes();
    mode_valid = drive_mode_step(foc);

    return expect_true(mode_valid, "已实现模式应返回有效。") &&
           expect_true(event_count == 1U, "模式应且只应调用一个实现。") &&
           expect_true(last_event == expected_event, "模式分派到错误的实现。") &&
           expect_true(commit_count == (expected_commit ? 1U : 0U),
                       "模式的 PWM 提交次数不正确。") &&
           expect_true(arguments_ok, "模式传给控制函数的参数不正确。");
}

static bool test_release_modes(foc_t *foc)
{
    static const mode_case_t cases[] = {
        {mit_mode, EVENT_MIT, false},
        {vel_mode, EVENT_VELOCITY, false},
        {pos_mode, EVENT_POSITION, false},
        {cst_mode, EVENT_CST, false},
        {csv_mode, EVENT_CSV, false},
        {csp_mode, EVENT_CSP, false}
    };
    size_t index;

    foc->mode.sys = release_mode;
    for (index = 0U; index < (sizeof(cases) / sizeof(cases[0])); index++)
    {
        foc->mode.release = (release_mode_e)cases[index].mode;
        if (!expect_dispatch(foc, cases[index].event, cases[index].commits_pwm) ||
            !expect_true(command_apply_count == 1U,
                         "发布模式必须先应用一次外部命令。"))
        {
            return false;
        }
    }

    reset_fakes();
    command_valid = false;
    foc->mode.release = cst_mode;

    return expect_true(!drive_mode_step(foc),
                       "无效发布命令必须终止本周期模式。") &&
           expect_true(event_count == 0U,
                       "无效发布命令不能进入控制算法。");
}

static bool test_halt_modes(foc_t *foc)
{
    foc->mode.sys = halt_mode;
    foc->mode.halt = quick_mode;
    if (!expect_dispatch(foc, EVENT_QUICK_STOP, false))
    {
        return false;
    }

    return true;
}

static bool test_debug_modes(foc_t *foc)
{
    static const mode_case_t cases[] = {
        {drag_vf, EVENT_FORCE_VOLTAGE, false},
        {drag_if, EVENT_FORCE_CURRENT, false},
        {volt_op, EVENT_FOC_VOLTAGE, true},
        {curr_cl, EVENT_FOC_CURRENT, true},
        {spd_curr_cl, EVENT_FOC_VELOCITY, true},
        {pos_spd_curr_cl, EVENT_FOC_POSITION, true}
    };
    size_t index;

    foc->mode.sys = debug_mode;
    for (index = 0U; index < (sizeof(cases) / sizeof(cases[0])); index++)
    {
        foc->mode.debug = (debug_mode_e)cases[index].mode;
        if (!expect_dispatch(foc, cases[index].event, cases[index].commits_pwm))
        {
            return false;
        }
    }

    reset_fakes();
    foc_result_valid = false;
    foc->mode.debug = volt_op;
    const bool mode_valid = drive_mode_step(foc);

    return expect_true(!mode_valid, "无效 FOC 结果应报告本周期模式失败。") &&
           expect_true(event_count == 1U, "无效 FOC 结果仍应完成本模式计算。") &&
           expect_true(commit_count == 0U, "无效 FOC 结果不能提交 PWM。");
}

static bool test_diagnostic_mode(foc_t *foc)
{
    foc->mode.sys = calibrat_mode;
    foc->mode.calibrat = iden_pm;

    return expect_dispatch(foc, EVENT_IDENTIFICATION, false);
}

static bool test_unimplemented_modes_do_nothing(foc_t *foc)
{
    static const release_mode_e release_modes[] = {
        tor_mode
    };
    static const calibrat_mode_e calibration_modes[] = {
        rotor_enc_mod,
        rotor_enc_cali,
        output_enc_mod,
        output_enc_cali,
        anticogging_pm
    };
    static const debug_mode_e debug_modes[] = {
        spd_volt_cl,
        pos_spd_volt_cl
    };
    size_t index;

    foc->mode.sys = release_mode;
    for (index = 0U; index < (sizeof(release_modes) / sizeof(release_modes[0])); index++)
    {
        reset_fakes();
        foc->mode.release = release_modes[index];
        if (!expect_true(!drive_mode_is_supported(foc),
                         "未验证的发布模式不能标记为正式支持。") ||
            !expect_true(!drive_mode_step(foc),
                         "未验证的发布模式必须明确返回失败。") ||
            !expect_true(event_count == 0U,
                         "未验证的发布模式不应调用算法实现。"))
        {
            return false;
        }
    }

    foc->mode.sys = calibrat_mode;
    for (index = 0U;
         index < (sizeof(calibration_modes) / sizeof(calibration_modes[0]));
         index++)
    {
        reset_fakes();
        foc->mode.calibrat = calibration_modes[index];
        const bool mode_valid = drive_mode_step(foc);
        if (!expect_true(!mode_valid,
                         "未实现的标定模式必须明确返回失败。") ||
            !expect_true(event_count == 0U,
                         "未实现的标定模式不应调用其他实现。"))
        {
            return false;
        }
    }

    reset_fakes();
    foc->mode.sys = halt_mode;
    foc->mode.halt = fault_mode;
    if (!expect_true(!drive_mode_is_supported(foc),
                     "故障停车由 Drive 故障链直接处理，不进入运行模式。") ||
        !expect_true(!drive_mode_step(foc),
                     "未接入的故障停车模式必须明确返回失败。"))
    {
        return false;
    }

    foc->mode.sys = debug_mode;
    for (index = 0U; index < (sizeof(debug_modes) / sizeof(debug_modes[0])); index++)
    {
        reset_fakes();
        foc->mode.debug = debug_modes[index];
        const bool mode_valid = drive_mode_step(foc);
        if (!expect_true(!mode_valid,
                         "未实现的调试模式必须明确返回失败。") ||
            !expect_true(event_count == 0U,
                         "未实现的调试模式不应调用其他实现。"))
        {
            return false;
        }
    }

    return true;
}

static bool test_unknown_system_mode_returns_failure(foc_t *foc)
{
    reset_fakes();
    foc->mode.sys = (sys_mode_e)99;

    return expect_true(!drive_mode_step(foc),
                       "未知系统模式必须明确返回失败。") &&
           expect_true(event_count == 0U,
                       "未知系统模式不应调用任何实现。");
}

int main(void)
{
    foc_t foc = {0};

    foc.ref.vd = 1.1f;
    foc.ref.vq = 1.2f;
    foc.ref.id = 1.3f;
    foc.ref.iq = 1.4f;
    foc.ref.spd_r = 1.5f;
    foc.ref.pos_r = 1.6f;
    foc.fb.theta_e = 1.7f;

    if (!test_release_modes(&foc))
    {
        return 1;
    }

    if (!test_halt_modes(&foc))
    {
        return 2;
    }

    if (!test_debug_modes(&foc))
    {
        return 3;
    }

    if (!test_diagnostic_mode(&foc))
    {
        return 4;
    }

    if (!test_unimplemented_modes_do_nothing(&foc))
    {
        return 5;
    }

    if (!test_unknown_system_mode_returns_failure(&foc))
    {
        return 6;
    }

    return 0;
}
