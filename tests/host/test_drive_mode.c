/**
 * @file test_drive_mode.c
 * @brief 在 Host 上验证生产 Drive 模式分派表。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

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
static bool foc_result_valid;
static bool arguments_ok;

static void reset_fakes(void)
{
    last_event = EVENT_MIT;
    event_count = 0U;
    commit_count = 0U;
    foc_result_valid = true;
    arguments_ok = true;
}

static void log_event(mode_event_e event)
{
    last_event = event;
    event_count++;
}

void pm_mit_mode(pmsm_t *pm)
{
    (void)pm;
    log_event(EVENT_MIT);
}

void pt_tor_mode(pmsm_t *pm)
{
    (void)pm;
    log_event(EVENT_TORQUE);
}

void pv_vel_mode(pmsm_t *pm)
{
    (void)pm;
    log_event(EVENT_VELOCITY);
}

void pp_pos_mode(pmsm_t *pm)
{
    (void)pm;
    log_event(EVENT_POSITION);
}

void cst_tor_mode(pmsm_t *pm)
{
    (void)pm;
    log_event(EVENT_CST);
}

void csv_vel_mode(pmsm_t *pm)
{
    (void)pm;
    log_event(EVENT_CSV);
}

void csp_pos_mode(pmsm_t *pm)
{
    (void)pm;
    log_event(EVENT_CSP);
}

void pmsm_quick_stop_mode(pmsm_t *pm)
{
    (void)pm;
    log_event(EVENT_QUICK_STOP);
}

void pmsm_fault_stop_mode(pmsm_t *pm)
{
    (void)pm;
    log_event(EVENT_FAULT_STOP);
}

void cali_mag_encoder(pmsm_t *pm)
{
    (void)pm;
    log_event(EVENT_ENCODER_CALIBRATION);
}

void iden_pmsm_first(idpm_t *idpm)
{
    arguments_ok = arguments_ok && (idpm != NULL);
    log_event(EVENT_IDENTIFICATION);
}

void anticogging_calibration(pmsm_t *pm)
{
    (void)pm;
    log_event(EVENT_ANTICOGGING);
}

void force_volt_mode(pmsm_t *pm)
{
    (void)pm;
    log_event(EVENT_FORCE_VOLTAGE);
}

void force_curr_mode(pmsm_t *pm)
{
    (void)pm;
    log_event(EVENT_FORCE_CURRENT);
}

bool foc_volt(pmsm_t *pm, float vd_ref, float vq_ref, float pos)
{
    arguments_ok = arguments_ok &&
                   (vd_ref == pm->ctrl.vd_set) &&
                   (vq_ref == pm->ctrl.vq_set) &&
                   (pos == pm->foc.p_e);
    log_event(EVENT_FOC_VOLTAGE);
    return foc_result_valid;
}

bool foc_curr(pmsm_t *pm, float id_ref, float iq_ref, float pos)
{
    arguments_ok = arguments_ok &&
                   (id_ref == pm->ctrl.id_set) &&
                   (iq_ref == pm->ctrl.iq_set) &&
                   (pos == pm->foc.p_e);
    log_event(EVENT_FOC_CURRENT);
    return foc_result_valid;
}

bool foc_vel(pmsm_t *pm, float vel_ref, float iq_ref, float pos)
{
    arguments_ok = arguments_ok &&
                   (vel_ref == pm->ctrl.wr_set) &&
                   (iq_ref == pm->ctrl.iq_set) &&
                   (pos == pm->foc.p_e);
    log_event(EVENT_FOC_VELOCITY);
    return foc_result_valid;
}

bool foc_pos(pmsm_t *pm,
             float pos_ref,
             float vel_ref,
             float iq_ref,
             float pos)
{
    arguments_ok = arguments_ok &&
                   (pos_ref == pm->ctrl.posr_set) &&
                   (vel_ref == pm->ctrl.wr_set) &&
                   (iq_ref == pm->ctrl.iq_set) &&
                   (pos == pm->foc.p_e);
    log_event(EVENT_FOC_POSITION);
    return foc_result_valid;
}

void drive_pwm_commit(pmsm_t *pm)
{
    (void)pm;
    commit_count++;
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

static bool expect_dispatch(pmsm_t *pm,
                            mode_event_e expected_event,
                            bool expected_commit)
{
    reset_fakes();
    drive_mode_step(pm);

    return expect_true(event_count == 1U, "模式应且只应调用一个实现。") &&
           expect_true(last_event == expected_event, "模式分派到错误的实现。") &&
           expect_true(commit_count == (expected_commit ? 1U : 0U),
                       "模式的 PWM 提交次数不正确。") &&
           expect_true(arguments_ok, "模式传给控制函数的参数不正确。");
}

static bool test_release_modes(pmsm_t *pm)
{
    static const mode_case_t cases[] = {
        {mit_mode, EVENT_MIT, false},
        {tor_mode, EVENT_TORQUE, false},
        {vel_mode, EVENT_VELOCITY, false},
        {pos_mode, EVENT_POSITION, false},
        {cst_mode, EVENT_CST, false},
        {csv_mode, EVENT_CSV, false},
        {csp_mode, EVENT_CSP, false}
    };
    size_t index;

    pm->mode.sys = release_mode;
    for (index = 0U; index < (sizeof(cases) / sizeof(cases[0])); index++)
    {
        pm->mode.release = (release_mode_e)cases[index].mode;
        if (!expect_dispatch(pm, cases[index].event, cases[index].commits_pwm))
        {
            return false;
        }
    }

    return true;
}

static bool test_halt_modes(pmsm_t *pm)
{
    pm->mode.sys = halt_mode;
    pm->mode.halt = quick_mode;
    if (!expect_dispatch(pm, EVENT_QUICK_STOP, false))
    {
        return false;
    }

    pm->mode.halt = fault_mode;
    return expect_dispatch(pm, EVENT_FAULT_STOP, false);
}

static bool test_calibration_modes(pmsm_t *pm)
{
    pm->mode.sys = calibrat_mode;
    pm->mode.calibrat = rotor_enc_cali;
    if (!expect_dispatch(pm, EVENT_ENCODER_CALIBRATION, false))
    {
        return false;
    }

    pm->mode.calibrat = iden_pm;
    if (!expect_dispatch(pm, EVENT_IDENTIFICATION, false))
    {
        return false;
    }

    pm->mode.calibrat = anticogging_pm;
    return expect_dispatch(pm, EVENT_ANTICOGGING, false);
}

static bool test_debug_modes(pmsm_t *pm)
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

    pm->mode.sys = debug_mode;
    for (index = 0U; index < (sizeof(cases) / sizeof(cases[0])); index++)
    {
        pm->mode.debug = (debug_mode_e)cases[index].mode;
        if (!expect_dispatch(pm, cases[index].event, cases[index].commits_pwm))
        {
            return false;
        }
    }

    reset_fakes();
    foc_result_valid = false;
    pm->mode.debug = volt_op;
    drive_mode_step(pm);

    return expect_true(event_count == 1U, "无效 FOC 结果仍应完成本模式计算。") &&
           expect_true(commit_count == 0U, "无效 FOC 结果不能提交 PWM。");
}

static bool test_unimplemented_modes_do_nothing(pmsm_t *pm)
{
    static const calibrat_mode_e calibration_modes[] = {
        rotor_enc_mod,
        output_enc_mod,
        output_enc_cali
    };
    static const debug_mode_e debug_modes[] = {
        spd_volt_cl,
        pos_spd_volt_cl
    };
    size_t index;

    pm->mode.sys = calibrat_mode;
    for (index = 0U;
         index < (sizeof(calibration_modes) / sizeof(calibration_modes[0]));
         index++)
    {
        reset_fakes();
        pm->mode.calibrat = calibration_modes[index];
        drive_mode_step(pm);
        if (!expect_true(event_count == 0U,
                         "未实现的标定模式应保持空操作。"))
        {
            return false;
        }
    }

    pm->mode.sys = debug_mode;
    for (index = 0U; index < (sizeof(debug_modes) / sizeof(debug_modes[0])); index++)
    {
        reset_fakes();
        pm->mode.debug = debug_modes[index];
        drive_mode_step(pm);
        if (!expect_true(event_count == 0U,
                         "未实现的调试模式应保持空操作。"))
        {
            return false;
        }
    }

    return true;
}

int main(void)
{
    pmsm_t pm = {0};

    pm.ctrl.vd_set = 1.1f;
    pm.ctrl.vq_set = 1.2f;
    pm.ctrl.id_set = 1.3f;
    pm.ctrl.iq_set = 1.4f;
    pm.ctrl.wr_set = 1.5f;
    pm.ctrl.posr_set = 1.6f;
    pm.foc.p_e = 1.7f;

    if (!test_release_modes(&pm))
    {
        return 1;
    }

    if (!test_halt_modes(&pm))
    {
        return 2;
    }

    if (!test_calibration_modes(&pm))
    {
        return 3;
    }

    if (!test_debug_modes(&pm))
    {
        return 4;
    }

    if (!test_unimplemented_modes_do_nothing(&pm))
    {
        return 5;
    }

    return 0;
}
