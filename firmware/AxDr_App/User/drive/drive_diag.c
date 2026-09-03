/**
 * @file drive_diag.c
 * @brief 把诊断算法的通用采样和命令接入现有电机控制链。
 *
 * 数据流：当前电机反馈 -> mc_sample_t -> 诊断算法 -> mc_command_t
 *        -> 现有 foc_cur_step/foc_volt_step -> drive_pwm_commit。
 */

#include "drive_diag.h"

#include <math.h>

#include "board_config.h"
#include "common.h"
#include "diag_runtime.h"
#include "drive_pwm.h"
#include "foc_core.h"

diag_runtime_t g_diag;

void drive_diag_init(const foc_t *foc)
{
    const encoder_data_t *encoder = (foc->enc.sensory1 == ENCODER_TYPE_MT6816)
        ? &foc->enc.mt6816
        : &foc->enc.ma732;
    const diag_seed_t seed = {
        .control_period_s = foc->rate.foc_ts,
        .phase_resistance_ohm = foc->motor.Rs,
        .d_axis_inductance_h = foc->motor.Ld,
        .q_axis_inductance_h = foc->motor.Lq,
        .flux_linkage_wb = foc->motor.flux,
        .pole_pairs = (uint32_t)foc->motor.pn,
        .encoder_full_scale = encoder->cpr,
        .encoder_direction = (int8_t)encoder->dir,
    };

    diag_runtime_init(&g_diag, &seed);
    g_diag.profile.current_limit_a = DRIVE_DIAG_CURRENT_LIMIT_A;
    g_diag.profile.voltage_limit_v = DRIVE_DIAG_VOLTAGE_LIMIT_V;
    g_diag.profile.minimum_vbus_v = foc->prot_cfg.under_voltage_v;
}

static _RAM_FUNC void drive_diag_build_sample(foc_t *foc,
                                               mc_sample_t *sample)
{
    const foc_sample_t foc_sample = {
        .i_a = foc->sig.i_a,
        .i_b = foc->sig.i_b,
        .i_c = foc->sig.i_c,
        .theta = foc->sig.p_e,
    };
    foc_frame_t frame;

    /* 这里重新计算本周期坐标变换，不读取上一拍 FOC 留下的 i_d/i_q。 */
    foc_core_prepare(&foc_sample, &frame);

    *sample = (mc_sample_t){
        .ia_a = foc->sig.i_a,
        .ib_a = foc->sig.i_b,
        .ic_a = foc->sig.i_c,
        .i_alpha_a = frame.i_alpha,
        .i_beta_a = frame.i_beta,
        .id_a = frame.i_d,
        .iq_a = frame.i_q,
        .vd_v = foc->sig.v_d,
        .vq_v = foc->sig.v_q,
        .theta_mech_rad = foc->sig.mp_r,
        .theta_elec_rad = foc->sig.p_e,
        .omega_mech_rad_s = foc->sig.wr_f,
        .vbus_v = foc->sig.vbus,
        .dt_s = foc->rate.foc_ts,
        .current_limit_a = g_diag.profile.current_limit_a,
        .encoder_raw = (uint32_t)foc->enc.raw_1,
        .encoder_full_scale = g_diag.profile.encoder_full_scale,
        .fault_code = foc->fault.all,
        .voltage_saturated = g_diag.voltage_saturated,
    };
}

static _RAM_FUNC bool drive_diag_voltage_is_allowed(
    const foc_t *foc,
    const mc_command_t *command)
{
    float voltage_v;

    if (command->mode == MC_CONTROL_VOLTAGE)
    {
        voltage_v = hypotf(command->vd_ref_v, command->vq_ref_v);
    }
    else
    {
        voltage_v = hypotf(foc->sig.v_d, foc->sig.v_q);
    }

    return isfinite(voltage_v)
        && (voltage_v <= g_diag.profile.voltage_limit_v);
}

static _RAM_FUNC void drive_diag_abort_task(void)
{
    mc_command_t stop_command;

    if (g_diag.active)
    {
        (void)diag_runtime_abort(&g_diag, &stop_command);
    }
}

_RAM_FUNC void drive_diag_poll_request(foc_t *foc)
{
    const diag_request_e request = (diag_request_e)g_diag.request;

    if (request == DIAG_REQUEST_NONE)
    {
        return;
    }

    g_diag.request = DIAG_REQUEST_NONE;
    if (request == DIAG_REQUEST_STOP)
    {
        drive_diag_abort_task();
        foc->req = DRIVE_REQ_STOP;
        return;
    }

    if ((request == DIAG_REQUEST_START)
        && !g_diag.active
        && !foc->pwm_active
        && (foc->state == DRIVE_STATE_STOP)
        && (foc->fault.all == 0U))
    {
        foc->mode.sys = calibrat_mode;
        foc->mode.calibrat = iden_pm;
        foc->req = DRIVE_REQ_START;
        return;
    }

    g_diag.last_status = MC_REJECTED;
}

bool drive_diag_is_supported(const foc_t *foc)
{
    return (foc->mode.sys == calibrat_mode)
        && (foc->mode.calibrat == iden_pm);
}

bool drive_diag_prepare(foc_t *foc)
{
    mc_status_t status;

    if (g_diag.active)
    {
        return true;
    }
    if (foc->pwm_active || (foc->state != DRIVE_STATE_STOP))
    {
        g_diag.last_status = MC_REJECTED;
        return false;
    }

    status = diag_runtime_start(&g_diag, foc->sig.vbus, true);
    g_diag.last_status = status;
    return status == MC_OK;
}

_RAM_FUNC bool drive_diag_step(foc_t *foc)
{
    mc_sample_t sample;
    mc_command_t command;
    mc_status_t status;
    float control_angle_rad;
    bool foc_valid;

    drive_diag_build_sample(foc, &sample);
    status = diag_runtime_step(&g_diag, &sample, &command);
    if ((status != MC_BUSY)
        || command.disable_request
        || !command.enable_request)
    {
        foc->req = DRIVE_REQ_STOP;
        return false;
    }

    control_angle_rad = command.openloop_enable
        ? command.openloop_theta_e_rad
        : foc->sig.p_e;

    switch (command.mode)
    {
        case MC_CONTROL_CURRENT:
            foc_valid = foc_cur_step(foc,
                                 command.id_ref_a,
                                 command.iq_ref_a,
                                 control_angle_rad);
            break;

        case MC_CONTROL_VOLTAGE:
            foc_valid = foc_volt_step(foc,
                                 command.vd_ref_v,
                                 command.vq_ref_v,
                                 control_angle_rad);
            break;

        case MC_CONTROL_IDLE:
        default:
            foc_valid = false;
            break;
    }

    if (!foc_valid || !drive_diag_voltage_is_allowed(foc, &command))
    {
        drive_diag_abort_task();
        foc->req = DRIVE_REQ_STOP;
        return false;
    }

    g_diag.voltage_saturated =
        (foc->sig.vs > 0.0f)
        && (hypotf(foc->sig.v_d, foc->sig.v_q) >= (0.999f * foc->sig.vs));

    if (!drive_pwm_commit(foc))
    {
        drive_diag_abort_task();
        foc->req = DRIVE_REQ_STOP;
        return false;
    }

    return true;
}

void drive_diag_on_stopped(void)
{
    diag_runtime_confirm_stopped(&g_diag, true);
}

void drive_diag_on_fault(void)
{
    drive_diag_abort_task();
}
