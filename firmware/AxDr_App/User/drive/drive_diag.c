/**
 * @file drive_diag.c
 * @brief 把诊断算法的通用采样和命令接入现有电机控制链。
 *
 * 数据流：当前电机反馈 -> mc_sample_t -> 诊断算法 -> mc_command_t
 *        -> 现有 foc_curr/foc_volt -> drive_pwm_commit。
 */

#include "drive_diag.h"

#include <math.h>

#include "common.h"
#include "drive_pwm.h"
#include "foc_core.h"

static _RAM_FUNC void drive_diag_build_sample(pmsm_t *pm,
                                               mc_sample_t *sample)
{
    const foc_sample_t foc_sample = {
        .i_a = pm->foc.i_a,
        .i_b = pm->foc.i_b,
        .i_c = pm->foc.i_c,
        .theta = pm->foc.p_e,
    };
    foc_frame_t frame;

    /* 这里重新计算本周期坐标变换，不读取上一拍 FOC 留下的 i_d/i_q。 */
    foc_core_prepare(&foc_sample, &frame);

    *sample = (mc_sample_t){
        .ia_a = pm->foc.i_a,
        .ib_a = pm->foc.i_b,
        .ic_a = pm->foc.i_c,
        .i_alpha_a = frame.i_alpha,
        .i_beta_a = frame.i_beta,
        .id_a = frame.i_d,
        .iq_a = frame.i_q,
        .vd_v = pm->foc.v_d,
        .vq_v = pm->foc.v_q,
        .theta_mech_rad = pm->foc.mp_r,
        .theta_elec_rad = pm->foc.p_e,
        .omega_mech_rad_s = pm->foc.wr_f,
        .vbus_v = pm->foc.vbus,
        .dt_s = pm->period.foc_ts,
        .current_limit_a = pm->diag.profile.current_limit_a,
        .encoder_raw = (uint32_t)pm->pos_box.raw_1,
        .encoder_full_scale = pm->diag.profile.encoder_full_scale,
        .fault_code = pm->fault.all,
        .voltage_saturated = pm->diag.voltage_saturated,
    };
}

static _RAM_FUNC bool drive_diag_voltage_is_allowed(
    const pmsm_t *pm,
    const mc_command_t *command)
{
    float voltage_v;

    if (command->mode == MC_CONTROL_VOLTAGE)
    {
        voltage_v = hypotf(command->vd_ref_v, command->vq_ref_v);
    }
    else
    {
        voltage_v = hypotf(pm->foc.v_d, pm->foc.v_q);
    }

    return isfinite(voltage_v)
        && (voltage_v <= pm->diag.profile.voltage_limit_v);
}

static _RAM_FUNC void drive_diag_abort(pmsm_t *pm)
{
    mc_command_t stop_command;

    if (pm->diag.active)
    {
        (void)diag_runtime_abort(&pm->diag, &stop_command);
    }
    pm->req = DRIVE_REQ_STOP;
}

_RAM_FUNC void drive_diag_poll_request(pmsm_t *pm)
{
    const diag_request_e request = (diag_request_e)pm->diag.request;

    if (request == DIAG_REQUEST_NONE)
    {
        return;
    }

    pm->diag.request = DIAG_REQUEST_NONE;
    if (request == DIAG_REQUEST_STOP)
    {
        drive_diag_abort(pm);
        return;
    }

    if ((request == DIAG_REQUEST_START)
        && !pm->diag.active
        && !pm->pwm_active
        && (pm->state == DRIVE_STATE_STOP)
        && (pm->fault.all == 0U))
    {
        pm->mode.sys = calibrat_mode;
        pm->mode.calibrat = iden_pm;
        pm->req = DRIVE_REQ_START;
        return;
    }

    pm->diag.last_status = MC_REJECTED;
}

bool drive_diag_is_supported(const pmsm_t *pm)
{
    return (pm->mode.sys == calibrat_mode)
        && (pm->mode.calibrat == iden_pm);
}

bool drive_diag_prepare(pmsm_t *pm)
{
    mc_status_t status;

    if (pm->diag.active)
    {
        return true;
    }
    if (pm->pwm_active || (pm->state != DRIVE_STATE_STOP))
    {
        pm->diag.last_status = MC_REJECTED;
        return false;
    }

    status = diag_runtime_start(&pm->diag, pm->foc.vbus, true);
    pm->diag.last_status = status;
    return status == MC_OK;
}

_RAM_FUNC bool drive_diag_step(pmsm_t *pm)
{
    mc_sample_t sample;
    mc_command_t command;
    mc_status_t status;
    float control_angle_rad;
    bool foc_valid;

    drive_diag_build_sample(pm, &sample);
    status = diag_runtime_step(&pm->diag, &sample, &command);
    if ((status != MC_BUSY)
        || command.disable_request
        || !command.enable_request)
    {
        pm->req = DRIVE_REQ_STOP;
        return false;
    }

    control_angle_rad = command.openloop_enable
        ? command.openloop_theta_e_rad
        : pm->foc.p_e;

    switch (command.mode)
    {
        case MC_CONTROL_CURRENT:
            foc_valid = foc_curr(pm,
                                 command.id_ref_a,
                                 command.iq_ref_a,
                                 control_angle_rad);
            break;

        case MC_CONTROL_VOLTAGE:
            foc_valid = foc_volt(pm,
                                 command.vd_ref_v,
                                 command.vq_ref_v,
                                 control_angle_rad);
            break;

        case MC_CONTROL_IDLE:
        default:
            foc_valid = false;
            break;
    }

    if (!foc_valid || !drive_diag_voltage_is_allowed(pm, &command))
    {
        drive_diag_abort(pm);
        return false;
    }

    pm->diag.voltage_saturated =
        (pm->foc.vs > 0.0f)
        && (hypotf(pm->foc.v_d, pm->foc.v_q) >= (0.999f * pm->foc.vs));

    if (!drive_pwm_commit(pm))
    {
        drive_diag_abort(pm);
        return false;
    }

    return true;
}

void drive_diag_on_stopped(pmsm_t *pm)
{
    diag_runtime_confirm_stopped(&pm->diag, true);
}

void drive_diag_on_fault(pmsm_t *pm)
{
    drive_diag_abort(pm);
}
