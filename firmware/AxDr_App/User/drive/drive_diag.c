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
#include "foc.h"
#include "foc_core.h"

diag_runtime_t g_diag;

/* 参数辨识成功落档（ISR 上下文，成功收尾拍，电机随即停机）。
 * 派生量按依赖顺序重算：Ls=(Ld+Lq)/2、Kt=1.5·pn·flux、div_Kt。
 * FULL 模式不测 Lq/磁链（result 留 0 哨兵），对应字段保持现值。
 * 电流环 PI 按 cur_pi_init 同式直写（SET_CURRENT_PI 先例），kp 分轴
 * 用辨识 Ld/Lq（原式统一用 Ls），ki=Rs·ibw（连续增益），ibw 不动；
 * 持久化（save/load）与遥测上报归 svc 段。 */
static void drive_diag_ident_apply(const mc_param_ident_result_t *result,
                                   void *user_ctx)
{
    (void)user_ctx;

    g_foc.motor.Rs = result->phase_resistance;
    g_foc.motor.Ld = result->phase_inductance;
    if (result->phase_inductance_q > 0.0f)
    {
        g_foc.motor.Lq = result->phase_inductance_q;
    }
    g_foc.motor.Ls = 0.5f * (g_foc.motor.Ld + g_foc.motor.Lq);

    if (result->flux_linkage_wb > 0.0f)
    {
        g_foc.motor.flux = result->flux_linkage_wb;
        g_foc.motor.Kt = 1.5f * g_foc.motor.pn * g_foc.motor.flux;
        g_foc.motor.div_Kt = 1.0f / g_foc.motor.Kt;
    }

    g_foc.id_pi.kp = g_foc.motor.Ld * g_foc.motor.ibw;
    g_foc.iq_pi.kp = g_foc.motor.Lq * g_foc.motor.ibw;
    g_foc.id_pi.ki = g_foc.motor.Rs * g_foc.motor.ibw;
    g_foc.iq_pi.ki = g_foc.motor.Rs * g_foc.motor.ibw;
}

void drive_diag_init(const foc_t *foc)
{
    const encoder_data_t *encoder = (foc->enc.primary == ENCODER_TYPE_MT6816)
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
    {
        /* attach 必须在 diag_runtime_init（memset 清 ctx，满足核的零
         * 初始化契约）之后；io 其余回调留 NULL（svc 段再接上报）。 */
        const mc_param_ident_io_t ident_io = {
            .apply_results = drive_diag_ident_apply,
        };
        mc_param_ident_attach(&g_diag.param_ident, &ident_io);
    }
    g_diag.profile.current_limit_a = DRIVE_DIAG_CURRENT_LIMIT_A;
    g_diag.profile.voltage_limit_v = DRIVE_DIAG_VOLTAGE_LIMIT_V;
    g_diag.profile.minimum_vbus_v = foc->prot_cfg.under_voltage_v;
}

static _RAM_FUNC void drive_diag_build_sample(foc_t *foc,
                                               mc_sample_t *sample)
{
    const foc_sample_t foc_sample = {
        .ia = foc->fb.ia,
        .ib = foc->fb.ib,
        .ic = foc->fb.ic,
        .theta = foc->fb.theta_e,
    };
    foc_frame_t frame;

    /* 这里重新计算本周期坐标变换，不读取上一拍 FOC 留下的 i_d/i_q。 */
    foc_core_prepare(&foc_sample, &frame);

    *sample = (mc_sample_t){
        .ia_a = foc->fb.ia,
        .ib_a = foc->fb.ib,
        .ic_a = foc->fb.ic,
        .i_alpha_a = frame.ialpha,
        .i_beta_a = frame.ibeta,
        .id_a = frame.id,
        .iq_a = frame.iq,
        .vd_v = foc->out.vd,
        .vq_v = foc->out.vq,
        .theta_mech_rad = foc->fb.pos_r,
        .theta_elec_rad = foc->fb.theta_e,
        .omega_mech_rad_s = foc->fb.spd_r,
        .vbus_v = foc->fb.vbus,
        .dt_s = foc->rate.foc_ts,
        .current_limit_a = g_diag.profile.current_limit_a,
        .encoder_raw = (uint32_t)foc->enc.raw,
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
        voltage_v = hypotf(foc->out.vd, foc->out.vq);
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

    status = diag_runtime_start(&g_diag, foc->fb.vbus, true);
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
        : foc->fb.theta_e;

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
        (foc->ref.v_lim > 0.0f)
        && (hypotf(foc->out.vd, foc->out.vq) >= (0.999f * foc->ref.v_lim));

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
