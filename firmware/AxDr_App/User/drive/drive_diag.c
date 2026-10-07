/**
 * @file drive_diag.c
 * @brief 把诊断算法的通用采样和命令接入现有电机控制链。
 *
 * 数据流：当前电机反馈 -> mc_sample_t -> 诊断算法 -> mc_command_t
 *        -> 现有 foc_cur_step/foc_volt_step -> drive_pwm_commit。
 */

#include "drive_diag.h"

#include <math.h>

#include <string.h>

#include "board_config.h"
#include "common.h"
#include "diag_runtime.h"
#include "drive_pwm.h"
#include "foc.h"
#include "foc_core.h"
#include "service_param_store.h"

diag_runtime_t g_diag;

/* 电机参数+电流环 PI 应用（辨识成功与 Flash 档案装载共用）。
 * 派生量按依赖顺序重算：Ls=(Ld+Lq)/2、Kt=1.5·pn·flux、div_Kt。
 * Lq/磁链 0 哨兵（FULL 模式未测字段）不覆盖现值。
 * 电流环 PI 按 cur_pi_init 同式直写（SET_CURRENT_PI 先例），kp 分轴
 * 用辨识 Ld/Lq（原式统一用 Ls），ki=Rs·ibw（连续增益），ibw 不动；
 * ibw>0 守卫：档案链若未建 ibw，防辨识成功反把 PI 增益清零。 */
static void drive_diag_ident_apply_motor(float rs_ohm,
                                         float ld_h,
                                         float lq_h,
                                         float flux_wb)
{
    g_foc.motor.Rs = rs_ohm;
    g_foc.motor.Ld = ld_h;
    if (lq_h > 0.0f)
    {
        g_foc.motor.Lq = lq_h;
    }
    g_foc.motor.Ls = 0.5f * (g_foc.motor.Ld + g_foc.motor.Lq);

    if (flux_wb > 0.0f)
    {
        g_foc.motor.flux = flux_wb;
        g_foc.motor.Kt = 1.5f * g_foc.motor.pn * g_foc.motor.flux;
        g_foc.motor.div_Kt = 1.0f / g_foc.motor.Kt;
    }

    if (g_foc.motor.ibw > 0.0f)
    {
        g_foc.id_pi.kp = g_foc.motor.Ld * g_foc.motor.ibw;
        g_foc.iq_pi.kp = g_foc.motor.Lq * g_foc.motor.ibw;
        g_foc.id_pi.ki = g_foc.motor.Rs * g_foc.motor.ibw;
        g_foc.iq_pi.ki = g_foc.motor.Rs * g_foc.motor.ibw;
    }
}

/* 参数辨识成功落 RAM（ISR 上下文，成功收尾拍，电机随即停机）。 */
static void drive_diag_ident_apply(const mc_param_ident_result_t *result,
                                   void *user_ctx)
{
    (void)user_ctx;
    drive_diag_ident_apply_motor(result->phase_resistance,
                                 result->phase_inductance,
                                 result->phase_inductance_q,
                                 result->flux_linkage_wb);
}

/* 辨识终态（ISR 上下文）：成功则把结果+质量摘要暂存待落盘；失败/中止
 * 不动旧档案。落盘（页 2 擦写）由主循环 service_param_store_ident_poll 执行。 */
static void drive_diag_ident_on_finish(bool success,
                                       mc_param_ident_fault_t fault_code,
                                       void *user_ctx)
{
    const mc_param_ident_result_t *result;
    mc_param_ident_quality_t quality;
    service_ident_data_t data;

    (void)fault_code;
    (void)user_ctx;
    if (!success)
    {
        return;
    }

    result = mc_param_ident_get_result(&g_diag.param_ident);
    mc_param_ident_get_quality(&g_diag.param_ident, &quality);

    memset(&data, 0, sizeof(data));
    data.rs_ohm = result->phase_resistance;
    data.ld_h = result->phase_inductance;
    data.lq_h = result->phase_inductance_q;
    data.flux_wb = result->flux_linkage_wb;
    data.ke_v_per_rad = result->back_emf_v_per_rad;
    data.kt_nm_a = result->torque_kt_nm_a;
    data.ld_r2 = quality.ld.fit_r2;
    data.lq_r2 = quality.lq.fit_r2;
    data.flux_std_wb = quality.flux_std_wb;
    data.deadtime_v = quality.ld.deadtime_drop_v;
    service_param_store_ident_stage(&data);
}

void drive_diag_ident_load(void)
{
    service_ident_data_t data;

    if (service_param_store_ident_read(&data))
    {
        drive_diag_ident_apply_motor(data.rs_ohm,
                                     data.ld_h,
                                     data.lq_h,
                                     data.flux_wb);
    }
}

void drive_diag_init(const foc_t *foc)
{
    /* 种子按主编码器类型取结构：ABZ 落 ma732 会带错标度/方向
     * （16384/+1 vs 真值 10000/-1），污染极对数/对齐等 raw 换算任务。 */
    const encoder_data_t *encoder = &foc->enc.ma732;
    if (foc->enc.primary == ENCODER_TYPE_MT6816)
    {
        encoder = &foc->enc.mt6816;
    }
    else if (foc->enc.primary == ENCODER_TYPE_ABZ)
    {
        encoder = &foc->enc.abz;
    }
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
         * 初始化契约）之后；load 回调留 NULL——档案装载由 boot 侧
         * drive_diag_ident_load 主动做（结果要落 g_foc.motor+PI，
         * 语义与核的手动 io.load 出参不同）。 */
        const mc_param_ident_io_t ident_io = {
            .apply_results = drive_diag_ident_apply,
            .on_finish = drive_diag_ident_on_finish,
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
