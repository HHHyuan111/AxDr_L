/**
 * @file mc_param_ident.c
 * @brief 电机参数辨识（分阶段）——平台无关核心实现
 *
 * 来源：suanfa 参考工程 G11-50 `app\User\motor_ident\motor_param_ident.c`
 * （1383 行，2026-10-05 迁入 AxDr_App diagnostic/core）。本文件相对原版
 * 的改动（其余数学与状态转移逐行保留）：
 *  1. 零全局：g_ident_ctx 单例与 g_algo_ops/g_last_sample/s_do_flux 三个
 *     static 全部收编为 mc_param_ident_t ctx（调用方持有，接口首参）。
 *  2. 接口换形：platform 8 回调（get_sample 拉 + set_* 推）改为家族
 *     step 推拉（mc_sample_t 入参 / mc_command_t 出参）；apply/on_finish
 *     等保留为 ctx->io 回调。详见 .h 头注释的四条语义差异。
 *  3. 依赖摘除：joint_attr.h 两宏（PWM 频率/极对数）改为 cfg 必填
 *     （control_period_s / pole_pairs）。
 *  4. Rs/磁链长和式升 double（万级样本累加精度，mc_rs_ident 先例）；
 *     L 回归统计量维持 float（与参考 Monte Carlo 基准的 f32 行为同构）。
 *
 * 辨识流程（全 standstill/自由旋转，无需机械锁轴）：
 *   FULL: IDLE→PREPARE→RS×2档→L(Ld)→FLUX_SPINUP→FLUX_SAMPLE→DONE/ERROR
 *   LQ:   IDLE→PREPARE→RS×2档→L(Ld)→LQ(Lq, 注入轴 d→q)→DONE/ERROR
 */

#include <math.h>
#include <string.h>

#include "mc_param_ident.h"

#define MC_IDENT_FOC_FAULT_DEBOUNCE_TICKS   (5u)
#define MC_IDENT_PREPARE_GRACE_TICKS        (50u)  /* PREPARE 建立期：使能/建流瞬态跳过电流安全检查 */
#define MC_IDENT_VBUS_LOW_DEBOUNCE_TICKS    (20u)
#define MC_IDENT_RS_READY_STREAK_TICKS      (40u)
#define MC_IDENT_RS_MIN_SETTLE_TIME_S       (0.10f)
#define MC_IDENT_FLUX_MIN_OMEGA_RAD_S       (5.0f)
#define MC_IDENT_RS_CURRENT_RAMP_A_PER_S    (50.0f)
#define MC_IDENT_FLUX_IQ_LIMIT_RATIO        (1.5f)
#define MC_IDENT_L_COMMAND_DELAY_TICKS      (1u)   /* 命令下一拍才进 PWM（双缓冲管线） */
#define MC_IDENT_L_MEASURE_DELAY_TICKS      (3u)
#define MC_IDENT_L_MIN_SAMPLES_PER_SIGN     (24u)
#define MC_IDENT_L_REG_DET_REL_MIN          (1.0e-5f)
/* L 采样允许最大电气转速：200eRPM（pp=5 时机械 40RPM；放宽以避免
 * 齿槽蠕动误判飞车，G11 原注释 pp=10 时机械 6RPM=61eRPM 过严）。 */
#define MC_IDENT_L_MAX_ELECTRICAL_RPM       (200.0f)
#define MC_IDENT_RS_MAX_CURRENT_CV          (0.05f)
#define MC_IDENT_FLUX_MIN_SPEED_RATIO       (0.70f)
#define MC_IDENT_FLUX_MAX_SPEED_RATIO       (1.30f)
#define MC_IDENT_FLUX_MAX_CV                (0.20f)
#define MC_IDENT_FLUX_MIN_SAMPLES           (200u)
#define MC_IDENT_FLUX_FILTER_SETTLE_S       (0.05f)
#define MC_IDENT_FLUX_SQRT_3_2              (1.2247448714f)
/* 死区压降 dq 几何换算：L 段锁 d 轴脉冲下回归解出的等效开关压降为
 * (4/3)·v_phase，而旋转基波电流矢量（近 q 轴）下逆变器扰动等效为
 * (4/π)·v_phase，两者之比 = 3/π ≈ 0.955。磁链修正用 L 段拟合值乘
 * 本系数；一阶近似残差约 3~4%（A5 死区补偿回流 foc 链后消除）。 */
#define MC_IDENT_FLUX_VDT_DQ_SCALE          (0.9549296586f)

static inline float clamp_f(float x, float lo, float hi);

/* =============================================================================
 * 内部辅助（全部 ctx 化）
 * ============================================================================*/

static inline uint32_t ticks_from_sec(const mc_param_ident_t *ctx, float sec)
{
    return (uint32_t)(sec / ctx->cfg.control_period_s + 0.5f);
}

static inline void reset_acc(mc_param_ident_t *ctx)
{
    ctx->acc = (mc_param_ident_acc_t){0};
}

static inline void reset_filters(mc_param_ident_t *ctx)
{
    ctx->filt = (mc_param_ident_filter_t){0};
}

static inline void reset_runtime_guards(mc_param_ident_t *ctx)
{
    ctx->rt.foc_fault_streak = 0u;
    ctx->rt.vbus_low_streak = 0u;
    ctx->rt.rs_ready_streak = 0u;
}

static void enter_state(mc_param_ident_t *ctx, mc_param_ident_state_t next)
{
    ctx->state = next;
    ctx->rt.tick = 0u;
    ctx->rt.toggle_tick = 0u;
    reset_runtime_guards(ctx);
}

/* 终态/停机命令：清全部输出并请求失能。 */
static void stop_command(mc_command_t *command)
{
    memset(command, 0, sizeof(*command));
    command->disable_request = true;
}

static inline float ident_flux_iq_limit(const mc_param_ident_t *ctx)
{
    float limit = ctx->rt.test_current_amp * MC_IDENT_FLUX_IQ_LIMIT_RATIO;
    const float hard_limit = ctx->cfg.current_limit_a * 0.5f;

    if (hard_limit > 0.0f && limit > hard_limit) limit = hard_limit;
    if (limit < 0.5f) limit = 0.5f;
    return limit;
}

/* 编码器闭环拖转外环：输出受辨识电流限幅约束的 iq，
 * 避免原电压开环法的失步/堵转。 */
static float ident_flux_speed_control(mc_param_ident_t *ctx,
                                      float target_mech_rad_s,
                                      const mc_sample_t *sample)
{
    const float iq_limit = ident_flux_iq_limit(ctx);
    const float omega_mech = sample->omega_mech_rad_s;
    float target_abs = fabsf(target_mech_rad_s);
    float kp, ki, err, iq_cmd;

    if (target_abs < 1.0f) target_abs = 1.0f;
    kp = 0.5f * iq_limit / target_abs;
    ki = 4.0f * kp; /* 外环积分时间常数约 250ms */
    err = target_mech_rad_s - omega_mech;
    ctx->rt.flux_iq_int += ki * err * ctx->rt.dt;
    ctx->rt.flux_iq_int = clamp_f(ctx->rt.flux_iq_int, -iq_limit, iq_limit);
    iq_cmd = clamp_f(kp * err + ctx->rt.flux_iq_int, -iq_limit, iq_limit);
    ctx->rt.flux_vq_cmd = sample->vq_v; /* 调试字段：实际 q 轴电压 */
    return iq_cmd;
}

static void write_fault_snapshot_fields(mc_param_ident_t *ctx,
                                        const mc_sample_t *sample)
{
    const mc_param_ident_l_quality_t *l_quality =
        (ctx->state == MC_PARAM_IDENT_LQ_SAMPLE) ? &ctx->quality.lq
                                                 : &ctx->quality.ld;

    ctx->fault_snapshot.state = ctx->state;
    ctx->fault_snapshot.tick = ctx->rt.tick;
    ctx->fault_snapshot.progress = ctx->rt.progress;
    ctx->fault_snapshot.align_v_cmd = ctx->rt.align_v_cmd;
    ctx->fault_snapshot.flux_vq_cmd = ctx->rt.flux_vq_cmd;
    ctx->fault_snapshot.sample_ia = sample->ia_a;
    ctx->fault_snapshot.sample_ib = sample->ib_a;
    ctx->fault_snapshot.sample_ic = sample->ic_a;
    ctx->fault_snapshot.sample_id = sample->id_a;
    ctx->fault_snapshot.sample_iq = sample->iq_a;
    ctx->fault_snapshot.sample_vd = sample->vd_v;
    ctx->fault_snapshot.sample_vq = sample->vq_v;
    ctx->fault_snapshot.sample_omega_mech_rad_s = sample->omega_mech_rad_s;
    ctx->fault_snapshot.sample_vbus = sample->vbus_v;
    ctx->fault_snapshot.filt_id = ctx->filt.id;
    ctx->fault_snapshot.filt_iq = ctx->filt.iq;
    ctx->fault_snapshot.filt_vd = ctx->filt.vd;
    ctx->fault_snapshot.filt_vq = ctx->filt.vq;
    ctx->fault_snapshot.rs_cnt = ctx->acc.rs_cnt;
    ctx->fault_snapshot.l_cnt = ctx->acc.l_cnt;
    ctx->fault_snapshot.flux_cnt = ctx->acc.flux_cnt;
    ctx->fault_snapshot.l_est_lo = l_quality->estimate_lo_h;
    ctx->fault_snapshot.l_est_hi = l_quality->estimate_hi_h;
    ctx->fault_snapshot.l_fit_r2 = l_quality->fit_r2;
    ctx->fault_snapshot.l_deadtime_v = l_quality->deadtime_drop_v;
    ctx->fault_snapshot.l_level_delta = l_quality->level_delta;
    ctx->fault_snapshot.l_quality_fail_mask = l_quality->fail_mask;
    ctx->fault_snapshot.l_reject_cnt = ctx->acc.l_reg.rejected;
    ctx->fault_snapshot.l_pos_cnt = ctx->acc.l_reg.positive;
    ctx->fault_snapshot.l_neg_cnt = ctx->acc.l_reg.negative;
    if (ctx->rt.l_level == 1u) {
        ctx->fault_snapshot.l_cnt += ctx->rt.l_reg_lo.accepted;
        ctx->fault_snapshot.l_reject_cnt += ctx->rt.l_reg_lo.rejected;
        ctx->fault_snapshot.l_pos_cnt += ctx->rt.l_reg_lo.positive;
        ctx->fault_snapshot.l_neg_cnt += ctx->rt.l_reg_lo.negative;
    }
}

static void update_fault_snapshot(mc_param_ident_t *ctx, const mc_sample_t *sample)
{
    if (!sample || ctx->fault_snapshot.valid) return;
    write_fault_snapshot_fields(ctx, sample);
}

/** 触发故障：记快照、发停机命令、回调 on_finish、进 ERROR（统一出口）。
 * 返回 false 供状态函数早退。 */
static bool abort_with_fault(mc_param_ident_t *ctx, mc_command_t *command,
                             mc_param_ident_fault_t code)
{
    ctx->result.fault_code = code;
    if (!ctx->fault_snapshot.valid) {
        write_fault_snapshot_fields(ctx, &ctx->last_sample);
        ctx->fault_snapshot.valid = 1u;
        ctx->fault_snapshot.fault_code = code;
    }
    stop_command(command);
    if (ctx->io.on_finish) {
        ctx->io.on_finish(false, code, ctx->io.user_ctx);
    }
    ctx->state = MC_PARAM_IDENT_ERROR;
    return false;
}

/** 成功收尾：停机命令、自动应用结果、on_finish(true)、进 DONE。 */
static void finish_success(mc_param_ident_t *ctx, mc_command_t *command)
{
    stop_command(command);
    if (ctx->io.apply_results) {
        ctx->io.apply_results(&ctx->result, ctx->io.user_ctx);
    }
    if (ctx->io.on_finish) {
        ctx->io.on_finish(true, MC_PARAM_IDENT_FAULT_NONE, ctx->io.user_ctx);
    }
    ctx->state = MC_PARAM_IDENT_DONE;
    ctx->rt.progress = 1000u;
}

/** 一阶低通滤波，alpha 限制在 [0.01, 0.5]。 */
static float apply_filter(const mc_param_ident_t *ctx, float prev, float input)
{
    float alpha = ctx->cfg.filter_alpha;
    if (alpha < 0.01f) alpha = 0.01f;
    if (alpha > 0.50f) alpha = 0.50f;
    return prev + alpha * (input - prev);
}

static void update_progress(mc_param_ident_t *ctx, uint32_t ticks,
                            uint32_t total, uint16_t base, uint16_t span)
{
    if (total == 0u) { ctx->rt.progress = base; return; }
    if (ticks > total) ticks = total;
    ctx->rt.progress = (uint16_t)(base + ((uint32_t)span * ticks) / total);
}

/** 相对误差校验：仅 nominal>0 且 tol>0 时生效（0=跳过）；超差触发故障。 */
static bool check_relative_error(mc_param_ident_t *ctx, mc_command_t *command,
                                 float value, float nominal, float tol,
                                 mc_param_ident_fault_t fault_code)
{
    if (nominal <= 0.0f || tol <= 0.0f) return true;
    if (fabsf(value - nominal) > (tol * nominal)) {
        return abort_with_fault(ctx, command, fault_code);
    }
    return true;
}

static inline float clamp_f(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static void ident_l_reg_merge(mc_param_ident_l_reg_t *dst,
                              const mc_param_ident_l_reg_t *src)
{
    dst->xx += src->xx;
    dst->xz += src->xz;
    dst->zz += src->zz;
    dst->xy += src->xy;
    dst->zy += src->zy;
    dst->yy += src->yy;
    dst->accepted += src->accepted;
    dst->rejected += src->rejected;
    dst->positive += src->positive;
    dst->negative += src->negative;
}

static float ident_std_from_sums(double sum, double sq_sum, uint32_t count)
{
    double mean, var;
    if (count < 2u) return 0.0f;
    mean = sum / (double)count;
    var = sq_sum / (double)count - mean * mean;
    return (var > 0.0) ? (float)sqrt(var) : 0.0f;
}

static bool ident_current_is_safe(mc_param_ident_t *ctx,
                                  const mc_sample_t *sample,
                                  mc_command_t *command)
{
    const float limit = ctx->cfg.current_limit_a;
    float max_i;
    if (limit <= 0.0f) return true;
    max_i = fmaxf(fabsf(sample->ia_a),
                  fmaxf(fabsf(sample->ib_a), fabsf(sample->ic_a)));
    if (max_i > limit || fabsf(sample->id_a) > limit || fabsf(sample->iq_a) > limit) {
        abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_OVERCURRENT);
        return false;
    }
    return true;
}

static inline bool ident_should_check_runtime_faults(const mc_param_ident_t *ctx)
{
    return (ctx->state != MC_PARAM_IDENT_PREPARE &&
            ctx->state != MC_PARAM_IDENT_DONE &&
            ctx->state != MC_PARAM_IDENT_ERROR &&
            ctx->state != MC_PARAM_IDENT_ABORTED);
}

/* 运行时故障监测：FOC 故障（含使能丢失，fault_code 承载）5 拍去抖、
 * 母线低压 20 拍去抖；RS_SETTLE 前 100/200 拍豁免建流瞬态。 */
static bool ident_runtime_faults_ok(mc_param_ident_t *ctx,
                                    const mc_sample_t *sample,
                                    mc_command_t *command)
{
    if (!ident_should_check_runtime_faults(ctx)) return true;

    const bool skip_foc_check = (ctx->state == MC_PARAM_IDENT_RS_SETTLE &&
                                 ctx->rt.tick < 100u);
    if (!skip_foc_check) {
        if (sample->fault_code != 0u) {
            if (ctx->rt.foc_fault_streak < 0xFFFFu) ctx->rt.foc_fault_streak++;
            if (ctx->rt.foc_fault_streak >= MC_IDENT_FOC_FAULT_DEBOUNCE_TICKS) {
                return abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_FOC_FAULT);
            }
        } else {
            ctx->rt.foc_fault_streak = 0u;
        }
    } else {
        ctx->rt.foc_fault_streak = 0u;
    }

    const bool skip_vbus = (ctx->state == MC_PARAM_IDENT_RS_SETTLE &&
                            ctx->rt.tick < 200u);
    if (!skip_vbus) {
        if (sample->vbus_v < ctx->cfg.vbus_min_v) {
            if (ctx->rt.vbus_low_streak < 0xFFFFu) ctx->rt.vbus_low_streak++;
            if (ctx->rt.vbus_low_streak >= MC_IDENT_VBUS_LOW_DEBOUNCE_TICKS) {
                return abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_VBUS_LOW);
            }
        } else {
            ctx->rt.vbus_low_streak = 0u;
        }
    } else {
        ctx->rt.vbus_low_streak = 0u;
    }

    return true;
}

/* =============================================================================
 * 纯估计器（公开实现，host 单测直用）
 * ============================================================================ */

mc_status_t mc_param_ident_solve_rs(float vd_lo_v, float id_lo_a,
                                    float vd_hi_v, float id_hi_a,
                                    float min_di_a, float *out_rs_ohm)
{
    const float dv = vd_hi_v - vd_lo_v;
    const float di = id_hi_a - id_lo_a;
    if (!out_rs_ohm) return MC_INVALID_ARGUMENT;
    if (!isfinite(dv) || !isfinite(di) || di <= min_di_a || dv <= 0.0f) {
        return MC_OUT_OF_RANGE;
    }
    *out_rs_ohm = dv / di;
    return MC_OK;
}

float mc_param_ident_sign_average(float i0_a, float i1_a)
{
    if (i0_a * i1_a < 0.0f) {
        const float denom = fabsf(i0_a) + fabsf(i1_a);
        return (denom > 1e-12f) ? ((i0_a + i1_a) / denom) : 0.0f;
    }
    if (i0_a > 0.0f || i1_a > 0.0f) return 1.0f;
    if (i0_a < 0.0f || i1_a < 0.0f) return -1.0f;
    return 0.0f;
}

void mc_param_ident_l_reg_add(mc_param_ident_l_reg_t *reg,
                              const mc_param_ident_l_gate_t *gate,
                              float u_cmd_v, float i0_a, float i1_a,
                              float i_avg_a, float sign_avg, float dt_meas_s)
{
    float di, x, y, z, l_inst;

    if (!reg || !gate || dt_meas_s <= 0.0f || !isfinite(u_cmd_v) ||
        !isfinite(i0_a) || !isfinite(i1_a) || !isfinite(i_avg_a) ||
        !isfinite(sign_avg)) {
        if (reg) reg->rejected++;
        return;
    }

    di = i1_a - i0_a;
    if (fabsf(di) < gate->min_di_a || u_cmd_v * di <= 0.0f) {
        reg->rejected++;
        return;
    }

    x = di / dt_meas_s;
    y = u_cmd_v; /* 调用方保证 Rs*i_avg 已被扣除（见 l_axis 窗口组装） */
    z = clamp_f(sign_avg, -1.0f, 1.0f);
    l_inst = fabsf(y / x);

    if (!isfinite(x) || !isfinite(y) || !isfinite(l_inst) || x * y <= 0.0f ||
        (gate->nominal_l > 0.0f &&
         (l_inst < 0.10f * gate->nominal_l ||
          l_inst > 10.0f * gate->nominal_l))) {
        reg->rejected++;
        return;
    }

    reg->xx += x * x;
    reg->xz += x * z;
    reg->zz += z * z;
    reg->xy += x * y;
    reg->zy += z * y;
    reg->yy += y * y;
    reg->accepted++;
    if (u_cmd_v > 0.0f) reg->positive++;
    else reg->negative++;
}

bool mc_param_ident_l_reg_solve(const mc_param_ident_l_reg_t *reg,
                                float *out_l_h, float *out_vdt_v, float *out_r2)
{
    float det, scale, l_est, vdt, sse, r2;

    if (!reg || reg->positive < MC_IDENT_L_MIN_SAMPLES_PER_SIGN ||
        reg->negative < MC_IDENT_L_MIN_SAMPLES_PER_SIGN || reg->yy <= 1e-12f) {
        return false;
    }

    det = reg->xx * reg->zz - reg->xz * reg->xz;
    scale = reg->xx * reg->zz;
    if (scale <= 1e-12f || det <= MC_IDENT_L_REG_DET_REL_MIN * scale) {
        return false;
    }

    l_est = (reg->xy * reg->zz - reg->zy * reg->xz) / det;
    vdt = (reg->zy * reg->xx - reg->xy * reg->xz) / det;
    sse = reg->yy - l_est * reg->xy - vdt * reg->zy;
    if (sse < 0.0f) sse = 0.0f; /* 浮点舍入 */
    r2 = 1.0f - sse / reg->yy;

    if (!isfinite(l_est) || !isfinite(vdt) || !isfinite(r2) || l_est <= 0.0f) {
        return false;
    }

    if (out_l_h) *out_l_h = l_est;
    if (out_vdt_v) *out_vdt_v = vdt;
    if (out_r2) *out_r2 = clamp_f(r2, 0.0f, 1.0f);
    return true;
}

bool mc_param_ident_l_reg_solve_level(const mc_param_ident_l_reg_t *reg,
                                      float switch_drop_v, float *out_l_h)
{
    float l_est;
    if (!reg || !out_l_h || reg->xx <= 1e-12f || !isfinite(switch_drop_v)) {
        return false;
    }
    /* 联合回归先确定共同开关压降，再在单激励档仅拟合 L：
     * 单档内 x 与 sign(i) 高度相关，独立二参数拟合近共线，
     * 不能用于档间一致性判断。 */
    l_est = (reg->xy - switch_drop_v * reg->xz) / reg->xx;
    if (!isfinite(l_est) || l_est <= 0.0f) return false;
    *out_l_h = l_est;
    return true;
}

bool mc_param_ident_eval_flux_sample(const mc_param_ident_flux_eval_input_t *in,
                                     float *out_flux_wb)
{
    float lambda;
    if (!in || !out_flux_wb) return false;
    if (fabsf(in->omega_e) <= MC_IDENT_FLUX_MIN_OMEGA_RAD_S) return false;

    /* 转子 dq 稳态模型：vq = Rs*iq + omega_e*(Ld*id + psi_f)。
     * 必须保留全部符号；使用绝对值会在反转/再生时产生系统性错误。
     * vq 取驱动回报的指令电压（电流环 PI 输出）：PI 为跟踪电流会自动
     * 顶掉逆变器死区压降，回报值因此含该分量，须按 deadtime_v·sign(iq)
     * 扣除，否则 λ̂ 带 +vdt/ωe 正偏置（deadtime_v=0 时退化为原式）。 */
    lambda = (in->vq - in->phase_resistance * in->iq
              - in->deadtime_v * ((in->iq >= 0.0f) ? 1.0f : -1.0f))
             / in->omega_e
             - in->phase_inductance * in->id;
    if (!isfinite(lambda) || lambda <= 0.0f) return false;

    *out_flux_wb = lambda;
    return true;
}

/* =============================================================================
 * 状态机各阶段
 * ============================================================================ */

static bool ident_state_prepare(mc_param_ident_t *ctx,
                                const mc_sample_t *sample,
                                mc_command_t *command)
{
    float align_current, ramp_step;

    /* 建立期容错：使能/openloop 建流瞬态（含 rs_current_cmd 起步斜坡）
     * 前 N 拍跳过电流安全检查，避免误触发 OVERCURRENT。 */
    if (ctx->rt.tick >= MC_IDENT_PREPARE_GRACE_TICKS) {
        if (!ident_current_is_safe(ctx, sample, command)) return false;
    }

    /* 固定电角度、闭环 Id 软启动锁转子；随后 Rs 低档从该电流平滑接管。 */
    align_current = fminf(ctx->rt.test_current_amp, ctx->cfg.rs_current_lo_a);
    ramp_step = MC_IDENT_RS_CURRENT_RAMP_A_PER_S * ctx->rt.dt;
    ctx->rt.rs_current_cmd += clamp_f(align_current - ctx->rt.rs_current_cmd,
                                      -ramp_step, ramp_step);
    command->mode = MC_CONTROL_CURRENT;
    command->id_ref_a = ctx->rt.rs_current_cmd;
    command->iq_ref_a = 0.0f;
    command->openloop_enable = true;
    ctx->rt.align_v_cmd = sample->vd_v;

    if (++ctx->rt.tick >= ticks_from_sec(ctx, ctx->cfg.align_time_s)) {
        enter_state(ctx, MC_PARAM_IDENT_RS_SETTLE);
    }
    update_progress(ctx, ctx->rt.tick, ticks_from_sec(ctx, ctx->cfg.align_time_s), 0u, 100u);
    return true;
}

static bool ident_state_rs_settle(mc_param_ident_t *ctx,
                                  const mc_sample_t *sample,
                                  mc_command_t *command)
{
    float target_i, ramp_step, i_err, v_ceiling;

    if (!ident_current_is_safe(ctx, sample, command)) return false;

    /* 电流闭环电阻标定：PI 建立两档稳态电流，读实际电压；
     * 两档斜率抵消死区/器件压降的常量部分。 */
    target_i = (ctx->rt.rs_level == 0u) ? ctx->cfg.rs_current_lo_a
                                        : ctx->cfg.rs_current_hi_a;
    if (target_i <= 0.0f) target_i = ctx->rt.test_current_amp;
    target_i = clamp_f(target_i, ctx->cfg.min_id_a, 0.8f * ctx->cfg.current_limit_a);
    ramp_step = MC_IDENT_RS_CURRENT_RAMP_A_PER_S * ctx->rt.dt;
    ctx->rt.rs_current_cmd += clamp_f(target_i - ctx->rt.rs_current_cmd,
                                      -ramp_step, ramp_step);
    command->mode = MC_CONTROL_CURRENT;
    command->id_ref_a = ctx->rt.rs_current_cmd;
    command->iq_ref_a = 0.0f;
    command->openloop_enable = true;
    ctx->rt.align_v_cmd = sample->vd_v;
    i_err = target_i - sample->id_a;

    {
        float stable_tol = target_i * 0.05f;
        if (stable_tol < 0.2f) stable_tol = 0.2f;
        if (fabsf(i_err) <= stable_tol) {
            if (ctx->rt.rs_ready_streak < 0xFFFFu) ctx->rt.rs_ready_streak++;
        } else {
            ctx->rt.rs_ready_streak = 0u;
        }
    }

    v_ceiling = fminf(ctx->cfg.align_voltage_v, 0.25f * ctx->cfg.voltage_limit_v);
    if (ctx->rt.tick > ticks_from_sec(ctx, MC_IDENT_RS_MIN_SETTLE_TIME_S) &&
        (sample->vd_v < -0.05f || fabsf(sample->vd_v) > v_ceiling)) {
        return abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_RS_OUT_OF_RANGE);
    }

    if (++ctx->rt.tick >= ticks_from_sec(ctx, ctx->cfg.rs_settle_s) ||
        (ctx->rt.tick >= ticks_from_sec(ctx, MC_IDENT_RS_MIN_SETTLE_TIME_S) &&
         ctx->rt.rs_ready_streak >= MC_IDENT_RS_READY_STREAK_TICKS)) {
        reset_acc(ctx);
        reset_filters(ctx);
        enter_state(ctx, MC_PARAM_IDENT_RS_SAMPLE);
    }
    update_progress(ctx, ctx->rt.tick, ticks_from_sec(ctx, ctx->cfg.rs_settle_s), 100u, 30u);
    return true;
}

static bool ident_state_rs_sample(mc_param_ident_t *ctx,
                                  const mc_sample_t *sample,
                                  mc_command_t *command)
{
    float target_i = (ctx->rt.rs_level == 0u) ? ctx->cfg.rs_current_lo_a
                                              : ctx->cfg.rs_current_hi_a;
    float id_m, vd_m, id_std;

    if (!ident_current_is_safe(ctx, sample, command)) return false;
    target_i = clamp_f(target_i, ctx->cfg.min_id_a, 0.8f * ctx->cfg.current_limit_a);
    command->mode = MC_CONTROL_CURRENT;
    command->id_ref_a = target_i;
    command->iq_ref_a = 0.0f;
    command->openloop_enable = true;
    ctx->rt.rs_current_cmd = target_i;
    ctx->rt.align_v_cmd = sample->vd_v;

    ctx->acc.rs_i_sum += (double)sample->id_a;
    ctx->acc.rs_v_sum += (double)sample->vd_v;
    ctx->acc.rs_i_sq_sum += (double)sample->id_a * (double)sample->id_a;
    ctx->acc.rs_cnt++;

    if (++ctx->rt.tick >= ticks_from_sec(ctx, ctx->cfg.rs_sample_s)) {
        /* 双平台 Rs：每级 (vd,id) 均值，两级斜率抵消共同压降。 */
        if (ctx->acc.rs_cnt < 16u) {
            abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_RS_NO_SAMPLE);
            return false;
        }
        vd_m = (float)(ctx->acc.rs_v_sum / (double)ctx->acc.rs_cnt);
        id_m = (float)(ctx->acc.rs_i_sum / (double)ctx->acc.rs_cnt);
        id_std = ident_std_from_sums(ctx->acc.rs_i_sum, ctx->acc.rs_i_sq_sum,
                                     ctx->acc.rs_cnt);

        if (id_m <= ctx->cfg.min_id_a || vd_m <= 0.0f) {
            if (ctx->rt.rs_retry_count < 1u) {
                ctx->rt.rs_retry_count++;
                reset_acc(ctx);
                reset_filters(ctx);
                enter_state(ctx, MC_PARAM_IDENT_RS_SETTLE);
                return true;
            }
            return abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_RS_NO_SAMPLE);
        }
        ctx->rt.rs_retry_count = 0u;

        if (id_std > fmaxf(0.10f, MC_IDENT_RS_MAX_CURRENT_CV * id_m)) {
            return abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_RS_UNSTABLE);
        }

        if (ctx->rt.rs_level == 0u) {
            /* 低电平级完成：缓存 (vd_lo,id_lo)，切高电平级重新建流。 */
            ctx->rt.rs_v_lvl0 = vd_m;
            ctx->rt.rs_i_lvl0 = id_m;
            ctx->rt.rs_std_lvl0 = id_std;
            ctx->quality.rs_current_lo_a = id_m;
            ctx->quality.rs_voltage_lo_v = vd_m;
            ctx->rt.rs_level = 1u;
            reset_acc(ctx);
            reset_filters(ctx);
            enter_state(ctx, MC_PARAM_IDENT_RS_SETTLE);
            return true;
        }

        /* 高电平级完成：差分得相电阻（死区压降在相减中消掉）。 */
        {
            const float commanded_di = ctx->cfg.rs_current_hi_a - ctx->cfg.rs_current_lo_a;
            const float min_di = fmaxf(ctx->cfg.min_id_a, 0.25f * fabsf(commanded_di));
            float rs = 0.0f;
            if (mc_param_ident_solve_rs(ctx->rt.rs_v_lvl0, ctx->rt.rs_i_lvl0,
                                        vd_m, id_m, min_di, &rs) != MC_OK) {
                return abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_RS_NO_SAMPLE);
            }
            ctx->result.phase_resistance = rs;
        }

        ctx->quality.rs_current_hi_a = id_m;
        ctx->quality.rs_voltage_hi_v = vd_m;
        ctx->quality.rs_current_std_max_a = fmaxf(id_std, ctx->rt.rs_std_lvl0);

        /* 平台残差直测死区：锁 d 轴稳态下 vd − Rs·id 即逆变器死区沿
         * 电流矢量方向的等效压降（锁轴几何），两档取平均压噪声。供磁链
         * 段修正（×3/π 换旋转几何）；不依赖 L 段回归拟合值——后者受
         * 梯形积分误差影响有 -5~-10% 系统偏差，仅作质量诊断输出。 */
        ctx->rt.vdt_platform_v = 0.5f *
            ((ctx->rt.rs_v_lvl0 - ctx->result.phase_resistance * ctx->rt.rs_i_lvl0)
             + (vd_m - ctx->result.phase_resistance * id_m));

        if (ctx->result.phase_resistance < ctx->cfg.r_min ||
            ctx->result.phase_resistance > ctx->cfg.r_max) {
            return abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_RS_OUT_OF_RANGE);
        }
        if (!check_relative_error(ctx, command, ctx->result.phase_resistance,
                                  ctx->cfg.nominal_r, ctx->cfg.tol_r,
                                  MC_PARAM_IDENT_FAULT_RS_DEVIATION)) {
            return false;
        }

        ctx->rt.l_level = 0u;
        ctx->rt.l_phase = 0u;
        ctx->rt.test_sign = 1;
        reset_acc(ctx);
        reset_filters(ctx);
        enter_state(ctx, MC_PARAM_IDENT_L_SAMPLE);
    }
    update_progress(ctx, ctx->rt.tick, ticks_from_sec(ctx, ctx->cfg.rs_sample_s), 130u, 250u);
    return true;
}

/* 公共 d/q 电感辨识器。返回 1=两档联合拟合完成，0=继续，-1=故障收尾。 */
static int ident_l_axis_process(mc_param_ident_t *ctx,
                                const mc_sample_t *sample,
                                mc_command_t *command,
                                bool q_axis, float *out_l)
{
    mc_param_ident_l_quality_t *quality = q_axis ? &ctx->quality.lq
                                                 : &ctx->quality.ld;
    const float axis_current = q_axis ? sample->iq_a : sample->id_a;
    const float axis_voltage = q_axis ? sample->vq_v : sample->vd_v;
    const uint32_t toggle_ticks = ticks_from_sec(ctx, ctx->cfg.l_toggle_s);
    const mc_param_ident_l_gate_t gate = {
        .min_di_a = ctx->cfg.min_di_a,
        .nominal_l = ctx->cfg.nominal_l,
    };
    float v_inj;

    if (!ident_current_is_safe(ctx, sample, command)) return -1;
    /* 停转判定延后到 settle 结束（l_phase>=1）：l_phase=0 是零电压退流/转子
     * 停转期，转子刚从 Rs 失锁（电流清零），齿槽/惯性残余转动属正常。 */
    if (ctx->rt.l_phase >= 1u) {
        const float pp = (ctx->cfg.pole_pairs < 1.0f) ? 1.0f : ctx->cfg.pole_pairs;
        const float omega_e = fabsf(sample->omega_mech_rad_s) * pp;
        if (omega_e > MC_IDENT_L_MAX_ELECTRICAL_RPM * (MC_TWO_PI_F / 60.0f)) {
            abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_LS_NOT_STANDSTILL);
            return -1;
        }
    }

    /* 输出：锁轴保持（继承 PREPARE 的 openloop 锁存角），电压模式注入。 */
    command->openloop_enable = true;

    if (ctx->rt.l_phase == 0u) {
        command->mode = MC_CONTROL_VOLTAGE;
        command->vd_ref_v = 0.0f;
        command->vq_ref_v = 0.0f;
        if (++ctx->rt.toggle_tick >= ticks_from_sec(ctx, ctx->cfg.l_settle_s)) {
            ctx->rt.l_phase = 1u;
            ctx->rt.tick = 0u;
            ctx->rt.toggle_tick = 0u;
            ctx->rt.test_sign = 1;
            reset_acc(ctx);
        }
        return 0;
    }

    v_inj = (ctx->rt.l_level == 0u) ? ctx->cfg.l_inject_lo_v
                                    : ctx->cfg.l_inject_hi_v;
    v_inj = clamp_f(v_inj, 0.05f, ctx->cfg.voltage_limit_v);

    if (ctx->rt.toggle_tick == 0u) {
        /* 新半周期：清窗口和，翻转注入极性。 */
        ctx->rt.l_current_sum = 0.0f;
        ctx->rt.l_sign_sum = 0.0f;
        ctx->rt.l_voltage_sum = 0.0f;
    }
    command->mode = MC_CONTROL_VOLTAGE;
    command->vd_ref_v = q_axis ? 0.0f : (float)ctx->rt.test_sign * v_inj;
    command->vq_ref_v = q_axis ? (float)ctx->rt.test_sign * v_inj : 0.0f;

    /* step 位于 foc 运行后：toggle_tick=1 时新命令才写入 PWM，该拍电流
     * 是正确窗口起点，不能把 toggle_tick=0 算进去。 */
    if (ctx->rt.toggle_tick == MC_IDENT_L_COMMAND_DELAY_TICKS) {
        ctx->rt.l_edge_current = axis_current;
        ctx->rt.l_prev_current = axis_current;
        ctx->rt.l_prev_voltage = axis_voltage;
    }

    if (ctx->rt.toggle_tick > MC_IDENT_L_COMMAND_DELAY_TICKS &&
        ctx->rt.toggle_tick <=
            (MC_IDENT_L_COMMAND_DELAY_TICKS + MC_IDENT_L_MEASURE_DELAY_TICKS)) {
        ctx->rt.l_current_sum += 0.5f * (ctx->rt.l_prev_current + axis_current);
        ctx->rt.l_sign_sum +=
            mc_param_ident_sign_average(ctx->rt.l_prev_current, axis_current);
        /* 区间 [k-1,k] 的实际电压是上一拍已写入的命令。 */
        ctx->rt.l_voltage_sum += ctx->rt.l_prev_voltage;
        ctx->rt.l_prev_current = axis_current;
        ctx->rt.l_prev_voltage = axis_voltage;
    }

    if (ctx->rt.toggle_tick ==
        (MC_IDENT_L_COMMAND_DELAY_TICKS + MC_IDENT_L_MEASURE_DELAY_TICKS)) {
        const float window_ticks = (float)MC_IDENT_L_MEASURE_DELAY_TICKS;
        const float u_eff = ctx->rt.l_voltage_sum / window_ticks
                            - ctx->result.phase_resistance *
                                  (ctx->rt.l_current_sum / window_ticks);
        mc_param_ident_l_reg_add(&ctx->acc.l_reg, &gate, u_eff,
                                 ctx->rt.l_edge_current, axis_current,
                                 ctx->rt.l_current_sum / window_ticks,
                                 ctx->rt.l_sign_sum / window_ticks,
                                 (float)MC_IDENT_L_MEASURE_DELAY_TICKS * ctx->rt.dt);
        ctx->acc.l_cnt = ctx->acc.l_reg.accepted;
    }

    if (++ctx->rt.toggle_tick >= toggle_ticks) {
        ctx->rt.toggle_tick = 0u;
        ctx->rt.test_sign = (int8_t)(-ctx->rt.test_sign);
    }

    if (++ctx->rt.tick >= ticks_from_sec(ctx, ctx->cfg.l_sample_s)) {
        if (ctx->acc.l_reg.positive < MC_IDENT_L_MIN_SAMPLES_PER_SIGN ||
            ctx->acc.l_reg.negative < MC_IDENT_L_MIN_SAMPLES_PER_SIGN) {
            quality->fail_mask = MC_PARAM_IDENT_L_QUALITY_FAIL_SAMPLE;
            quality->accepted = ctx->acc.l_reg.accepted;
            quality->rejected = ctx->acc.l_reg.rejected;
            quality->positive = ctx->acc.l_reg.positive;
            quality->negative = ctx->acc.l_reg.negative;
            abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_LS_NO_SAMPLE);
            return -1;
        }

        if (ctx->rt.l_level == 0u) {
            ctx->rt.l_reg_lo = ctx->acc.l_reg;
            ctx->rt.l_level = 1u;
            ctx->rt.l_phase = 0u;
            ctx->rt.tick = 0u;
            ctx->rt.toggle_tick = 0u;
            ctx->rt.test_sign = 1;
            command->vd_ref_v = 0.0f;
            command->vq_ref_v = 0.0f;
            reset_acc(ctx);
            return 0;
        }

        {
            mc_param_ident_l_reg_t combined = ctx->rt.l_reg_lo;
            float combined_l = 0.0f, combined_vdt = 0.0f, combined_r2 = 0.0f;
            float low_l = 0.0f, high_l = 0.0f;
            float mean_l, level_delta;
            uint8_t fail_mask = MC_PARAM_IDENT_L_QUALITY_FAIL_NONE;

            ident_l_reg_merge(&combined, &ctx->acc.l_reg);
            quality->accepted = combined.accepted;
            quality->rejected = combined.rejected;
            quality->positive = combined.positive;
            quality->negative = combined.negative;

            if (!mc_param_ident_l_reg_solve(&combined, &combined_l,
                                            &combined_vdt, &combined_r2)) {
                quality->fail_mask = MC_PARAM_IDENT_L_QUALITY_FAIL_SOLVE;
                abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_LS_FIT_QUALITY);
                return -1;
            }
            if (!mc_param_ident_l_reg_solve_level(&ctx->rt.l_reg_lo,
                                                  combined_vdt, &low_l) ||
                !mc_param_ident_l_reg_solve_level(&ctx->acc.l_reg,
                                                  combined_vdt, &high_l)) {
                quality->fail_mask = MC_PARAM_IDENT_L_QUALITY_FAIL_SOLVE;
                abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_LS_FIT_QUALITY);
                return -1;
            }

            quality->estimate_lo_h = low_l;
            quality->estimate_hi_h = high_l;
            quality->combined_h = combined_l;
            quality->fit_r2 = combined_r2;
            quality->deadtime_drop_v = combined_vdt;
            ctx->rt.l_est_lo = low_l;

            mean_l = 0.5f * (low_l + high_l);
            level_delta = fabsf(high_l - low_l) / fmaxf(mean_l, 1e-12f);
            quality->level_delta = level_delta;
            if (combined_r2 < ctx->cfg.l_fit_min_r2) {
                fail_mask |= MC_PARAM_IDENT_L_QUALITY_FAIL_R2;
            }
            if (level_delta > ctx->cfg.l_level_tolerance) {
                fail_mask |= MC_PARAM_IDENT_L_QUALITY_FAIL_LEVEL_DELTA;
            }
            if (fabsf(combined_vdt) > 0.95f * ctx->cfg.l_inject_lo_v) {
                fail_mask |= MC_PARAM_IDENT_L_QUALITY_FAIL_VSWITCH;
            }
            quality->fail_mask = fail_mask;
            if (fail_mask != MC_PARAM_IDENT_L_QUALITY_FAIL_NONE) {
                abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_LS_FIT_QUALITY);
                return -1;
            }
            if (combined_l < ctx->cfg.l_min || combined_l > ctx->cfg.l_max) {
                abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_LS_OUT_OF_RANGE);
                return -1;
            }
            *out_l = combined_l;
        }
        return 1;
    }

    return 0;
}

static bool ident_state_l_sample(mc_param_ident_t *ctx,
                                 const mc_sample_t *sample,
                                 mc_command_t *command)
{
    float ld = 0.0f;
    const int rc = ident_l_axis_process(ctx, sample, command, false, &ld);
    const uint16_t base = (ctx->rt.l_level == 0u) ? 380u : 455u;
    update_progress(ctx, ctx->rt.tick, ticks_from_sec(ctx, ctx->cfg.l_sample_s),
                    base, 75u);
    if (rc < 0) return false;
    if (rc == 0) return true;

    ctx->result.phase_inductance = ld;
    if (!check_relative_error(ctx, command, ld, ctx->cfg.nominal_l,
                              ctx->cfg.tol_l,
                              MC_PARAM_IDENT_FAULT_LS_DEVIATION)) {
        return false;
    }

    if (ctx->mode == MC_PARAM_IDENT_MODE_FULL) {
        /* 放开开环，转子自由旋转进入磁链拖转；过渡拍清零注入电压。 */
        command->vd_ref_v = 0.0f;
        command->vq_ref_v = 0.0f;
        command->openloop_enable = false;
        ctx->rt.flux_iq_int = 0.0f;
        ctx->rt.flux_speed_cmd = 0.0f;
        ctx->rt.flux_vq_cmd = 0.0f;
        enter_state(ctx, MC_PARAM_IDENT_FLUX_SPINUP);
    } else {
        ctx->rt.l_level = 0u;
        ctx->rt.l_phase = 0u;
        ctx->rt.test_sign = 1;
        reset_acc(ctx);
        enter_state(ctx, MC_PARAM_IDENT_LQ_SAMPLE);
    }
    return true;
}

static bool ident_state_lq_sample(mc_param_ident_t *ctx,
                                  const mc_sample_t *sample,
                                  mc_command_t *command)
{
    float lq = 0.0f;
    const int rc = ident_l_axis_process(ctx, sample, command, true, &lq);
    const uint16_t base = (ctx->rt.l_level == 0u) ? 530u : 605u;
    update_progress(ctx, ctx->rt.tick, ticks_from_sec(ctx, ctx->cfg.l_sample_s),
                    base, 75u);
    if (rc < 0) return false;
    if (rc == 0) return true;

    if (!check_relative_error(ctx, command, lq, ctx->cfg.nominal_l,
                              ctx->cfg.tol_l,
                              MC_PARAM_IDENT_FAULT_LS_DEVIATION)) {
        return false;
    }
    ctx->result.phase_inductance_q = lq;
    finish_success(ctx, command);
    return false; /* 已终结 */
}

static bool ident_state_flux_spinup(mc_param_ident_t *ctx,
                                    const mc_sample_t *sample,
                                    mc_command_t *command)
{
    const float pp = (ctx->cfg.pole_pairs < 1.0f) ? 1.0f : ctx->cfg.pole_pairs;
    const float target_omega_e = ctx->cfg.flux_target_erpm * (MC_TWO_PI_F / 60.0f);
    const uint32_t total = ticks_from_sec(ctx, ctx->cfg.flux_spinup_s);
    float ratio = (total > 0u) ? (float)(ctx->rt.tick + 1u) / (float)total : 1.0f;
    float iq_cmd;

    if (!ident_current_is_safe(ctx, sample, command)) return false;
    ratio = clamp_f(ratio, 0.0f, 1.0f);
    ctx->rt.flux_speed_cmd = (target_omega_e / pp) * ratio; /* 机械 rad/s 斜坡 */
    iq_cmd = ident_flux_speed_control(ctx, ctx->rt.flux_speed_cmd, sample);
    command->mode = MC_CONTROL_CURRENT;
    command->id_ref_a = 0.0f;
    command->iq_ref_a = iq_cmd;
    command->openloop_enable = false;

    /* 电压逼近上限时停止积分继续抬升，避免弱磁区/母线饱和污染磁链模型。 */
    if (fabsf(sample->vq_v) > ctx->cfg.flux_voltage_v) {
        ctx->rt.flux_iq_int *= 0.995f;
    }

    if (++ctx->rt.tick >= total) {
        reset_acc(ctx);
        reset_filters(ctx);
        enter_state(ctx, MC_PARAM_IDENT_FLUX_SAMPLE);
    }
    update_progress(ctx, ctx->rt.tick, total, 650u, 100u);
    return true;
}

static bool ident_state_flux_sample(mc_param_ident_t *ctx,
                                    const mc_sample_t *sample,
                                    mc_command_t *command)
{
    const float pp = (ctx->cfg.pole_pairs < 1.0f) ? 1.0f : ctx->cfg.pole_pairs;
    const float target_omega_e = ctx->cfg.flux_target_erpm * (MC_TWO_PI_F / 60.0f);
    float omega_e = sample->omega_mech_rad_s * pp;
    float speed_ratio, iq_cmd;

    if (!ident_current_is_safe(ctx, sample, command)) return false;
    ctx->rt.flux_speed_cmd = target_omega_e / pp;
    iq_cmd = ident_flux_speed_control(ctx, ctx->rt.flux_speed_cmd, sample);
    command->mode = MC_CONTROL_CURRENT;
    command->id_ref_a = 0.0f;
    command->iq_ref_a = iq_cmd;
    command->openloop_enable = false;

    ctx->filt.id = apply_filter(ctx, ctx->filt.id, sample->id_a);
    ctx->filt.iq = apply_filter(ctx, ctx->filt.iq, sample->iq_a);
    ctx->filt.vq = apply_filter(ctx, ctx->filt.vq, sample->vq_v);
    speed_ratio = (target_omega_e > 1e-6f) ? (omega_e / target_omega_e) : 0.0f;

    if (ctx->rt.tick >= ticks_from_sec(ctx, MC_IDENT_FLUX_FILTER_SETTLE_S) &&
        speed_ratio >= MC_IDENT_FLUX_MIN_SPEED_RATIO &&
        speed_ratio <= MC_IDENT_FLUX_MAX_SPEED_RATIO &&
        fabsf(ctx->filt.vq) <= ctx->cfg.flux_voltage_v) {
        float flux_sample = 0.0f;
        mc_param_ident_flux_eval_input_t flux_in = {
            .vq = ctx->filt.vq,
            .iq = ctx->filt.iq,
            .id = ctx->filt.id,
            .omega_e = omega_e,
            .phase_resistance = ctx->result.phase_resistance,
            .phase_inductance = ctx->result.phase_inductance,
            /* Rs 平台残差直测的死区压降（锁轴几何），按 dq 几何换算
             * 到旋转 q 轴；FLUX 段 iq 恒号，等价于扣除常数偏置。 */
            .deadtime_v = ctx->rt.vdt_platform_v
                          * MC_IDENT_FLUX_VDT_DQ_SCALE,
        };
        const mc_param_ident_algo_ops_t *ops = &ctx->algo_ops;
        bool (*eval)(const mc_param_ident_flux_eval_input_t *, float *) =
            ops->eval_flux_sample ? ops->eval_flux_sample
                                  : mc_param_ident_eval_flux_sample;
        if (eval(&flux_in, &flux_sample)) {
            ctx->acc.flux_acc += (double)flux_sample;
            ctx->acc.flux_sq_sum += (double)flux_sample * (double)flux_sample;
            ctx->acc.flux_speed_sum += (double)omega_e;
            ctx->acc.flux_cnt++;
        }
    }

    if (++ctx->rt.tick >= ticks_from_sec(ctx, ctx->cfg.flux_sample_s)) {
        if (ctx->acc.flux_cnt < MC_IDENT_FLUX_MIN_SAMPLES) {
            if (ctx->rt.flux_retry_count < 1u) {
                ctx->rt.flux_retry_count++;
                ctx->rt.flux_vq_cmd = 0.0f;
                ctx->rt.flux_iq_int = 0.0f;
                ctx->rt.flux_speed_cmd = 0.0f;
                reset_acc(ctx);
                reset_filters(ctx);
                enter_state(ctx, MC_PARAM_IDENT_FLUX_SPINUP);
                return true;
            }
            return abort_with_fault(ctx, command,
                                    MC_PARAM_IDENT_FAULT_FLUX_SPEED_NOT_REACHED);
        }

        ctx->rt.flux_retry_count = 0u;
        ctx->result.flux_linkage_wb =
            (float)(ctx->acc.flux_acc / (double)ctx->acc.flux_cnt);
        ctx->quality.flux_std_wb = ident_std_from_sums(ctx->acc.flux_acc,
                                                       ctx->acc.flux_sq_sum,
                                                       ctx->acc.flux_cnt);
        ctx->quality.flux_speed_mean_erpm =
            (float)(ctx->acc.flux_speed_sum / (double)ctx->acc.flux_cnt) *
            (float)(60.0 / MC_TWO_PI_F);

        if (ctx->quality.flux_std_wb >
            MC_IDENT_FLUX_MAX_CV * ctx->result.flux_linkage_wb) {
            return abort_with_fault(ctx, command,
                                    MC_PARAM_IDENT_FAULT_FLUX_NO_SAMPLE);
        }

        {
            /* Ke 线线RMS/机械rad/s 约定（与 foc.cfg.motor.flux 换算互逆）。 */
            ctx->result.back_emf_v_per_rad =
                ctx->result.flux_linkage_wb * pp * MC_IDENT_FLUX_SQRT_3_2;
            ctx->result.torque_kt_nm_a =
                1.5f * pp * ctx->result.flux_linkage_wb;
        }

        if (!check_relative_error(ctx, command, ctx->result.flux_linkage_wb,
                                  ctx->cfg.nominal_flux, ctx->cfg.tol_flux,
                                  MC_PARAM_IDENT_FAULT_FLUX_DEVIATION)) {
            return false;
        }
        if (!check_relative_error(ctx, command, ctx->result.back_emf_v_per_rad,
                                  ctx->cfg.nominal_ke, ctx->cfg.tol_ke,
                                  MC_PARAM_IDENT_FAULT_KE_DEVIATION)) {
            return false;
        }
        if (!check_relative_error(ctx, command, ctx->result.torque_kt_nm_a,
                                  ctx->cfg.nominal_kt, ctx->cfg.tol_kt,
                                  MC_PARAM_IDENT_FAULT_KT_DEVIATION)) {
            return false;
        }

        finish_success(ctx, command);
        return false; /* 已终结 */
    }
    update_progress(ctx, ctx->rt.tick, ticks_from_sec(ctx, ctx->cfg.flux_sample_s), 750u, 200u);
    return true;
}

static bool ident_run_current_state(mc_param_ident_t *ctx,
                                    const mc_sample_t *sample,
                                    mc_command_t *command)
{
    switch (ctx->state) {
    case MC_PARAM_IDENT_PREPARE:
        return ident_state_prepare(ctx, sample, command);
    case MC_PARAM_IDENT_RS_SETTLE:
        return ident_state_rs_settle(ctx, sample, command);
    case MC_PARAM_IDENT_RS_SAMPLE:
        return ident_state_rs_sample(ctx, sample, command);
    case MC_PARAM_IDENT_L_SAMPLE:
        return ident_state_l_sample(ctx, sample, command);
    case MC_PARAM_IDENT_LQ_SAMPLE:
        return ident_state_lq_sample(ctx, sample, command);
    case MC_PARAM_IDENT_FLUX_SPINUP:
        return ident_state_flux_spinup(ctx, sample, command);
    case MC_PARAM_IDENT_FLUX_SAMPLE:
        return ident_state_flux_sample(ctx, sample, command);
    default:
        return true;
    }
}

/* =============================================================================
 * 公共接口
 * ============================================================================ */

void mc_param_ident_default_config(mc_param_ident_config_t *cfg)
{
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));

    /* 算法层：G11 原值（时序/门限）。 */
    cfg->align_time_s = 0.5f;
    cfg->rs_settle_s = 0.5f;
    cfg->rs_sample_s = 0.5f;
    cfg->l_sample_s = 0.5f;
    cfg->l_toggle_s = 0.3e-3f;       /* L 方波半周期：电流呈三角波(峰值有界) */
    cfg->l_settle_s = 0.010f;        /* 每档先退流约 8 个电气时间常数 */
    cfg->flux_spinup_s = 1.0f;
    cfg->flux_sample_s = 1.0f;
    cfg->min_id_a = 0.1f;
    cfg->min_di_a = 0.05f;
    cfg->r_min = 0.001f;
    cfg->r_max = 10.0f;
    cfg->l_min = 1e-7f;
    cfg->l_max = 0.1f;
    cfg->l_fit_min_r2 = 0.80f;
    cfg->l_level_tolerance = 0.35f;
    cfg->filter_alpha = 0.10f;
    cfg->vbus_min_v = 1.0f;          /* 保守阈值：仅检测完全断电 */

    /* 激励层：沉沙电机档案（5 对极；平台电流指令限 8A，
     * G11 的 Rs 4/10A 平台超限，改 2/4A）。 */
    cfg->pole_pairs = 5.0f;
    cfg->test_current_a = 3.0f;
    cfg->align_voltage_v = 3.0f;
    cfg->rs_current_lo_a = 2.0f;
    cfg->rs_current_hi_a = 4.0f;
    cfg->l_inject_lo_v = 1.5f;
    cfg->l_inject_hi_v = 3.0f;
    cfg->flux_voltage_v = 3.0f;
    cfg->flux_target_erpm = 900.0f;  /* 沉沙 3000rpm 额定：180rpm 机械 */

    /* 安全闸门：0=未确认实物试验条件，禁止启动（start 强制校验，
     * 家族 mc_rs_ident 的 safe_current_limit 同规）。 */
    cfg->current_limit_a = 0.0f;
    cfg->voltage_limit_v = 0.0f;

    /* 标称值与容差：待沉沙首次辨识后回填；0=不检查偏差。 */
}

void mc_param_ident_attach(mc_param_ident_t *ctx, const mc_param_ident_io_t *io)
{
    if (!ctx) return;
    if (io) {
        ctx->io = *io;
    } else {
        memset(&ctx->io, 0, sizeof(ctx->io));
    }
}

mc_status_t mc_param_ident_start(mc_param_ident_t *ctx,
                                 const mc_param_ident_config_t *cfg,
                                 mc_param_ident_mode_t mode)
{
    mc_param_ident_config_t c;
    const float *fields;
    size_t i;

    if (!ctx) return MC_INVALID_ARGUMENT;
    if (ctx->state != MC_PARAM_IDENT_IDLE && ctx->state != MC_PARAM_IDENT_DONE &&
        ctx->state != MC_PARAM_IDENT_ERROR && ctx->state != MC_PARAM_IDENT_ABORTED) {
        return MC_REJECTED;
    }
    if (mode != MC_PARAM_IDENT_MODE_FULL && mode != MC_PARAM_IDENT_MODE_LQ) {
        return MC_INVALID_ARGUMENT;
    }

    if (cfg) {
        c = *cfg;
    } else {
        mc_param_ident_default_config(&c);
    }

    /* 全字段有限性（NaN 与任何数比较恒假，纯范围检查挡不住——14 号数值
     * 政策；闸门字段随后的 >0 校验同时覆盖 NaN）。结构体全 float 无
     * 混排成员，可整块扫描。 */
    fields = (const float *)&c;
    for (i = 0u; i < sizeof(c) / sizeof(float); ++i) {
        if (!mc_float_is_finite(fields[i])) return MC_INVALID_ARGUMENT;
    }

    /* 安全闸门：必须显式确认试验条件。 */
    if (!(c.control_period_s > 0.0f) || !(c.pole_pairs >= 1.0f) ||
        !(c.current_limit_a > 0.0f) || !(c.voltage_limit_v > 0.0f)) {
        return MC_INVALID_ARGUMENT;
    }

    /* 填充缺省（仅对未设置的字段生效，=0 代表使用默认值）。 */
#define SET_DEFAULT(field, dval) \
    if (c.field <= 0.0f) c.field = (dval)

    SET_DEFAULT(align_time_s,     0.5f);
    SET_DEFAULT(rs_settle_s,      0.5f);
    SET_DEFAULT(rs_sample_s,      0.5f);
    SET_DEFAULT(l_sample_s,       0.5f);
    SET_DEFAULT(l_toggle_s,       0.3e-3f);
    SET_DEFAULT(l_settle_s,       0.010f);
    SET_DEFAULT(test_current_a,   3.0f);
    SET_DEFAULT(align_voltage_v,  3.0f);
    SET_DEFAULT(min_id_a,         0.1f);
    SET_DEFAULT(min_di_a,         0.05f);
    SET_DEFAULT(r_min,            0.001f);
    SET_DEFAULT(r_max,            10.0f);
    SET_DEFAULT(l_min,            1e-7f);
    SET_DEFAULT(l_max,            0.1f);
    SET_DEFAULT(l_fit_min_r2,     0.80f);
    SET_DEFAULT(l_level_tolerance, 0.35f);
    SET_DEFAULT(filter_alpha,     0.10f);
    SET_DEFAULT(vbus_min_v,       1.0f);
    SET_DEFAULT(flux_voltage_v,   3.0f);
    SET_DEFAULT(flux_target_erpm, 900.0f);
    SET_DEFAULT(flux_spinup_s,    1.0f);
    SET_DEFAULT(flux_sample_s,    1.0f);

#undef SET_DEFAULT

    /* 参数一致性约束。 */
    c.flux_voltage_v = clamp_f(c.flux_voltage_v, 0.1f, c.voltage_limit_v);
    c.align_voltage_v = clamp_f(c.align_voltage_v, 0.1f, c.voltage_limit_v);
    c.rs_current_lo_a = clamp_f(c.rs_current_lo_a, 0.2f, 0.6f * c.current_limit_a);
    c.rs_current_hi_a = clamp_f(c.rs_current_hi_a, c.rs_current_lo_a + 0.5f,
                                0.8f * c.current_limit_a);
    c.l_inject_lo_v = clamp_f(c.l_inject_lo_v, 0.05f, 0.25f * c.voltage_limit_v);
    c.l_inject_hi_v = clamp_f(c.l_inject_hi_v, 1.25f * c.l_inject_lo_v,
                              0.40f * c.voltage_limit_v);
    c.l_fit_min_r2 = clamp_f(c.l_fit_min_r2, 0.10f, 0.999f);
    c.l_level_tolerance = clamp_f(c.l_level_tolerance, 0.05f, 1.0f);
    {
        const float min_toggle = (float)(MC_IDENT_L_COMMAND_DELAY_TICKS +
                                         MC_IDENT_L_MEASURE_DELAY_TICKS + 2u) *
                                 c.control_period_s;
        if (c.l_toggle_s < min_toggle) c.l_toggle_s = min_toggle;
    }

    /* 运行时清零。 */
    ctx->cfg = c;
    ctx->mode = mode;
    ctx->rt.test_current_amp = clamp_f(fabsf(c.test_current_a), 0.1f,
                                       c.current_limit_a);
    ctx->state = MC_PARAM_IDENT_PREPARE;
    ctx->rt.tick = 0u;
    ctx->rt.toggle_tick = 0u;
    ctx->rt.test_sign = 1;
    ctx->rt.dt = c.control_period_s;
    ctx->rt.align_v_cmd = 0.0f;
    ctx->rt.flux_vq_cmd = 0.0f;
    ctx->rt.openloop_angle = 0.0f;
    ctx->rt.progress = 0u;
    ctx->rt.rs_retry_count = 0u;
    ctx->rt.flux_retry_count = 0u;
    ctx->rt.rs_level = 0u;
    ctx->rt.l_level = 0u;
    ctx->rt.l_phase = 0u;
    ctx->rt.rs_current_cmd = 0.0f; /* 清上次残留，防 PREPARE 首拍灌入大电流 */
    reset_acc(ctx);
    reset_filters(ctx);
    reset_runtime_guards(ctx);
    memset(&ctx->fault_snapshot, 0, sizeof(ctx->fault_snapshot));
    memset(&ctx->quality, 0, sizeof(ctx->quality));
    memset(&ctx->last_sample, 0, sizeof(ctx->last_sample));
    memset(&ctx->result, 0, sizeof(ctx->result));
    return MC_OK;
}

mc_status_t mc_param_ident_step(mc_param_ident_t *ctx,
                                const mc_sample_t *sample,
                                mc_command_t *command)
{
    if (!ctx || !sample || !command) return MC_INVALID_ARGUMENT;

    switch (ctx->state) {
    case MC_PARAM_IDENT_IDLE:
        stop_command(command);
        return MC_REJECTED;
    case MC_PARAM_IDENT_DONE:
        stop_command(command);
        return MC_DONE;
    case MC_PARAM_IDENT_ERROR:
        stop_command(command);
        return MC_FAULT;
    case MC_PARAM_IDENT_ABORTED:
        stop_command(command);
        return MC_ABORTED;
    default:
        break;
    }

    /* 采样有限性防护（14 号数值政策）。 */
    if (!mc_float_is_finite(sample->ia_a) || !mc_float_is_finite(sample->ib_a) ||
        !mc_float_is_finite(sample->ic_a) || !mc_float_is_finite(sample->id_a) ||
        !mc_float_is_finite(sample->iq_a) || !mc_float_is_finite(sample->vd_v) ||
        !mc_float_is_finite(sample->vq_v) ||
        !mc_float_is_finite(sample->theta_elec_rad) ||
        !mc_float_is_finite(sample->omega_mech_rad_s) ||
        !mc_float_is_finite(sample->vbus_v)) {
        abort_with_fault(ctx, command, MC_PARAM_IDENT_FAULT_FOC_FAULT);
        return MC_NUMERIC_ERROR;
    }

    ctx->last_sample = *sample;
    update_fault_snapshot(ctx, sample);

    if (!ident_runtime_faults_ok(ctx, sample, command)) return MC_FAULT;

    /* PREPARE 首拍先锁存电角度（电角契约），命令基底才带得上它。 */
    if (ctx->state == MC_PARAM_IDENT_PREPARE && ctx->rt.tick == 0u) {
        ctx->rt.openloop_angle = sample->theta_elec_rad;
    }

    /* 命令基底：每拍全量重填（幂等）；辨识期间持续请求使能保活。 */
    memset(command, 0, sizeof(*command));
    command->enable_request = true;
    command->openloop_theta_e_rad = ctx->rt.openloop_angle;

    /* on_start 必须在任何命令输出被调度层应用之前发出（首拍）。 */
    if (ctx->state == MC_PARAM_IDENT_PREPARE && ctx->rt.tick == 0u &&
        ctx->io.on_start) {
        ctx->io.on_start(ctx->io.user_ctx);
    }

    (void)ident_run_current_state(ctx, sample, command);

    switch (ctx->state) {
    case MC_PARAM_IDENT_DONE:    return MC_DONE;
    case MC_PARAM_IDENT_ERROR:   return MC_FAULT;
    case MC_PARAM_IDENT_ABORTED: return MC_ABORTED;
    default:                     return MC_BUSY;
    }
}

mc_status_t mc_param_ident_abort(mc_param_ident_t *ctx, mc_command_t *command)
{
    if (!ctx || !command) return MC_INVALID_ARGUMENT;
    if (ctx->state == MC_PARAM_IDENT_IDLE || ctx->state == MC_PARAM_IDENT_DONE ||
        ctx->state == MC_PARAM_IDENT_ERROR ||
        ctx->state == MC_PARAM_IDENT_ABORTED) {
        stop_command(command);
        return MC_REJECTED;
    }
    stop_command(command);
    /* 主动中止非故障：不触发 apply_results（语义差异 2）。 */
    if (ctx->io.on_finish) {
        ctx->io.on_finish(false, ctx->result.fault_code, ctx->io.user_ctx);
    }
    ctx->state = MC_PARAM_IDENT_ABORTED;
    return MC_ABORTED;
}

mc_status_t mc_param_ident_apply(const mc_param_ident_t *ctx)
{
    if (!ctx) return MC_INVALID_ARGUMENT;
    if (!ctx->io.apply_results) return MC_REJECTED;
    ctx->io.apply_results(&ctx->result, ctx->io.user_ctx);
    return MC_OK;
}

bool mc_param_ident_save(const mc_param_ident_t *ctx)
{
    return (ctx && ctx->io.save) ? ctx->io.save(&ctx->result, ctx->io.user_ctx)
                                 : false;
}

bool mc_param_ident_load(mc_param_ident_t *ctx)
{
    mc_param_ident_result_t tmp;
    if (!ctx || !ctx->io.load) return false;
    memset(&tmp, 0, sizeof(tmp));
    if (!ctx->io.load(&tmp, ctx->io.user_ctx)) return false;
    ctx->result = tmp;
    return true;
}

mc_param_ident_state_t mc_param_ident_get_state(const mc_param_ident_t *ctx)
{
    return ctx ? ctx->state : MC_PARAM_IDENT_IDLE;
}

uint16_t mc_param_ident_get_progress_permille(const mc_param_ident_t *ctx)
{
    return ctx ? ctx->rt.progress : 0u;
}

const mc_param_ident_result_t *mc_param_ident_get_result(const mc_param_ident_t *ctx)
{
    return ctx ? &ctx->result : NULL;
}

void mc_param_ident_get_quality(const mc_param_ident_t *ctx,
                                mc_param_ident_quality_t *out_quality)
{
    if (ctx && out_quality) *out_quality = ctx->quality;
}

void mc_param_ident_get_fault_snapshot(const mc_param_ident_t *ctx,
                                       mc_param_ident_fault_snapshot_t *out_snapshot)
{
    if (ctx && out_snapshot) *out_snapshot = ctx->fault_snapshot;
}

void mc_param_ident_set_algo_ops(mc_param_ident_t *ctx,
                                 const mc_param_ident_algo_ops_t *ops)
{
    if (!ctx) return;
    if (!ops) {
        ctx->algo_ops.eval_flux_sample = mc_param_ident_eval_flux_sample;
        return;
    }
    ctx->algo_ops.eval_flux_sample = ops->eval_flux_sample
                                         ? ops->eval_flux_sample
                                         : mc_param_ident_eval_flux_sample;
}
