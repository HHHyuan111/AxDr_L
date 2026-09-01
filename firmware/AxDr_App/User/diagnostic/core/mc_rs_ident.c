#include "mc_rs_ident.h"

#include <math.h>
#include <string.h>

static void rs_zero_command(mc_command_t *command, bool request_disable)
{
    if (command == NULL) return;
    memset(command, 0, sizeof(*command));
    command->mode = MC_CONTROL_VOLTAGE;
    command->disable_request = request_disable;
}

static void rs_voltage_command(mc_command_t *command, float voltage_v)
{
    memset(command, 0, sizeof(*command));
    command->mode = MC_CONTROL_VOLTAGE;
    command->vd_ref_v = voltage_v;
    command->openloop_theta_e_rad = 0.0f;
    command->openloop_enable = true;
    command->enable_request = true;
}

static mc_status_t rs_fail(mc_rs_ident_t *ctx,
                           mc_rs_ident_fault_t fault,
                           mc_status_t status,
                           mc_command_t *command)
{
    ctx->fault = fault;
    ctx->state = MC_RS_IDENT_ERROR;
    ctx->result.valid = false;
    rs_zero_command(command, true);
    return status;
}

void mc_rs_ident_default_config(mc_rs_ident_config_t *cfg)
{
    if (cfg == NULL) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->current_min_a = 6.0f;
    cfg->current_max_a = 12.0f;
    cfg->point_count = MC_RS_IDENT_DEFAULT_POINTS;
    cfg->total_duration_s = 8.0f;
    cfg->settling_fraction = 0.75f;
    cfg->integral_gain_v_per_a_s = 2.0f;
    cfg->control_period_s = 1.0f / 20000.0f;
    /* Invalid until the target explicitly confirms a safe bridge/motor limit. */
    cfg->safe_current_limit_a = 0.0f;
    cfg->max_abs_voltage_v = 0.0f; /* source behavior: no software clamp */
    cfg->min_vbus_v = 0.0f;
    cfg->denominator_epsilon = 1e-9;
}

static bool rs_config_valid(const mc_rs_ident_config_t *cfg)
{
    if (cfg == NULL) return false;
    if (!mc_float_is_finite(cfg->current_min_a) || !mc_float_is_finite(cfg->current_max_a) ||
        !mc_float_is_finite(cfg->total_duration_s) ||
        !mc_float_is_finite(cfg->settling_fraction) ||
        !mc_float_is_finite(cfg->integral_gain_v_per_a_s) ||
        !mc_float_is_finite(cfg->control_period_s) ||
        !mc_float_is_finite(cfg->safe_current_limit_a) ||
        !mc_float_is_finite(cfg->max_abs_voltage_v) ||
        !mc_float_is_finite(cfg->min_vbus_v)) {
        return false;
    }
    if (cfg->current_min_a < 0.0f ||
        cfg->current_max_a <= cfg->current_min_a ||
        cfg->current_max_a > cfg->safe_current_limit_a ||
        cfg->safe_current_limit_a <= 0.0f) {
        return false;
    }
    if (cfg->point_count < 2u || cfg->point_count > MC_RS_IDENT_MAX_POINTS)
        return false;
    if (cfg->total_duration_s <= 0.0f || cfg->control_period_s <= 1e-9f)
        return false;
    if (cfg->settling_fraction < 0.0f || cfg->settling_fraction >= 1.0f)
        return false;
    if (cfg->integral_gain_v_per_a_s <= 0.0f ||
        cfg->max_abs_voltage_v < 0.0f || cfg->min_vbus_v < 0.0f)
        return false;
    if (!(cfg->denominator_epsilon > 0.0) ||
        !mc_float_is_finite(cfg->denominator_epsilon))
        return false;
    return true;
}

mc_status_t mc_rs_ident_start(mc_rs_ident_t *ctx,
                              const mc_rs_ident_config_t *cfg)
{
    uint32_t total_ticks;

    if (ctx == NULL || !rs_config_valid(cfg)) return MC_INVALID_ARGUMENT;

    total_ticks = (uint32_t)(cfg->total_duration_s / cfg->control_period_s);
    if (total_ticks < (uint32_t)cfg->point_count) return MC_OUT_OF_RANGE;

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->total_ticks = total_ticks;
    ctx->ticks_per_point = total_ticks / (uint32_t)cfg->point_count;
    if (ctx->ticks_per_point == 0u) {
        ctx->state = MC_RS_IDENT_ERROR;
        ctx->fault = MC_RS_IDENT_FAULT_BAD_CONFIG;
        return MC_OUT_OF_RANGE;
    }
    ctx->state = MC_RS_IDENT_RUNNING;
    ctx->fault = MC_RS_IDENT_FAULT_NONE;
    return MC_OK;
}

mc_status_t mc_rs_ident_solve(const float *voltage_avg_v,
                              const float *current_avg_a,
                              uint8_t point_count,
                              double denominator_epsilon,
                              float *phase_resistance_ohm)
{
    double numerator = 0.0;
    double denominator = 0.0;
    uint8_t i;
    uint8_t j;

    if (voltage_avg_v == NULL || current_avg_a == NULL ||
        phase_resistance_ohm == NULL || point_count < 2u ||
        point_count > MC_RS_IDENT_MAX_POINTS ||
        !(denominator_epsilon > 0.0)) {
        return MC_INVALID_ARGUMENT;
    }

    for (i = 0u; i < point_count; ++i) {
        if (!mc_float_is_finite(voltage_avg_v[i]) || !mc_float_is_finite(current_avg_a[i]))
            return MC_NUMERIC_ERROR;
        for (j = (uint8_t)(i + 1u); j < point_count; ++j) {
            const double dv = (double)voltage_avg_v[i] - voltage_avg_v[j];
            const double di = (double)current_avg_a[i] - current_avg_a[j];
            numerator += dv * di;
            denominator += di * di;
        }
    }

    if (denominator <= denominator_epsilon) return MC_OUT_OF_RANGE;
    *phase_resistance_ohm = (float)(numerator / denominator);
    return mc_float_is_finite(*phase_resistance_ohm) ? MC_OK : MC_NUMERIC_ERROR;
}

mc_status_t mc_rs_ident_step(mc_rs_ident_t *ctx,
                             const mc_sample_t *sample,
                             mc_command_t *command)
{
    uint32_t point;
    uint32_t local_tick;
    uint32_t sample_start;
    float target;
    mc_status_t solve_status;
    uint8_t i;

    if (ctx == NULL || sample == NULL || command == NULL)
        return MC_INVALID_ARGUMENT;
    if (ctx->state == MC_RS_IDENT_DONE) {
        rs_zero_command(command, true);
        return MC_DONE;
    }
    if (ctx->state == MC_RS_IDENT_ABORTED) {
        rs_zero_command(command, true);
        return MC_ABORTED;
    }
    if (ctx->state != MC_RS_IDENT_RUNNING)
        return rs_fail(ctx, MC_RS_IDENT_FAULT_BAD_CONFIG,
                       MC_REJECTED, command);

    if (sample->fault_code != 0u)
        return rs_fail(ctx, MC_RS_IDENT_FAULT_DRIVE, MC_FAULT, command);
    if (ctx->cfg.min_vbus_v > 0.0f && sample->vbus_v < ctx->cfg.min_vbus_v)
        return rs_fail(ctx, MC_RS_IDENT_FAULT_VBUS_LOW, MC_FAULT, command);
    if (sample->current_limit_a > 0.0f &&
        ctx->cfg.current_max_a > sample->current_limit_a)
        return rs_fail(ctx, MC_RS_IDENT_FAULT_CURRENT_LIMIT,
                       MC_OUT_OF_RANGE, command);
    if (!mc_float_is_finite(sample->ia_a))
        return rs_fail(ctx, MC_RS_IDENT_FAULT_NUMERIC,
                       MC_NUMERIC_ERROR, command);

    point = ctx->tick / ctx->ticks_per_point;
    if (point >= ctx->cfg.point_count) point = ctx->cfg.point_count - 1u;
    local_tick = ctx->tick - point * ctx->ticks_per_point;
    sample_start = (uint32_t)((float)ctx->ticks_per_point *
                              ctx->cfg.settling_fraction);
    target = ctx->cfg.current_min_a +
             (ctx->cfg.current_max_a - ctx->cfg.current_min_a) *
             ((float)point / (float)(ctx->cfg.point_count - 1u));
    ctx->current_target_a = target;

    ctx->voltage_integrator_v += ctx->cfg.integral_gain_v_per_a_s *
                                 ctx->cfg.control_period_s *
                                 (target - sample->ia_a);
    if (ctx->cfg.max_abs_voltage_v > 0.0f) {
        const float unclamped = ctx->voltage_integrator_v;
        ctx->voltage_integrator_v = mc_clampf(
            unclamped, -ctx->cfg.max_abs_voltage_v,
            ctx->cfg.max_abs_voltage_v);
        if (ctx->voltage_integrator_v != unclamped)
            ctx->result.voltage_was_clamped = true;
    }
    if (!mc_float_is_finite(ctx->voltage_integrator_v))
        return rs_fail(ctx, MC_RS_IDENT_FAULT_NUMERIC,
                       MC_NUMERIC_ERROR, command);

    rs_voltage_command(command, ctx->voltage_integrator_v);

    if (local_tick >= sample_start) {
        ctx->voltage_sum[point] += ctx->voltage_integrator_v;
        ctx->current_sum[point] += sample->ia_a;
        ctx->result.sample_count[point]++;
    }

    ctx->tick++;
    if (ctx->tick < ctx->total_ticks) return MC_BUSY;

    for (i = 0u; i < ctx->cfg.point_count; ++i) {
        if (ctx->result.sample_count[i] == 0u)
            return rs_fail(ctx, MC_RS_IDENT_FAULT_DEGENERATE,
                           MC_OUT_OF_RANGE, command);
        ctx->result.voltage_avg_v[i] =
            (float)(ctx->voltage_sum[i] / ctx->result.sample_count[i]);
        ctx->result.current_avg_a[i] =
            (float)(ctx->current_sum[i] / ctx->result.sample_count[i]);
    }
    ctx->result.point_count = ctx->cfg.point_count;
    solve_status = mc_rs_ident_solve(
        ctx->result.voltage_avg_v, ctx->result.current_avg_a,
        ctx->cfg.point_count, ctx->cfg.denominator_epsilon,
        &ctx->result.phase_resistance_ohm);
    if (solve_status != MC_OK) {
        return rs_fail(ctx,
                       solve_status == MC_NUMERIC_ERROR
                           ? MC_RS_IDENT_FAULT_NUMERIC
                           : MC_RS_IDENT_FAULT_DEGENERATE,
                       solve_status, command);
    }

    ctx->result.valid = true;
    ctx->state = MC_RS_IDENT_DONE;
    rs_zero_command(command, true);
    return MC_DONE;
}

mc_status_t mc_rs_ident_abort(mc_rs_ident_t *ctx, mc_command_t *command)
{
    if (ctx == NULL || command == NULL) return MC_INVALID_ARGUMENT;
    ctx->state = MC_RS_IDENT_ABORTED;
    ctx->result.valid = false;
    rs_zero_command(command, true);
    return MC_ABORTED;
}




