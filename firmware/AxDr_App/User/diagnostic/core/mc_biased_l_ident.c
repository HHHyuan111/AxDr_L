#include "mc_biased_l_ident.h"

#include <math.h>
#include <string.h>

static void l_zero_command(mc_command_t *command, bool request_disable)
{
    if (command == NULL) return;
    memset(command, 0, sizeof(*command));
    command->mode = MC_CONTROL_VOLTAGE;
    command->disable_request = request_disable;
}

static void l_voltage_command(mc_command_t *command, float voltage_v)
{
    memset(command, 0, sizeof(*command));
    command->mode = MC_CONTROL_VOLTAGE;
    command->vd_ref_v = voltage_v;
    command->openloop_theta_e_rad = 0.0f;
    command->openloop_enable = true;
    command->enable_request = true;
}

static mc_status_t l_fail(mc_biased_l_ident_t *ctx,
                          mc_biased_l_ident_fault_t fault,
                          mc_status_t status,
                          mc_command_t *command)
{
    ctx->fault = fault;
    ctx->state = MC_BIASED_L_IDENT_ERROR;
    ctx->result.valid = false;
    l_zero_command(command, true);
    return status;
}

void mc_biased_l_ident_default_config(mc_biased_l_ident_config_t *cfg)
{
    if (cfg == NULL) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->control_period_s = 1.0f / 20000.0f;
    cfg->auto_tune = true;
    cfg->bias_ratio = 0.03f;
    cfg->ripple_ratio = 0.25f;
    cfg->manual_bias_current_a = 0.0f;
    cfg->initial_injection_voltage_v = 1.0f;
    cfg->level_ticks = 10u;
    cfg->edge_skip_ticks = 3u;
    cfg->bias_settle_ticks = 2000u;
    cfg->bias_stable_ticks = 500u;
    cfg->bias_timeout_ticks = 100000u;
    cfg->bias_tolerance_ratio = 0.05f;
    cfg->bias_integral_gain_v_per_a_s = 2.0f;
    cfg->tune_pairs = 20u;
    cfg->target_accepted_pairs = 1000u;
    cfg->max_measure_pair_multiplier = 3u;
    cfg->min_bias_current_a = 0.5f;
    cfg->max_bias_current_limit_ratio = 0.4f;
    cfg->min_target_ripple_a = 0.1f;
    cfg->min_current_delta_a = 0.02f;
    cfg->min_positive_current_ratio = 0.2f;
    cfg->min_injection_voltage_v = 0.05f;
    cfg->max_injection_vbus_ratio = 0.10f;
    cfg->tune_scale_min = 0.75f;
    cfg->tune_scale_max = 1.25f;
    cfg->tune_ripple_floor_a = 0.02f;
    cfg->min_slope_difference_a_s = 1.0f;
    cfg->min_valid_inductance_h = 1e-6f;
    cfg->max_valid_inductance_h = 5e-3f;
    cfg->current_limit_a = 0.0f;
    cfg->voltage_limit_v = 0.0f;
    cfg->min_vbus_v = 0.0f;
    cfg->request_deadtime_compensation = true;
}

static bool l_config_valid(const mc_biased_l_ident_config_t *cfg)
{
    if (cfg == NULL) return false;
    if (!mc_float_is_finite(cfg->control_period_s) ||
        !mc_float_is_finite(cfg->bias_ratio) || !mc_float_is_finite(cfg->ripple_ratio) ||
        !mc_float_is_finite(cfg->manual_bias_current_a) ||
        !mc_float_is_finite(cfg->initial_injection_voltage_v) ||
        !mc_float_is_finite(cfg->bias_tolerance_ratio) ||
        !mc_float_is_finite(cfg->bias_integral_gain_v_per_a_s) ||
        !mc_float_is_finite(cfg->min_bias_current_a) ||
        !mc_float_is_finite(cfg->max_bias_current_limit_ratio) ||
        !mc_float_is_finite(cfg->min_target_ripple_a) ||
        !mc_float_is_finite(cfg->min_current_delta_a) ||
        !mc_float_is_finite(cfg->min_positive_current_ratio) ||
        !mc_float_is_finite(cfg->min_injection_voltage_v) ||
        !mc_float_is_finite(cfg->max_injection_vbus_ratio) ||
        !mc_float_is_finite(cfg->tune_scale_min) || !mc_float_is_finite(cfg->tune_scale_max) ||
        !mc_float_is_finite(cfg->tune_ripple_floor_a) ||
        !mc_float_is_finite(cfg->min_slope_difference_a_s) ||
        !mc_float_is_finite(cfg->min_valid_inductance_h) ||
        !mc_float_is_finite(cfg->max_valid_inductance_h) ||
        !mc_float_is_finite(cfg->current_limit_a) ||
        !mc_float_is_finite(cfg->voltage_limit_v) ||
        !mc_float_is_finite(cfg->min_vbus_v)) return false;

    if (cfg->control_period_s <= 1e-9f || cfg->current_limit_a <= 0.0f ||
        cfg->voltage_limit_v <= 0.0f || cfg->bias_ratio <= 0.0f ||
        cfg->bias_ratio > 0.5f || cfg->ripple_ratio <= 0.05f ||
        cfg->ripple_ratio > 0.5f) return false;
    if (cfg->level_ticks < 5u ||
        cfg->edge_skip_ticks >= (uint16_t)(cfg->level_ticks - 1u))
        return false;
    if (cfg->bias_stable_ticks == 0u ||
        cfg->bias_timeout_ticks <= cfg->bias_settle_ticks ||
        cfg->target_accepted_pairs == 0u ||
        cfg->max_measure_pair_multiplier == 0u) return false;
    if (cfg->bias_tolerance_ratio <= 0.0f ||
        cfg->bias_integral_gain_v_per_a_s <= 0.0f ||
        cfg->min_bias_current_a < 0.0f ||
        cfg->max_bias_current_limit_ratio <= 0.0f ||
        cfg->max_bias_current_limit_ratio > 1.0f ||
        cfg->min_target_ripple_a <= 0.0f ||
        cfg->min_current_delta_a <= 0.0f ||
        cfg->min_positive_current_ratio <= 0.0f ||
        cfg->min_positive_current_ratio >= 1.0f) return false;
    if (cfg->initial_injection_voltage_v <= 0.0f ||
        cfg->min_injection_voltage_v <= 0.0f ||
        cfg->max_injection_vbus_ratio <= 0.0f ||
        cfg->max_injection_vbus_ratio > 1.0f ||
        cfg->tune_scale_min <= 0.0f ||
        cfg->tune_scale_max < cfg->tune_scale_min ||
        cfg->tune_ripple_floor_a <= 0.0f) return false;
    if (cfg->min_slope_difference_a_s <= 0.0f ||
        cfg->min_valid_inductance_h <= 0.0f ||
        cfg->max_valid_inductance_h <= cfg->min_valid_inductance_h)
        return false;
    return true;
}

static float l_clamp_injection(const mc_biased_l_ident_t *ctx,
                               float injection_v,
                               float vbus_v)
{
    float maximum = ctx->cfg.max_injection_vbus_ratio * vbus_v;
    float bridge_headroom = ctx->cfg.voltage_limit_v - ctx->base_voltage_v;
    if (maximum < ctx->cfg.min_injection_voltage_v)
        maximum = ctx->cfg.min_injection_voltage_v;
    if (bridge_headroom < maximum) maximum = bridge_headroom;
    if (maximum < 0.0f) maximum = 0.0f;
    if (injection_v < ctx->cfg.min_injection_voltage_v)
        injection_v = ctx->cfg.min_injection_voltage_v;
    if (injection_v > maximum) injection_v = maximum;
    return injection_v;
}

mc_status_t mc_biased_l_ident_start(mc_biased_l_ident_t *ctx,
                                    const mc_biased_l_ident_config_t *cfg,
                                    float phase_resistance_ohm,
                                    float initial_vbus_v)
{
    float bias;

    if (ctx == NULL || !l_config_valid(cfg) ||
        !mc_float_is_finite(phase_resistance_ohm) || phase_resistance_ohm < 0.0f ||
        !mc_float_is_finite(initial_vbus_v) || initial_vbus_v <= 0.0f) {
        return MC_INVALID_ARGUMENT;
    }
    if (cfg->min_vbus_v > 0.0f && initial_vbus_v < cfg->min_vbus_v)
        return MC_OUT_OF_RANGE;

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->phase_resistance_ohm = phase_resistance_ohm;

    if (cfg->auto_tune || cfg->manual_bias_current_a <= 0.0f)
        bias = cfg->current_limit_a * cfg->bias_ratio;
    else
        bias = cfg->manual_bias_current_a;
    if (bias < cfg->min_bias_current_a) bias = cfg->min_bias_current_a;
    if (bias > cfg->max_bias_current_limit_ratio * cfg->current_limit_a)
        bias = cfg->max_bias_current_limit_ratio * cfg->current_limit_a;
    if (bias <= 0.0f || bias > cfg->current_limit_a)
        return MC_OUT_OF_RANGE;

    ctx->bias_target_a = bias;
    ctx->target_ripple_a = bias * cfg->ripple_ratio;
    if (ctx->target_ripple_a < cfg->min_target_ripple_a)
        ctx->target_ripple_a = cfg->min_target_ripple_a;
    ctx->base_voltage_v = phase_resistance_ohm * bias;
    if (ctx->base_voltage_v >= cfg->voltage_limit_v)
        return MC_OUT_OF_RANGE;
    ctx->injection_voltage_v = cfg->initial_injection_voltage_v;
    ctx->injection_voltage_v = l_clamp_injection(
        ctx, ctx->injection_voltage_v, initial_vbus_v);
    if (ctx->injection_voltage_v <= 0.0f ||
        ctx->base_voltage_v + ctx->injection_voltage_v >
            cfg->voltage_limit_v) return MC_OUT_OF_RANGE;

    ctx->level_sign = 1;
    ctx->state = MC_BIASED_L_IDENT_BIAS_SETTLE;
    ctx->fault = MC_BIASED_L_IDENT_FAULT_NONE;
    ctx->result.bias_current_target_a = bias;
    ctx->result.tuned_injection_voltage_v = ctx->injection_voltage_v;
    ctx->result.injection_frequency_hz =
        1.0f / (2.0f * (float)cfg->level_ticks * cfg->control_period_s);
    return MC_OK;
}

mc_status_t mc_biased_l_ident_estimate_pair(
    const mc_biased_l_pair_t *pair,
    float phase_resistance_ohm,
    float min_slope_difference_a_s,
    float *phase_inductance_h)
{
    float slope_difference;
    float voltage_difference;

    if (pair == NULL || phase_inductance_h == NULL ||
        !mc_float_is_finite(phase_resistance_ohm) || phase_resistance_ohm < 0.0f ||
        !mc_float_is_finite(min_slope_difference_a_s) ||
        min_slope_difference_a_s <= 0.0f) return MC_INVALID_ARGUMENT;
    if (!mc_float_is_finite(pair->positive_slope_a_s) ||
        !mc_float_is_finite(pair->negative_slope_a_s) ||
        !mc_float_is_finite(pair->positive_current_avg_a) ||
        !mc_float_is_finite(pair->negative_current_avg_a) ||
        !mc_float_is_finite(pair->positive_voltage_avg_v) ||
        !mc_float_is_finite(pair->negative_voltage_avg_v)) return MC_NUMERIC_ERROR;

    slope_difference = pair->positive_slope_a_s -
                       pair->negative_slope_a_s;
    if (slope_difference <= min_slope_difference_a_s)
        return MC_OUT_OF_RANGE;
    voltage_difference = pair->positive_voltage_avg_v -
                         pair->negative_voltage_avg_v -
                         phase_resistance_ohm *
                         (pair->positive_current_avg_a -
                          pair->negative_current_avg_a);
    *phase_inductance_h = fabsf(voltage_difference / slope_difference);
    return mc_float_is_finite(*phase_inductance_h) ? MC_OK : MC_NUMERIC_ERROR;
}

mc_status_t mc_biased_l_ident_step(mc_biased_l_ident_t *ctx,
                                   const mc_sample_t *sample,
                                   mc_command_t *command)
{
    const float current = sample != NULL ? sample->i_alpha_a : 0.0f;

    if (ctx == NULL || sample == NULL || command == NULL)
        return MC_INVALID_ARGUMENT;
    if (ctx->state == MC_BIASED_L_IDENT_DONE) {
        l_zero_command(command, true);
        return MC_DONE;
    }
    if (ctx->state == MC_BIASED_L_IDENT_ABORTED) {
        l_zero_command(command, true);
        return MC_ABORTED;
    }
    if (ctx->state != MC_BIASED_L_IDENT_BIAS_SETTLE &&
        ctx->state != MC_BIASED_L_IDENT_INJECT)
        return l_fail(ctx, MC_BIASED_L_IDENT_FAULT_BAD_CONFIG,
                      MC_REJECTED, command);
    if (sample->fault_code != 0u)
        return l_fail(ctx, MC_BIASED_L_IDENT_FAULT_DRIVE,
                      MC_FAULT, command);
    if (ctx->cfg.min_vbus_v > 0.0f && sample->vbus_v < ctx->cfg.min_vbus_v)
        return l_fail(ctx, MC_BIASED_L_IDENT_FAULT_VBUS_LOW,
                      MC_FAULT, command);
    if (sample->current_limit_a > 0.0f &&
        ctx->cfg.current_limit_a > sample->current_limit_a)
        return l_fail(ctx, MC_BIASED_L_IDENT_FAULT_CURRENT_LIMIT,
                      MC_OUT_OF_RANGE, command);
    if (!mc_float_is_finite(current) || !mc_float_is_finite(sample->vd_v) ||
        !mc_float_is_finite(sample->vbus_v))
        return l_fail(ctx, MC_BIASED_L_IDENT_FAULT_NUMERIC,
                      MC_NUMERIC_ERROR, command);

    if (ctx->state == MC_BIASED_L_IDENT_BIAS_SETTLE) {
        float base_max;
        ctx->base_voltage_v += ctx->cfg.bias_integral_gain_v_per_a_s *
                               ctx->cfg.control_period_s *
                               (ctx->bias_target_a - current);
        base_max = ctx->cfg.voltage_limit_v - ctx->injection_voltage_v;
        if (base_max < 0.0f) base_max = 0.0f;
        ctx->base_voltage_v = mc_clampf(ctx->base_voltage_v, 0.0f, base_max);
        l_voltage_command(command, ctx->base_voltage_v);

        if (fabsf(current - ctx->bias_target_a) <=
            ctx->cfg.bias_tolerance_ratio * ctx->bias_target_a)
            ctx->stable_tick++;
        else
            ctx->stable_tick = 0u;

        if (ctx->settle_tick >= ctx->cfg.bias_settle_ticks &&
            ctx->stable_tick >= ctx->cfg.bias_stable_ticks) {
            ctx->state = MC_BIASED_L_IDENT_INJECT;
            ctx->level_tick = 0u;
            ctx->edge_current_a = current;
            ctx->previous_current_a = current;
            ctx->current_integral = 0.0;
            ctx->voltage_integral = 0.0;
            l_voltage_command(command,
                              ctx->base_voltage_v + ctx->injection_voltage_v);
        } else if (ctx->settle_tick >= ctx->cfg.bias_timeout_ticks) {
            return l_fail(ctx, MC_BIASED_L_IDENT_FAULT_BIAS_TIMEOUT,
                          MC_OUT_OF_RANGE, command);
        }
        ctx->settle_tick++;
        return MC_BUSY;
    }

    l_voltage_command(command, ctx->base_voltage_v +
                               (float)ctx->level_sign *
                               ctx->injection_voltage_v);

    if (ctx->level_tick < ctx->cfg.edge_skip_ticks) {
        ctx->level_tick++;
        if (ctx->level_tick == ctx->cfg.edge_skip_ticks) {
            ctx->edge_current_a = current;
            ctx->previous_current_a = current;
            ctx->current_integral = 0.0;
            ctx->voltage_integral = 0.0;
        }
        return MC_BUSY;
    }

    ctx->current_integral += 0.5 *
        ((double)ctx->previous_current_a + current);
    ctx->voltage_integral += sample->vd_v;
    ctx->previous_current_a = current;
    ctx->level_tick++;

    if (ctx->level_tick >= ctx->cfg.level_ticks) {
        const float n = (float)(ctx->cfg.level_ticks -
                                ctx->cfg.edge_skip_ticks);
        const float i_avg = (float)(ctx->current_integral / n);
        const float v_avg = (float)(ctx->voltage_integral / n);
        const float di = current - ctx->edge_current_a;
        const float slope = di / (n * ctx->cfg.control_period_s);

        if (ctx->level_sign > 0) {
            ctx->positive_slope_a_s = slope;
            ctx->positive_current_avg_a = i_avg;
            ctx->positive_voltage_avg_v = v_avg;
            ctx->positive_valid =
                current > ctx->cfg.min_positive_current_ratio *
                              ctx->bias_target_a &&
                ctx->edge_current_a > ctx->cfg.min_positive_current_ratio *
                                          ctx->bias_target_a &&
                fabsf(di) > ctx->cfg.min_current_delta_a;
        } else {
            mc_biased_l_pair_t pair;
            float inductance_h = 0.0f;
            mc_status_t pair_status;
            const float ripple = 0.5f *
                (fabsf(ctx->positive_slope_a_s) + fabsf(slope)) *
                n * ctx->cfg.control_period_s;
            ctx->pair_count++;
            ctx->last_ripple_a = ripple;

            pair.positive_slope_a_s = ctx->positive_slope_a_s;
            pair.negative_slope_a_s = slope;
            pair.positive_current_avg_a = ctx->positive_current_avg_a;
            pair.negative_current_avg_a = i_avg;
            pair.positive_voltage_avg_v = ctx->positive_voltage_avg_v;
            pair.negative_voltage_avg_v = v_avg;
            pair_status = mc_biased_l_ident_estimate_pair(
                &pair, ctx->phase_resistance_ohm,
                ctx->cfg.min_slope_difference_a_s, &inductance_h);

            if (ctx->cfg.auto_tune && ctx->pair_count <= ctx->cfg.tune_pairs) {
                if (ripple > ctx->cfg.tune_ripple_floor_a) {
                    float scale = ctx->target_ripple_a / ripple;
                    scale = mc_clampf(scale, ctx->cfg.tune_scale_min,
                                      ctx->cfg.tune_scale_max);
                    ctx->injection_voltage_v *= scale;
                } else {
                    ctx->injection_voltage_v *= ctx->cfg.tune_scale_max;
                }
                ctx->injection_voltage_v = l_clamp_injection(
                    ctx, ctx->injection_voltage_v, sample->vbus_v);
                ctx->result.tune_pairs = ctx->pair_count;
                ctx->result.tuned_injection_voltage_v =
                    ctx->injection_voltage_v;
            } else if (ctx->positive_valid &&
                       current > ctx->cfg.min_positive_current_ratio *
                                     ctx->bias_target_a &&
                       ctx->edge_current_a >
                           ctx->cfg.min_positive_current_ratio *
                               ctx->bias_target_a &&
                       fabsf(di) > ctx->cfg.min_current_delta_a &&
                       pair_status == MC_OK && mc_float_is_finite(inductance_h) &&
                       inductance_h > ctx->cfg.min_valid_inductance_h &&
                       inductance_h < ctx->cfg.max_valid_inductance_h) {
                ctx->inductance_sum_h += inductance_h;
                ctx->result.accepted_pairs++;
                ctx->last_inductance_h = inductance_h;
            } else {
                ctx->result.rejected_pairs++;
            }
            ctx->positive_valid = false;
        }

        ctx->level_sign = (int8_t)-ctx->level_sign;
        ctx->level_tick = 0u;
        ctx->edge_current_a = current;
        ctx->previous_current_a = current;
        ctx->current_integral = 0.0;
        ctx->voltage_integral = 0.0;
        l_voltage_command(command, ctx->base_voltage_v +
                                   (float)ctx->level_sign *
                                   ctx->injection_voltage_v);
    }

    if (ctx->result.accepted_pairs >= ctx->cfg.target_accepted_pairs ||
        ctx->pair_count >= ctx->cfg.tune_pairs +
                           ctx->cfg.target_accepted_pairs *
                           ctx->cfg.max_measure_pair_multiplier) {
        if (ctx->result.accepted_pairs == 0u)
            return l_fail(ctx, MC_BIASED_L_IDENT_FAULT_NO_VALID_PAIR,
                          MC_OUT_OF_RANGE, command);
        ctx->result.phase_inductance_h =
            (float)(ctx->inductance_sum_h /
                    (double)ctx->result.accepted_pairs);
        ctx->result.line_inductance_h =
            2.0f * ctx->result.phase_inductance_h;
        ctx->result.valid = true;
        ctx->state = MC_BIASED_L_IDENT_DONE;
        l_zero_command(command, true);
        return MC_DONE;
    }

    return MC_BUSY;
}

mc_status_t mc_biased_l_ident_abort(mc_biased_l_ident_t *ctx,
                                    mc_command_t *command)
{
    if (ctx == NULL || command == NULL) return MC_INVALID_ARGUMENT;
    ctx->state = MC_BIASED_L_IDENT_ABORTED;
    ctx->result.valid = false;
    l_zero_command(command, true);
    return MC_ABORTED;
}




