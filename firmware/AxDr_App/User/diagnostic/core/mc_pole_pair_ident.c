#include "mc_pole_pair_ident.h"

#include <limits.h>
#include <math.h>
#include <string.h>

static void pp_zero_command(mc_command_t *command, bool request_disable)
{
    if (command == NULL) return;
    memset(command, 0, sizeof(*command));
    command->mode = MC_CONTROL_VOLTAGE;
    command->disable_request = request_disable;
}

static void pp_voltage_command(mc_command_t *command,
                               float voltage_v,
                               float theta_e_rad)
{
    memset(command, 0, sizeof(*command));
    command->mode = MC_CONTROL_VOLTAGE;
    command->vd_ref_v = voltage_v;
    command->openloop_theta_e_rad = theta_e_rad;
    command->openloop_enable = true;
    command->enable_request = true;
}

static mc_status_t pp_fail(mc_pole_pair_ident_t *ctx,
                           mc_pole_pair_ident_fault_t fault,
                           mc_status_t status,
                           mc_command_t *command)
{
    ctx->fault = fault;
    ctx->state = MC_POLE_PAIR_IDENT_ERROR;
    ctx->result.valid = false;
    pp_zero_command(command, true);
    return status;
}

void mc_pole_pair_ident_default_config(mc_pole_pair_ident_config_t *cfg)
{
    if (cfg == NULL) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->control_period_s = 1.0f / 20000.0f;
    cfg->drive_current_a = 8.0f;
    cfg->electrical_turns = 16.0f;
    cfg->electrical_velocity_rad_s = MC_PI_F;
    cfg->ramp_duration_s = 2.0f;
    cfg->min_pole_pairs = 1;
    cfg->max_pole_pairs = 30;
    /* Invalid until the target explicitly confirms a safe bridge/motor limit. */
    cfg->safe_current_limit_a = 0.0f;
    cfg->max_abs_voltage_v = 0.0f; /* source behavior: no software clamp */
    cfg->min_vbus_v = 0.0f;
}

static bool pp_config_valid(const mc_pole_pair_ident_config_t *cfg)
{
    if (cfg == NULL) return false;
    if (!mc_float_is_finite(cfg->control_period_s) ||
        !mc_float_is_finite(cfg->phase_resistance_ohm) ||
        !mc_float_is_finite(cfg->drive_current_a) ||
        !mc_float_is_finite(cfg->electrical_turns) ||
        !mc_float_is_finite(cfg->electrical_velocity_rad_s) ||
        !mc_float_is_finite(cfg->ramp_duration_s) ||
        !mc_float_is_finite(cfg->safe_current_limit_a) ||
        !mc_float_is_finite(cfg->max_abs_voltage_v) ||
        !mc_float_is_finite(cfg->min_vbus_v)) return false;
    if (cfg->control_period_s <= 1e-9f ||
        cfg->phase_resistance_ohm <= 0.0f ||
        cfg->drive_current_a <= 0.0f ||
        cfg->drive_current_a > cfg->safe_current_limit_a ||
        cfg->safe_current_limit_a <= 0.0f ||
        cfg->electrical_turns <= 0.0f ||
        cfg->electrical_velocity_rad_s <= 0.0f ||
        cfg->ramp_duration_s <= 0.0f ||
        cfg->encoder_full_scale < 2u ||
        cfg->encoder_full_scale > (uint32_t)INT32_MAX ||
        cfg->min_pole_pairs < 1 ||
        cfg->max_pole_pairs < cfg->min_pole_pairs ||
        cfg->max_abs_voltage_v < 0.0f || cfg->min_vbus_v < 0.0f)
        return false;
    return true;
}

mc_status_t mc_pole_pair_ident_start(mc_pole_pair_ident_t *ctx,
                                     const mc_pole_pair_ident_config_t *cfg)
{
    float drive_voltage_v;
    uint32_t ramp_ticks;

    if (ctx == NULL || !pp_config_valid(cfg)) return MC_INVALID_ARGUMENT;
    drive_voltage_v = cfg->drive_current_a * cfg->phase_resistance_ohm;
    if (!mc_float_is_finite(drive_voltage_v)) return MC_NUMERIC_ERROR;
    if (cfg->max_abs_voltage_v > 0.0f &&
        drive_voltage_v > cfg->max_abs_voltage_v) return MC_OUT_OF_RANGE;
    ramp_ticks = (uint32_t)(cfg->ramp_duration_s / cfg->control_period_s);
    if (ramp_ticks == 0u) return MC_OUT_OF_RANGE;

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->ramp_ticks = ramp_ticks;
    ctx->drive_voltage_v = drive_voltage_v;
    ctx->state = MC_POLE_PAIR_IDENT_RAMP;
    ctx->fault = MC_POLE_PAIR_IDENT_FAULT_NONE;
    return MC_OK;
}

int32_t mc_encoder_unwrap_delta(uint32_t current_raw,
                                uint32_t previous_raw,
                                uint32_t full_scale)
{
    int64_t delta;
    int64_t fs;

    if (full_scale < 2u || full_scale > (uint32_t)INT32_MAX) return 0;
    current_raw %= full_scale;
    previous_raw %= full_scale;
    fs = (int64_t)full_scale;
    delta = (int64_t)current_raw - previous_raw;
    if (delta > fs / 2) delta -= fs;
    if (delta < -fs / 2) delta += fs;
    return (int32_t)delta;
}

mc_status_t mc_pole_pair_ident_solve(int64_t accumulated_counts,
                                     uint32_t encoder_full_scale,
                                     float electrical_turns,
                                     int32_t min_pole_pairs,
                                     int32_t max_pole_pairs,
                                     mc_pole_pair_ident_result_t *result)
{
    double absolute_counts;
    float mechanical_turns;
    float unrounded;
    int32_t rounded;

    if (result == NULL || encoder_full_scale < 2u ||
        encoder_full_scale > (uint32_t)INT32_MAX ||
        !mc_float_is_finite(electrical_turns) || electrical_turns <= 0.0f ||
        min_pole_pairs < 1 || max_pole_pairs < min_pole_pairs)
        return MC_INVALID_ARGUMENT;

    memset(result, 0, sizeof(*result));
    result->accumulated_counts = accumulated_counts;
    if (accumulated_counts == 0) return MC_OUT_OF_RANGE;
    absolute_counts = accumulated_counts < 0
        ? -(double)accumulated_counts
        : (double)accumulated_counts;
    mechanical_turns = (float)(absolute_counts /
                               (double)encoder_full_scale);
    if (!(mechanical_turns > 1e-6f) || !mc_float_is_finite(mechanical_turns))
        return MC_OUT_OF_RANGE;
    unrounded = electrical_turns / mechanical_turns;
    if (!mc_float_is_finite(unrounded) || unrounded > (float)INT32_MAX)
        return MC_NUMERIC_ERROR;
    rounded = (int32_t)floorf(unrounded + 0.5f);

    result->encoder_direction = accumulated_counts > 0 ? (int8_t)1 : (int8_t)-1;
    result->mechanical_turns = mechanical_turns;
    result->pole_pairs_unrounded = unrounded;
    result->pole_pairs = rounded;
    if (rounded < min_pole_pairs || rounded > max_pole_pairs)
        return MC_OUT_OF_RANGE;
    result->valid = true;
    return MC_OK;
}

mc_status_t mc_pole_pair_ident_step(mc_pole_pair_ident_t *ctx,
                                    const mc_sample_t *sample,
                                    mc_command_t *command)
{
    if (ctx == NULL || sample == NULL || command == NULL)
        return MC_INVALID_ARGUMENT;
    if (ctx->state == MC_POLE_PAIR_IDENT_DONE) {
        pp_zero_command(command, true);
        return MC_DONE;
    }
    if (ctx->state == MC_POLE_PAIR_IDENT_ABORTED) {
        pp_zero_command(command, true);
        return MC_ABORTED;
    }
    if (ctx->state != MC_POLE_PAIR_IDENT_RAMP &&
        ctx->state != MC_POLE_PAIR_IDENT_ROTATE)
        return pp_fail(ctx, MC_POLE_PAIR_IDENT_FAULT_BAD_CONFIG,
                       MC_REJECTED, command);
    if (sample->fault_code != 0u)
        return pp_fail(ctx, MC_POLE_PAIR_IDENT_FAULT_DRIVE,
                       MC_FAULT, command);
    if (ctx->cfg.min_vbus_v > 0.0f && sample->vbus_v < ctx->cfg.min_vbus_v)
        return pp_fail(ctx, MC_POLE_PAIR_IDENT_FAULT_VBUS_LOW,
                       MC_FAULT, command);
    if (sample->current_limit_a > 0.0f &&
        ctx->cfg.drive_current_a > sample->current_limit_a)
        return pp_fail(ctx, MC_POLE_PAIR_IDENT_FAULT_CURRENT_LIMIT,
                       MC_OUT_OF_RANGE, command);

    if (ctx->state == MC_POLE_PAIR_IDENT_RAMP) {
        float ramp = (float)ctx->ramp_tick /
                     (float)ctx->ramp_ticks;
        if (ramp > 1.0f) ramp = 1.0f;
        pp_voltage_command(command, ctx->drive_voltage_v * ramp, 0.0f);
        if (ctx->ramp_tick >= ctx->ramp_ticks) {
            ctx->previous_encoder_raw = sample->encoder_raw;
            ctx->accumulated_counts = 0;
            ctx->state = MC_POLE_PAIR_IDENT_ROTATE;
        } else {
            ctx->ramp_tick++;
        }
        return MC_BUSY;
    }

    ctx->electrical_phase_rad += ctx->cfg.electrical_velocity_rad_s *
                                 ctx->cfg.control_period_s;
    pp_voltage_command(command, ctx->drive_voltage_v,
                       ctx->electrical_phase_rad);
    ctx->accumulated_counts += mc_encoder_unwrap_delta(
        sample->encoder_raw, ctx->previous_encoder_raw,
        ctx->cfg.encoder_full_scale);
    ctx->previous_encoder_raw = sample->encoder_raw;

    if (ctx->electrical_phase_rad >=
        ctx->cfg.electrical_turns * MC_TWO_PI_F) {
        mc_status_t status = mc_pole_pair_ident_solve(
            ctx->accumulated_counts, ctx->cfg.encoder_full_scale,
            ctx->cfg.electrical_turns, ctx->cfg.min_pole_pairs,
            ctx->cfg.max_pole_pairs, &ctx->result);
        if (status != MC_OK) {
            return pp_fail(ctx,
                ctx->accumulated_counts == 0
                    ? MC_POLE_PAIR_IDENT_FAULT_NO_MOTION
                    : MC_POLE_PAIR_IDENT_FAULT_OUT_OF_RANGE,
                status, command);
        }
        ctx->state = MC_POLE_PAIR_IDENT_DONE;
        pp_zero_command(command, true);
        return MC_DONE;
    }
    return MC_BUSY;
}

mc_status_t mc_pole_pair_ident_abort(mc_pole_pair_ident_t *ctx,
                                     mc_command_t *command)
{
    if (ctx == NULL || command == NULL) return MC_INVALID_ARGUMENT;
    ctx->state = MC_POLE_PAIR_IDENT_ABORTED;
    ctx->result.valid = false;
    pp_zero_command(command, true);
    return MC_ABORTED;
}




