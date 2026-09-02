#include "mc_current_sweep.h"

#include <math.h>
#include <string.h>

#define MC_SWEEP_RAD_TO_DEG    (57.295779513082320876f)

static float mc_sweep_frequency(const mc_current_sweep_t *sweep, uint32_t index)
{
    if (sweep->config.single_point || sweep->config.requested_points <= 1U)
    {
        return sweep->config.single_frequency_hz;
    }

    return sweep->config.start_frequency_hz * expf(
        ((float)index / (float)(sweep->config.requested_points - 1U)) *
        logf(sweep->config.end_frequency_hz / sweep->config.start_frequency_hz));
}

static void mc_sweep_command(const mc_current_sweep_t *sweep, mc_command_t *command)
{
    float injection;
    memset(command, 0, sizeof(*command));
    injection = sweep->config.offset_a + sweep->config.amplitude_a * sinf(sweep->phase_rad);
    command->mode = MC_CONTROL_CURRENT;
    command->enable_request = true;
    if (sweep->config.axis == MC_AXIS_D) command->id_ref_a = injection;
    else command->iq_ref_a = injection;
}

static void mc_sweep_prepare_point(mc_current_sweep_t *sweep)
{
    float fs = 1.0f / sweep->dt_s;
    float settle_min = sweep->config.minimum_settle_time_s;
    float measure_min = sweep->config.minimum_measure_time_s;
    uint32_t settle_cycles;
    uint32_t settle_time;
    uint32_t measure_cycles;
    uint32_t measure_time;

    sweep->active_frequency_hz = mc_sweep_frequency(sweep, sweep->point_index);
    settle_cycles = (uint32_t)((float)sweep->config.settle_cycles * fs /
                               sweep->active_frequency_hz + 0.5f);
    settle_time = (uint32_t)(settle_min * fs + 0.5f);
    measure_cycles = (uint32_t)((float)sweep->config.measure_cycles * fs /
                                sweep->active_frequency_hz + 0.5f);
    measure_time = (uint32_t)(measure_min * fs + 0.5f);
    sweep->settle_ticks = settle_cycles > settle_time ? settle_cycles : settle_time;
    sweep->measure_ticks = measure_cycles > measure_time ? measure_cycles : measure_time;
    if (sweep->measure_ticks < sweep->config.minimum_measure_samples)
    {
        sweep->measure_ticks = sweep->config.minimum_measure_samples;
    }

    /* 对所有频点按完整周期测量，避免直流偏置和频谱泄漏影响幅相结果。 */
    {
        const uint32_t whole_cycles = (uint32_t)ceilf(
            (float)sweep->measure_ticks * sweep->active_frequency_hz / fs);

        sweep->measure_ticks = (uint32_t)(
            (float)whole_cycles * fs / sweep->active_frequency_hz + 0.5f);
    }
    if (sweep->settle_ticks == 0U)
    {
        sweep->settle_ticks = 1U;
    }
    if (sweep->measure_ticks < 8U)
    {
        sweep->measure_ticks = 8U;
    }
    sweep->phase_rad = 0.0f;
    sweep->tick = 0u;
    sweep->sum_y_sin = 0.0;
    sweep->sum_y_cos = 0.0;
    sweep->point_saturated = false;
}

static bool mc_sweep_finish_point(mc_current_sweep_t *sweep)
{
    uint32_t index = sweep->point_index;
    float norm = 2.0f / ((float)sweep->measure_ticks * fabsf(sweep->config.amplitude_a));
    float tr = (float)sweep->sum_y_sin * norm;
    float ti = (float)sweep->sum_y_cos * norm;
    float t_mag = sqrtf(tr * tr + ti * ti);
    float hr = 1.0f;
    float hi = 0.0f;
    float h_mag;
    float fr;
    float fi;
    float divisor;
    float lr;
    float li;
    float l_mag;
    bool numeric_valid;

    if (sweep->config.feedback_filter_alpha > 0.0f &&
        sweep->config.feedback_filter_alpha < 1.0f) {
        float alpha = sweep->config.feedback_filter_alpha;
        float r = 1.0f - alpha;
        float omega = MC_TWO_PI_F * sweep->active_frequency_hz * sweep->dt_s;
        float den_re = 1.0f - r * cosf(omega);
        float den_im = r * sinf(omega);
        float den_mag2 = den_re * den_re + den_im * den_im;
        hr = alpha * den_re / den_mag2;
        hi = -alpha * den_im / den_mag2;
    }
    h_mag = sqrtf(hr * hr + hi * hi);
    fr = hr * tr - hi * ti;
    fi = hr * ti + hi * tr;
    divisor = (1.0f - fr) * (1.0f - fr) + fi * fi;
    lr = divisor > 1e-12f ? (fr - fr * fr - fi * fi) / divisor : 0.0f;
    li = divisor > 1e-12f ? fi / divisor : 0.0f;
    l_mag = sqrtf(lr * lr + li * li);

    sweep->result.frequency_hz[index] = sweep->active_frequency_hz;
    sweep->result.input_amplitude_a[index] = sweep->config.amplitude_a;
    sweep->result.input_offset_a[index] = sweep->config.offset_a;
    sweep->result.closed_magnitude_db[index] = 20.0f * log10f(t_mag > 1e-12f ? t_mag : 1e-12f);
    sweep->result.closed_phase_deg[index] = atan2f(ti, tr) * MC_SWEEP_RAD_TO_DEG;
    sweep->result.filter_magnitude_db[index] = 20.0f * log10f(h_mag > 1e-12f ? h_mag : 1e-12f);
    sweep->result.filter_phase_deg[index] = atan2f(hi, hr) * MC_SWEEP_RAD_TO_DEG;
    sweep->result.open_magnitude_db[index] = 20.0f * log10f(l_mag > 1e-12f ? l_mag : 1e-12f);
    sweep->result.open_phase_deg[index] = atan2f(li, lr) * MC_SWEEP_RAD_TO_DEG;
    numeric_valid =
        mc_float_is_finite(sweep->result.closed_magnitude_db[index]) &&
        mc_float_is_finite(sweep->result.closed_phase_deg[index]) &&
        mc_float_is_finite(sweep->result.filter_magnitude_db[index]) &&
        mc_float_is_finite(sweep->result.filter_phase_deg[index]) &&
        mc_float_is_finite(sweep->result.open_magnitude_db[index]) &&
        mc_float_is_finite(sweep->result.open_phase_deg[index]);
    sweep->result.samples[index] = sweep->measure_ticks;
    sweep->result.saturated[index] = sweep->point_saturated ? 1u : 0u;
    sweep->result.valid[index] =
        (numeric_valid && (!sweep->point_saturated ||
                           !sweep->config.reject_saturated_points)) ? 1u : 0u;
    if (sweep->result.completed_points < index + 1u)
        sweep->result.completed_points = index + 1u;
    return numeric_valid;
}

void mc_current_sweep_default_config(mc_current_sweep_config_t *config)
{
    if (!config) return;
    memset(config, 0, sizeof(*config));
    config->axis = MC_AXIS_D;
    config->single_frequency_hz = 100.0f;
    config->start_frequency_hz = 20.0f;
    config->end_frequency_hz = 2000.0f;
    config->amplitude_a = 0.5f;
    config->safe_current_limit_a = 0.0f;
    config->requested_points = MC_SWEEP_MAX_POINTS;
    config->settle_cycles = 12u;
    config->measure_cycles = 20u;
    config->minimum_settle_time_s = 0.20f;
    config->minimum_measure_time_s = 0.20f;
    config->minimum_measure_samples = 4000u;
    /* 通用算法不假设反馈通道存在固定低通滤波器。 */
    config->feedback_filter_alpha = 0.0f;
    config->reject_saturated_points = true;
}

mc_status_t mc_current_sweep_start(mc_current_sweep_t *sweep,
                                   const mc_current_sweep_config_t *config,
                                   float sample_time_s,
                                   mc_command_t *first_command)
{
    if (!sweep || !config || !first_command ||
        !mc_float_is_finite(sample_time_s) || !mc_float_is_finite(config->single_frequency_hz) ||
        !mc_float_is_finite(config->start_frequency_hz) ||
        !mc_float_is_finite(config->end_frequency_hz) ||
        !mc_float_is_finite(config->amplitude_a) || !mc_float_is_finite(config->offset_a) ||
        !mc_float_is_finite(config->safe_current_limit_a) ||
        !mc_float_is_finite(config->minimum_settle_time_s) ||
        !mc_float_is_finite(config->minimum_measure_time_s) ||
        !mc_float_is_finite(config->feedback_filter_alpha) || sample_time_s <= 0.0f ||
        (config->axis != MC_AXIS_D && config->axis != MC_AXIS_Q) ||
        fabsf(config->amplitude_a) < 1e-4f ||
        config->safe_current_limit_a <= 0.0f ||
        fabsf(config->offset_a) + fabsf(config->amplitude_a) >
            config->safe_current_limit_a ||
        config->start_frequency_hz <= 0.0f ||
        config->end_frequency_hz < config->start_frequency_hz ||
        config->single_frequency_hz <= 0.0f ||
        config->minimum_settle_time_s < 0.0f ||
        config->minimum_measure_time_s < 0.0f ||
        config->feedback_filter_alpha < 0.0f ||
        config->feedback_filter_alpha >= 1.0f ||
        config->requested_points == 0u ||
        config->requested_points > MC_SWEEP_MAX_POINTS ||
        config->settle_cycles == 0u || config->measure_cycles == 0u) {
        return MC_INVALID_ARGUMENT;
    }
    if (((config->single_point || config->requested_points <= 1u) ?
         config->single_frequency_hz :
         config->end_frequency_hz) >= 0.5f / sample_time_s)
        return MC_OUT_OF_RANGE;
    memset(sweep, 0, sizeof(*sweep));
    sweep->config = *config;
    sweep->dt_s = sample_time_s;
    sweep->status = MC_BUSY;
    mc_sweep_prepare_point(sweep);
    mc_sweep_command(sweep, first_command);
    return MC_OK;
}

mc_status_t mc_current_sweep_step(mc_current_sweep_t *sweep,
                                  const mc_sample_t *sample,
                                  mc_command_t *next_command)
{
    float sine;
    float cosine;
    float measured;

    if (!sweep || !sample || !next_command || sweep->status != MC_BUSY)
        return MC_REJECTED;
    if (sample->fault_code != 0u) return mc_current_sweep_abort(sweep, next_command);
    if (!mc_float_is_finite(sample->i_alpha_a) ||
        !mc_float_is_finite(sample->i_beta_a) ||
        !mc_float_is_finite(sample->theta_elec_rad) ||
        !mc_float_is_finite(sample->current_limit_a)) {
        mc_current_sweep_abort(sweep, next_command);
        return MC_NUMERIC_ERROR;
    }
    if (sample->current_limit_a > 0.0f &&
        fabsf(sweep->config.offset_a) + fabsf(sweep->config.amplitude_a) >
            sample->current_limit_a) {
        mc_current_sweep_abort(sweep, next_command);
        return MC_OUT_OF_RANGE;
    }

    sine = sinf(sweep->phase_rad);
    cosine = cosf(sweep->phase_rad);
    if (sweep->config.axis == MC_AXIS_D) {
        measured = sample->i_alpha_a * cosf(sample->theta_elec_rad)
                 + sample->i_beta_a * sinf(sample->theta_elec_rad);
    } else {
        measured = sample->i_beta_a * cosf(sample->theta_elec_rad)
                 - sample->i_alpha_a * sinf(sample->theta_elec_rad);
    }
    if (sweep->tick >= sweep->settle_ticks) {
        sweep->sum_y_sin += (double)measured * sine;
        sweep->sum_y_cos += (double)measured * cosine;
        if (sample->voltage_saturated) sweep->point_saturated = true;
    }

    sweep->tick++;
    sweep->phase_rad = mc_wrap_0_2pi(sweep->phase_rad +
        MC_TWO_PI_F * sweep->active_frequency_hz * sweep->dt_s);
    mc_sweep_command(sweep, next_command);
    if (sweep->tick < sweep->settle_ticks + sweep->measure_ticks) return MC_BUSY;

    if (!mc_sweep_finish_point(sweep)) {
        mc_current_sweep_abort(sweep, next_command);
        return MC_NUMERIC_ERROR;
    }
    if (sweep->config.single_point) {
        sweep->tick = sweep->settle_ticks;
        sweep->sum_y_sin = 0.0;
        sweep->sum_y_cos = 0.0;
        sweep->point_saturated = false;
        return MC_BUSY;
    }
    sweep->point_index++;
    if (sweep->point_index >= sweep->config.requested_points) {
        memset(next_command, 0, sizeof(*next_command));
        next_command->disable_request = true;
        sweep->status = MC_DONE;
        return MC_DONE;
    }
    mc_sweep_prepare_point(sweep);
    mc_sweep_command(sweep, next_command);
    return MC_BUSY;
}

mc_status_t mc_current_sweep_abort(mc_current_sweep_t *sweep,
                                   mc_command_t *stop_command)
{
    if (!sweep || !stop_command) return MC_INVALID_ARGUMENT;
    memset(stop_command, 0, sizeof(*stop_command));
    stop_command->disable_request = true;
    sweep->status = MC_ABORTED;
    return MC_ABORTED;
}


