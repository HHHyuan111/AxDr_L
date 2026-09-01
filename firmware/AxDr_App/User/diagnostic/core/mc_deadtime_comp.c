#include "mc_deadtime_comp.h"

#include <math.h>
#include <string.h>

static float mc_deadtime_phase_drop(const mc_deadtime_config_t *config,
                                    float current_a, float fixed_drop_v)
{
    float blend = config->current_blend_a;
    float direction;
    if (blend <= 1e-6f) blend = 1e-6f;
    direction = current_a / (fabsf(current_a) + blend);
    return config->polarity *
           (direction * fixed_drop_v + current_a * config->mos_rds_on_ohm);
}

mc_status_t mc_deadtime_compensate(const mc_deadtime_config_t *config,
                                   float ia_a, float ib_a, float ic_a,
                                   float vbus_v, float sample_time_s,
                                   mc_deadtime_output_t *output)
{
    if (!config || !output || !mc_float_is_finite(ia_a) || !mc_float_is_finite(ib_a) ||
        !mc_float_is_finite(ic_a) || !mc_float_is_finite(vbus_v) || !mc_float_is_finite(sample_time_s) ||
        !mc_float_is_finite(config->compensation_deadtime_ns) ||
        !mc_float_is_finite(config->current_blend_a) ||
        !mc_float_is_finite(config->mos_rds_on_ohm) ||
        !mc_float_is_finite(config->diode_vf_v) || !mc_float_is_finite(config->polarity) ||
        sample_time_s <= 0.0f || vbus_v <= 0.0f ||
        config->compensation_deadtime_ns < 0.0f ||
        config->compensation_deadtime_ns * 1e-9f >= sample_time_s ||
        config->current_blend_a <= 0.0f || config->mos_rds_on_ohm < 0.0f ||
        config->diode_vf_v < 0.0f ||
        fabsf(fabsf(config->polarity) - 1.0f) > 1e-6f) {
        return MC_INVALID_ARGUMENT;
    }

    memset(output, 0, sizeof(*output));
    output->deadtime_ratio = config->compensation_deadtime_ns * 1e-9f /
                             sample_time_s;
    output->fixed_drop_v = output->deadtime_ratio * (vbus_v + config->diode_vf_v);
    output->dv_a_v = mc_deadtime_phase_drop(config, ia_a, output->fixed_drop_v);
    output->dv_b_v = mc_deadtime_phase_drop(config, ib_a, output->fixed_drop_v);
    output->dv_c_v = mc_deadtime_phase_drop(config, ic_a, output->fixed_drop_v);
    output->dv_alpha_v = (2.0f * output->dv_a_v - output->dv_b_v - output->dv_c_v) / 3.0f;
    output->dv_beta_v = (output->dv_b_v - output->dv_c_v) / MC_SQRT3_F;
    return (mc_float_is_finite(output->dv_alpha_v) && mc_float_is_finite(output->dv_beta_v)) ?
           MC_OK : MC_NUMERIC_ERROR;
}




