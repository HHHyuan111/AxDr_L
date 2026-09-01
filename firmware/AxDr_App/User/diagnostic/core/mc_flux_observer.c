#include "mc_flux_observer.h"

#include <math.h>
#include <string.h>

void mc_flux_observer_init(mc_flux_observer_t *observer)
{
    if (observer) memset(observer, 0, sizeof(*observer));
}

mc_status_t mc_flux_observer_step(mc_flux_observer_t *observer,
                                  const mc_flux_config_t *config,
                                  const mc_sample_t *sample)
{
    float omega_e;
    float alpha;

    if (!observer || !config || !sample ||
        !mc_float_is_finite(config->rs_ohm) || !mc_float_is_finite(config->pole_pairs) ||
        !mc_float_is_finite(config->sample_time_s) ||
        !mc_float_is_finite(config->filter_cutoff_hz) ||
        !mc_float_is_finite(config->minimum_electrical_speed_rad_s) ||
        !mc_float_is_finite(sample->omega_mech_rad_s) || !mc_float_is_finite(sample->id_a) ||
        !mc_float_is_finite(sample->iq_a) || !mc_float_is_finite(sample->vd_v) ||
        !mc_float_is_finite(sample->vq_v) || config->rs_ohm < 0.0f ||
        config->sample_time_s <= 0.0f || config->pole_pairs <= 0.0f ||
        config->filter_cutoff_hz <= 0.0f ||
        config->filter_cutoff_hz >= 0.5f / config->sample_time_s ||
        config->minimum_electrical_speed_rad_s <= 0.0f) {
        return MC_INVALID_ARGUMENT;
    }

    omega_e = sample->omega_mech_rad_s * config->pole_pairs;
    if (!mc_float_is_finite(omega_e)) return MC_NUMERIC_ERROR;
    if (fabsf(omega_e) < config->minimum_electrical_speed_rad_s ||
        (config->reject_voltage_saturation && sample->voltage_saturated)) {
        observer->rejected_samples++;
        return MC_REJECTED;
    }

    observer->psi_d_instant_wb = (sample->vq_v - config->rs_ohm * sample->iq_a) / omega_e;
    observer->psi_q_instant_wb = (config->rs_ohm * sample->id_a - sample->vd_v) / omega_e;
    observer->psi_magnitude_instant_wb = sqrtf(
        observer->psi_d_instant_wb * observer->psi_d_instant_wb +
        observer->psi_q_instant_wb * observer->psi_q_instant_wb);
    if (!mc_float_is_finite(observer->psi_d_instant_wb) ||
        !mc_float_is_finite(observer->psi_q_instant_wb) ||
        !mc_float_is_finite(observer->psi_magnitude_instant_wb))
        return MC_NUMERIC_ERROR;

    alpha = 1.0f - expf(-MC_TWO_PI_F * config->filter_cutoff_hz * config->sample_time_s);
    if (!observer->initialized) {
        observer->psi_d_filtered_wb = observer->psi_d_instant_wb;
        observer->psi_q_filtered_wb = observer->psi_q_instant_wb;
        observer->psi_magnitude_filtered_wb = observer->psi_magnitude_instant_wb;
        observer->initialized = true;
    } else {
        observer->psi_d_filtered_wb += alpha *
            (observer->psi_d_instant_wb - observer->psi_d_filtered_wb);
        observer->psi_q_filtered_wb += alpha *
            (observer->psi_q_instant_wb - observer->psi_q_filtered_wb);
        observer->psi_magnitude_filtered_wb += alpha *
            (observer->psi_magnitude_instant_wb - observer->psi_magnitude_filtered_wb);
    }
    if (!mc_float_is_finite(observer->psi_d_filtered_wb) ||
        !mc_float_is_finite(observer->psi_q_filtered_wb) ||
        !mc_float_is_finite(observer->psi_magnitude_filtered_wb))
        return MC_NUMERIC_ERROR;
    observer->accepted_samples++;
    return MC_OK;
}




