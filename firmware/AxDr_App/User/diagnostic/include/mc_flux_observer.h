#ifndef MC_FLUX_OBSERVER_H
#define MC_FLUX_OBSERVER_H

#include "mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float rs_ohm;
    float pole_pairs;
    float sample_time_s;
    float filter_cutoff_hz;
    float minimum_electrical_speed_rad_s;
    bool reject_voltage_saturation;
} mc_flux_config_t;

typedef struct {
    bool initialized;
    uint32_t accepted_samples;
    uint32_t rejected_samples;
    float psi_d_instant_wb;
    float psi_q_instant_wb;
    float psi_magnitude_instant_wb;
    float psi_d_filtered_wb;
    float psi_q_filtered_wb;
    float psi_magnitude_filtered_wb;
} mc_flux_observer_t;

void mc_flux_observer_init(mc_flux_observer_t *observer);
mc_status_t mc_flux_observer_step(mc_flux_observer_t *observer,
                                  const mc_flux_config_t *config,
                                  const mc_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif




