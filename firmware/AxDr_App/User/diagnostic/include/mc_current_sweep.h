#ifndef MC_CURRENT_SWEEP_H
#define MC_CURRENT_SWEEP_H

#include "mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MC_SWEEP_MAX_POINTS 128u

typedef struct {
    mc_axis_t axis;
    bool single_point;
    float single_frequency_hz;
    float start_frequency_hz;
    float end_frequency_hz;
    float amplitude_a;
    float offset_a;
    /* Must be set explicitly by the target.  Requested peak current is
     * abs(offset_a) + abs(amplitude_a). */
    float safe_current_limit_a;
    uint32_t requested_points;
    uint16_t settle_cycles;
    uint16_t measure_cycles;
    float minimum_settle_time_s;
    float minimum_measure_time_s;
    uint32_t minimum_measure_samples;
    float feedback_filter_alpha;
    bool reject_saturated_points;
} mc_current_sweep_config_t;

typedef struct {
    float frequency_hz[MC_SWEEP_MAX_POINTS];
    float input_amplitude_a[MC_SWEEP_MAX_POINTS];
    float input_offset_a[MC_SWEEP_MAX_POINTS];
    float closed_magnitude_db[MC_SWEEP_MAX_POINTS];
    float closed_phase_deg[MC_SWEEP_MAX_POINTS];
    float filter_magnitude_db[MC_SWEEP_MAX_POINTS];
    float filter_phase_deg[MC_SWEEP_MAX_POINTS];
    float open_magnitude_db[MC_SWEEP_MAX_POINTS];
    float open_phase_deg[MC_SWEEP_MAX_POINTS];
    uint32_t samples[MC_SWEEP_MAX_POINTS];
    uint8_t saturated[MC_SWEEP_MAX_POINTS];
    uint8_t valid[MC_SWEEP_MAX_POINTS];
    uint32_t completed_points;
} mc_current_sweep_result_t;

typedef struct {
    mc_current_sweep_config_t config;
    mc_current_sweep_result_t result;
    mc_status_t status;
    float dt_s;
    float active_frequency_hz;
    float phase_rad;
    uint32_t point_index;
    uint32_t tick;
    uint32_t settle_ticks;
    uint32_t measure_ticks;
    double sum_y_sin;
    double sum_y_cos;
    bool point_saturated;
} mc_current_sweep_t;

void mc_current_sweep_default_config(mc_current_sweep_config_t *config);
mc_status_t mc_current_sweep_start(mc_current_sweep_t *sweep,
                                   const mc_current_sweep_config_t *config,
                                   float sample_time_s,
                                   mc_command_t *first_command);
mc_status_t mc_current_sweep_step(mc_current_sweep_t *sweep,
                                  const mc_sample_t *sample,
                                  mc_command_t *next_command);
mc_status_t mc_current_sweep_abort(mc_current_sweep_t *sweep,
                                   mc_command_t *stop_command);

#ifdef __cplusplus
}
#endif

#endif




