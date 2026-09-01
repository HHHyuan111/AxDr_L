#ifndef MC_BIAS_BANDWIDTH_H
#define MC_BIAS_BANDWIDTH_H

#include "mc_current_sweep.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MC_BIAS_MAX_POINTS 16u

typedef struct {
    float bias_current_a[MC_BIAS_MAX_POINTS];
    float bandwidth_hz[MC_BIAS_MAX_POINTS];
    float bandwidth_ratio[MC_BIAS_MAX_POINTS];
    float suggested_kp[MC_BIAS_MAX_POINTS];
    uint8_t valid[MC_BIAS_MAX_POINTS];
    uint32_t point_count;
} mc_bias_bandwidth_result_t;

mc_status_t mc_closed_loop_bandwidth(const mc_current_sweep_result_t *sweep,
                                     bool reject_saturated_points,
                                     float *bandwidth_hz);
mc_status_t mc_bias_bandwidth_build(const float *bias_current_a,
                                    const mc_current_sweep_result_t *sweeps,
                                    uint32_t count,
                                    float initial_kp,
                                    bool reject_saturated_points,
                                    mc_bias_bandwidth_result_t *result);

#ifdef __cplusplus
}
#endif

#endif




