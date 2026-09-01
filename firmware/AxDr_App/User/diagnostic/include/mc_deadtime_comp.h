#ifndef MC_DEADTIME_COMP_H
#define MC_DEADTIME_COMP_H

#include "mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float compensation_deadtime_ns;
    float current_blend_a;
    float mos_rds_on_ohm;
    float diode_vf_v;
    float polarity;
} mc_deadtime_config_t;

typedef struct {
    float dv_a_v;
    float dv_b_v;
    float dv_c_v;
    float dv_alpha_v;
    float dv_beta_v;
    float fixed_drop_v;
    float deadtime_ratio;
} mc_deadtime_output_t;

mc_status_t mc_deadtime_compensate(const mc_deadtime_config_t *config,
                                   float ia_a, float ib_a, float ic_a,
                                   float vbus_v, float sample_time_s,
                                   mc_deadtime_output_t *output);

#ifdef __cplusplus
}
#endif

#endif




