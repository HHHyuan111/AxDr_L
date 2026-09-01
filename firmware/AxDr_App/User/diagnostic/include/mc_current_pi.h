#ifndef MC_CURRENT_PI_H
#define MC_CURRENT_PI_H

#include "mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Ki is the increment applied once per control tick, not the continuous-time
 * integral gain. */
mc_status_t mc_current_pi_from_bandwidth(float bandwidth_hz,
                                         float sample_time_s,
                                         float rs_ohm,
                                         float ld_h,
                                         float lq_h,
                                         float *kp_d,
                                         float *ki_d_per_tick,
                                         float *kp_q,
                                         float *ki_q_per_tick);

/* First-pass motor/platform scaling. Per-tick Ki includes the sample-time
 * ratio. Sweep verification is still required. */
mc_status_t mc_current_pi_scale_from_reference(
    float reference_kp_d,
    float reference_ki_d_per_tick,
    float reference_kp_q,
    float reference_ki_q_per_tick,
    float reference_rs_ohm,
    float reference_ld_h,
    float reference_lq_h,
    float reference_sample_time_s,
    float target_rs_ohm,
    float target_ld_h,
    float target_lq_h,
    float target_sample_time_s,
    float *target_kp_d,
    float *target_ki_d_per_tick,
    float *target_kp_q,
    float *target_ki_q_per_tick);

#ifdef __cplusplus
}
#endif

#endif




