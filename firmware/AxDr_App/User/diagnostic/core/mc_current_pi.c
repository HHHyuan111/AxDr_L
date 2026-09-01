#include "mc_current_pi.h"

#include <math.h>

mc_status_t mc_current_pi_from_bandwidth(float bandwidth_hz,
                                         float sample_time_s,
                                         float rs_ohm,
                                         float ld_h,
                                         float lq_h,
                                         float *kp_d,
                                         float *ki_d_per_tick,
                                         float *kp_q,
                                         float *ki_q_per_tick)
{
    float omega_c;
    float ki;

    if (!kp_d || !ki_d_per_tick || !kp_q || !ki_q_per_tick ||
        !mc_float_is_finite(bandwidth_hz) || !mc_float_is_finite(sample_time_s) ||
        !mc_float_is_finite(rs_ohm) || !mc_float_is_finite(ld_h) || !mc_float_is_finite(lq_h) ||
        bandwidth_hz <= 0.0f || sample_time_s <= 0.0f ||
        rs_ohm <= 0.0f || ld_h <= 0.0f || lq_h <= 0.0f) {
        return MC_INVALID_ARGUMENT;
    }
    if (bandwidth_hz > 0.1f / sample_time_s) return MC_OUT_OF_RANGE;

    omega_c = MC_TWO_PI_F * bandwidth_hz;
    ki = rs_ohm * omega_c * sample_time_s;
    *kp_d = ld_h * omega_c;
    *kp_q = lq_h * omega_c;
    *ki_d_per_tick = ki;
    *ki_q_per_tick = ki;
    return (mc_float_is_finite(*kp_d) &&
            mc_float_is_finite(*ki_d_per_tick) &&
            mc_float_is_finite(*kp_q) &&
            mc_float_is_finite(*ki_q_per_tick)) ? MC_OK : MC_NUMERIC_ERROR;
}

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
    float *target_ki_q_per_tick)
{
    if (!target_kp_d || !target_ki_d_per_tick || !target_kp_q ||
        !target_ki_q_per_tick ||
        !mc_float_is_finite(reference_kp_d) ||
        !mc_float_is_finite(reference_ki_d_per_tick) ||
        !mc_float_is_finite(reference_kp_q) ||
        !mc_float_is_finite(reference_ki_q_per_tick) ||
        !mc_float_is_finite(reference_rs_ohm) || !mc_float_is_finite(reference_ld_h) ||
        !mc_float_is_finite(reference_lq_h) ||
        !mc_float_is_finite(reference_sample_time_s) ||
        !mc_float_is_finite(target_rs_ohm) || !mc_float_is_finite(target_ld_h) ||
        !mc_float_is_finite(target_lq_h) || !mc_float_is_finite(target_sample_time_s) ||
        reference_rs_ohm <= 0.0f || reference_ld_h <= 0.0f ||
        reference_lq_h <= 0.0f || reference_sample_time_s <= 0.0f ||
        target_rs_ohm <= 0.0f || target_ld_h <= 0.0f ||
        target_lq_h <= 0.0f || target_sample_time_s <= 0.0f) {
        return MC_INVALID_ARGUMENT;
    }

    *target_kp_d = reference_kp_d * target_ld_h / reference_ld_h;
    *target_kp_q = reference_kp_q * target_lq_h / reference_lq_h;
    *target_ki_d_per_tick = reference_ki_d_per_tick *
                            target_rs_ohm / reference_rs_ohm *
                            target_sample_time_s / reference_sample_time_s;
    *target_ki_q_per_tick = reference_ki_q_per_tick *
                            target_rs_ohm / reference_rs_ohm *
                            target_sample_time_s / reference_sample_time_s;
    return (mc_float_is_finite(*target_kp_d) &&
            mc_float_is_finite(*target_ki_d_per_tick) &&
            mc_float_is_finite(*target_kp_q) &&
            mc_float_is_finite(*target_ki_q_per_tick)) ?
           MC_OK : MC_NUMERIC_ERROR;
}




