#ifndef MC_ENCODER_ALIGNMENT_H
#define MC_ENCODER_ALIGNMENT_H

#include "mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    /* Reproduces the reference arithmetic mechanical-angle averaging rule. */
    MC_ENCODER_ALIGN_REFERENCE_ARITHMETIC = 0,
    /* Recommended portable option when samples may cross 0/2*pi. */
    MC_ENCODER_ALIGN_CIRCULAR_ELECTRICAL
} mc_encoder_alignment_method_t;

typedef struct {
    int8_t encoder_direction;
    float theta_e_offset_rad;
    float mean_corrected_mechanical_rad;
    float electrical_resultant_ratio;
    uint32_t motor_zero_raw;
    bool valid;
} mc_encoder_alignment_result_t;

mc_status_t mc_encoder_direction_from_motion(float mechanical_motion_rad,
                                             float minimum_motion_rad,
                                             int8_t *encoder_direction);

mc_status_t mc_encoder_alignment_solve(
    const float *raw_mechanical_angle_rad,
    uint32_t sample_count,
    uint32_t pole_pairs,
    int8_t encoder_direction,
    float target_electrical_angle_rad,
    uint32_t encoder_full_scale,
    mc_encoder_alignment_method_t method,
    float minimum_resultant_ratio,
    mc_encoder_alignment_result_t *result);

/* Inverse of theta_e = wrap(pp*dir*theta_raw + theta_offset). */
mc_status_t mc_encoder_offset_to_raw(float theta_e_offset_rad,
                                     uint32_t pole_pairs,
                                     int8_t encoder_direction,
                                     uint32_t encoder_full_scale,
                                     uint32_t *motor_zero_raw);

#ifdef __cplusplus
}
#endif

#endif




