#include "mc_encoder_alignment.h"

#include <math.h>
#include <string.h>

mc_status_t mc_encoder_direction_from_motion(float mechanical_motion_rad,
                                             float minimum_motion_rad,
                                             int8_t *encoder_direction)
{
    if (!encoder_direction || !mc_float_is_finite(mechanical_motion_rad) ||
        !mc_float_is_finite(minimum_motion_rad) || minimum_motion_rad < 0.0f) {
        return MC_INVALID_ARGUMENT;
    }
    if (fabsf(mechanical_motion_rad) <= minimum_motion_rad)
        return MC_OUT_OF_RANGE;
    *encoder_direction = mechanical_motion_rad > 0.0f ? 1 : -1;
    return MC_OK;
}

mc_status_t mc_encoder_offset_to_raw(float theta_e_offset_rad,
                                     uint32_t pole_pairs,
                                     int8_t encoder_direction,
                                     uint32_t encoder_full_scale,
                                     uint32_t *motor_zero_raw)
{
    float raw_angle;
    double raw_count;

    if (!motor_zero_raw || !mc_float_is_finite(theta_e_offset_rad) ||
        pole_pairs == 0u || encoder_full_scale < 2u ||
        (encoder_direction != 1 && encoder_direction != -1)) {
        return MC_INVALID_ARGUMENT;
    }

    raw_angle = mc_wrap_0_2pi(-(float)encoder_direction *
                              theta_e_offset_rad / (float)pole_pairs);
    raw_count = (double)raw_angle * (double)encoder_full_scale /
                (double)MC_TWO_PI_F;
    *motor_zero_raw = (uint32_t)(raw_count + 0.5) % encoder_full_scale;
    return MC_OK;
}

mc_status_t mc_encoder_alignment_solve(
    const float *raw_mechanical_angle_rad,
    uint32_t sample_count,
    uint32_t pole_pairs,
    int8_t encoder_direction,
    float target_electrical_angle_rad,
    uint32_t encoder_full_scale,
    mc_encoder_alignment_method_t method,
    float minimum_resultant_ratio,
    mc_encoder_alignment_result_t *result)
{
    double mechanical_sum = 0.0;
    double electrical_sin_sum = 0.0;
    double electrical_cos_sum = 0.0;
    float electrical_mean;
    uint32_t i;

    if (!raw_mechanical_angle_rad || !result || sample_count == 0u ||
        pole_pairs == 0u || encoder_full_scale < 2u ||
        (encoder_direction != 1 && encoder_direction != -1) ||
        !mc_float_is_finite(target_electrical_angle_rad) ||
        !mc_float_is_finite(minimum_resultant_ratio) || minimum_resultant_ratio < 0.0f ||
        minimum_resultant_ratio > 1.0f ||
        (method != MC_ENCODER_ALIGN_REFERENCE_ARITHMETIC &&
         method != MC_ENCODER_ALIGN_CIRCULAR_ELECTRICAL)) {
        return MC_INVALID_ARGUMENT;
    }

    memset(result, 0, sizeof(*result));
    result->encoder_direction = encoder_direction;
    for (i = 0u; i < sample_count; ++i) {
        float corrected;
        float electrical;
        if (!mc_float_is_finite(raw_mechanical_angle_rad[i]))
            return MC_NUMERIC_ERROR;
        corrected = mc_wrap_0_2pi((float)encoder_direction *
                                  raw_mechanical_angle_rad[i]);
        electrical = mc_wrap_0_2pi((float)pole_pairs * corrected);
        mechanical_sum += corrected;
        electrical_sin_sum += sinf(electrical);
        electrical_cos_sum += cosf(electrical);
    }

    result->mean_corrected_mechanical_rad =
        (float)(mechanical_sum / (double)sample_count);
    result->electrical_resultant_ratio = (float)(
        sqrt(electrical_sin_sum * electrical_sin_sum +
             electrical_cos_sum * electrical_cos_sum) /
        (double)sample_count);
    if (result->electrical_resultant_ratio < minimum_resultant_ratio)
        return MC_REJECTED;

    if (method == MC_ENCODER_ALIGN_REFERENCE_ARITHMETIC) {
        electrical_mean = (float)pole_pairs *
                          result->mean_corrected_mechanical_rad;
    } else {
        electrical_mean = atan2f((float)electrical_sin_sum,
                                 (float)electrical_cos_sum);
    }
    result->theta_e_offset_rad = mc_wrap_0_2pi(
        target_electrical_angle_rad - electrical_mean);
    if (mc_encoder_offset_to_raw(result->theta_e_offset_rad, pole_pairs,
                                 encoder_direction, encoder_full_scale,
                                 &result->motor_zero_raw) != MC_OK) {
        return MC_NUMERIC_ERROR;
    }
    result->valid = true;
    return MC_OK;
}




