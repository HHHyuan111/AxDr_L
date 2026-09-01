#ifndef MC_PROFILES_H
#define MC_PROFILES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float line_resistance_ohm;
    float line_inductance_h;
    float ld_h;
    float lq_h;
    float flux_wb;
    float kt_nm_per_a;
    float rated_current_a;
    float peak_current_a;
    float rated_speed_rpm;
    float max_speed_rpm;
    float inertia_kg_m2;
    uint32_t pole_pairs;
    uint32_t encoder_full_scale;
} mc_motor_profile_t;

typedef struct {
    float nominal_vbus_v;
    float max_vbus_v;
    float control_frequency_hz;
    float pwm_frequency_hz;
    float hardware_deadtime_ns;
    float compensation_deadtime_ns;
    float current_blend_a;
    float mos_rds_on_ohm;
    float diode_vf_v;
    float current_gain_a_per_count;
    float current_offset_count;
    uint32_t adc_bits;
} mc_inverter_profile_t;

typedef struct {
    float current_kp_d;
    float current_ki_d_per_tick;
    float current_kp_q;
    float current_ki_q_per_tick;
    float feedback_filter_alpha;
    float current_limit_a;
    float voltage_limit_v;
} mc_control_profile_t;

#ifdef __cplusplus
}
#endif

#endif




