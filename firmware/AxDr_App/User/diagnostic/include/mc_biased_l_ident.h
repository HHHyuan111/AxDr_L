#ifndef MC_BIASED_L_IDENT_H
#define MC_BIASED_L_IDENT_H

#include "mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MC_BIASED_L_IDENT_IDLE = 0,
    MC_BIASED_L_IDENT_BIAS_SETTLE,
    MC_BIASED_L_IDENT_INJECT,
    MC_BIASED_L_IDENT_DONE,
    MC_BIASED_L_IDENT_ABORTED,
    MC_BIASED_L_IDENT_ERROR
} mc_biased_l_ident_state_t;

typedef enum {
    MC_BIASED_L_IDENT_FAULT_NONE = 0,
    MC_BIASED_L_IDENT_FAULT_BAD_CONFIG,
    MC_BIASED_L_IDENT_FAULT_DRIVE,
    MC_BIASED_L_IDENT_FAULT_VBUS_LOW,
    MC_BIASED_L_IDENT_FAULT_CURRENT_LIMIT,
    MC_BIASED_L_IDENT_FAULT_BIAS_TIMEOUT,
    MC_BIASED_L_IDENT_FAULT_NO_VALID_PAIR,
    MC_BIASED_L_IDENT_FAULT_NUMERIC
} mc_biased_l_ident_fault_t;

typedef struct {
    float control_period_s;

    /* The 3% bias ratio is the actual reference-code value. */
    bool auto_tune;
    float bias_ratio;
    float ripple_ratio;
    float manual_bias_current_a;
    float initial_injection_voltage_v;

    uint16_t level_ticks;
    uint16_t edge_skip_ticks;
    uint32_t bias_settle_ticks;
    uint32_t bias_stable_ticks;
    uint32_t bias_timeout_ticks;
    float bias_tolerance_ratio;
    float bias_integral_gain_v_per_a_s;

    uint32_t tune_pairs;
    uint32_t target_accepted_pairs;
    uint32_t max_measure_pair_multiplier;

    float min_bias_current_a;
    float max_bias_current_limit_ratio;
    float min_target_ripple_a;
    float min_current_delta_a;
    float min_positive_current_ratio;

    float min_injection_voltage_v;
    float max_injection_vbus_ratio;
    float tune_scale_min;
    float tune_scale_max;
    float tune_ripple_floor_a;

    float min_slope_difference_a_s;
    float min_valid_inductance_h;
    float max_valid_inductance_h;

    /* New motor/bridge values: these are intentionally not inferred. */
    float current_limit_a;
    float voltage_limit_v;
    float min_vbus_v;

    /* Kept as a request for the adapter/supervisor.  The portable core never
     * touches the target's dead-time global or PWM registers. */
    bool request_deadtime_compensation;
} mc_biased_l_ident_config_t;

typedef struct {
    float phase_inductance_h;
    float line_inductance_h;
    float bias_current_target_a;
    float tuned_injection_voltage_v;
    float injection_frequency_hz;
    uint32_t accepted_pairs;
    uint32_t rejected_pairs;
    uint32_t tune_pairs;
    bool valid;
} mc_biased_l_ident_result_t;

typedef struct {
    float positive_slope_a_s;
    float negative_slope_a_s;
    float positive_current_avg_a;
    float negative_current_avg_a;
    float positive_voltage_avg_v;
    float negative_voltage_avg_v;
} mc_biased_l_pair_t;

typedef struct {
    mc_biased_l_ident_state_t state;
    mc_biased_l_ident_fault_t fault;
    mc_biased_l_ident_config_t cfg;
    mc_biased_l_ident_result_t result;

    float phase_resistance_ohm;
    float bias_target_a;
    float target_ripple_a;
    float base_voltage_v;
    float injection_voltage_v;

    uint32_t settle_tick;
    uint32_t stable_tick;
    uint32_t level_tick;
    uint32_t pair_count;
    int8_t level_sign;

    float edge_current_a;
    float previous_current_a;
    double current_integral;
    double voltage_integral;

    float positive_slope_a_s;
    float positive_current_avg_a;
    float positive_voltage_avg_v;
    bool positive_valid;

    double inductance_sum_h;
    float last_inductance_h;
    float last_ripple_a;
} mc_biased_l_ident_t;

void mc_biased_l_ident_default_config(mc_biased_l_ident_config_t *cfg);

mc_status_t mc_biased_l_ident_start(mc_biased_l_ident_t *ctx,
                                    const mc_biased_l_ident_config_t *cfg,
                                    float phase_resistance_ohm,
                                    float initial_vbus_v);
mc_status_t mc_biased_l_ident_step(mc_biased_l_ident_t *ctx,
                                   const mc_sample_t *sample,
                                   mc_command_t *command);
mc_status_t mc_biased_l_ident_abort(mc_biased_l_ident_t *ctx,
                                    mc_command_t *command);

/* Pure two-level estimator:
 * L = |(V+ - V-) - Rs*(I+ - I-)| / (slope+ - slope-)
 */
mc_status_t mc_biased_l_ident_estimate_pair(
    const mc_biased_l_pair_t *pair,
    float phase_resistance_ohm,
    float min_slope_difference_a_s,
    float *phase_inductance_h);

#ifdef __cplusplus
}
#endif

#endif




