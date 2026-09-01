#ifndef MC_POLE_PAIR_IDENT_H
#define MC_POLE_PAIR_IDENT_H

#include "mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MC_POLE_PAIR_IDENT_IDLE = 0,
    MC_POLE_PAIR_IDENT_RAMP,
    MC_POLE_PAIR_IDENT_ROTATE,
    MC_POLE_PAIR_IDENT_DONE,
    MC_POLE_PAIR_IDENT_ABORTED,
    MC_POLE_PAIR_IDENT_ERROR
} mc_pole_pair_ident_state_t;

typedef enum {
    MC_POLE_PAIR_IDENT_FAULT_NONE = 0,
    MC_POLE_PAIR_IDENT_FAULT_BAD_CONFIG,
    MC_POLE_PAIR_IDENT_FAULT_DRIVE,
    MC_POLE_PAIR_IDENT_FAULT_VBUS_LOW,
    MC_POLE_PAIR_IDENT_FAULT_CURRENT_LIMIT,
    MC_POLE_PAIR_IDENT_FAULT_NO_MOTION,
    MC_POLE_PAIR_IDENT_FAULT_OUT_OF_RANGE,
    MC_POLE_PAIR_IDENT_FAULT_NUMERIC
} mc_pole_pair_ident_fault_t;

typedef struct {
    float control_period_s;
    float phase_resistance_ohm;

    /* Reference defaults: 8 A-equivalent open-loop voltage, 16 electrical
     * turns, pi rad/s, and a 2 s voltage ramp. */
    float drive_current_a;
    float electrical_turns;
    float electrical_velocity_rad_s;
    float ramp_duration_s;

    uint32_t encoder_full_scale;
    int32_t min_pole_pairs;
    int32_t max_pole_pairs;

    /* Explicit target safety limits. */
    float safe_current_limit_a;
    float max_abs_voltage_v;
    float min_vbus_v;
} mc_pole_pair_ident_config_t;

typedef struct {
    int32_t pole_pairs;
    int8_t encoder_direction;
    int64_t accumulated_counts;
    float mechanical_turns;
    float pole_pairs_unrounded;
    bool valid;
} mc_pole_pair_ident_result_t;

typedef struct {
    mc_pole_pair_ident_state_t state;
    mc_pole_pair_ident_fault_t fault;
    mc_pole_pair_ident_config_t cfg;
    mc_pole_pair_ident_result_t result;

    uint32_t ramp_tick;
    uint32_t ramp_ticks;
    uint32_t previous_encoder_raw;
    int64_t accumulated_counts;
    float electrical_phase_rad;
    float drive_voltage_v;
} mc_pole_pair_ident_t;

void mc_pole_pair_ident_default_config(mc_pole_pair_ident_config_t *cfg);

mc_status_t mc_pole_pair_ident_start(mc_pole_pair_ident_t *ctx,
                                     const mc_pole_pair_ident_config_t *cfg);
mc_status_t mc_pole_pair_ident_step(mc_pole_pair_ident_t *ctx,
                                    const mc_sample_t *sample,
                                    mc_command_t *command);
mc_status_t mc_pole_pair_ident_abort(mc_pole_pair_ident_t *ctx,
                                     mc_command_t *command);

/* Incremental single-turn encoder unwrap. */
int32_t mc_encoder_unwrap_delta(uint32_t current_raw,
                                uint32_t previous_raw,
                                uint32_t full_scale);

/* Pure final calculation used by host tests and replay tools. */
mc_status_t mc_pole_pair_ident_solve(int64_t accumulated_counts,
                                     uint32_t encoder_full_scale,
                                     float electrical_turns,
                                     int32_t min_pole_pairs,
                                     int32_t max_pole_pairs,
                                     mc_pole_pair_ident_result_t *result);

#ifdef __cplusplus
}
#endif

#endif




