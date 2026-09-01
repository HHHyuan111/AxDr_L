#ifndef MC_RS_IDENT_H
#define MC_RS_IDENT_H

#include "mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The reference implementation uses six plateaus.  The larger fixed capacity
 * lets another motor use more points without dynamic allocation. */
#define MC_RS_IDENT_DEFAULT_POINTS  (6u)
#define MC_RS_IDENT_MAX_POINTS      (16u)

typedef enum {
    MC_RS_IDENT_IDLE = 0,
    MC_RS_IDENT_RUNNING,
    MC_RS_IDENT_DONE,
    MC_RS_IDENT_ABORTED,
    MC_RS_IDENT_ERROR
} mc_rs_ident_state_t;

typedef enum {
    MC_RS_IDENT_FAULT_NONE = 0,
    MC_RS_IDENT_FAULT_BAD_CONFIG,
    MC_RS_IDENT_FAULT_DRIVE,
    MC_RS_IDENT_FAULT_VBUS_LOW,
    MC_RS_IDENT_FAULT_CURRENT_LIMIT,
    MC_RS_IDENT_FAULT_DEGENERATE,
    MC_RS_IDENT_FAULT_NUMERIC
} mc_rs_ident_fault_t;

typedef struct {
    /* Reference defaults: 6 A .. 12 A, six equally spaced points, 8 s total. These
     * are experiment settings, not universal safe values.  A new target must
     * replace them according to motor/bridge thermal and current limits. */
    float current_min_a;
    float current_max_a;
    uint8_t point_count;
    float total_duration_s;
    float settling_fraction;

    /* v[k+1] = v[k] + integral_gain * dt * (i_ref - i_meas). */
    float integral_gain_v_per_a_s;
    float control_period_s;

    /* Explicit porting guards.  safe_current_limit_a is mandatory.  A zero
     * voltage limit preserves the source algorithm's unclamped integrator;
     * production adapters should normally provide a positive limit. */
    float safe_current_limit_a;
    float max_abs_voltage_v;
    float min_vbus_v;

    /* Pairwise least-squares denominator threshold from the source code. */
    double denominator_epsilon;
} mc_rs_ident_config_t;

typedef struct {
    float phase_resistance_ohm;
    uint8_t point_count;
    float voltage_avg_v[MC_RS_IDENT_MAX_POINTS];
    float current_avg_a[MC_RS_IDENT_MAX_POINTS];
    uint32_t sample_count[MC_RS_IDENT_MAX_POINTS];
    bool voltage_was_clamped;
    bool valid;
} mc_rs_ident_result_t;

typedef struct {
    mc_rs_ident_state_t state;
    mc_rs_ident_fault_t fault;
    mc_rs_ident_config_t cfg;
    mc_rs_ident_result_t result;

    uint32_t tick;
    uint32_t total_ticks;
    uint32_t ticks_per_point;
    float current_target_a;
    float voltage_integrator_v;
    double voltage_sum[MC_RS_IDENT_MAX_POINTS];
    double current_sum[MC_RS_IDENT_MAX_POINTS];
} mc_rs_ident_t;

/* Populate behavior-compatible excitation defaults. safe_current_limit_a stays
 * zero so the caller must explicitly confirm a motor/bridge limit. */
void mc_rs_ident_default_config(mc_rs_ident_config_t *cfg);

mc_status_t mc_rs_ident_start(mc_rs_ident_t *ctx,
                              const mc_rs_ident_config_t *cfg);
mc_status_t mc_rs_ident_step(mc_rs_ident_t *ctx,
                             const mc_sample_t *sample,
                             mc_command_t *command);
mc_status_t mc_rs_ident_abort(mc_rs_ident_t *ctx, mc_command_t *command);

/* Pure result solver used by host tests and import tools.  It implements the
 * all-pairs differential regression:
 *     Rs = sum(dV_ij*dI_ij) / sum(dI_ij^2)
 */
mc_status_t mc_rs_ident_solve(const float *voltage_avg_v,
                              const float *current_avg_a,
                              uint8_t point_count,
                              double denominator_epsilon,
                              float *phase_resistance_ohm);

#ifdef __cplusplus
}
#endif

#endif




