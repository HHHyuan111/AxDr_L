#ifndef MC_DEADTIME_TEST_H
#define MC_DEADTIME_TEST_H

#include "mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MC_DEADTIME_TEST_IDLE = 0,
    MC_DEADTIME_TEST_RUNNING,
    MC_DEADTIME_TEST_ABORTED,
    MC_DEADTIME_TEST_ERROR
} mc_deadtime_test_state_t;

typedef struct {
    float target_line_voltage_v;
    float minimum_vbus_v;
    float maximum_abs_d_axis_voltage_v;
    float maximum_abs_phase_current_a;
} mc_deadtime_test_config_t;

typedef struct {
    mc_deadtime_test_state_t state;
    mc_deadtime_test_config_t config;
    uint32_t sample_count;
} mc_deadtime_test_t;

void mc_deadtime_test_default_config(mc_deadtime_test_config_t *config);

/* Reference fixture uses theta_e=0, vq=0 and Vab=1.5*vd. */
mc_status_t mc_deadtime_line_voltage_to_d_axis(float line_voltage_v,
                                              float *d_axis_voltage_v);
mc_status_t mc_deadtime_test_start(mc_deadtime_test_t *test,
                                   const mc_deadtime_test_config_t *config,
                                   mc_command_t *command);
mc_status_t mc_deadtime_test_step(mc_deadtime_test_t *test,
                                  const mc_sample_t *sample,
                                  mc_command_t *command);
mc_status_t mc_deadtime_test_abort(mc_deadtime_test_t *test,
                                   mc_command_t *command);

#ifdef __cplusplus
}
#endif

#endif




