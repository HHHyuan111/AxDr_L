#include "mc_deadtime_test.h"

#include <math.h>
#include <string.h>

static void deadtime_stop_command(mc_command_t *command)
{
    memset(command, 0, sizeof(*command));
    command->disable_request = true;
}

static void deadtime_run_command(float d_axis_voltage_v,
                                 mc_command_t *command)
{
    memset(command, 0, sizeof(*command));
    command->mode = MC_CONTROL_VOLTAGE;
    command->vd_ref_v = d_axis_voltage_v;
    command->vq_ref_v = 0.0f;
    command->openloop_theta_e_rad = 0.0f;
    command->openloop_enable = true;
    command->enable_request = true;
}

void mc_deadtime_test_default_config(mc_deadtime_test_config_t *config)
{
    if (!config) return;
    memset(config, 0, sizeof(*config));
    config->target_line_voltage_v = 0.15f;
    /* The target adapter must explicitly provide safe voltage/current bounds. */
    config->maximum_abs_d_axis_voltage_v = 0.0f;
    config->maximum_abs_phase_current_a = 0.0f;
}

mc_status_t mc_deadtime_line_voltage_to_d_axis(float line_voltage_v,
                                              float *d_axis_voltage_v)
{
    if (!d_axis_voltage_v || !mc_float_is_finite(line_voltage_v))
        return MC_INVALID_ARGUMENT;
    *d_axis_voltage_v = line_voltage_v / 1.5f;
    return MC_OK;
}

mc_status_t mc_deadtime_test_start(mc_deadtime_test_t *test,
                                   const mc_deadtime_test_config_t *config,
                                   mc_command_t *command)
{
    float vd = 0.0f;
    if (!test || !config || !command ||
        !mc_float_is_finite(config->target_line_voltage_v) ||
        !mc_float_is_finite(config->minimum_vbus_v) ||
        !mc_float_is_finite(config->maximum_abs_d_axis_voltage_v) ||
        !mc_float_is_finite(config->maximum_abs_phase_current_a) ||
        config->minimum_vbus_v < 0.0f ||
        config->maximum_abs_d_axis_voltage_v <= 0.0f ||
        config->maximum_abs_phase_current_a <= 0.0f) {
        return MC_INVALID_ARGUMENT;
    }
    if (mc_deadtime_line_voltage_to_d_axis(
            config->target_line_voltage_v, &vd) != MC_OK)
        return MC_INVALID_ARGUMENT;
    if (fabsf(vd) > config->maximum_abs_d_axis_voltage_v)
        return MC_OUT_OF_RANGE;
    memset(test, 0, sizeof(*test));
    test->config = *config;
    test->state = MC_DEADTIME_TEST_RUNNING;
    deadtime_run_command(vd, command);
    return MC_OK;
}

mc_status_t mc_deadtime_test_step(mc_deadtime_test_t *test,
                                  const mc_sample_t *sample,
                                  mc_command_t *command)
{
    float vd = 0.0f;
    if (!test || !sample || !command ||
        test->state != MC_DEADTIME_TEST_RUNNING) return MC_REJECTED;
    if (!mc_float_is_finite(sample->vbus_v) ||
        !mc_float_is_finite(sample->ia_a) ||
        !mc_float_is_finite(sample->ib_a) ||
        !mc_float_is_finite(sample->ic_a) ||
        !mc_float_is_finite(sample->current_limit_a)) {
        deadtime_stop_command(command);
        test->state = MC_DEADTIME_TEST_ERROR;
        return MC_NUMERIC_ERROR;
    }
    if (fabsf(sample->ia_a) > test->config.maximum_abs_phase_current_a ||
        fabsf(sample->ib_a) > test->config.maximum_abs_phase_current_a ||
        fabsf(sample->ic_a) > test->config.maximum_abs_phase_current_a ||
        (sample->current_limit_a > 0.0f &&
         test->config.maximum_abs_phase_current_a >
             sample->current_limit_a)) {
        deadtime_stop_command(command);
        test->state = MC_DEADTIME_TEST_ERROR;
        return MC_OUT_OF_RANGE;
    }
    if (sample->fault_code != 0u ||
        (test->config.minimum_vbus_v > 0.0f &&
         sample->vbus_v < test->config.minimum_vbus_v)) {
        deadtime_stop_command(command);
        test->state = MC_DEADTIME_TEST_ERROR;
        return MC_FAULT;
    }
    if (mc_deadtime_line_voltage_to_d_axis(
            test->config.target_line_voltage_v, &vd) != MC_OK) {
        deadtime_stop_command(command);
        test->state = MC_DEADTIME_TEST_ERROR;
        return MC_NUMERIC_ERROR;
    }
    deadtime_run_command(vd, command);
    test->sample_count++;
    return MC_BUSY;
}

mc_status_t mc_deadtime_test_abort(mc_deadtime_test_t *test,
                                   mc_command_t *command)
{
    if (!test || !command) return MC_INVALID_ARGUMENT;
    deadtime_stop_command(command);
    test->state = MC_DEADTIME_TEST_ABORTED;
    return MC_ABORTED;
}




