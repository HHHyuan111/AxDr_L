#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "diag_runtime.h"

static bool near_value(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static diag_seed_t make_pr60_seed(void)
{
    const diag_seed_t seed = {
        .control_period_s = 50.0e-6f,
        .phase_resistance_ohm = 0.162977806f,
        .d_axis_inductance_h = 0.000108778855f,
        .q_axis_inductance_h = 0.000112416135f,
        .flux_linkage_wb = 0.00498822471f,
        .pole_pairs = 10U,
        .encoder_full_scale = 16384U,
        .encoder_direction = 1,
    };

    return seed;
}

static void set_test_limits(diag_runtime_t *runtime)
{
    runtime->profile.current_limit_a = 1.0f;
    runtime->profile.voltage_limit_v = 2.0f;
    runtime->profile.minimum_vbus_v = 10.0f;
}

static void test_default_profile_cannot_start_power(void)
{
    const diag_seed_t seed = make_pr60_seed();
    diag_runtime_t runtime;

    diag_runtime_init(&runtime, &seed);
    runtime.requested_job = DIAG_JOB_CURRENT_SWEEP;
    assert(runtime.profile.current_limit_a == 0.0f);
    assert(runtime.profile.voltage_limit_v == 0.0f);
    assert(diag_runtime_start(&runtime, 24.0f, true)
           == MC_INVALID_ARGUMENT);
    assert(!runtime.active);
}

static void test_sweep_start_abort_and_stop(void)
{
    const diag_seed_t seed = make_pr60_seed();
    diag_runtime_t runtime;
    mc_command_t command;

    diag_runtime_init(&runtime, &seed);
    set_test_limits(&runtime);
    runtime.requested_job = DIAG_JOB_CURRENT_SWEEP;
    assert(diag_runtime_start(&runtime, 24.0f, false) == MC_REJECTED);
    assert(diag_runtime_start(&runtime, 24.0f, true) == MC_OK);
    assert(runtime.active);
    assert(runtime.manager.state == MC_DIAG_RUNNING);

    memset(&command, 0, sizeof(command));
    assert(diag_runtime_step(&runtime, &runtime.last_sample, &command)
           == MC_BUSY);
    assert(command.mode == MC_CONTROL_CURRENT);
    assert(command.enable_request);

    assert(diag_runtime_abort(&runtime, &command) == MC_ABORTED);
    assert(command.disable_request);
    assert(runtime.manager.state == MC_DIAG_STOPPING);
    diag_runtime_confirm_stopped(&runtime, true);
    assert(!runtime.active);
    assert(runtime.manager.owner == MC_DIAG_OWNER_NONE);
    assert(runtime.last_status == MC_ABORTED);
}

static void test_rs_requires_explicit_current_points(void)
{
    const diag_seed_t seed = make_pr60_seed();
    diag_runtime_t runtime;
    mc_command_t command;

    diag_runtime_init(&runtime, &seed);
    set_test_limits(&runtime);
    runtime.requested_job = DIAG_JOB_RS_IDENT;
    assert(diag_runtime_start(&runtime, 24.0f, true)
           == MC_INVALID_ARGUMENT);

    runtime.profile.rs.current_min_a = 0.2f;
    runtime.profile.rs.current_max_a = 0.4f;
    runtime.profile.rs.total_duration_s = 0.1f;
    assert(diag_runtime_start(&runtime, 24.0f, true) == MC_OK);
    assert(diag_runtime_abort(&runtime, &command) == MC_ABORTED);
    diag_runtime_confirm_stopped(&runtime, true);
}

static void test_project_pi_unit_conversion(void)
{
    float kp_d;
    float ki_d;
    float kp_q;
    float ki_q;
    const float bandwidth_hz = 300.0f;
    const float resistance_ohm = 0.16f;

    assert(diag_current_pi_for_project(
        bandwidth_hz,
        50.0e-6f,
        resistance_ohm,
        100.0e-6f,
        110.0e-6f,
        &kp_d,
        &ki_d,
        &kp_q,
        &ki_q) == MC_OK);
    assert(near_value(ki_d,
                      resistance_ohm * MC_TWO_PI_F * bandwidth_hz,
                      1.0e-4f));
    assert(near_value(ki_d, ki_q, 1.0e-6f));
    assert(kp_q > kp_d);
}

int main(void)
{
    test_default_profile_cannot_start_power();
    test_sweep_start_abort_and_stop();
    test_rs_requires_explicit_current_points();
    test_project_pi_unit_conversion();
    puts("diagnostic runtime tests: PASS");
    return 0;
}
