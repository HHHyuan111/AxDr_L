#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "algorithm_config.h"
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
    assert(runtime.profile.sweep.axis == DIAG_SWEEP_AXIS);
    assert(runtime.profile.sweep.single_point == DIAG_SWEEP_SINGLE_POINT);
    assert(runtime.profile.sweep.single_frequency_hz
           == DIAG_SWEEP_SINGLE_FREQ_HZ);
    assert(runtime.profile.sweep.start_frequency_hz
           == DIAG_SWEEP_START_FREQ_HZ);
    assert(runtime.profile.sweep.end_frequency_hz
           == DIAG_SWEEP_END_FREQ_HZ);
    assert(runtime.profile.sweep.amplitude_a == DIAG_SWEEP_AMPLITUDE_A);
    assert(runtime.profile.sweep.feedback_filter_alpha
           == DIAG_SWEEP_FEEDBACK_FILTER_ALPHA);
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

static void test_param_ident_gate_and_bridge(void)
{
    const diag_seed_t seed = make_pr60_seed();
    diag_runtime_t runtime;
    mc_command_t command;

    diag_runtime_init(&runtime, &seed);
    runtime.requested_job = DIAG_JOB_PARAM_IDENT;
    /* 闸门未开（默认限值 0）：runtime 层拒绝，与核的 start 校验同语义。 */
    assert(diag_runtime_start(&runtime, 24.0f, true)
           == MC_INVALID_ARGUMENT);
    assert(!runtime.active);

    /* 限值开后可启动；桥接断言：seed 透传、profile 限值覆盖核默认。 */
    set_test_limits(&runtime);
    assert(diag_runtime_start(&runtime, 24.0f, true) == MC_OK);
    assert(runtime.active);
    assert(runtime.manager.owner == MC_DIAG_OWNER_PARAM);
    assert(runtime.param_ident.state == MC_PARAM_IDENT_PREPARE);
    assert(runtime.param_ident.cfg.control_period_s
           == seed.control_period_s);
    assert(runtime.param_ident.cfg.pole_pairs
           == (float)seed.pole_pairs);
    assert(runtime.param_ident.cfg.current_limit_a == 1.0f);
    assert(runtime.param_ident.cfg.voltage_limit_v == 2.0f);
    assert(runtime.param_ident.cfg.vbus_min_v == 10.0f);

    assert(diag_runtime_abort(&runtime, &command) == MC_ABORTED);
    assert(command.disable_request);
    diag_runtime_confirm_stopped(&runtime, true);
    assert(!runtime.active);
}

static void start_abort_and_release(diag_runtime_t *runtime)
{
    mc_command_t command;

    assert(diag_runtime_start(runtime, 24.0f, true) == MC_OK);
    assert(runtime->active);
    assert(diag_runtime_abort(runtime, &command) == MC_ABORTED);
    assert(command.disable_request);
    diag_runtime_confirm_stopped(runtime, true);
    assert(!runtime->active);
}

static void test_all_active_job_routes(void)
{
    const diag_seed_t seed = make_pr60_seed();
    diag_runtime_t runtime;

    diag_runtime_init(&runtime, &seed);
    set_test_limits(&runtime);
    runtime.requested_job = DIAG_JOB_PARAM_IDENT;
    start_abort_and_release(&runtime);

    diag_runtime_init(&runtime, &seed);
    set_test_limits(&runtime);
    runtime.requested_job = DIAG_JOB_PARAM_IDENT_LQ;
    start_abort_and_release(&runtime);

    diag_runtime_init(&runtime, &seed);
    set_test_limits(&runtime);
    runtime.profile.pole_pair.drive_current_a = 0.5f;
    runtime.requested_job = DIAG_JOB_POLE_PAIR_IDENT;
    start_abort_and_release(&runtime);

    diag_runtime_init(&runtime, &seed);
    set_test_limits(&runtime);
    runtime.profile.encoder_align.align_current_a = 0.5f;
    runtime.requested_job = DIAG_JOB_ENCODER_ALIGN;
    start_abort_and_release(&runtime);

    diag_runtime_init(&runtime, &seed);
    set_test_limits(&runtime);
    runtime.requested_job = DIAG_JOB_DEADTIME_TEST;
    start_abort_and_release(&runtime);
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
    test_param_ident_gate_and_bridge();
    test_all_active_job_routes();
    test_project_pi_unit_conversion();
    puts("diagnostic runtime tests: PASS");
    return 0;
}
