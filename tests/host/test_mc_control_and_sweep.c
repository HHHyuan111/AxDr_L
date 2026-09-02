#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "mc_bias_bandwidth.h"
#include "mc_current_pi.h"
#include "mc_current_sweep.h"
#include "mc_deadtime_comp.h"
#include "mc_deadtime_test.h"
#include "mc_decoupling.h"
#include "mc_diag_manager.h"
#include "mc_encoder_alignment.h"
#include "mc_flux_observer.h"

static int nearf(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

static float make_nan(void)
{
    const uint32_t bits = UINT32_C(0x7FC00000);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void test_decoupling(void)
{
    mc_decoupling_config_t cfg = {0};
    mc_decoupling_input_t in = {0};
    mc_decoupling_output_t out;
    float kp_d, ki_d, kp_q, ki_q;

    assert(!mc_float_is_finite(make_nan()));

    cfg.rs_ohm = 0.12f;
    cfg.ld_h = 300.0e-6f;
    cfg.lq_h = 350.0e-6f;
    cfg.flux_wb = 0.018f;
    cfg.pole_pairs = 7.0f;
    cfg.dt_s = 100e-6f;
    in.id_ref_a = 1.0f;
    in.iq_ref_a = 3.0f;
    in.error_d_a = 0.2f;
    in.error_q_a = -0.1f;
    in.omega_mech_rad_s = 8.0f;
    in.kp_d = 0.25f;
    in.kp_q = 0.25f;

    assert(mc_decoupling_calculate(MC_DECOUPLING_MODEL_FEEDFORWARD, &cfg, &in, &out) == MC_OK);
    assert(nearf(out.vd_feedforward_v, -56.0f * 350.0e-6f * 3.0f, 1e-6f));
    assert(nearf(out.vq_feedforward_v,
                 56.0f * (300.0e-6f + 0.018f), 1e-5f));

    assert(mc_decoupling_calculate(MC_DECOUPLING_COMPLEX_VECTOR, &cfg, &in, &out) == MC_OK);
    assert(out.id_integrator_delta_v > 0.0f);
    assert(out.iq_integrator_delta_v > 0.0f);
    in.omega_mech_rad_s = make_nan();
    assert(mc_decoupling_calculate(MC_DECOUPLING_COMPLEX_VECTOR,
                                    &cfg, &in, &out) == MC_INVALID_ARGUMENT);
    in.omega_mech_rad_s = 8.0f;

    assert(mc_current_pi_from_bandwidth(300.0f, 100e-6f, 0.12f,
                                        300.0e-6f, 350.0e-6f,
                                        &kp_d, &ki_d, &kp_q, &ki_q) == MC_OK);
    assert(nearf(kp_d, 300.0e-6f * MC_TWO_PI_F * 300.0f, 1e-5f));
    assert(nearf(ki_d, 0.12f * MC_TWO_PI_F * 300.0f * 100e-6f, 1e-6f));
    assert(nearf(ki_d, ki_q, 1e-8f));

    assert(mc_current_pi_scale_from_reference(
        0.8f, 0.035f, 0.8f, 0.035f,
        0.15f, 300.0e-6f, 320.0e-6f, 50.0e-6f,
        0.10f, 200.0e-6f, 250.0e-6f, 100.0e-6f,
        &kp_d, &ki_d, &kp_q, &ki_q) == MC_OK);
    assert(nearf(kp_d, 0.8f * 200.0e-6f / 300.0e-6f, 1e-6f));
    assert(nearf(ki_d, 0.035f * 0.10f / 0.15f * 2.0f, 1e-6f));
}

static void test_deadtime(void)
{
    mc_deadtime_config_t cfg = {800.0f, 0.4f, 0.002f, 0.9f, 1.0f};
    mc_deadtime_output_t out;
    assert(mc_deadtime_compensate(&cfg, 2.0f, -1.0f, -1.0f,
                                  24.0f, 100e-6f, &out) == MC_OK);
    assert(nearf(out.fixed_drop_v, 0.1992f, 0.001f));
    assert(out.dv_alpha_v > 0.0f);
    assert(nearf(out.dv_beta_v, 0.0f, 1e-6f));
    cfg.polarity = 0.0f;
    assert(mc_deadtime_compensate(&cfg, 2.0f, -1.0f, -1.0f,
                                  24.0f, 100e-6f, &out) == MC_INVALID_ARGUMENT);
}

static void test_deadtime_fixture(void)
{
    mc_deadtime_test_config_t cfg;
    mc_deadtime_test_t test;
    mc_command_t command;
    mc_sample_t sample = {0};
    float vd;

    assert(mc_deadtime_line_voltage_to_d_axis(0.15f, &vd) == MC_OK);
    assert(nearf(vd, 0.10f, 1e-7f));
    mc_deadtime_test_default_config(&cfg);
    assert(mc_deadtime_test_start(&test, &cfg, &command) == MC_INVALID_ARGUMENT);
    cfg.maximum_abs_d_axis_voltage_v = 0.2f;
    cfg.maximum_abs_phase_current_a = 5.0f;
    cfg.minimum_vbus_v = 10.0f;
    assert(mc_deadtime_test_start(&test, &cfg, &command) == MC_OK);
    assert(command.openloop_enable && command.enable_request);
    assert(nearf(command.vd_ref_v, 0.10f, 1e-7f));
    sample.vbus_v = 24.0f;
    sample.ia_a = 6.0f;
    assert(mc_deadtime_test_step(&test, &sample, &command) == MC_OUT_OF_RANGE);
    assert(command.disable_request);
    sample.ia_a = 0.0f;
    assert(mc_deadtime_test_start(&test, &cfg, &command) == MC_OK);
    sample.vbus_v = 9.0f;
    assert(mc_deadtime_test_step(&test, &sample, &command) == MC_FAULT);
    assert(command.disable_request);
}

static void test_encoder_alignment(void)
{
    const float samples[] = {
        MC_TWO_PI_F - 0.003f,
        MC_TWO_PI_F - 0.001f,
        0.001f,
        0.003f
    };
    mc_encoder_alignment_result_t result;
    float reconstructed;

    assert(mc_encoder_direction_from_motion(-1.0f, 0.5f,
                                             &result.encoder_direction) == MC_OK);
    assert(result.encoder_direction == -1);
    assert(mc_encoder_alignment_solve(
        samples, 4u, 3u, 1, 0.0f, 65536u,
        MC_ENCODER_ALIGN_CIRCULAR_ELECTRICAL, 0.9f, &result) == MC_OK);
    assert(result.valid);
    assert(result.electrical_resultant_ratio > 0.99f);
    assert(result.theta_e_offset_rad < 0.02f ||
           result.theta_e_offset_rad > MC_TWO_PI_F - 0.02f);
    reconstructed = mc_wrap_0_2pi(
        3.0f * MC_TWO_PI_F * (float)result.motor_zero_raw / 65536.0f +
        result.theta_e_offset_rad);
    assert(reconstructed < 0.001f || reconstructed > MC_TWO_PI_F - 0.001f);
}

static void test_diag_manager(void)
{
    mc_diag_manager_t manager;
    mc_diag_manager_init(&manager);
    assert(mc_diag_acquire(&manager, MC_DIAG_OWNER_CURRENT_SWEEP, true) == MC_OK);
    assert(mc_diag_acquire(&manager, MC_DIAG_OWNER_FLUX, true) == MC_BUSY);
    assert(mc_diag_begin(&manager, MC_DIAG_OWNER_CURRENT_SWEEP) == MC_OK);
    assert(mc_diag_finish(&manager, MC_DIAG_OWNER_CURRENT_SWEEP, MC_DONE) == MC_OK);
    assert(mc_diag_release(&manager, MC_DIAG_OWNER_CURRENT_SWEEP) == MC_REJECTED);
    assert(mc_diag_confirm_stopped(&manager, MC_DIAG_OWNER_CURRENT_SWEEP,
                                   false) == MC_BUSY);
    assert(mc_diag_confirm_stopped(&manager, MC_DIAG_OWNER_CURRENT_SWEEP,
                                   true) == MC_OK);
    assert(mc_diag_release(&manager, MC_DIAG_OWNER_CURRENT_SWEEP) == MC_OK);
    assert(manager.owner == MC_DIAG_OWNER_NONE);
}

static void test_flux(void)
{
    mc_flux_observer_t observer;
    mc_flux_config_t cfg = {0.12f, 7.0f, 0.001f, 10.0f, 5.0f, true};
    mc_sample_t sample = {0};
    mc_flux_observer_init(&observer);
    sample.omega_mech_rad_s = 8.0f;
    sample.iq_a = 2.0f;
    sample.vq_v = 0.12f * 2.0f + 56.0f * 0.018f;
    assert(mc_flux_observer_step(&observer, &cfg, &sample) == MC_OK);
    assert(nearf(observer.psi_d_instant_wb, 0.018f, 1e-6f));
    cfg.minimum_electrical_speed_rad_s = 0.0f;
    assert(mc_flux_observer_step(&observer, &cfg, &sample) == MC_INVALID_ARGUMENT);
    cfg.minimum_electrical_speed_rad_s = 5.0f;
    sample.omega_mech_rad_s = make_nan();
    assert(mc_flux_observer_step(&observer, &cfg, &sample) == MC_INVALID_ARGUMENT);
    sample.omega_mech_rad_s = -8.0f;
    sample.vq_v = 0.12f * 2.0f - 56.0f * 0.018f;
    assert(mc_flux_observer_step(&observer, &cfg, &sample) == MC_OK);
    assert(nearf(observer.psi_d_instant_wb, 0.018f, 1e-6f));
}

static void test_sweep(void)
{
    mc_current_sweep_t sweep;
    mc_current_sweep_config_t cfg;
    mc_command_t command;
    mc_sample_t sample;
    uint32_t guard;

    mc_current_sweep_default_config(&cfg);
    assert(mc_current_sweep_start(&sweep, &cfg, 50e-6f, &command) ==
           MC_INVALID_ARGUMENT);
    cfg.single_point = true;
    cfg.single_frequency_hz = 100.0f;
    cfg.amplitude_a = 0.5f;
    cfg.safe_current_limit_a = 2.0f;
    cfg.settle_cycles = 2u;
    cfg.measure_cycles = 10u;
    cfg.minimum_settle_time_s = 0.0f;
    cfg.minimum_measure_time_s = 0.0f;
    cfg.minimum_measure_samples = 100u;
    cfg.feedback_filter_alpha = 0.0f;

    /* 频率范围由项目配置决定，通用算法只检查正频率和奈奎斯特边界。 */
    cfg.single_frequency_hz = 10.0f;
    assert(mc_current_sweep_start(&sweep, &cfg, 50e-6f, &command) == MC_OK);
    assert(mc_current_sweep_abort(&sweep, &command) == MC_ABORTED);
    cfg.single_frequency_hz = 10000.0f;
    assert(mc_current_sweep_start(&sweep, &cfg, 50e-6f, &command)
           == MC_OUT_OF_RANGE);

    cfg.single_frequency_hz = 100.0f;
    assert(mc_current_sweep_start(&sweep, &cfg, 50e-6f, &command) == MC_OK);

    memset(&sample, 0, sizeof(sample));
    /* 理想反馈应在一个完整测量窗口后得到 0 dB、0°。 */
    for (guard = 0u; guard < 30000u && sweep.result.completed_points == 0u; ++guard) {
        sample.i_alpha_a = command.id_ref_a;
        assert(mc_current_sweep_step(&sweep, &sample, &command) == MC_BUSY);
    }
    assert(sweep.result.completed_points == 1u);
    assert(nearf(sweep.result.closed_magnitude_db[0], 0.0f, 0.05f));
    assert(nearf(sweep.result.closed_phase_deg[0], 0.0f, 0.5f));
}

static void test_bandwidth(void)
{
    mc_current_sweep_result_t sweeps[2];
    mc_bias_bandwidth_result_t result;
    float biases[2] = {0.0f, 5.0f};
    float bw;
    uint32_t i;
    memset(sweeps, 0, sizeof(sweeps));
    for (i = 0u; i < 5u; ++i) {
        sweeps[0].frequency_hz[i] = (float[]){20, 50, 100, 200, 400}[i];
        sweeps[0].closed_magnitude_db[i] = (float[]){0, 0, 0, -3.5f, -10}[i];
        sweeps[0].valid[i] = 1u;
        sweeps[1].frequency_hz[i] = sweeps[0].frequency_hz[i];
        sweeps[1].closed_magnitude_db[i] = (float[]){0, 0, 0, -6, -15}[i];
        sweeps[1].valid[i] = 1u;
    }
    sweeps[0].completed_points = sweeps[1].completed_points = 5u;
    assert(mc_closed_loop_bandwidth(&sweeps[0], true, &bw) == MC_OK);
    assert(bw > 100.0f && bw < 250.0f);
    assert(mc_bias_bandwidth_build(biases, sweeps, 2u, 0.33f, true, &result) == MC_OK);
    assert(result.valid[0] && result.valid[1]);
    assert(result.suggested_kp[1] > result.suggested_kp[0]);
}

int main(void)
{
    test_decoupling();
    test_deadtime();
    test_deadtime_fixture();
    test_encoder_alignment();
    test_diag_manager();
    test_flux();
    test_sweep();
    test_bandwidth();
    puts("portable control and sweep tests: PASS");
    return 0;
}


