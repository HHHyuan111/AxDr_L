#include "mc_biased_l_ident.h"
#include "mc_pole_pair_ident.h"
#include "mc_rs_ident.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_failures;

#define CHECK_TRUE(expr) do {                                                \
    if (!(expr)) {                                                          \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr);             \
        g_failures++;                                                       \
    }                                                                       \
} while (0)

#define CHECK_NEAR(actual, expected, tolerance) do {                        \
    const double a_ = (double)(actual);                                     \
    const double e_ = (double)(expected);                                   \
    const double t_ = (double)(tolerance);                                  \
    if (fabs(a_ - e_) > t_) {                                              \
        printf("FAIL %s:%d: %.9g != %.9g (tol %.3g)\n",                   \
               __FILE__, __LINE__, a_, e_, t_);                            \
        g_failures++;                                                       \
    }                                                                       \
} while (0)

static void test_rs_pairwise_regression(void)
{
    const float current[6] = {6.0f, 7.2f, 8.4f, 9.6f, 10.8f, 12.0f};
    float voltage[6];
    float resistance = 0.0f;
    uint8_t i;

    for (i = 0u; i < 6u; ++i)
        voltage[i] = 0.12f * current[i] + 0.37f;
    CHECK_TRUE(mc_rs_ident_solve(voltage, current, 6u, 1e-9,
                                 &resistance) == MC_OK);
    CHECK_NEAR(resistance, 0.12f, 1e-6f);

    for (i = 0u; i < 6u; ++i) voltage[i] = 1.0f;
    {
        const float no_motion[6] = {2, 2, 2, 2, 2, 2};
        CHECK_TRUE(mc_rs_ident_solve(voltage, no_motion, 6u, 1e-9,
                                     &resistance) == MC_OUT_OF_RANGE);
    }
}

static void test_rs_defaults_and_safety(void)
{
    mc_rs_ident_config_t cfg;
    mc_rs_ident_t ident;

    mc_rs_ident_default_config(&cfg);
    CHECK_NEAR(cfg.current_min_a, 6.0f, 0.0f);
    CHECK_NEAR(cfg.current_max_a, 12.0f, 0.0f);
    CHECK_TRUE(cfg.point_count == 6u);
    CHECK_NEAR(cfg.total_duration_s, 8.0f, 0.0f);
    CHECK_TRUE(mc_rs_ident_start(&ident, &cfg) == MC_INVALID_ARGUMENT);
    cfg.safe_current_limit_a = 12.0f;
    CHECK_TRUE(mc_rs_ident_start(&ident, &cfg) == MC_OK);

    cfg.safe_current_limit_a = 11.0f;
    CHECK_TRUE(mc_rs_ident_start(&ident, &cfg) == MC_INVALID_ARGUMENT);
}

static void test_rs_state_machine(void)
{
    mc_rs_ident_config_t cfg;
    mc_rs_ident_t ident;
    mc_sample_t sample;
    mc_command_t command;
    mc_status_t status;
    const float resistance = 0.05f;
    const float fixed_drop = 0.20f;
    float applied_voltage = 0.0f;
    uint32_t iterations = 0u;

    mc_rs_ident_default_config(&cfg);
    cfg.control_period_s = 0.001f;
    cfg.total_duration_s = 0.6f;
    cfg.integral_gain_v_per_a_s = 10.0f;
    cfg.safe_current_limit_a = 15.0f;
    cfg.max_abs_voltage_v = 2.0f;
    CHECK_TRUE(mc_rs_ident_start(&ident, &cfg) == MC_OK);

    memset(&sample, 0, sizeof(sample));
    sample.vbus_v = 24.0f;
    sample.current_limit_a = 15.0f;
    do {
        sample.ia_a = (applied_voltage - fixed_drop) / resistance;
        status = mc_rs_ident_step(&ident, &sample, &command);
        applied_voltage = command.vd_ref_v;
        iterations++;
    } while (status == MC_BUSY && iterations < 2000u);

    CHECK_TRUE(status == MC_DONE);
    CHECK_TRUE(ident.result.valid);
    CHECK_NEAR(ident.result.phase_resistance_ohm, resistance, 2e-4f);
    CHECK_TRUE(command.disable_request);
}

static void test_biased_l_pair_formula(void)
{
    mc_biased_l_pair_t pair;
    float inductance = 0.0f;
    const float resistance = 0.12f;
    const float expected_l = 300.0e-6f;

    memset(&pair, 0, sizeof(pair));
    pair.positive_slope_a_s = 1000.0f;
    pair.negative_slope_a_s = -1000.0f;
    pair.positive_current_avg_a = 1.2f;
    pair.negative_current_avg_a = 1.0f;
    pair.negative_voltage_avg_v = 0.10f;
    pair.positive_voltage_avg_v = pair.negative_voltage_avg_v +
        resistance * (pair.positive_current_avg_a -
                      pair.negative_current_avg_a) +
        expected_l * (pair.positive_slope_a_s -
                      pair.negative_slope_a_s);

    CHECK_TRUE(mc_biased_l_ident_estimate_pair(
        &pair, resistance, 1.0f, &inductance) == MC_OK);
    CHECK_NEAR(inductance, expected_l, 1e-9f);
}

static void test_biased_l_three_percent_default(void)
{
    mc_biased_l_ident_config_t cfg;
    mc_biased_l_ident_t ident;

    mc_biased_l_ident_default_config(&cfg);
    cfg.current_limit_a = 100.0f;
    cfg.voltage_limit_v = 24.0f;
    CHECK_TRUE(mc_biased_l_ident_start(&ident, &cfg, 0.12f, 24.0f) == MC_OK);
    CHECK_NEAR(ident.bias_target_a, 3.0f, 1e-6f);
    CHECK_NEAR(ident.target_ripple_a, 0.75f, 1e-6f);
    CHECK_NEAR(ident.result.injection_frequency_hz, 1000.0f, 1e-3f);
}

static void test_biased_l_state_machine(void)
{
    mc_biased_l_ident_config_t cfg;
    mc_biased_l_ident_t ident;
    mc_sample_t sample;
    mc_command_t command;
    mc_status_t status;
    const float true_l = 1.0e-3f;
    float current = 0.5f;
    float applied_voltage = 0.0f;
    uint32_t iterations = 0u;

    mc_biased_l_ident_default_config(&cfg);
    cfg.control_period_s = 1.0e-4f;
    cfg.auto_tune = false;
    cfg.manual_bias_current_a = 0.5f;
    cfg.initial_injection_voltage_v = 0.1f;
    cfg.level_ticks = 5u;
    cfg.edge_skip_ticks = 1u;
    cfg.bias_settle_ticks = 0u;
    cfg.bias_stable_ticks = 1u;
    cfg.bias_timeout_ticks = 10u;
    cfg.tune_pairs = 0u;
    cfg.target_accepted_pairs = 5u;
    cfg.max_measure_pair_multiplier = 3u;
    cfg.min_current_delta_a = 0.001f;
    cfg.current_limit_a = 10.0f;
    cfg.voltage_limit_v = 10.0f;
    CHECK_TRUE(mc_biased_l_ident_start(&ident, &cfg, 0.0f, 24.0f) == MC_OK);

    memset(&sample, 0, sizeof(sample));
    sample.vbus_v = 24.0f;
    sample.current_limit_a = 10.0f;
    sample.i_alpha_a = current;
    sample.vd_v = 0.0f;
    status = mc_biased_l_ident_step(&ident, &sample, &command);
    CHECK_TRUE(status == MC_BUSY);
    applied_voltage = command.vd_ref_v;

    do {
        current += applied_voltage * cfg.control_period_s / true_l;
        sample.i_alpha_a = current;
        sample.vd_v = applied_voltage;
        status = mc_biased_l_ident_step(&ident, &sample, &command);
        applied_voltage = command.vd_ref_v;
        iterations++;
    } while (status == MC_BUSY && iterations < 1000u);

    CHECK_TRUE(status == MC_DONE);
    CHECK_TRUE(ident.result.valid);
    CHECK_TRUE(ident.result.accepted_pairs == 5u);
    CHECK_NEAR(ident.result.phase_inductance_h, true_l, 2e-5f);
    CHECK_NEAR(ident.result.line_inductance_h, 2.0f * true_l, 4e-5f);
}

static void test_encoder_unwrap_and_pole_pair_solver(void)
{
    mc_pole_pair_ident_result_t result;

    CHECK_TRUE(mc_encoder_unwrap_delta(2u, 65534u, 65536u) == 4);
    CHECK_TRUE(mc_encoder_unwrap_delta(65534u, 2u, 65536u) == -4);
    CHECK_TRUE(mc_pole_pair_ident_solve(65536, 65536u, 16.0f,
                                        1, 30, &result) == MC_OK);
    CHECK_TRUE(result.pole_pairs == 16);
    CHECK_TRUE(result.encoder_direction == 1);
    CHECK_NEAR(result.mechanical_turns, 1.0f, 1e-7f);

    CHECK_TRUE(mc_pole_pair_ident_solve(-65536, 65536u, 16.0f,
                                        1, 30, &result) == MC_OK);
    CHECK_TRUE(result.pole_pairs == 16);
    CHECK_TRUE(result.encoder_direction == -1);

    CHECK_TRUE(mc_pole_pair_ident_solve(0, 65536u, 16.0f,
                                        1, 30, &result) == MC_OUT_OF_RANGE);
}

static void test_pole_pair_state_machine(void)
{
    mc_pole_pair_ident_config_t cfg;
    mc_pole_pair_ident_t ident;
    mc_sample_t sample;
    mc_command_t command;
    mc_status_t status;
    const float true_pp = 4.0f;
    uint32_t iterations = 0u;

    mc_pole_pair_ident_default_config(&cfg);
    cfg.control_period_s = 0.001f;
    cfg.phase_resistance_ohm = 0.05f;
    cfg.drive_current_a = 1.0f;
    cfg.safe_current_limit_a = 2.0f;
    cfg.max_abs_voltage_v = 2.0f;
    cfg.electrical_turns = 1.0f;
    cfg.electrical_velocity_rad_s = MC_TWO_PI_F;
    cfg.ramp_duration_s = 0.002f;
    cfg.encoder_full_scale = 65536u;

    CHECK_TRUE(mc_pole_pair_ident_start(&ident, &cfg) == MC_OK);
    memset(&sample, 0, sizeof(sample));
    sample.vbus_v = 24.0f;
    sample.current_limit_a = 2.0f;

    do {
        const float mechanical_turns = ident.electrical_phase_rad /
                                       (MC_TWO_PI_F * true_pp);
        const double counts = mechanical_turns * cfg.encoder_full_scale;
        sample.encoder_raw = (uint32_t)((uint64_t)(counts + 0.5) %
                                        cfg.encoder_full_scale);
        status = mc_pole_pair_ident_step(&ident, &sample, &command);
        iterations++;
    } while (status == MC_BUSY && iterations < 5000u);

    CHECK_TRUE(status == MC_DONE);
    CHECK_TRUE(ident.result.valid);
    CHECK_TRUE(ident.result.pole_pairs == 4);
    CHECK_TRUE(ident.result.encoder_direction == 1);
    CHECK_TRUE(command.disable_request);
}

int main(void)
{
    test_rs_pairwise_regression();
    test_rs_defaults_and_safety();
    test_rs_state_machine();
    test_biased_l_pair_formula();
    test_biased_l_three_percent_default();
    test_biased_l_state_machine();
    test_encoder_unwrap_and_pole_pair_solver();
    test_pole_pair_state_machine();

    if (g_failures != 0) {
        printf("identification tests: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("identification tests: PASS\n");
    return 0;
}




