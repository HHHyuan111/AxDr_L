#include "mc_decoupling.h"

#include <math.h>
#include <string.h>

mc_status_t mc_decoupling_calculate(mc_decoupling_mode_t mode,
                                    const mc_decoupling_config_t *config,
                                    const mc_decoupling_input_t *input,
                                    mc_decoupling_output_t *output)
{
    float omega_e;

    if (!config || !input || !output ||
        (mode != MC_DECOUPLING_NONE &&
         mode != MC_DECOUPLING_MODEL_FEEDFORWARD &&
         mode != MC_DECOUPLING_COMPLEX_VECTOR) ||
        !mc_float_is_finite(config->rs_ohm) || !mc_float_is_finite(config->ld_h) ||
        !mc_float_is_finite(config->lq_h) || !mc_float_is_finite(config->flux_wb) ||
        !mc_float_is_finite(config->pole_pairs) || !mc_float_is_finite(config->dt_s) ||
        !mc_float_is_finite(input->id_ref_a) || !mc_float_is_finite(input->iq_ref_a) ||
        !mc_float_is_finite(input->error_d_a) || !mc_float_is_finite(input->error_q_a) ||
        !mc_float_is_finite(input->omega_mech_rad_s) || !mc_float_is_finite(input->kp_d) ||
        !mc_float_is_finite(input->kp_q) || config->rs_ohm < 0.0f ||
        config->ld_h <= 0.0f || config->lq_h <= 0.0f ||
        config->flux_wb < 0.0f || config->dt_s <= 0.0f ||
        config->pole_pairs <= 0.0f) {
        return MC_INVALID_ARGUMENT;
    }

    memset(output, 0, sizeof(*output));
    omega_e = input->omega_mech_rad_s * config->pole_pairs;
    if (!mc_float_is_finite(omega_e)) return MC_NUMERIC_ERROR;
    output->omega_elec_rad_s = omega_e;

    if (mode == MC_DECOUPLING_NONE || fabsf(omega_e) <= 0.01f) {
        return MC_OK;
    }

    if (mode == MC_DECOUPLING_MODEL_FEEDFORWARD) {
        output->vd_feedforward_v = -omega_e * config->lq_h * input->iq_ref_a;
        output->vq_feedforward_v = omega_e * config->ld_h * input->id_ref_a
                                 + omega_e * config->flux_wb;
        if (config->include_resistive_feedforward) {
            output->vd_feedforward_v += config->rs_ohm * input->id_ref_a;
            output->vq_feedforward_v += config->rs_ohm * input->iq_ref_a;
        }
        return (mc_float_is_finite(output->vd_feedforward_v) &&
                mc_float_is_finite(output->vq_feedforward_v)) ?
               MC_OK : MC_NUMERIC_ERROR;
    }

    if (mode == MC_DECOUPLING_COMPLEX_VECTOR) {
        output->id_integrator_delta_v = -omega_e * input->kp_d
                                      * input->error_q_a * config->dt_s;
        output->iq_integrator_delta_v = omega_e * input->kp_q
                                      * input->error_d_a * config->dt_s;
        return (mc_float_is_finite(output->id_integrator_delta_v) &&
                mc_float_is_finite(output->iq_integrator_delta_v)) ?
               MC_OK : MC_NUMERIC_ERROR;
    }

    return MC_OUT_OF_RANGE;
}




