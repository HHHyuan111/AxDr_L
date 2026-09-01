#ifndef MC_DECOUPLING_H
#define MC_DECOUPLING_H

#include "mc_common.h"
#include "mc_current_pi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MC_DECOUPLING_NONE = 0,
    MC_DECOUPLING_MODEL_FEEDFORWARD,
    MC_DECOUPLING_COMPLEX_VECTOR
} mc_decoupling_mode_t;

typedef struct {
    float rs_ohm;
    float ld_h;
    float lq_h;
    float flux_wb;
    float pole_pairs;
    float dt_s;
    bool include_resistive_feedforward;
} mc_decoupling_config_t;

typedef struct {
    float id_ref_a;
    float iq_ref_a;
    float error_d_a;
    float error_q_a;
    float omega_mech_rad_s;
    float kp_d;
    float kp_q;
} mc_decoupling_input_t;

typedef struct {
    float omega_elec_rad_s;
    float vd_feedforward_v;
    float vq_feedforward_v;
    float id_integrator_delta_v;
    float iq_integrator_delta_v;
} mc_decoupling_output_t;

mc_status_t mc_decoupling_calculate(mc_decoupling_mode_t mode,
                                    const mc_decoupling_config_t *config,
                                    const mc_decoupling_input_t *input,
                                    mc_decoupling_output_t *output);

#ifdef __cplusplus
}
#endif

#endif




