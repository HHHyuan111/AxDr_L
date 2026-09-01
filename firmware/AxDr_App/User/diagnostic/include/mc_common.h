#ifndef MC_COMMON_H
#define MC_COMMON_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MC_PI_F       (3.14159265358979323846f)
#define MC_TWO_PI_F   (6.28318530717958647692f)
#define MC_SQRT3_F    (1.73205080756887729353f)

typedef enum {
    MC_OK = 0,
    MC_BUSY,
    MC_DONE,
    MC_REJECTED,
    MC_ABORTED,
    MC_FAULT,
    MC_INVALID_ARGUMENT,
    MC_OUT_OF_RANGE,
    MC_NUMERIC_ERROR
} mc_status_t;

typedef enum {
    MC_AXIS_D = 0,
    MC_AXIS_Q = 1
} mc_axis_t;

typedef enum {
    MC_CONTROL_IDLE = 0,
    MC_CONTROL_CURRENT,
    MC_CONTROL_VOLTAGE
} mc_control_mode_t;

typedef struct {
    float ia_a;
    float ib_a;
    float ic_a;
    float i_alpha_a;
    float i_beta_a;
    float id_a;
    float iq_a;
    float vd_v;
    float vq_v;
    float theta_mech_rad;
    float theta_elec_rad;
    float omega_mech_rad_s;
    float vbus_v;
    float dt_s;
    float current_limit_a;
    uint32_t encoder_raw;
    uint32_t encoder_full_scale;
    uint32_t fault_code;
    bool voltage_saturated;
} mc_sample_t;

typedef struct {
    mc_control_mode_t mode;
    float id_ref_a;
    float iq_ref_a;
    float vd_ref_v;
    float vq_ref_v;
    float openloop_theta_e_rad;
    bool openloop_enable;
    bool enable_request;
    bool disable_request;
} mc_command_t;

float mc_clampf(float value, float lower, float upper);
float mc_wrap_0_2pi(float angle);
float mc_wrap_pm_pi(float angle);
/* Bit-level IEEE-754 binary32 check. It remains effective even when an
 * embedded compiler is configured with finite-math assumptions. */
bool mc_float_is_finite(float value);

#ifdef __cplusplus
}
#endif

#endif




