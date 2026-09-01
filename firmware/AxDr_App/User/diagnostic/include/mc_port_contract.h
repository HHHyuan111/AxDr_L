#ifndef MC_PORT_CONTRACT_H
#define MC_PORT_CONTRACT_H

#include "mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool (*read_sample)(void *user, mc_sample_t *sample);
    bool (*apply_command)(void *user, const mc_command_t *command);
    bool (*is_ready_and_disabled)(void *user);
    void (*reset_current_controller)(void *user);
    void (*set_current_pi)(void *user,
                           float kp_d, float ki_d_per_tick,
                           float kp_q, float ki_q_per_tick);
    void (*request_safe_stop)(void *user);
    void *user;
} mc_port_t;

#ifdef __cplusplus
}
#endif

#endif




