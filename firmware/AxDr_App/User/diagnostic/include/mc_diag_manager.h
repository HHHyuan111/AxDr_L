#ifndef MC_DIAG_MANAGER_H
#define MC_DIAG_MANAGER_H

#include "mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MC_DIAG_OWNER_NONE = 0,
    MC_DIAG_OWNER_RS,
    MC_DIAG_OWNER_L,
    MC_DIAG_OWNER_POLE_PAIR,
    MC_DIAG_OWNER_ENCODER_ALIGN,
    MC_DIAG_OWNER_CURRENT_SWEEP,
    MC_DIAG_OWNER_BIAS_SWEEP,
    MC_DIAG_OWNER_FLUX,
    MC_DIAG_OWNER_DEADTIME
} mc_diag_owner_t;

typedef enum {
    MC_DIAG_IDLE = 0,
    MC_DIAG_PRECHECK,
    MC_DIAG_RUNNING,
    MC_DIAG_STOPPING,
    MC_DIAG_DONE,
    MC_DIAG_FAILED
} mc_diag_state_t;

typedef struct {
    mc_diag_owner_t owner;
    mc_diag_state_t state;
    mc_status_t last_status;
    uint32_t generation;
} mc_diag_manager_t;

void mc_diag_manager_init(mc_diag_manager_t *manager);
mc_status_t mc_diag_acquire(mc_diag_manager_t *manager,
                            mc_diag_owner_t owner,
                            bool platform_ready_and_disabled);
mc_status_t mc_diag_begin(mc_diag_manager_t *manager, mc_diag_owner_t owner);
mc_status_t mc_diag_finish(mc_diag_manager_t *manager,
                           mc_diag_owner_t owner,
                           mc_status_t result);
/* Call after request_safe_stop(). The manager stays in STOPPING until the
 * target confirms READY and disabled. */
mc_status_t mc_diag_confirm_stopped(mc_diag_manager_t *manager,
                                    mc_diag_owner_t owner,
                                    bool platform_ready_and_disabled);
mc_status_t mc_diag_release(mc_diag_manager_t *manager, mc_diag_owner_t owner);

#ifdef __cplusplus
}
#endif

#endif




