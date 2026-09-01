#include "mc_diag_manager.h"

#include <string.h>

void mc_diag_manager_init(mc_diag_manager_t *manager)
{
    if (manager) memset(manager, 0, sizeof(*manager));
}

mc_status_t mc_diag_acquire(mc_diag_manager_t *manager,
                            mc_diag_owner_t owner,
                            bool platform_ready_and_disabled)
{
    if (!manager || owner == MC_DIAG_OWNER_NONE) return MC_INVALID_ARGUMENT;
    if (!platform_ready_and_disabled) return MC_REJECTED;
    if (manager->owner != MC_DIAG_OWNER_NONE) return MC_BUSY;
    manager->owner = owner;
    manager->state = MC_DIAG_PRECHECK;
    manager->last_status = MC_BUSY;
    manager->generation++;
    return MC_OK;
}

mc_status_t mc_diag_begin(mc_diag_manager_t *manager, mc_diag_owner_t owner)
{
    if (!manager || manager->owner != owner || manager->state != MC_DIAG_PRECHECK)
        return MC_REJECTED;
    manager->state = MC_DIAG_RUNNING;
    return MC_OK;
}

mc_status_t mc_diag_finish(mc_diag_manager_t *manager,
                           mc_diag_owner_t owner,
                           mc_status_t result)
{
    if (!manager || manager->owner != owner ||
        (manager->state != MC_DIAG_RUNNING &&
         manager->state != MC_DIAG_PRECHECK)) return MC_REJECTED;
    manager->last_status = result;
    manager->state = MC_DIAG_STOPPING;
    return MC_OK;
}

mc_status_t mc_diag_confirm_stopped(mc_diag_manager_t *manager,
                                    mc_diag_owner_t owner,
                                    bool platform_ready_and_disabled)
{
    if (!manager || manager->owner != owner ||
        manager->state != MC_DIAG_STOPPING) return MC_REJECTED;
    if (!platform_ready_and_disabled) return MC_BUSY;
    manager->state = (manager->last_status == MC_DONE ||
                      manager->last_status == MC_OK) ?
                     MC_DIAG_DONE : MC_DIAG_FAILED;
    return MC_OK;
}

mc_status_t mc_diag_release(mc_diag_manager_t *manager, mc_diag_owner_t owner)
{
    if (!manager || manager->owner != owner ||
        (manager->state != MC_DIAG_DONE &&
         manager->state != MC_DIAG_FAILED)) return MC_REJECTED;
    manager->owner = MC_DIAG_OWNER_NONE;
    manager->state = MC_DIAG_IDLE;
    return MC_OK;
}




