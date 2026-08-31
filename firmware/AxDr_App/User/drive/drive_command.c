/**
 * @file drive_command.c
 * @brief Drive 发布命令校验、限幅和极性转换。
 */

#include "drive_command.h"

#include <math.h>

#include "common.h"
#include "compiler.h"
#include "control_limit.h"

static PLATFORM_FAST_CODE bool drive_cmd_range_valid(float lower, float upper)
{
    return isfinite(lower) && isfinite(upper) && (lower <= upper);
}

PLATFORM_FAST_CODE bool drive_cmd_apply(pmsm_t *pm)
{
    float position_rad;
    float speed_rad_s;
    float torque_nm;

    if (!isfinite(pm->cmd.torm_set) ||
        !isfinite(pm->cmd.wm_set) ||
        !isfinite(pm->cmd.posm_set) ||
        !drive_cmd_range_valid(pm->app_ctrl.nmax_torm, pm->app_ctrl.pmax_torm) ||
        !drive_cmd_range_valid(pm->app_ctrl.nmax_velm, pm->app_ctrl.pmax_velm) ||
        !drive_cmd_range_valid(pm->app_ctrl.nmax_posm, pm->app_ctrl.pmax_posm))
    {
        return false;
    }

    torque_nm = control_limit(pm->cmd.torm_set,
                              pm->app_ctrl.pmax_torm,
                              pm->app_ctrl.nmax_torm);
    speed_rad_s = control_limit(pm->cmd.wm_set,
                                pm->app_ctrl.pmax_velm,
                                pm->app_ctrl.nmax_velm);
    position_rad = control_limit(pm->cmd.posm_set,
                                 pm->app_ctrl.pmax_posm,
                                 pm->app_ctrl.nmax_posm);

    if (pm->app_ctrl.polarity == motor_polarity_n)
    {
        torque_nm = -torque_nm;
        speed_rad_s = -speed_rad_s;
        position_rad = -position_rad;
    }
    else if (pm->app_ctrl.polarity != motor_polarity_p)
    {
        return false;
    }

    pm->ctrl.torm_set = torque_nm;
    pm->ctrl.wm_set = speed_rad_s;
    pm->ctrl.posm_set = position_rad;
    return true;
}
