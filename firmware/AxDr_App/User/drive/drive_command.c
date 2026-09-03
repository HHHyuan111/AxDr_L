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

PLATFORM_FAST_CODE bool drive_cmd_apply(foc_t *foc)
{
    float position_rad;
    float speed_rad_s;
    float torque_nm;

    if (!isfinite(foc->cmd.torm_set) ||
        !isfinite(foc->cmd.wm_set) ||
        !isfinite(foc->cmd.posm_set) ||
        !drive_cmd_range_valid(foc->app.nmax_torm, foc->app.pmax_torm) ||
        !drive_cmd_range_valid(foc->app.nmax_velm, foc->app.pmax_velm) ||
        !drive_cmd_range_valid(foc->app.nmax_posm, foc->app.pmax_posm))
    {
        return false;
    }

    torque_nm = control_limit(foc->cmd.torm_set,
                              foc->app.pmax_torm,
                              foc->app.nmax_torm);
    speed_rad_s = control_limit(foc->cmd.wm_set,
                                foc->app.pmax_velm,
                                foc->app.nmax_velm);
    position_rad = control_limit(foc->cmd.posm_set,
                                 foc->app.pmax_posm,
                                 foc->app.nmax_posm);

    if (foc->app.polarity == motor_polarity_n)
    {
        torque_nm = -torque_nm;
        speed_rad_s = -speed_rad_s;
        position_rad = -position_rad;
    }
    else if (foc->app.polarity != motor_polarity_p)
    {
        return false;
    }

    foc->ctrl.torm_set = torque_nm;
    foc->ctrl.wm_set = speed_rad_s;
    foc->ctrl.posm_set = position_rad;
    return true;
}
