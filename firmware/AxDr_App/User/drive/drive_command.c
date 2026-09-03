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
    const bool reverse = foc->app.polarity == motor_polarity_n;

    if ((foc->app.polarity != motor_polarity_p) && !reverse)
    {
        return false;
    }

    switch (foc->mode.release)
    {
        case mit_mode:
        {
            float pos;
            float spd;
            float torq_ff;

            if (!isfinite(foc->cmd.pos) ||
                !isfinite(foc->cmd.spd) ||
                !isfinite(foc->cmd.mit_ff) ||
                !isfinite(foc->cmd.kp) ||
                !isfinite(foc->cmd.kd) ||
                (foc->cmd.kp < 0.0f) ||
                (foc->cmd.kd < 0.0f) ||
                !drive_cmd_range_valid(foc->app.nmax_torm, foc->app.pmax_torm) ||
                !drive_cmd_range_valid(foc->app.nmax_velm, foc->app.pmax_velm) ||
                !drive_cmd_range_valid(foc->app.nmax_posm, foc->app.pmax_posm))
            {
                return false;
            }

            pos = control_limit(foc->cmd.pos,
                                foc->app.pmax_posm,
                                foc->app.nmax_posm);
            spd = control_limit(foc->cmd.spd,
                                foc->app.pmax_velm,
                                foc->app.nmax_velm);
            torq_ff = control_limit(foc->cmd.mit_ff,
                                    foc->app.pmax_torm,
                                    foc->app.nmax_torm);

            foc->ctrl.posm_set = reverse ? -pos : pos;
            foc->ctrl.wm_set = reverse ? -spd : spd;
            foc->ctrl.mit_tor_set = reverse ? -torq_ff : torq_ff;
            foc->ctrl.kp = foc->cmd.kp;
            foc->ctrl.kd = foc->cmd.kd;
            return true;
        }

        case cst_mode:
            if (!isfinite(foc->cmd.torq) ||
                !drive_cmd_range_valid(foc->app.nmax_torm, foc->app.pmax_torm))
            {
                return false;
            }
            foc->ctrl.torm_set = control_limit(foc->cmd.torq,
                                               foc->app.pmax_torm,
                                               foc->app.nmax_torm);
            if (reverse)
            {
                foc->ctrl.torm_set = -foc->ctrl.torm_set;
            }
            return true;

        case vel_mode:
        case csv_mode:
            if (!isfinite(foc->cmd.torq) ||
                !isfinite(foc->cmd.spd) ||
                !drive_cmd_range_valid(foc->app.nmax_torm, foc->app.pmax_torm) ||
                !drive_cmd_range_valid(foc->app.nmax_velm, foc->app.pmax_velm))
            {
                return false;
            }
            foc->ctrl.torm_set = control_limit(foc->cmd.torq,
                                               foc->app.pmax_torm,
                                               foc->app.nmax_torm);
            foc->ctrl.wm_set = control_limit(foc->cmd.spd,
                                             foc->app.pmax_velm,
                                             foc->app.nmax_velm);
            if (reverse)
            {
                foc->ctrl.torm_set = -foc->ctrl.torm_set;
                foc->ctrl.wm_set = -foc->ctrl.wm_set;
            }
            return true;

        case pos_mode:
        case csp_mode:
            if (!isfinite(foc->cmd.torq) ||
                !isfinite(foc->cmd.spd) ||
                !isfinite(foc->cmd.pos) ||
                !drive_cmd_range_valid(foc->app.nmax_torm, foc->app.pmax_torm) ||
                !drive_cmd_range_valid(foc->app.nmax_velm, foc->app.pmax_velm) ||
                !drive_cmd_range_valid(foc->app.nmax_posm, foc->app.pmax_posm))
            {
                return false;
            }
            foc->ctrl.torm_set = control_limit(foc->cmd.torq,
                                               foc->app.pmax_torm,
                                               foc->app.nmax_torm);
            foc->ctrl.wm_set = control_limit(foc->cmd.spd,
                                             foc->app.pmax_velm,
                                             foc->app.nmax_velm);
            foc->ctrl.posm_set = control_limit(foc->cmd.pos,
                                               foc->app.pmax_posm,
                                               foc->app.nmax_posm);
            if (reverse)
            {
                foc->ctrl.torm_set = -foc->ctrl.torm_set;
                foc->ctrl.wm_set = -foc->ctrl.wm_set;
                foc->ctrl.posm_set = -foc->ctrl.posm_set;
            }
            return true;

        default:
            return false;
    }
}
