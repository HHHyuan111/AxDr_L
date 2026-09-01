/**
 * @file drive_mode.c
 * @brief Drive 正式控制模式判定和分派。
 */

#include "drive_mode.h"

#include "common.h"
#include "drive_command.h"
#include "drive_diag.h"
#include "drive_pwm.h"

bool drive_mode_is_supported(const pmsm_t *pm)
{
    switch (pm->mode.sys)
    {
        case release_mode:
            return (pm->mode.release == cst_mode) ||
                   (pm->mode.release == csv_mode) ||
                   (pm->mode.release == csp_mode);

        case halt_mode:
            return pm->mode.halt == quick_mode;

        case debug_mode:
            return (pm->mode.debug == drag_vf) ||
                   (pm->mode.debug == drag_if) ||
                   (pm->mode.debug == volt_op) ||
                   (pm->mode.debug == curr_cl) ||
                   (pm->mode.debug == spd_curr_cl) ||
                   (pm->mode.debug == pos_spd_curr_cl);

        case calibrat_mode:
            return drive_diag_is_supported(pm);

        default:
            return false;
    }
}

bool drive_mode_prepare(pmsm_t *pm)
{
    if (!drive_mode_is_supported(pm))
    {
        return false;
    }

    if (pm->mode.sys == release_mode)
    {
        return drive_cmd_apply(pm);
    }
    if (pm->mode.sys == calibrat_mode)
    {
        return drive_diag_prepare(pm);
    }

    return true;
}

_RAM_FUNC bool drive_mode_step(pmsm_t *pm)
{
    if (!drive_mode_prepare(pm))
    {
        return false;
    }

    switch (pm->mode.sys)
    {
        case release_mode:
            switch (pm->mode.release)
            {
                case cst_mode:
                    cst_tor_mode(pm);
                    return true;

                case csv_mode:
                    csv_vel_mode(pm);
                    return true;

                case csp_mode:
                    csp_pos_mode(pm);
                    return true;

                default:
                    return false;
            }

        case halt_mode:
            pmsm_quick_stop_mode(pm);
            return true;

        case debug_mode:
            switch (pm->mode.debug)
            {
                case drag_vf:
                    force_volt_mode(pm);
                    return true;

                case volt_op:
                    if (!foc_volt(pm,
                                  pm->ctrl.vd_set,
                                  pm->ctrl.vq_set,
                                  pm->foc.p_e))
                    {
                        return false;
                    }
                    return drive_pwm_commit(pm);

                case drag_if:
                    force_curr_mode(pm);
                    return true;

                case curr_cl:
                    if (!foc_curr(pm,
                                  pm->ctrl.id_set,
                                  pm->ctrl.iq_set,
                                  pm->foc.p_e))
                    {
                        return false;
                    }
                    return drive_pwm_commit(pm);

                case spd_curr_cl:
                    if (!foc_vel(pm,
                                 pm->ctrl.wr_set,
                                 pm->ctrl.iq_set,
                                 pm->foc.p_e))
                    {
                        return false;
                    }
                    return drive_pwm_commit(pm);

                case pos_spd_curr_cl:
                    if (!foc_pos(pm,
                                 pm->ctrl.posr_set,
                                 pm->ctrl.wr_set,
                                 pm->ctrl.iq_set,
                                 pm->foc.p_e))
                    {
                        return false;
                    }
                    return drive_pwm_commit(pm);

                default:
                    return false;
            }

        case calibrat_mode:
            return drive_diag_step(pm);

        default:
            return false;
    }
}
