/**
 * @file drive_mode.c
 * @brief Drive 正式控制模式判定和分派。
 */

#include "drive_mode.h"

#include "common.h"
#include "drive_command.h"
#include "drive_diag.h"
#include "drive_pwm.h"

bool drive_mode_is_supported(const foc_t *foc)
{
    switch (foc->mode.sys)
    {
        case release_mode:
            return (foc->mode.release == mit_mode) ||
                   (foc->mode.release == vel_mode) ||
                   (foc->mode.release == pos_mode) ||
                   (foc->mode.release == cst_mode) ||
                   (foc->mode.release == csv_mode) ||
                   (foc->mode.release == csp_mode);

        case halt_mode:
            return foc->mode.halt == quick_mode;

        case debug_mode:
            return (foc->mode.debug == drag_vf) ||
                   (foc->mode.debug == drag_if) ||
                   (foc->mode.debug == volt_op) ||
                   (foc->mode.debug == curr_cl) ||
                   (foc->mode.debug == spd_curr_cl) ||
                   (foc->mode.debug == pos_spd_curr_cl);

        case calibrat_mode:
            return drive_diag_is_supported(foc);

        default:
            return false;
    }
}

bool drive_mode_prepare(foc_t *foc)
{
    if (!drive_mode_is_supported(foc))
    {
        return false;
    }

    if (foc->mode.sys == release_mode)
    {
        return drive_cmd_apply(foc);
    }
    if (foc->mode.sys == calibrat_mode)
    {
        return drive_diag_prepare(foc);
    }

    return true;
}

_RAM_FUNC bool drive_mode_step(foc_t *foc)
{
    if (!drive_mode_prepare(foc))
    {
        return false;
    }

    switch (foc->mode.sys)
    {
        case release_mode:
            switch (foc->mode.release)
            {
                case mit_mode:
                    mit_step(foc);
                    return true;

                case vel_mode:
                    pv_step(foc);
                    return true;

                case pos_mode:
                    pp_step(foc);
                    return true;

                case cst_mode:
                    cst_step(foc);
                    return true;

                case csv_mode:
                    csv_step(foc);
                    return true;

                case csp_mode:
                    csp_step(foc);
                    return true;

                default:
                    return false;
            }

        case halt_mode:
            quick_stop_step(foc);
            return true;

        case debug_mode:
            switch (foc->mode.debug)
            {
                case drag_vf:
                    open_volt_step(foc);
                    return true;

                case volt_op:
                    if (!foc_volt_step(foc,
                                  foc->ctrl.vd_set,
                                  foc->ctrl.vq_set,
                                  foc->sig.p_e))
                    {
                        return false;
                    }
                    return drive_pwm_commit(foc);

                case drag_if:
                    open_cur_step(foc);
                    return true;

                case curr_cl:
                    if (!foc_cur_step(foc,
                                  foc->ctrl.id_set,
                                  foc->ctrl.iq_set,
                                  foc->sig.p_e))
                    {
                        return false;
                    }
                    return drive_pwm_commit(foc);

                case spd_curr_cl:
                    if (!foc_spd_step(foc,
                                 foc->ctrl.wr_set,
                                 foc->ctrl.iq_set,
                                 foc->sig.p_e))
                    {
                        return false;
                    }
                    return drive_pwm_commit(foc);

                case pos_spd_curr_cl:
                    if (!foc_pos_step(foc,
                                 foc->ctrl.posr_set,
                                 foc->ctrl.wr_set,
                                 foc->ctrl.iq_set,
                                 foc->sig.p_e))
                    {
                        return false;
                    }
                    return drive_pwm_commit(foc);

                default:
                    return false;
            }

        case calibrat_mode:
            return drive_diag_step(foc);

        default:
            return false;
    }
}
