/**
 * @file drive_mode.c
 * @brief Drive 正式控制模式判定和分派。
 */

#include "drive_mode.h"

#include "common.h"
#include "foc_run.h"
#include "drive_command.h"
#include "drive_diag.h"


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

_RAM_FUNC bool drive_mode_step(foc_t *foc, foc_plan_t *cmd)
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
                    return mit_step(foc, cmd);

                case vel_mode:
                    return pv_step(foc, cmd);

                case pos_mode:
                    return pp_step(foc, cmd);

                case cst_mode:
                    return cst_step(foc, cmd);

                case csv_mode:
                    return csv_step(foc, cmd);

                case csp_mode:
                    return csp_step(foc, cmd);

                default:
                    return false;
            }

        case halt_mode:
            return quick_stop_step(foc, cmd);

        case debug_mode:
            switch (foc->mode.debug)
            {
                case drag_vf:
                    return open_volt_step(foc, cmd);

                case volt_op:
                    *cmd = (foc_plan_t){
                        .ref = {
                            .mode = FOC_CTRL_MODE_VOLT,
                            .vd = foc->ref.vd,
                            .vq = foc->ref.vq,
                        },
                        .theta = foc->fb.theta_e,
                    };
                    return true;

                case drag_if:
                    return open_cur_step(foc, cmd);

                case curr_cl:
                    *cmd = (foc_plan_t){
                        .ref = {
                            .mode = FOC_CTRL_MODE_CUR,
                            .id_ref = foc->ref.id,
                            .iq_ref = foc->ref.iq,
                        },
                        .theta = foc->fb.theta_e,
                    };
                    return true;

                case spd_curr_cl:
                    *cmd = (foc_plan_t){
                        .ref = {
                            .mode = FOC_CTRL_MODE_SPD,
                            .spd_ref = foc->ref.spd_r,
                            .cur_lim = foc->ref.iq,
                            .id_ref = foc->ref.id,
                        },
                        .theta = foc->fb.theta_e,
                    };
                    return true;

                case pos_spd_curr_cl:
                    *cmd = (foc_plan_t){
                        .ref = {
                            .mode = FOC_CTRL_MODE_POS,
                            .pos_ref = foc->ref.pos_r,
                            .spd_lim = foc->ref.spd_r,
                            .cur_lim = foc->ref.iq,
                            .id_ref = foc->ref.id,
                        },
                        .theta = foc->fb.theta_e,
                    };
                    return true;

                default:
                    return false;
            }

        case calibrat_mode:
            return drive_diag_step(foc, cmd);

        default:
            return false;
    }
}
