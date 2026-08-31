/**
 * @file drive_mode.c
 * @brief Drive 控制、标定和辨识模式分派。
 */

#include "drive_mode.h"

#include "common.h"
#include "drive_pwm.h"

_RAM_FUNC bool drive_mode_step(pmsm_t *pm)
{
    switch (pm->mode.sys)
    {
        case release_mode:
            /* 对外运行模式：MIT、轮廓模式和周期同步模式。 */
            switch (pm->mode.release)
            {
                case mit_mode:
                    pm_mit_mode(pm);
                    return true;

                case tor_mode:
                    pt_tor_mode(pm);
                    return true;

                case vel_mode:
                    pv_vel_mode(pm);
                    return true;

                case pos_mode:
                    pp_pos_mode(pm);
                    return true;

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
            /* 停车模式：快速停车或故障停车。 */
            switch (pm->mode.halt)
            {
                case quick_mode:
                    pmsm_quick_stop_mode(pm);
                    return true;

                case fault_mode:
                    pmsm_fault_stop_mode(pm);
                    return true;

                default:
                    return false;
            }

        case calibrat_mode:
            /* 标定与辨识模式。没有实现的分支保持空操作。 */
            switch (pm->mode.calibrat)
            {
                case rotor_enc_cali:
                    cali_mag_encoder(pm);
                    return true;

                case output_enc_mod:
                    return false;

                case output_enc_cali:
                    return false;

                case iden_pm:
                    iden_pmsm_first(&pm->idpm);
                    return true;

                case anticogging_pm:
                    anticogging_calibration(pm);
                    return true;

                default:
                    return false;
            }

        case debug_mode:
            /* 旧实验控制路径：V/f、I/f 和各级闭环控制。 */
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

                case spd_volt_cl:
                    return false;

                case pos_spd_volt_cl:
                    return false;

                default:
                    return false;
            }

        default:
            return false;
    }
}
