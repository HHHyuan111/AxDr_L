/**
 * @file drive_mode.c
 * @brief Drive 控制、标定和辨识模式分派。
 */

#include "drive_mode.h"

#include "common.h"
#include "drive_pwm.h"

_RAM_FUNC void drive_mode_step(pmsm_t *pm)
{
    switch (pm->mode.sys)
    {
        case release_mode:
            /* 对外运行模式：MIT、轮廓模式和周期同步模式。 */
            switch (pm->mode.release)
            {
                case mit_mode:
                    pm_mit_mode(pm);
                    break;

                case tor_mode:
                    pt_tor_mode(pm);
                    break;

                case vel_mode:
                    pv_vel_mode(pm);
                    break;

                case pos_mode:
                    pp_pos_mode(pm);
                    break;

                case cst_mode:
                    cst_tor_mode(pm);
                    break;

                case csv_mode:
                    csv_vel_mode(pm);
                    break;

                case csp_mode:
                    csp_pos_mode(pm);
                    break;

                default:
                    break;
            }
            break;

        case halt_mode:
            /* 停车模式：快速停车或故障停车。 */
            switch (pm->mode.halt)
            {
                case quick_mode:
                    pmsm_quick_stop_mode(pm);
                    break;

                case fault_mode:
                    pmsm_fault_stop_mode(pm);
                    break;

                default:
                    break;
            }
            break;

        case calibrat_mode:
            /* 标定与辨识模式。没有实现的分支保持空操作。 */
            switch (pm->mode.calibrat)
            {
                case rotor_enc_cali:
                    cali_mag_encoder(pm);
                    break;

                case output_enc_mod:
                    /* 当前未实现，本周期不下发新的 FOC 指令。 */
                    break;

                case output_enc_cali:
                    /* 当前未实现，本周期不下发新的 FOC 指令。 */
                    break;

                case iden_pm:
                    iden_pmsm_first(&pm->idpm);
                    break;

                case anticogging_pm:
                    anticogging_calibration(pm);
                    break;

                default:
                    break;
            }
            break;

        case debug_mode:
            /* 旧实验控制路径：V/f、I/f 和各级闭环控制。 */
            switch (pm->mode.debug)
            {
                case drag_vf:
                    force_volt_mode(pm);
                    break;

                case volt_op:
                    if (foc_volt(pm,
                                 pm->ctrl.vd_set,
                                 pm->ctrl.vq_set,
                                 pm->foc.p_e))
                    {
                        drive_pwm_commit(pm);
                    }
                    break;

                case drag_if:
                    force_curr_mode(pm);
                    break;

                case curr_cl:
                    if (foc_curr(pm,
                                 pm->ctrl.id_set,
                                 pm->ctrl.iq_set,
                                 pm->foc.p_e))
                    {
                        drive_pwm_commit(pm);
                    }
                    break;

                case spd_curr_cl:
                    if (foc_vel(pm,
                                pm->ctrl.wr_set,
                                pm->ctrl.iq_set,
                                pm->foc.p_e))
                    {
                        drive_pwm_commit(pm);
                    }
                    break;

                case pos_spd_curr_cl:
                    if (foc_pos(pm,
                                pm->ctrl.posr_set,
                                pm->ctrl.wr_set,
                                pm->ctrl.iq_set,
                                pm->foc.p_e))
                    {
                        drive_pwm_commit(pm);
                    }
                    break;

                case spd_volt_cl:
                    /* 当前未实现，本周期不下发新的 FOC 指令。 */
                    break;

                case pos_spd_volt_cl:
                    /* 当前未实现，本周期不下发新的 FOC 指令。 */
                    break;

                default:
                    break;
            }
            break;

        default:
            /* 未知系统模式保持空操作。 */
            break;
    }
}
