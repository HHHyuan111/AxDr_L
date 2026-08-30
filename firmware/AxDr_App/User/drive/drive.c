/**
 * @file drive.c
 * @brief 电机启停、故障和运行请求的最小状态闭环。
 */

#include "drive.h"
#include "drive_pwm.h"

/**
 * @brief 执行当前选定的 PMSM 控制模式。
 *
 * @param[in,out] pm 电机控制对象。
 *
 * 本函数只负责模式分派，不修改各模式内部算法。旧工程中的 debug_mode 是实验控制
 * 路径，不是日志或显示开关；这里保留原有用途和全部映射关系。
 */
static _RAM_FUNC void drive_run_selected_mode(pmsm_t *pm)
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
            /* 保留原行为：未知系统模式不执行动作。 */
            break;
    }
}

/**
 * @brief 启动三相 PWM，并把控制器置于已知初始状态。
 *
 * @param[in,out] pm 电机控制对象。
 *
 * 这里保留原硬件顺序：先启动三相输出，再写 50% 占空比，最后清控制器。
 * pwm_active 记录软件已经发出启动动作，避免每个快速周期重复启动硬件。
 */
static _RAM_FUNC void drive_start_pwm(pmsm_t *pm)
{
    if (pm->pwm_active)
    {
        return;
    }

    drive_pwm_start();
    drive_pwm_set_neutral(pm);
    pmsm_reset(pm);
    pm->pwm_active = true;
}

/**
 * @brief 关闭三相 PWM，并清空控制器历史状态。
 *
 * @param[in,out] pm 电机控制对象。
 *
 * 仅在软件记录为已启动时执行一次停止动作，空闲周期不重复调用 HAL 停止接口。
 */
static _RAM_FUNC void drive_stop_pwm(pmsm_t *pm)
{
    if (!pm->pwm_active)
    {
        return;
    }

    drive_pwm_stop();
    pmsm_reset(pm);
    pm->pwm_active = false;
}

/**
 * @brief 执行本周期入口请求对应的动作。
 *
 * @param[in,out] pm 电机控制对象。
 * @param[in] req 本周期入口锁存的请求。
 */
static _RAM_FUNC void drive_exec_action(pmsm_t *pm, drive_req_e req)
{
    switch (req)
    {
        case DRIVE_REQ_STOP:
            drive_stop_pwm(pm);
            break;

        case DRIVE_REQ_START:
            drive_start_pwm(pm);
            break;

        case DRIVE_REQ_RUN:
            if (pm->pwm_active)
            {
                drive_run_selected_mode(pm);
            }
            break;

        default:
            /* 非法请求不允许继续保持功率输出。 */
            drive_stop_pwm(pm);
            break;
    }
}

/**
 * @brief 根据入口请求和动作结果更新 Drive 状态。
 *
 * @param[in,out] pm 电机控制对象。
 * @param[in] req 本周期入口锁存的请求。
 */
static _RAM_FUNC void drive_update_state(pmsm_t *pm, drive_req_e req)
{
    switch (req)
    {
        case DRIVE_REQ_STOP:
            pm->state = DRIVE_STATE_STOP;
            break;

        case DRIVE_REQ_START:
            if (pm->pwm_active)
            {
                pm->state = DRIVE_STATE_STARTING;
            }
            else
            {
                pm->state = DRIVE_STATE_STOP;
            }
            break;

        case DRIVE_REQ_RUN:
            if (pm->pwm_active)
            {
                pm->state = DRIVE_STATE_RUN;
            }
            else
            {
                pm->state = DRIVE_STATE_STOP;
                pm->req = DRIVE_REQ_STOP;
            }
            break;

        default:
            pm->state = DRIVE_STATE_STOP;
            pm->req = DRIVE_REQ_STOP;
            break;
    }
}

_RAM_FUNC void drive_fast_step(pmsm_t *pm)
{
    const drive_req_e req = pm->req;

    /* 每周期从空命令开始；只有实际执行的启动或控制路径可以重新生成命令。 */
    pm->pwm_cmd.seq = pm->fast_seq;
    pm->pwm_cmd.valid = false;

    /* 保持原快速链先执行本周期动作、再汇总故障的先后关系。 */
    drive_exec_action(pm, req);

    if (pm->fault.all > 0U)
    {
        /* 故障在本周期末关闭输出；已关闭时不会重复调用停止接口。 */
        drive_stop_pwm(pm);
        pm->state = DRIVE_STATE_FAULT;
        pm->req = DRIVE_REQ_STOP;
        return;
    }

    drive_update_state(pm, req);

    /* START 是一次性请求；启动成功后，下一快速周期进入正常 RUN。 */
    if ((req == DRIVE_REQ_START) &&
        (pm->req == DRIVE_REQ_START) &&
        (pm->state == DRIVE_STATE_STARTING))
    {
        pm->req = DRIVE_REQ_RUN;
    }
}
