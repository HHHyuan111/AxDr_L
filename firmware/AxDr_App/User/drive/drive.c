/**
 * @file drive.c
 * @brief 电机启停、故障和运行请求的最小状态闭环。
 */

#include "drive.h"

#include "common.h"
#include "drive_diag.h"
#include "drive_mode.h"
#include "drive_pwm.h"
#include "drive_reset.h"

static _RAM_FUNC void drive_update_protection(foc_t *foc)
{
    const bool power_requested = foc->pwm_active ||
                                 (foc->req == DRIVE_REQ_START) ||
                                 (foc->req == DRIVE_REQ_RUN);
    const bool currents_valid = foc->fb_status.i_valid &&
                                isfinite(foc->sig.i_a) &&
                                isfinite(foc->sig.i_b) &&
                                isfinite(foc->sig.i_c);
    const bool bus_voltage_valid = foc->fb_status.vbus_valid &&
                                   isfinite(foc->sig.vbus) &&
                                   (foc->sig.vbus >= 0.0f);
    const bool position_valid = foc->fb_status.pos_valid &&
                                isfinite(foc->sig.p_e) &&
                                isfinite(foc->sig.mp_r) &&
                                isfinite(foc->sig.mp_m);
    const drive_protection_sample_t sample = {
        .current_a_a = foc->sig.i_a,
        .current_b_a = foc->sig.i_b,
        .current_c_a = foc->sig.i_c,
        .bus_voltage_v = foc->sig.vbus,
        .mos_temperature_c = foc->sig.Tmos,
        .coil_temperature_c = foc->sig.Tcoil,
        .rotor_speed_rad_s = foc->sig.wr_f,
        .currents_valid = currents_valid,
        .bus_voltage_valid = bus_voltage_valid,
        /* 当前板级采样链尚未接入两个温度 ADC，不宣称温度数据有效。 */
        .mos_temperature_valid = false,
        .coil_temperature_valid = false,
        .rotor_speed_valid = position_valid,
        .position_valid = position_valid,
        .power_stage_active = power_requested,
    };
    const uint32_t faults = drive_protection_step(&foc->prot_state,
                                                  &foc->prot_cfg,
                                                  &sample);

    if ((faults & DRIVE_PROTECTION_FAULT_OVER_CURRENT) != 0U)
    {
        foc->fault.bit.ov_curr = 1U;
    }
    if ((faults & DRIVE_PROTECTION_FAULT_UNDER_VOLTAGE) != 0U)
    {
        foc->fault.bit.un_volt = 1U;
    }
    if ((faults & DRIVE_PROTECTION_FAULT_OVER_VOLTAGE) != 0U)
    {
        foc->fault.bit.ov_volt = 1U;
    }
    if ((faults & DRIVE_PROTECTION_FAULT_MOS_OVER_TEMPERATURE) != 0U)
    {
        foc->fault.bit.ov_tmos = 1U;
    }
    if ((faults & DRIVE_PROTECTION_FAULT_COIL_OVER_TEMPERATURE) != 0U)
    {
        foc->fault.bit.ov_tcoi = 1U;
    }
    if ((faults & DRIVE_PROTECTION_FAULT_OVER_SPEED) != 0U)
    {
        foc->fault.bit.ov_speed = 1U;
    }
    if ((faults & DRIVE_PROTECTION_FAULT_CURRENT_FEEDBACK) != 0U)
    {
        foc->fault.bit.ioff_err = 1U;
    }
    if ((faults & DRIVE_PROTECTION_FAULT_BUS_FEEDBACK) != 0U)
    {
        foc->fault.bit.un_volt = 1U;
    }
    if ((faults & DRIVE_PROTECTION_FAULT_POSITION_FEEDBACK) != 0U)
    {
        foc->fault.bit.enc_err = 1U;
    }
}

/**
 * @brief 启动三相 PWM，并把控制器置于已知初始状态。
 *
 * @param[in,out] foc 电机控制对象。
 *
 * 先把三个比较寄存器写成 50%，再打开三相输出，避免启动瞬间沿用旧 CCR。
 * 最后清空控制器历史状态。
 * pwm_active 记录软件已经发出启动动作，避免每个快速周期重复启动硬件。
 */
static _RAM_FUNC void drive_start_pwm(foc_t *foc)
{
    if (foc->pwm_active || !drive_mode_prepare(foc))
    {
        return;
    }

    drive_pwm_set_neutral(foc);
    if (drive_pwm_start())
    {
        drive_control_reset(foc);
        foc->pwm_active = true;
    }
    else
    {
        foc->fault.bit.pwm_err = 1U;
    }
}

/**
 * @brief 关闭三相 PWM，并清空控制器历史状态。
 *
 * @param[in,out] foc 电机控制对象。
 *
 * 仅在软件记录为已启动时执行一次停止动作，空闲周期不重复调用 HAL 停止接口。
 */
static _RAM_FUNC void drive_stop_pwm(foc_t *foc)
{
    if (!foc->pwm_active)
    {
        drive_diag_on_stopped();
        return;
    }

    const bool stopped = drive_pwm_stop();

    drive_control_reset(foc);
    if (stopped)
    {
        foc->pwm_active = false;
        drive_diag_on_stopped();
    }
    else
    {
        foc->fault.bit.pwm_err = 1U;
    }
}

/**
 * @brief 执行本周期入口请求对应的动作。
 *
 * @param[in,out] foc 电机控制对象。
 * @param[in] req 本周期入口锁存的请求。
 */
static _RAM_FUNC void drive_exec_action(foc_t *foc, drive_req_e req)
{
    switch (req)
    {
        case DRIVE_REQ_STOP:
            drive_stop_pwm(foc);
            break;

        case DRIVE_REQ_START:
            drive_start_pwm(foc);
            break;

        case DRIVE_REQ_RUN:
            if (foc->pwm_active)
            {
                const bool mode_valid = drive_mode_step(foc);

                if (!mode_valid || !foc->pwm_cmd.valid)
                {
                    /* 未实现模式或无效计算不得继续沿用上一拍物理占空比。 */
                    if (foc->pwm_active)
                    {
                        drive_stop_pwm(foc);
                    }
                    else
                    {
                        /* PWM 提交层已经关断硬件时，仍需清空控制器历史。 */
                        drive_control_reset(foc);
                    }
                    foc->req = DRIVE_REQ_STOP;
                }
            }
            break;

        default:
            /* 非法请求不允许继续保持功率输出。 */
            drive_stop_pwm(foc);
            break;
    }
}

/**
 * @brief 根据入口请求和动作结果更新 Drive 状态。
 *
 * @param[in,out] foc 电机控制对象。
 * @param[in] req 本周期入口锁存的请求。
 */
static _RAM_FUNC void drive_update_state(foc_t *foc, drive_req_e req)
{
    switch (req)
    {
        case DRIVE_REQ_STOP:
            foc->state = DRIVE_STATE_STOP;
            break;

        case DRIVE_REQ_START:
            if (foc->pwm_active)
            {
                foc->state = DRIVE_STATE_STARTING;
            }
            else
            {
                foc->state = DRIVE_STATE_STOP;
                foc->req = DRIVE_REQ_STOP;
            }
            break;

        case DRIVE_REQ_RUN:
            if (foc->pwm_active)
            {
                foc->state = DRIVE_STATE_RUN;
            }
            else
            {
                foc->state = DRIVE_STATE_STOP;
                foc->req = DRIVE_REQ_STOP;
            }
            break;

        default:
            foc->state = DRIVE_STATE_STOP;
            foc->req = DRIVE_REQ_STOP;
            break;
    }
}

bool drive_fault_clear(foc_t *foc)
{
    if (foc->pwm_active)
    {
        return false;
    }

    foc->fault.all = 0U;
    drive_protection_reset(&foc->prot_state);
    drive_control_reset(foc);
    foc->req = DRIVE_REQ_STOP;
    foc->state = DRIVE_STATE_STOP;
    return true;
}

_RAM_FUNC void drive_fast_step(foc_t *foc)
{
    drive_diag_poll_request(foc);
    const drive_req_e req = foc->req;

    /* 每周期从空命令开始；只有实际执行的启动或控制路径可以重新生成命令。 */
    foc->pwm_cmd.seq = foc->fast_seq;
    foc->pwm_cmd.valid = false;

    drive_update_protection(foc);

    /* 已锁存故障时，当前周期禁止执行 START 或 RUN。 */
    if (foc->fault.all > 0U)
    {
        drive_diag_on_fault();
        drive_stop_pwm(foc);
        foc->state = DRIVE_STATE_FAULT;
        foc->req = DRIVE_REQ_STOP;
        return;
    }

    drive_exec_action(foc, req);

    if (foc->fault.all > 0U)
    {
        /* 故障在本周期末关闭输出；已关闭时不会重复调用停止接口。 */
        drive_diag_on_fault();
        drive_stop_pwm(foc);
        foc->state = DRIVE_STATE_FAULT;
        foc->req = DRIVE_REQ_STOP;
        return;
    }

    drive_update_state(foc, req);

    /* START 是一次性请求；启动成功后，下一快速周期进入正常 RUN。 */
    if ((req == DRIVE_REQ_START) &&
        (foc->req == DRIVE_REQ_START) &&
        (foc->state == DRIVE_STATE_STARTING))
    {
        foc->req = DRIVE_REQ_RUN;
    }
}
