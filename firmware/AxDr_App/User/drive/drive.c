/**
 * @file drive.c
 * @brief 电机启停、故障和运行请求的最小状态闭环。
 */

#include "drive.h"

#include <math.h>

#include "common.h"
#include "fast_trig.h"
#include "drive_diag.h"
#include "drive_mode.h"
#include "drive_pwm.h"
#include "drive_reset.h"

static _RAM_FUNC void drive_update_protection(foc_t *foc)
{
    const bool power_requested = foc->pwm_active ||
                                 (foc->req == DRIVE_REQ_START) ||
                                 (foc->req == DRIVE_REQ_RUN);
    const bool currents_valid = foc->fb.i_valid &&
                                isfinite(foc->fb.ia) &&
                                isfinite(foc->fb.ib) &&
                                isfinite(foc->fb.ic);
    const bool bus_voltage_valid = foc->fb.vbus_valid &&
                                   isfinite(foc->fb.vbus) &&
                                   (foc->fb.vbus >= 0.0f);
    const bool position_valid = foc->fb.pos_valid &&
                                isfinite(foc->fb.theta_e) &&
                                isfinite(foc->fb.pos_r) &&
                                isfinite(foc->fb.pos_m);
    const drive_protection_sample_t sample = {
        .ia = foc->fb.ia,
        .ib = foc->fb.ib,
        .ic = foc->fb.ic,
        .vbus = foc->fb.vbus,
        .temp_mos = foc->fb.temp_mos,
        .temp_coil = foc->fb.temp_coil,
        .spd = foc->fb.spd_r,
        .i_valid = currents_valid,
        .vbus_valid = bus_voltage_valid,
        /* 当前板级采样链尚未接入两个温度 ADC，不宣称温度数据有效。 */
        .temp_mos_valid = false,
        .temp_coil_valid = false,
        .spd_valid = position_valid,
        .pos_valid = position_valid,
        .pwm_on = power_requested,
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
/*
 * ABZ 上电自动对齐（B 库门控哲学落地）：START 期间以固定电角度 0 注入
 * 档案对齐电流（foc_cur_step 的角度参数与编码器无关）——转子被吸到
 * "真实电角度=0"位置；后段对 fb.theta_e 做圆均值，e_off += -均值 (mod 2pi)。
 * 完成：置 enc_aligned 并自动晋升 req=RUN（本设计的显式行为，代码库中
 * 唯一的 RUN 写入点）。时长 = 保持 1.2s + 取平均 0.3s（B 库契约 >=300ms）。
 */
#define DRIVE_ALIGN_SETTLE_S   (1.2f)
#define DRIVE_ALIGN_AVG_S      (0.3f)
/*
 * 对齐验收双判据（26 号手册 §2，理论依据见 25/26 号）：
 * ① 角度收敛：圆均值矢量长度 R̄ >= 0.99（循环统计学 s=sqrt(-2 ln R̄)，
 *    0.99 对应角标准差约 8.1° 电角——转子锁定则样本集中于单点，R̄→1；
 *    转子被卡/仍在转/编码器异常则样本分散，R̄→0）；
 * ② 电流建立：|fb.id| >= 0.8x 指令（场给了电流没到位=采样或功率链问题，
 *    0.8 为工程容差，参照 B 库"对齐最小有效电流"概念）。
 * 任一不过 → 置 enc_err 故障拒绝对齐结果（错误零点闭环比不运行危险，
 *    P6 讲解决策 D1）。 */
#define DRIVE_ALIGN_MIN_RESULT_LEN  (0.99f)
#define DRIVE_ALIGN_CURRENT_RATIO   (0.8f)

static _RAM_FUNC void drive_align_reset(foc_t *foc)
{
    foc->align_ticks = 0U;
    foc->align_sin_sum = 0.0f;
    foc->align_cos_sum = 0.0f;
}

static _RAM_FUNC void drive_align_tick(foc_t *foc)
{
    const uint32_t settle_ticks =
        (uint32_t)(DRIVE_ALIGN_SETTLE_S * foc->rate.foc_fs);
    const uint32_t avg_ticks =
        (uint32_t)(DRIVE_ALIGN_AVG_S * foc->rate.foc_fs);
    float s;
    float c;

    /* 固定角 0 的 id 场：角度参数恒 0（开环），与编码器反馈无关 */
    if (foc_cur_step(foc, foc->motor.align_current_a, 0.0f, 0.0f))
    {
        (void)drive_pwm_commit(foc);
    }

    if (foc->align_ticks < settle_ticks + avg_ticks)
    {
        foc->align_ticks++;
        if (foc->align_ticks > settle_ticks)
        {
            /* 保持期后取平均：theta_e 圆均值（规避 0/2pi 边界线性均值失效） */
            fast_sincos(foc->fb.theta_e, &s, &c);
            foc->align_sin_sum += s;
            foc->align_cos_sum += c;
        }
        return;
    }

    {
        const float result_len =
            sqrtf(foc->align_sin_sum * foc->align_sin_sum +
                  foc->align_cos_sum * foc->align_cos_sum) / (float)avg_ticks;
        const bool angle_converged = (result_len >= DRIVE_ALIGN_MIN_RESULT_LEN);
        const bool current_established =
            (fabsf(foc->fb.id) >=
             DRIVE_ALIGN_CURRENT_RATIO * foc->motor.align_current_a);

        if (angle_converged && current_established)
        {
            /* 增量更新：圆均值由 fb.theta_e（已含现行 e_off）算出，新零点
             * = 旧零点 − 均值。绝对替换在重对齐（现行 e_off≠0，如 flash
             * 装载残留）时多扣旧值，产生 e_off_old 大小的常量角误差
             * （真机实测形态 δ=e_off_old，锁轴电流被拉开到 q 轴）。 */
            float e_off = foc->motor.e_off
                          + atan2f(-foc->align_sin_sum, foc->align_cos_sum);
            if (e_off >= 6.28318530718f)
            {
                e_off -= 6.28318530718f;
            }
            if (e_off < 0.0f)
            {
                e_off += 6.28318530718f;
            }
            foc->motor.e_off = e_off;
            foc->enc_aligned = true;
            foc->req = DRIVE_REQ_RUN; /* 自动晋升：对齐完成即进 RUN */
        }
        else
        {
            /* 验收不过：置编码器故障（fault 汇聚路径同拍停机拒绝 RUN），
             * 清对齐进度——清故障后重新 START 可重试。 */
            foc->fault.bit.enc_err = 1U;
            drive_align_reset(foc);
        }
    }
}

static _RAM_FUNC void drive_exec_action(foc_t *foc, drive_req_e req)
{
    switch (req)
    {
        case DRIVE_REQ_STOP:
            drive_stop_pwm(foc);
            if (!foc->enc_aligned)
            {
                drive_align_reset(foc); /* 中断对齐则进度作废，下次重来 */
            }
            break;

        case DRIVE_REQ_START:
            drive_start_pwm(foc); /* 内部一次性动作：中性占空比+启动+置 pwm_active */
            if (foc->pwm_active)
            {
                if ((foc->enc.primary == ENCODER_TYPE_ABZ) &&
                    !foc->enc_aligned)
                {
                    /* STARTING 期间逐拍执行自动对齐；完成时内部晋升 RUN */
                    drive_align_tick(foc);
                }
                else
                {
                    /* 绝对编码器或本上电周期已对齐：直接进 RUN */
                    foc->req = DRIVE_REQ_RUN;
                }
            }
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

    /* START 一次性语义由 exec_action 内完成晋升（非对齐场景启动即置 RUN；
     * ABZ 对齐场景保持 START 直到对齐完成由 drive_align_tick 晋升——
     * 此处原先的尾部自动晋升会把对齐第一拍后 premature 晋升，已移除）。 */
}
