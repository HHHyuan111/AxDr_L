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

#define DRIVE_DIRECTION_TARGET_MIN_RAD_S   (5.0f)
#define DRIVE_DIRECTION_FEEDBACK_MIN_RAD_S (5.0f)
#define DRIVE_DIRECTION_GUARD_WINDOW_S     (0.050f)
#define DRIVE_DIRECTION_MISMATCH_HOLD_S    (0.005f)

static uint32_t drive_seconds_to_ticks(const foc_t *foc, float seconds)
{
    const float fs = foc->rate.foc_fs;
    const float ticks = seconds * fs;

    if (!isfinite(ticks) || (ticks < 1.0f))
    {
        return 1U;
    }
    return (uint32_t)ticks;
}

static void drive_direction_guard_step(foc_t *foc)
{
    const float target = foc->ref.spd_r;
    const float feedback = foc->fb.spd_r;
    const bool target_valid = isfinite(target) &&
                               (fabsf(target) >=
                                DRIVE_DIRECTION_TARGET_MIN_RAD_S);
    const bool feedback_valid = isfinite(feedback) &&
                                 (fabsf(feedback) >=
                                  DRIVE_DIRECTION_FEEDBACK_MIN_RAD_S);
    const bool opposite = target_valid && feedback_valid &&
                          ((target * feedback) < 0.0f);
    const uint32_t guard_window =
        drive_seconds_to_ticks(foc, DRIVE_DIRECTION_GUARD_WINDOW_S);
    const uint32_t mismatch_hold =
        drive_seconds_to_ticks(foc, DRIVE_DIRECTION_MISMATCH_HOLD_S);

    if (!foc->direction_guard_armed || !foc->pwm_active ||
        (foc->req != DRIVE_REQ_RUN))
    {
        foc->direction_guard_ticks = 0U;
        foc->direction_mismatch_ticks = 0U;
        return;
    }

    if (foc->direction_guard_ticks++ >= guard_window)
    {
        foc->direction_guard_armed = false;
        foc->direction_mismatch_ticks = 0U;
        return;
    }

    if (!opposite)
    {
        foc->direction_mismatch_ticks = 0U;
        return;
    }

    if (foc->direction_mismatch_ticks < mismatch_hold)
    {
        foc->direction_mismatch_ticks++;
    }
    if (foc->direction_mismatch_ticks >= mismatch_hold)
    {
        /* 编码器信号可能有效但方向定义相反；这里先安全停机，保留
         * enc_err 供上位机区分，后续再单独校验 A/B 方向和相序。 */
        foc->fault.bit.enc_err = 1U;
        foc->direction_guard_armed = false;
    }
}

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
    const bool start_from_rest = isfinite(foc->fb.spd_r) &&
                                 (fabsf(foc->fb.spd_r) <
                                  DRIVE_DIRECTION_FEEDBACK_MIN_RAD_S);

    if (foc->pwm_active || !drive_mode_prepare(foc))
    {
        return;
    }

    drive_pwm_set_neutral(foc);
    if (drive_pwm_start())
    {
        drive_control_reset_for_start(foc);
        foc->direction_guard_armed = start_from_rest;
        foc->pwm_active = true;
    }
    else
    {
        foc->fault.bit.pwm_err = 1U;
    }
}

/* 对齐状态清理定义在对齐代码块内，此处先行声明供停机路径调用。 */
static _RAM_FUNC void drive_align_reset(foc_t *foc);

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
    if (!foc->enc_aligned)
    {
        drive_align_reset(foc); /* PWM 关断即对齐中止：进度作废，
                                 * 故障停机路径同样生效，下次 START 重来 */
    }
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
 * ABZ 上电自动对齐（参考预定位+整圈扫场）：电流幅值 1s 线性渐变
 * 0→档案值（参考 CALIB_RISE_CNT），随后场角扫一整圈收尾到 0，
 * 全电流保持，末 0.3s 对 fb.theta_e 做圆均值定零。
 * 扫场注：本台架联轴静摩擦 ~3A 等效，转子停位离场 0 仅 ~34° 电角
 * （sin 小），纯静场（参考做法）拉力上限不足以破摩擦——多轮真机
 * 对齐期位置曲线全平（2026-10-18 三轮实测）。场扫一整圈必经过
 * 转子正交位（拉力=满幅），转子被移动场拖到场 0 落位，参考
 * "吸合停位=零位"的前提才成立。定零=参考 __HAL_TIM_SET_COUNTER
 * (htim3,0) 的等价实现：e_off -= 均值 (mod 2pi)（不清计数器，
 * 位置链无差分突变）。
 * 完成：置 enc_aligned 并回 STOP（对齐是校准不是运行授权，上位机
 * 轮询 alignment_valid 后自行失能，再显式使能进模式）。
 * 唯一拒绝条件 R̄ < 0.9（在转/编码器异常样本分散，同 0x12 诊断档；
 * 静止转子恒通过）——参考无到位验收，不设。
 */
#define DRIVE_ALIGN_RISE_S         (1.0f)  /* 电流线性渐变（参考 CALIB_RISE_CNT） */
#define DRIVE_ALIGN_SWEEP_S        (1.5f)  /* 场扫一整圈 0→2pi 收尾到 0 */
#define DRIVE_ALIGN_HOLD_S         (0.7f)  /* 全电流保持（含采样窗） */
#define DRIVE_ALIGN_AVG_S          (0.3f)  /* 末段定零采样窗（抗编码器噪声） */
#define DRIVE_ALIGN_MIN_RESULT_LEN (0.9f)  /* 同 DIAG_ALIGN_MIN_RESULTANT_RATIO */

static _RAM_FUNC void drive_align_reset(foc_t *foc)
{
    foc->align_ticks = 0U;
    foc->align_sin_sum = 0.0f;
    foc->align_cos_sum = 0.0f;
}

static _RAM_FUNC void drive_align_reject(foc_t *foc)
{
    /* 验收不过：置编码器故障（fault 汇聚路径同拍停机拒绝 RUN），
     * 清对齐进度——清故障后重新 START 可重试。 */
    foc->fault.bit.enc_err = 1U;
    drive_align_reset(foc);
}

static _RAM_FUNC void drive_align_tick(foc_t *foc)
{
    const uint32_t rise_ticks =
        (uint32_t)(DRIVE_ALIGN_RISE_S * foc->rate.foc_fs);
    const uint32_t sweep_ticks =
        (uint32_t)(DRIVE_ALIGN_SWEEP_S * foc->rate.foc_fs);
    const uint32_t hold_ticks =
        (uint32_t)(DRIVE_ALIGN_HOLD_S * foc->rate.foc_fs);
    const uint32_t avg_ticks =
        (uint32_t)(DRIVE_ALIGN_AVG_S * foc->rate.foc_fs);
    const uint32_t sweep_end = rise_ticks + sweep_ticks;
    const uint32_t total_ticks = sweep_end + hold_ticks;
    const uint32_t avg_start = total_ticks - avg_ticks;
    const uint32_t n = foc->align_ticks;

    {
        /* 参考预定位：电流线性渐变+保持；渐变后场扫一整圈收尾到 0，
         * 移动场拖转子到场 0 落位（纯静场在本台架拉不动，见块注释） */
        float id_cmd = foc->motor.align_current_a;
        float field = 0.0f;

        if (n < rise_ticks)
        {
            id_cmd *= (float)n / (float)rise_ticks;
        }
        else if (n < sweep_end)
        {
            field = M_2PI * (float)(n - rise_ticks) / (float)sweep_ticks;
        }
        if (foc_cur_step(foc, id_cmd, 0.0f, field))
        {
            (void)drive_pwm_commit(foc);
        }
    }

    if (n < total_ticks)
    {
        if (n >= avg_start)
        {
            /* θ_e 圆均值累积（规避 0/2pi 边界线性均值失效） */
            float s;
            float c;

            fast_sincos(foc->fb.theta_e, &s, &c);
            foc->align_sin_sum += s;
            foc->align_cos_sum += c;
        }
        foc->align_ticks++;
        return;
    }

    {
        const float result_len =
            sqrtf(foc->align_sin_sum * foc->align_sin_sum +
                  foc->align_cos_sum * foc->align_cos_sum) /
            (float)avg_ticks;
        const float mean = atan2f(foc->align_sin_sum, foc->align_cos_sum);

        if (result_len >= DRIVE_ALIGN_MIN_RESULT_LEN)
        {
            /* 定零=参考清 TIM3 计数器的等价实现：吸合停位即零位，
             * e_off −= 均值 (mod 2pi)。增量语义：均值由 fb.theta_e
             * （已含现行 e_off）算出，重对齐/flash 装载残留不叠加。 */
            float e_off = foc->motor.e_off - mean;
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

            /* 对齐是校准动作，不是运行授权。立即关闭 PWM，避免对齐命令
             * 与此前缓存的速度/电流目标在同一个状态链中自动接通。下一次
             * 显式 START 才允许走已对齐的正常 RUN 入口。 */
            drive_stop_pwm(foc);
            foc->req = DRIVE_REQ_STOP;
        }
        else
        {
            drive_align_reject(foc);
        }
    }
}

static _RAM_FUNC void drive_exec_action(foc_t *foc, drive_req_e req)
{
    switch (req)
    {
        case DRIVE_REQ_STOP:
            drive_stop_pwm(foc); /* 内部含未完成对齐的进度作废 */
            break;

        case DRIVE_REQ_START:
            drive_start_pwm(foc); /* 内部一次性动作：中性占空比+启动+置 pwm_active */
            if (foc->pwm_active)
            {
                if ((foc->enc.primary == ENCODER_TYPE_ABZ) &&
                    !foc->enc_aligned)
                {
                    /* STARTING 期间逐拍执行自动对齐；完成后由对齐逻辑
                     * 关断 PWM 并回到 STOP，等待下一次显式 START。 */
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
    drive_direction_guard_step(foc);

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
     * ABZ 对齐场景由 drive_align_tick 在成功后主动关断并回到 STOP）。 */
}
