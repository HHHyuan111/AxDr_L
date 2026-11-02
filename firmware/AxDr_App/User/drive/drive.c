/**
 * @file drive.c
 * @brief 电机启停、故障和运行请求的状态闭环。主链位置：第 2~4、7 步（保护、状态、给定、输出）。
 */

#include "drive.h"
#include "drive_cycle.h"

#include <math.h>

#include "common.h"
#include "fast_trig.h"
#include "drive_diag.h"
#include "drive_mode.h"
#include "drive_pwm.h"
#include "drive_reset.h"

/* 状态只保存在 foc->state；req 是外部请求。
 * STOP --START--> STARTING --下一拍--> RUN / DIAG
 * 未定零的 ABZ：STARTING --> ALIGN --完成--> STOP
 * 任意状态：STOP 请求回 STOP；故障回 FAULT；清故障后才允许重新启动。
 */
static void drive_start_pwm(foc_t *foc);
static void drive_stop_pwm(foc_t *foc);
static void drive_align_reset(foc_t *foc);
static void drive_align_prepare(foc_t *foc, foc_plan_t *cmd);

/* 进入动作集中在这里。持续运行不重复启动、清 PI 或重置对齐进度。 */
static _RAM_FUNC void drive_enter_state(foc_t *foc, drive_state_e next)
{
    if (next == DRIVE_STATE_STARTING)
    {
        /* prepare 必须看到原 STOP 状态；先中性占空比，再开 PWM。 */
        drive_start_pwm(foc);
        if (foc->fault.all != 0U)
        {
            next = DRIVE_STATE_FAULT;
        }
        else if (!foc->pwm_active)
        {
            next = DRIVE_STATE_STOP; /* 模式未获准，消费这次启动请求。 */
        }
        else
        {
            foc->req = DRIVE_REQ_RUN;
        }
    }

    if (next == DRIVE_STATE_STOP || next == DRIVE_STATE_FAULT)
    {
        if (next == DRIVE_STATE_FAULT)
        {
            drive_diag_on_fault();
        }
        drive_stop_pwm(foc);
        if (!foc->enc_aligned)
        {
            drive_align_reset(foc);
        }
        foc->req = DRIVE_REQ_STOP;
        /* 关断失败保留 pwm_active，下一拍重试。 */
        if (foc->fault.all != 0U)
        {
            next = DRIVE_STATE_FAULT;
        }
    }
    foc->state = next;
}

/* 第三步：先处理故障和停止，再看当前状态允许向哪里转换。 */
_RAM_FUNC void drive_update_state(foc_t *foc)
{
    if (foc->fault.all != 0U || foc->state == DRIVE_STATE_FAULT)
    {
        drive_enter_state(foc, DRIVE_STATE_FAULT);
        return;
    }
    if (foc->req != DRIVE_REQ_START && foc->req != DRIVE_REQ_RUN)
    {
        drive_enter_state(foc, DRIVE_STATE_STOP);
        return;
    }

    switch (foc->state)
    {
        case DRIVE_STATE_STOP:
            if (foc->req != DRIVE_REQ_START || foc->pwm_active)
            {
                drive_enter_state(foc, DRIVE_STATE_STOP);
                break;
            }
            drive_enter_state(foc, DRIVE_STATE_STARTING);
            /* 对齐门控按模式：VF/IF 用虚拟角度，不要求编码器对齐（02 规范 §3）。 */
            if (foc->state == DRIVE_STATE_STARTING &&
                drive_mode_requires_encoder(foc) &&
                foc->enc.primary == ENCODER_TYPE_ABZ && !foc->enc_aligned)
            {
                drive_enter_state(foc, DRIVE_STATE_ALIGN);
            }
            break;

        case DRIVE_STATE_STARTING:
            if (!foc->pwm_active)
            {
                drive_enter_state(foc, DRIVE_STATE_STOP);
            }
            else if (foc->mode.sys == calibrat_mode)
            {
                drive_enter_state(foc, DRIVE_STATE_DIAG);
            }
            else
            {
                drive_enter_state(foc, DRIVE_STATE_RUN);
            }
            break;

        case DRIVE_STATE_ALIGN:
        case DRIVE_STATE_RUN:
        case DRIVE_STATE_DIAG:
            if (!foc->pwm_active ||
                (foc->state == DRIVE_STATE_RUN && foc->mode.sys == calibrat_mode) ||
                (foc->state == DRIVE_STATE_DIAG && foc->mode.sys != calibrat_mode))
            {
                /* 普通运行与诊断之间必须经过停止和重新授权。 */
                drive_enter_state(foc, DRIVE_STATE_STOP);
            }
            else
            {
                foc->req = DRIVE_REQ_RUN; /* 重复 START 保持已运行状态。 */
            }
            break;

        default:
            drive_enter_state(foc, DRIVE_STATE_STOP);
            break;
    }
}

/* 第四步：本拍做什么。这里只生成给定，PI 和 PWM 仍在七步主流程后部。 */
_RAM_FUNC void drive_handle_state(foc_t *foc, drive_cycle_t *cycle)
{
    *cycle = (drive_cycle_t){0};
    switch (foc->state)
    {
        case DRIVE_STATE_ALIGN:
            drive_align_prepare(foc, &cycle->cmd);
            cycle->ready = true;
            break;
        case DRIVE_STATE_RUN:
        case DRIVE_STATE_DIAG:
            cycle->ready = drive_mode_step(foc, &cycle->cmd);
            break;
        default:
            break; /* 其余状态无控制给定。 */
    }
}

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
    bool position_valid = foc->fb.pos_valid &&
                          isfinite(foc->fb.theta_e) &&
                          isfinite(foc->fb.pos_r) &&
                          isfinite(foc->fb.pos_m);
    if (!drive_mode_requires_encoder(foc))
    {
        /* VF/IF 用虚拟角度：位置前提不参与保护（02 规范 §6.4）。
         * 本板编码器仍工作，过速保护继续使用实测速度；无感前提在 S5 定义。 */
        position_valid = true;
    }
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

/* ABZ 上电定零：1 s 电流渐变 → 1.5 s 扫场 → 0.7 s 保持。
 * 末 0.3 s 对电角度做圆均值，更新 e_off；不清编码器计数器。
 * 扫场沿用既有台架方案和参数。圆均值集中度只检查样本是否集中。
 * 成功后关闭输出，等待下一次显式 START。
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

static _RAM_FUNC void drive_align_prepare(foc_t *foc, foc_plan_t *cmd)
{
    const uint32_t rise_ticks = (uint32_t)(DRIVE_ALIGN_RISE_S * foc->rate.foc_fs);
    const uint32_t sweep_ticks = (uint32_t)(DRIVE_ALIGN_SWEEP_S * foc->rate.foc_fs);
    const uint32_t sweep_end = rise_ticks + sweep_ticks;
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
        *cmd = (foc_plan_t){
            .ref = {.mode = FOC_CTRL_MODE_CUR, .id_ref = id_cmd, .iq_ref = 0.0f},
            .theta = field,
        };
    }

}

static _RAM_FUNC bool drive_align_finish(foc_t *foc)
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
        return false;
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
        }
        else
        {
            drive_align_reject(foc);
        }
    }
    return true; /* 完成或拒绝；退出状态统一在 drive_output 中处理。 */
}

bool drive_fault_clear(foc_t *foc)
{
    if (foc->pwm_active)
    {
        return false;
    }

    foc->fault.all = 0U;
    foc->first_fault_bits = 0U;
    foc->first_fault_seq = 0U;
    drive_protection_reset(&foc->prot_state);
    drive_control_reset(foc);
    foc->req = DRIVE_REQ_STOP;
    foc->state = DRIVE_STATE_STOP;
    return true;
}

_RAM_FUNC void drive_check_protection(foc_t *foc)
{
    drive_diag_poll_request(foc);

    /* 首故障定格：上一拍新置的锁存位在此首拍记录（含保护/对齐拒绝/
     * 方向守卫/换向异常全部来源）。多故障同亮时保住首因不覆盖。 */
    if ((foc->fault.all != 0U) && (foc->first_fault_bits == 0U))
    {
        foc->first_fault_bits = foc->fault.all;
        foc->first_fault_seq = foc->fast_seq;
    }

    /* 每周期从空命令开始；只有实际执行的启动或控制路径可以重新生成命令。 */
    foc->pwm_cmd.seq = foc->fast_seq;
    foc->pwm_cmd.valid = false;

    drive_update_protection(foc);
    drive_direction_guard_step(foc);
}

/* 第七步：普通占空比只有这里提交；计算或提交失败立即退出运行。 */
_RAM_FUNC void drive_output(foc_t *foc, drive_cycle_t *cycle)
{
    if (foc->state == DRIVE_STATE_FAULT)
    {
        return; /* 本拍已执行关断。 */
    }
    if (foc->fault.all != 0U)
    {
        drive_enter_state(foc, DRIVE_STATE_FAULT);
        return;
    }
    if (foc->state != DRIVE_STATE_RUN && foc->state != DRIVE_STATE_DIAG &&
        foc->state != DRIVE_STATE_ALIGN)
    {
        return;
    }

    bool valid = cycle->ready && cycle->valid;
    if (valid && foc->state == DRIVE_STATE_DIAG)
    {
        valid = drive_diag_check_output(foc, &cycle->cmd);
    }
    if (valid)
    {
        valid = drive_pwm_commit(foc);
    }
    if (!valid)
    {
        foc->pwm_cmd.valid = false;
        if (foc->state == DRIVE_STATE_DIAG)
        {
            drive_diag_on_fault();
        }
        /* commit 可能已关断硬件，仍需清控制历史和未完成的对齐。 */
        if (!foc->pwm_active)
        {
            drive_control_reset(foc);
        }
        {
            drive_state_e next = DRIVE_STATE_STOP;

            if (foc->fault.all != 0U)
            {
                next = DRIVE_STATE_FAULT;
            }
            drive_enter_state(foc, next);
        }
        return;
    }
    if (foc->state == DRIVE_STATE_ALIGN && drive_align_finish(foc))
    {
        /* 对齐只是校准；完成后必须再次显式 START。 */
        {
            drive_state_e next = DRIVE_STATE_STOP;

            if (foc->fault.all != 0U)
            {
                next = DRIVE_STATE_FAULT;
            }
            drive_enter_state(foc, next);
        }
    }
}
