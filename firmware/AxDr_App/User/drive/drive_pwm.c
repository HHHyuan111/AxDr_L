/**
 * @file drive_pwm.c
 * @brief Drive 层三相 PWM 执行适配。
 *
 * 数据流：逻辑 A/B/C 相占空比 -> 相序映射 -> Target 物理通道。
 */

#include "drive_pwm.h"

#include <math.h>

#include "common.h"
#include "target_pwm.h"

static _RAM_FUNC bool drive_pwm_duty_is_valid(float duty)
{
    return isfinite(duty) && (duty >= 0.0f) && (duty <= 1.0f);
}

_RAM_FUNC bool drive_pwm_start(void)
{
    return target_pwm_start_phase_outputs();
}

_RAM_FUNC bool drive_pwm_stop(void)
{
    return target_pwm_stop_phase_outputs();
}

_RAM_FUNC void drive_pwm_set_neutral(foc_t *foc)
{
    foc->pwm_cmd = (drive_pwm_cmd_t){
        .seq = foc->fast_seq,
        .valid = true,
        .duty_a = 0.5f,
        .duty_b = 0.5f,
        .duty_c = 0.5f
    };

    target_pwm_set_duty_ratios(0.5f, 0.5f, 0.5f);

    foc->pwm_commit = (drive_pwm_commit_t){
        .seq = foc->fast_seq,
        .valid = true,
        .duty_a = 0.5f,
        .duty_b = 0.5f,
        .duty_c = 0.5f
    };
}

_RAM_FUNC bool drive_pwm_commit(foc_t *foc)
{
    foc->pwm_cmd = (drive_pwm_cmd_t){
        .seq = foc->fast_seq,
        .valid = false,
        .duty_a = foc->out.duty_a,
        .duty_b = foc->out.duty_b,
        .duty_c = foc->out.duty_c
    };

    if (!drive_pwm_duty_is_valid(foc->out.duty_a) ||
        !drive_pwm_duty_is_valid(foc->out.duty_b) ||
        !drive_pwm_duty_is_valid(foc->out.duty_c))
    {
        if (target_pwm_stop_phase_outputs())
        {
            foc->pwm_active = false;
        }
        return false;
    }

    switch (foc->motor.phase_order)
    {
        case PHASE_ORDER_ABC:
            target_pwm_set_duty_ratios(foc->out.duty_a,
                                       foc->out.duty_b,
                                       foc->out.duty_c);
            break;

        case PHASE_ORDER_ACB:
            target_pwm_set_duty_ratios(foc->out.duty_a,
                                       foc->out.duty_c,
                                       foc->out.duty_b);
            break;

        default:
            /* 无法确定物理相序时立即撤销功率输出，不能继续沿用上一拍占空比。 */
            if (target_pwm_stop_phase_outputs())
            {
                foc->pwm_active = false;
            }
            return false;
    }

    foc->pwm_cmd.valid = true;

    foc->pwm_commit = (drive_pwm_commit_t){
        .seq = foc->fast_seq,
        .valid = true,
        .duty_a = foc->out.duty_a,
        .duty_b = foc->out.duty_b,
        .duty_c = foc->out.duty_c
    };

    return true;
}
