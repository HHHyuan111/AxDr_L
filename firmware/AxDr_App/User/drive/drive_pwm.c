/**
 * @file drive_pwm.c
 * @brief Drive 层三相 PWM 执行适配。
 *
 * 数据流：逻辑 A/B/C 相占空比 -> 相序映射 -> Target 物理通道。
 */

#include "drive_pwm.h"

#include "target_pwm.h"

_RAM_FUNC void drive_pwm_start(void)
{
    target_pwm_start_phase_outputs();
}

_RAM_FUNC void drive_pwm_stop(void)
{
    target_pwm_stop_phase_outputs();
}

_RAM_FUNC void drive_pwm_set_neutral(pmsm_t *pm)
{
    pm->pwm_cmd = (drive_pwm_cmd_t){
        .seq = pm->fast_seq,
        .valid = true,
        .duty_a = 0.5f,
        .duty_b = 0.5f,
        .duty_c = 0.5f
    };

    target_pwm_set_duty_ratios(0.5f, 0.5f, 0.5f);

    pm->pwm_commit = (drive_pwm_commit_t){
        .seq = pm->fast_seq,
        .valid = true,
        .duty_a = 0.5f,
        .duty_b = 0.5f,
        .duty_c = 0.5f
    };
}

_RAM_FUNC void drive_pwm_commit(pmsm_t *pm)
{
    pm->pwm_cmd = (drive_pwm_cmd_t){
        .seq = pm->fast_seq,
        .valid = true,
        .duty_a = pm->foc.dtc_a,
        .duty_b = pm->foc.dtc_b,
        .duty_c = pm->foc.dtc_c
    };

    switch (pm->para.phase_order)
    {
        case ABC_PHASE:
            target_pwm_set_duty_ratios(pm->foc.dtc_a,
                                       pm->foc.dtc_b,
                                       pm->foc.dtc_c);
            break;

        case ACB_PHASE:
            target_pwm_set_duty_ratios(pm->foc.dtc_a,
                                       pm->foc.dtc_c,
                                       pm->foc.dtc_b);
            break;

        default:
            /* 相序无效时不写物理通道，也不伪造提交记录。 */
            return;
    }

    pm->pwm_commit = (drive_pwm_commit_t){
        .seq = pm->fast_seq,
        .valid = true,
        .duty_a = pm->foc.dtc_a,
        .duty_b = pm->foc.dtc_b,
        .duty_c = pm->foc.dtc_c
    };
}
