/**
 * @file legacy_foc_core.c
 * @brief FOC 核心抽取前的固定计算顺序，仅供电脑端迁移对照。
 */

#include "legacy_control.h"

#include "foc_core.h"
#include "foc_svm.h"
#include "foc_transform.h"

void legacy_foc_core_prepare(const foc_sample_t *sample, foc_frame_t *frame)
{
    foc_clarke(sample->i_a,
               sample->i_b,
               sample->i_c,
               &frame->i_alpha,
               &frame->i_beta);

    frame->theta = sample->theta;
    wrap_0_2pi(frame->theta);
    foc_sin_cos(frame->theta, &frame->sin_theta, &frame->cos_theta);
    foc_park(frame->i_alpha,
             frame->i_beta,
             frame->sin_theta,
             frame->cos_theta,
             &frame->i_d,
             &frame->i_q);
}

bool legacy_foc_core_modulate(const foc_frame_t *frame,
                              const foc_voltage_t *voltage,
                              foc_duty_t *duty)
{
    foc_inv_park(voltage->v_d,
                 voltage->v_q,
                 frame->sin_theta,
                 frame->cos_theta,
                 &duty->v_alpha,
                 &duty->v_beta);

    return foc_svm(duty->v_alpha * voltage->inv_vbus,
                   duty->v_beta * voltage->inv_vbus,
                   &duty->duty_a,
                   &duty->duty_b,
                   &duty->duty_c) == 0;
}
