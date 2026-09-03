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
    foc_clarke(sample->ia,
               sample->ib,
               sample->ic,
               &frame->ialpha,
               &frame->ibeta);

    frame->theta = sample->theta;
    wrap_0_2pi(frame->theta);
    foc_sin_cos(frame->theta, &frame->sin, &frame->cos);
    foc_park(frame->ialpha,
             frame->ibeta,
             frame->sin,
             frame->cos,
             &frame->id,
             &frame->iq);
}

bool legacy_foc_core_modulate(const foc_frame_t *frame,
                              const foc_voltage_t *voltage,
                              foc_duty_t *duty)
{
    foc_inv_park(voltage->vd,
                 voltage->vq,
                 frame->sin,
                 frame->cos,
                 &duty->valpha,
                 &duty->vbeta);

    return foc_svm(duty->valpha * voltage->inv_vbus,
                   duty->vbeta * voltage->inv_vbus,
                   &duty->duty_a,
                   &duty->duty_b,
                   &duty->duty_c) == 0;
}
