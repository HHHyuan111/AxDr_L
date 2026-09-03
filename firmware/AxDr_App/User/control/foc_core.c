/**
 * @file foc_core.c
 * @brief 与硬件和电机大对象无关的 FOC 核心数据链实现。
 */

#include "foc_core.h"

#include "compiler.h"
#include "foc_svm.h"
#include "foc_transform.h"

static const float foc_core_two_pi = 6.28318530716f;

static PLATFORM_FAST_CODE float foc_core_wrap_0_2pi(float theta)
{
    theta = (theta > foc_core_two_pi) ? theta - foc_core_two_pi : theta;
    theta = (theta < 0.0f) ? theta + foc_core_two_pi : theta;

    return theta;
}

PLATFORM_FAST_CODE void foc_core_prepare(const foc_sample_t *sample,
                                        foc_frame_t *frame)
{
    foc_clarke(sample->ia,
               sample->ib,
               sample->ic,
               &frame->ialpha,
               &frame->ibeta);

    frame->theta = foc_core_wrap_0_2pi(sample->theta);
    foc_sin_cos(frame->theta, &frame->sin, &frame->cos);
    foc_park(frame->ialpha,
             frame->ibeta,
             frame->sin,
             frame->cos,
             &frame->id,
             &frame->iq);
}

PLATFORM_FAST_CODE bool foc_core_modulate(const foc_frame_t *frame,
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
