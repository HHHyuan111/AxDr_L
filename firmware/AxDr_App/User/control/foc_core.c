/**
 * @file foc_core.c
 * @brief 与硬件和电机大对象无关的 FOC 核心数据链实现。
 */

#include "foc_core.h"

#include "foc_svm.h"
#include "foc_transform.h"

#define FOC_CORE_RAM_FUNC __attribute__((section(".RamFunc")))

static const float foc_core_two_pi = 6.28318530716f;

static FOC_CORE_RAM_FUNC float foc_core_wrap_0_2pi(float theta)
{
    theta = (theta > foc_core_two_pi) ? theta - foc_core_two_pi : theta;
    theta = (theta < 0.0f) ? theta + foc_core_two_pi : theta;

    return theta;
}

FOC_CORE_RAM_FUNC void foc_core_prepare(const foc_sample_t *sample,
                                        foc_frame_t *frame)
{
    foc_clarke(sample->i_a,
               sample->i_b,
               sample->i_c,
               &frame->i_alpha,
               &frame->i_beta);

    frame->theta = foc_core_wrap_0_2pi(sample->theta);
    foc_sin_cos(frame->theta, &frame->sin_theta, &frame->cos_theta);
    foc_park(frame->i_alpha,
             frame->i_beta,
             frame->sin_theta,
             frame->cos_theta,
             &frame->i_d,
             &frame->i_q);
}

FOC_CORE_RAM_FUNC bool foc_core_modulate(const foc_frame_t *frame,
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
