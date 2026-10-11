/**
 * @file foc_core.c
 * @brief 与硬件和电机大对象无关的 FOC 核心数据链实现。
 */

#include "foc_core.h"

#include "compiler.h"
#include "foc_svm.h"
#include "foc_transform.h"

#include <math.h>

static const float foc_core_two_pi = 6.28318530716f;

static PLATFORM_FAST_CODE float foc_core_wrap_0_2pi(float theta)
{
    theta = (theta > foc_core_two_pi) ? theta - foc_core_two_pi : theta;
    theta = (theta < 0.0f) ? theta + foc_core_two_pi : theta;

    return theta;
}

static PLATFORM_FAST_CODE float foc_core_clamp_duty_delta(float value,
                                                           float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static PLATFORM_FAST_CODE float foc_core_deadtime_delta(float current_a,
                                                         const foc_deadtime_config_t *config)
{
    const float blend = (config->current_blend_a > 1.0e-6f)
                            ? config->current_blend_a
                            : 1.0e-6f;
    const float ratio = config->deadtime_s / config->pwm_period_s;
    const float direction = current_a / (fabsf(current_a) + blend);
    return foc_core_clamp_duty_delta(config->polarity * direction * ratio,
                                     config->max_duty_delta);
}

static PLATFORM_FAST_CODE void foc_core_apply_deadtime(
    const foc_frame_t *frame,
    const foc_deadtime_config_t *config,
    foc_duty_t *duty)
{
    const float sqrt3_by_2 = 0.8660254037844386f;
    const float ia = frame->ialpha;
    const float ib = -0.5f * frame->ialpha + sqrt3_by_2 * frame->ibeta;
    const float ic = -0.5f * frame->ialpha - sqrt3_by_2 * frame->ibeta;
    const float delta_a = foc_core_deadtime_delta(ia, config);
    const float delta_b = foc_core_deadtime_delta(ib, config);
    const float delta_c = foc_core_deadtime_delta(ic, config);

    duty->duty_a += delta_a;
    duty->duty_b += delta_b;
    duty->duty_c += delta_c;
    duty->duty_a = (duty->duty_a < 0.0f) ? 0.0f :
                   (duty->duty_a > 1.0f) ? 1.0f : duty->duty_a;
    duty->duty_b = (duty->duty_b < 0.0f) ? 0.0f :
                   (duty->duty_b > 1.0f) ? 1.0f : duty->duty_b;
    duty->duty_c = (duty->duty_c < 0.0f) ? 0.0f :
                   (duty->duty_c > 1.0f) ? 1.0f : duty->duty_c;
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

    if (foc_svm(duty->valpha * voltage->inv_vbus,
                duty->vbeta * voltage->inv_vbus,
                &duty->duty_a,
                &duty->duty_b,
                &duty->duty_c) != 0) {
        return false;
    }

    if (voltage->deadtime.enabled &&
        isfinite(voltage->deadtime.deadtime_s) &&
        isfinite(voltage->deadtime.pwm_period_s) &&
        isfinite(voltage->deadtime.current_blend_a) &&
        isfinite(voltage->deadtime.polarity) &&
        isfinite(voltage->deadtime.max_duty_delta) &&
        voltage->deadtime.deadtime_s >= 0.0f &&
        voltage->deadtime.pwm_period_s > 0.0f &&
        voltage->deadtime.deadtime_s < voltage->deadtime.pwm_period_s &&
        voltage->deadtime.current_blend_a > 0.0f &&
        fabsf(fabsf(voltage->deadtime.polarity) - 1.0f) < 1.0e-6f &&
        voltage->deadtime.max_duty_delta > 0.0f) {
        foc_core_apply_deadtime(frame, &voltage->deadtime, duty);
    }

    return true;
}
