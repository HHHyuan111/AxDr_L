/**
 * @file control_filter.c
 * @brief 与硬件无关的一阶低通滤波器实现。
 */

#include "control_filter.h"

#include "compiler.h"

static const float control_filter_two_pi = 6.28318530716f;

void control_lpf_init(lpf_t *filter)
{
    const float angular_cutoff_rad_s = control_filter_two_pi * filter->fc;

    filter->filt_a = filter->fs / (filter->fs + angular_cutoff_rad_s);
    filter->filt_b = 1.0f - filter->filt_a;
    filter->val = 0.0f;
    filter->val_f = 0.0f;
}

PLATFORM_FAST_CODE float control_lpf_step(lpf_t *filter, float value)
{
    filter->val = value;
    filter->val_f = filter->filt_b * filter->val +
                    filter->filt_a * filter->val_f;
    return filter->val_f;
}
