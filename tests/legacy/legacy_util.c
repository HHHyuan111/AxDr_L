/**
 * @file legacy_util.c
 * @brief 阶段 2 数值回归测试使用的旧版基础控制公式。
 *
 * 本文件只由电脑端测试编译，用于确认迁移后的限幅、低通和角度差分测速
 * 与迁移前结果一致；不再进入 MCU 固件工程。
 */

#include "legacy_control.h"

float sat1_datf(float val, float up, float low)
{
	if (val > up)
		return up;
	else if (val < low)
		return low;
	else
		return val;
}

void low_pf_init(lpf_t *x)
{
    float wf = M_2PI * x->fc;

    x->filt_a = x->fs / (x->fs+wf);
    x->filt_b = 1.0f - x->filt_a;

    //
    x->val = 0.0f;
    x->val_f = 0.0f;
}

_RAM_FUNC float low_pf(lpf_t *x, float val)
{
    x->val = val;
    x->val_f = x->filt_b * x->val + x->filt_a * x->val_f;
    return x->val_f;
}

_RAM_FUNC float angle_speed_calc(float pos, float fs)
{
    static float d_pos;
    static float pos_last;
    float vel;

    d_pos = pos - pos_last;
    wrap_pm_pi(d_pos);

    vel = d_pos * fs;
    pos_last = pos;

    return vel;
}
