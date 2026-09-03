/**
 * @file legacy_control_loop.c
 * @brief FOC 模式主链迁移前的固定执行顺序，仅供 Host 对照。
 */

#include "legacy_control.h"

bool legacy_foc_ctrl_step(foc_ctrl_t *ctrl,
                          const foc_fb_t *fb,
                          const foc_ref_t *ref,
                          foc_out_t *out)
{
    foc_voltage_t voltage;

    *out = (foc_out_t){0};
    legacy_foc_core_prepare(&fb->sample, &out->frame);

    switch (ref->mode)
    {
        case FOC_CTRL_MODE_VOLT:
            ctrl->vd = ref->vd;
            ctrl->vq = ref->vq;
            break;

        case FOC_CTRL_MODE_CUR:
            (void)legacy_control_cur_step(&ctrl->cur_rate,
                                          ctrl->id_pi,
                                          ctrl->iq_pi,
                                          ref->id_ref,
                                          ref->iq_ref,
                                          out->frame.id,
                                          out->frame.iq,
                                          &ctrl->vd,
                                          &ctrl->vq);
            break;

        case FOC_CTRL_MODE_SPD:
            (void)legacy_control_spd_step(&ctrl->spd_rate,
                                          ctrl->spd_pi,
                                          ref->spd_ref,
                                          fb->spd,
                                          ref->cur_lim,
                                          &ctrl->iq_ref);
            (void)legacy_control_cur_step(&ctrl->cur_rate,
                                          ctrl->id_pi,
                                          ctrl->iq_pi,
                                          ref->id_ref,
                                          ctrl->iq_ref,
                                          out->frame.id,
                                          out->frame.iq,
                                          &ctrl->vd,
                                          &ctrl->vq);
            break;

        case FOC_CTRL_MODE_POS:
            (void)legacy_control_pos_step(&ctrl->pos_rate,
                                          ctrl->pos_pi,
                                          ref->pos_ref,
                                          fb->pos,
                                          ref->spd_lim,
                                          &ctrl->spd_ref);
            (void)legacy_control_spd_step(&ctrl->spd_rate,
                                          ctrl->spd_pi,
                                          ctrl->spd_ref,
                                          fb->spd,
                                          ref->cur_lim,
                                          &ctrl->iq_ref);
            (void)legacy_control_cur_step(&ctrl->cur_rate,
                                          ctrl->id_pi,
                                          ctrl->iq_pi,
                                          ref->id_ref,
                                          ctrl->iq_ref,
                                          out->frame.id,
                                          out->frame.iq,
                                          &ctrl->vd,
                                          &ctrl->vq);
            break;

        default:
            return false;
    }

    voltage = (foc_voltage_t){
        .vd = ctrl->vd,
        .vq = ctrl->vq,
        .inv_vbus = fb->inv_vbus,
    };

    out->vd = ctrl->vd;
    out->vq = ctrl->vq;
    out->iq_ref = ctrl->iq_ref;
    out->spd_ref = ctrl->spd_ref;
    out->valid = legacy_foc_core_modulate(&out->frame,
                                          &voltage,
                                          &out->pwm);

    return out->valid;
}
