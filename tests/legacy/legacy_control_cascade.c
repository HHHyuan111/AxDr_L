/**
 * @file legacy_control_cascade.c
 * @brief 级联控制抽取前的固定执行顺序，仅供电脑端迁移对照。
 */

#include "legacy_control.h"

#include "control_cascade.h"

bool legacy_control_cur_step(control_rate_t *rate,
                             pid_para_t *d_pid,
                             pid_para_t *q_pid,
                             float id_ref,
                             float iq_ref,
                             float id_feedback,
                             float iq_feedback,
                             float *v_d,
                             float *v_q)
{
    if (++rate->count >= rate->divider)
    {
        parallel_pid_ctrl(d_pid, id_ref, id_feedback);
        *v_d = d_pid->out_value;
        parallel_pid_ctrl(q_pid, iq_ref, iq_feedback);
        *v_q = q_pid->out_value;
        return true;
    }

    return false;
}

bool legacy_control_spd_step(control_rate_t *rate,
                             pid_para_t *speed_pid,
                             float speed_ref,
                             float speed_feedback,
                             float iq_limit_abs,
                             float *iq_ref)
{
    if (++rate->count >= rate->divider)
    {
        rate->count = 0U;
        pdff_ctrl(speed_pid, speed_ref, speed_feedback);
        *iq_ref = speed_pid->out_value;

        if (ABS(iq_limit_abs) > 0)
        {
            *iq_ref = sat1_datf(*iq_ref,
                                ABS(iq_limit_abs),
                                -ABS(iq_limit_abs));
        }

        return true;
    }

    return false;
}

bool legacy_control_pos_step(control_rate_t *rate,
                             pid_para_t *position_pid,
                             float position_ref,
                             float position_feedback,
                             float speed_limit_abs,
                             float *speed_ref)
{
    if (++rate->count >= rate->divider)
    {
        rate->count = 0U;
        parallel_pid_ctrl(position_pid, position_ref, position_feedback);
        *speed_ref = position_pid->out_value;

        if (ABS(speed_limit_abs) > 0)
        {
            *speed_ref = sat1_datf(*speed_ref,
                                   ABS(speed_limit_abs),
                                   -ABS(speed_limit_abs));
        }

        return true;
    }

    return false;
}
