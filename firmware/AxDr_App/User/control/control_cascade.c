/**
 * @file control_cascade.c
 * @brief 电流、速度和位置级联控制的可移植实现。
 */

#include "control_cascade.h"

#include "control_limit.h"

#define CONTROL_CASCADE_RAM_FUNC __attribute__((section(".RamFunc")))

static CONTROL_CASCADE_RAM_FUNC float control_abs(float value)
{
    return (value > 0.0f) ? value : -value;
}

CONTROL_CASCADE_RAM_FUNC bool control_cur_step(control_rate_t *rate,
                                               pid_para_t *d_pid,
                                               pid_para_t *q_pid,
                                               float id_ref,
                                               float iq_ref,
                                               float id_feedback,
                                               float iq_feedback,
                                               float *v_d,
                                               float *v_q)
{
    if (++rate->count < rate->divider)
    {
        return false;
    }

    control_pid_parallel_step(d_pid, id_ref, id_feedback);
    *v_d = d_pid->out_value;
    control_pid_parallel_step(q_pid, iq_ref, iq_feedback);
    *v_q = q_pid->out_value;

    return true;
}

CONTROL_CASCADE_RAM_FUNC bool control_spd_step(control_rate_t *rate,
                                               pid_para_t *speed_pid,
                                               float speed_ref,
                                               float speed_feedback,
                                               float iq_limit_abs,
                                               float *iq_ref)
{
    float limit;

    if (++rate->count < rate->divider)
    {
        return false;
    }

    rate->count = 0U;
    control_pid_pdff_step(speed_pid, speed_ref, speed_feedback);
    *iq_ref = speed_pid->out_value;

    limit = control_abs(iq_limit_abs);
    if (limit > 0.0f)
    {
        *iq_ref = control_limit(*iq_ref, limit, -limit);
    }

    return true;
}

CONTROL_CASCADE_RAM_FUNC bool control_pos_step(control_rate_t *rate,
                                               pid_para_t *position_pid,
                                               float position_ref,
                                               float position_feedback,
                                               float speed_limit_abs,
                                               float *speed_ref)
{
    float limit;

    if (++rate->count < rate->divider)
    {
        return false;
    }

    rate->count = 0U;
    control_pid_parallel_step(position_pid, position_ref, position_feedback);
    *speed_ref = position_pid->out_value;

    limit = control_abs(speed_limit_abs);
    if (limit > 0.0f)
    {
        *speed_ref = control_limit(*speed_ref, limit, -limit);
    }

    return true;
}
