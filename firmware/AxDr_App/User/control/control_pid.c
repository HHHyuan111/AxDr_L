/**
 * @file control_pid.c
 * @brief 现有并联 PID、串联 PID 和 PDFF 的可移植数值实现。
 */

#include "control_pid.h"

#include "compiler.h"

#define CONTROL_PID_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define CONTROL_PID_MAX(a, b) (((a) > (b)) ? (a) : (b))

void control_pid_set_limits(pid_para_t *pid,
                            float i_term_max,
                            float i_term_min,
                            float out_max,
                            float out_min)
{
    pid->i_term_max = i_term_max;
    pid->i_term_min = i_term_min;
    pid->out_max = out_max;
    pid->out_min = out_min;
}

void control_pid_clear(pid_para_t *pid)
{
    pid->ref_value = 0.0f;
    pid->fback_value = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->error = 0.0f;
    pid->pre_err = 0.0f;
    pid->out_value = 0.0f;
}

PLATFORM_FAST_CODE float control_pid_parallel_step(pid_para_t *pid,
                                                      float ref_value,
                                                      float feedback_value)
{
    pid->ref_value = ref_value;
    pid->fback_value = feedback_value;

    pid->error = pid->ref_value - pid->fback_value;
    pid->p_term = pid->kp * pid->error;
    pid->d_term = pid->kd * (pid->error - pid->pre_err);
    pid->pre_err = pid->error;

    pid->i_term_max = CONTROL_PID_MAX(pid->out_max - pid->p_term, 0.0f);
    pid->i_term_min = CONTROL_PID_MIN(pid->out_min - pid->p_term, 0.0f);
    pid->i_term += pid->ki * pid->error * pid->ts;

    if (pid->i_term > pid->i_term_max)
    {
        pid->i_term = pid->i_term_max;
    }
    else if (pid->i_term < pid->i_term_min)
    {
        pid->i_term = pid->i_term_min;
    }

    pid->out_value = pid->p_term + pid->i_term + pid->d_term;

    if (pid->out_value > pid->out_max)
    {
        pid->out_value = pid->out_max;
    }
    else if (pid->out_value < pid->out_min)
    {
        pid->out_value = pid->out_min;
    }

    return pid->out_value;
}

PLATFORM_FAST_CODE float control_pid_serial_step(pid_para_t *pid,
                                                    float ref_value,
                                                    float feedback_value)
{
    pid->ref_value = ref_value;
    pid->fback_value = feedback_value;

    pid->error = pid->ref_value - pid->fback_value;
    pid->p_term = pid->kp * pid->error;
    pid->i_term += pid->ki * pid->p_term * pid->ts;

    if (pid->i_term > pid->i_term_max)
    {
        pid->i_term = pid->i_term_max;
    }
    else if (pid->i_term < pid->i_term_min)
    {
        pid->i_term = pid->i_term_min;
    }

    pid->out_value = pid->p_term + pid->i_term;

    if (pid->out_value > pid->out_max)
    {
        pid->out_value = pid->out_max;
    }
    else if (pid->out_value < pid->out_min)
    {
        pid->out_value = pid->out_min;
    }

    return pid->out_value;
}

PLATFORM_FAST_CODE float control_pid_pdff_step(pid_para_t *pid,
                                                  float ref_value,
                                                  float feedback_value)
{
    float ref_temp;
    float feedback_temp;
    float error_kf;

    pid->ref_value = ref_value;
    pid->fback_value = feedback_value;

    ref_temp = pid->ref_value * pid->kfp;
    feedback_temp = pid->fback_value * (1.0f + pid->kf_damp);
    error_kf = ref_temp - feedback_temp;

    pid->error = pid->ref_value - pid->fback_value;
    pid->p_term = pid->kp * error_kf;

    pid->i_term_max = CONTROL_PID_MAX(pid->out_max - pid->p_term, 0.0f);
    pid->i_term_min = CONTROL_PID_MIN(pid->out_min - pid->p_term, 0.0f);
    pid->i_term += pid->ki * pid->error * pid->ts;

    if (pid->i_term > pid->i_term_max)
    {
        pid->i_term = pid->i_term_max;
    }
    else if (pid->i_term < pid->i_term_min)
    {
        pid->i_term = pid->i_term_min;
    }

    pid->d_term = pid->kd * (pid->error - pid->pre_err);
    pid->pre_err = pid->error;
    pid->out_value = pid->p_term + pid->i_term + pid->d_term;

    if (pid->out_value > pid->out_max)
    {
        pid->out_value = pid->out_max;
    }
    else if (pid->out_value < pid->out_min)
    {
        pid->out_value = pid->out_min;
    }

    return pid->out_value;
}
