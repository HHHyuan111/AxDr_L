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

/**
 * @brief PDFF 条件积分步进（B 库 pdff_ctrl_conditional 原样移植）。
 *
 * 与 control_pid_pdff_step 的动态积分限幅抗饱和不同，本变体采用条件积分：
 * 仅当积分增量不会把（试探）输出进一步推过限幅时才采纳该增量。
 * abs_output_limit 为调用方给定的绝对输出限幅，与 pid 内部限幅取更严者；
 * 限幅非正时退化为普通 PDFF。B 库沉沙速度环即用此变体。
 */
PLATFORM_FAST_CODE float control_pid_pdff_conditional_step(pid_para_t *pid,
                                                            float ref_value,
                                                            float feedback_value,
                                                            float abs_output_limit)
{
    float limit;
    float increment;
    float trial;
    float output;

    if (pid == 0)
    {
        return 0.0f;
    }
    if (!(abs_output_limit > 0.0f))
    {
        return control_pid_pdff_step(pid, ref_value, feedback_value);
    }

    limit = abs_output_limit;
    if (limit > pid->out_max)
    {
        limit = pid->out_max;
    }
    if (-limit < pid->out_min)
    {
        limit = -pid->out_min;
    }

    pid->ref_value = ref_value;
    pid->fback_value = feedback_value;
    pid->error = ref_value - feedback_value;

    pid->p_term = pid->kp * (ref_value * pid->kfp - feedback_value * (1.0f + pid->kf_damp));
    pid->d_term = pid->kd * (pid->error - pid->pre_err);
    pid->pre_err = pid->error;

    increment = pid->ki * pid->error * pid->ts;
    trial = pid->i_term + increment;
    output = pid->p_term + trial + pid->d_term;

    /* 条件积分：积分只会推着输出更饱和时，放弃本拍增量 */
    if (!((output > limit && increment > 0.0f) || (output < -limit && increment < 0.0f)))
    {
        pid->i_term = trial;
    }

    pid->out_value = pid->p_term + pid->i_term + pid->d_term;
    if (pid->out_value > limit)
    {
        pid->out_value = limit;
    }
    if (pid->out_value < -limit)
    {
        pid->out_value = -limit;
    }
    return pid->out_value;
}
