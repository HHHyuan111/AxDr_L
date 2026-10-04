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
        /* oracle 修正（23 号审查）：历史代码此处缺 count 清零（divider>=2 时分频
         * 失效的休眠 bug），生产实现已修，对照基准同步修正。 */
        rate->count = 0U;
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
        /* oracle 升级（23 号审查）：对齐 B 库编码器反馈速度环真身——pdff_ctrl_limited
         * 语义（临时收紧 pid 限幅到指令限值再跑 pdff，恢复现场），替代 plain+后置钳位。
         * 证据：B 库 foc_drv.c:934 spd_curr_cl 主路径。 */
        rate->count = 0U;
        {
            float lim = LEGACY_ABS(iq_limit_abs);
            const float saved_max = speed_pid->out_max;
            const float saved_min = speed_pid->out_min;
            const float saved_imax = speed_pid->i_term_max;
            const float saved_imin = speed_pid->i_term_min;

            if (lim > 0.0f)
            {
                if (lim > saved_max)
                {
                    lim = saved_max;
                }
                if (-lim < saved_min)
                {
                    lim = -saved_min;
                }
                speed_pid->out_max = lim;
                speed_pid->out_min = -lim;
            }
            pdff_ctrl(speed_pid, speed_ref, speed_feedback);
            speed_pid->out_max = saved_max;
            speed_pid->out_min = saved_min;
            speed_pid->i_term_max = saved_imax;
            speed_pid->i_term_min = saved_imin;
        }
        *iq_ref = speed_pid->out_value;

        if (LEGACY_ABS(iq_limit_abs) > 0)
        {
            *iq_ref = sat1_datf(*iq_ref,
                                LEGACY_ABS(iq_limit_abs),
                                -LEGACY_ABS(iq_limit_abs));
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

        if (LEGACY_ABS(speed_limit_abs) > 0)
        {
            *speed_ref = sat1_datf(*speed_ref,
                                   LEGACY_ABS(speed_limit_abs),
                                   -LEGACY_ABS(speed_limit_abs));
        }

        return true;
    }

    return false;
}
