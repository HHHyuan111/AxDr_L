/**
 * @file control_pid.h
 * @brief 现有 PID/PDFF 控制器的可移植上下文和数值接口。
 */

#ifndef AXDR_CONTROL_PID_H
#define AXDR_CONTROL_PID_H

/**
 * @brief PID 兼容上下文。
 *
 * 本结构保持旧工程字段、顺序和 volatile 属性不变。它暂时同时保存配置、运行历史
 * 和诊断值，用于先完成无 HAL 的等价迁移；配置与状态将在后续独立节点再评估拆分。
 */
typedef struct
{
    /* 基本 PID 增益。 */
    volatile float kp;
    volatile float ki;
    volatile float kd;

    /* PDFF 的目标权重和反馈阻尼系数。 */
    volatile float kfp;
    volatile float kf_damp;

    /* 本周期各控制项，单位由具体控制环决定。 */
    volatile float p_term;
    volatile float i_term;
    volatile float d_term;

    /* 积分项允许范围。 */
    volatile float i_term_max;
    volatile float i_term_min;

    /* 控制器离散执行周期，单位为秒。 */
    volatile float ts;

    /* 目标、反馈和误差，三者使用相同物理单位。 */
    volatile float ref_value;
    volatile float fback_value;

    volatile float error;
    volatile float pre_err;

    /* 控制器最终输出范围和本周期输出。 */
    volatile float out_min;
    volatile float out_max;

    volatile float out_value;
} pid_para_t;

/**
 * @brief 设置积分项和控制器输出的上下限。
 *
 * @param[in,out] pid 已初始化的 PID 上下文。
 * @param[in] i_term_max 积分项上限。
 * @param[in] i_term_min 积分项下限。
 * @param[in] out_max 控制器输出上限。
 * @param[in] out_min 控制器输出下限。
 * @pre pid 指向当前调用者独占写入的有效上下文，各上限不小于对应下限。
 */
void control_pid_set_limits(pid_para_t *pid,
                            float i_term_max,
                            float i_term_min,
                            float out_max,
                            float out_min);

/**
 * @brief 清空控制器运行历史，保留增益、周期和限值。
 *
 * @param[in,out] pid 已初始化的 PID 上下文。
 * @pre pid 指向当前调用者独占写入的有效上下文。
 */
void control_pid_clear(pid_para_t *pid);

/**
 * @brief 执行一次并联 PID 计算。
 *
 * @param[in,out] pid PID 上下文。
 * @param[in] ref_value 本周期目标值。
 * @param[in] feedback_value 本周期反馈值。
 * @return 限幅后的控制器输出。
 * @pre pid 指向当前调用者独占写入的有效上下文，周期和限值已经设置。
 */
float control_pid_parallel_step(pid_para_t *pid,
                                float ref_value,
                                float feedback_value);

/**
 * @brief 执行一次串联 PID 计算。
 *
 * @param[in,out] pid PID 上下文。
 * @param[in] ref_value 本周期目标值。
 * @param[in] feedback_value 本周期反馈值。
 * @return 限幅后的控制器输出。
 * @pre pid 指向当前调用者独占写入的有效上下文，周期和限值已经设置。
 */
float control_pid_serial_step(pid_para_t *pid,
                              float ref_value,
                              float feedback_value);

/**
 * @brief 执行一次 PDFF 计算。
 *
 * @param[in,out] pid PDFF 兼容上下文。
 * @param[in] ref_value 本周期目标值。
 * @param[in] feedback_value 本周期反馈值。
 * @return 限幅后的控制器输出。
 * @pre pid 指向当前调用者独占写入的有效上下文，周期和限值已经设置。
 */
float control_pid_pdff_step(pid_para_t *pid,
                            float ref_value,
                            float feedback_value);

/**
 * @brief PDFF 条件积分步进（B 库变体）：积分增量会把试探输出进一步推过
 *        限幅时放弃该增量。abs_output_limit 为绝对限幅，非正时退化为普通 PDFF。
 */
float control_pid_pdff_conditional_step(pid_para_t *pid,
                                         float ref_value,
                                         float feedback_value,
                                         float abs_output_limit);

/**
 * @brief PDFF 按指令限幅步进（B 库编码器反馈速度环实际变体）：临时收紧 pid
 *        限幅到 abs_output_limit 执行普通 PDFF 后恢复，积分抗饱和随限值收紧。
 *        非正或宽于内部限幅时等价于普通 PDFF。
 */
float control_pid_pdff_limited_step(pid_para_t *pid,
                                    float ref_value,
                                    float feedback_value,
                                    float abs_output_limit);

#endif /* AXDR_CONTROL_PID_H */
