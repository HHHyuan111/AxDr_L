/**
 * @file control_mit.h
 * @brief MIT 位置、速度和前馈转矩合成的可移植接口。
 */

#ifndef CONTROL_MIT_H
#define CONTROL_MIT_H

typedef struct
{
    float position_ref_rad;
    float position_feedback_rad;
    float speed_ref_rad_s;
    float speed_feedback_rad_s;
    float torque_feedforward_nm;
    float position_gain_nm_per_rad;
    float speed_gain_nm_s_per_rad;
} control_mit_input_t;

typedef struct
{
    float torque_cmd_nm;
} control_mit_output_t;

/**
 * @brief 计算 MIT 模式的位置项、速度项和前馈转矩之和。
 *
 * @pre input 和 output 指向有效且互不重叠的对象，所有输入为有限值。
 */
void control_mit_step(const control_mit_input_t *input,
                      control_mit_output_t *output);

#endif /* CONTROL_MIT_H */
