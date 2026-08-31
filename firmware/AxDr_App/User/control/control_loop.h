/**
 * @file control_loop.h
 * @brief 电压、电流、速度和位置 FOC 主链的可移植执行接口。
 */

#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <stdbool.h>

#include "control_cascade.h"
#include "foc_core.h"

typedef enum
{
    CONTROL_LOOP_MODE_VOLTAGE = 0,
    CONTROL_LOOP_MODE_CURRENT = 1,
    CONTROL_LOOP_MODE_SPEED = 2,
    CONTROL_LOOP_MODE_POSITION = 3,
} control_loop_mode_t;

/**
 * @brief FOC 三级控制环运行状态和控制器引用。
 *
 * PID 对象由调用者创建并独占写入。本结构保存分频状态和未到更新拍时需要沿用的
 * 电压、电流及速度参考，不持有硬件对象或电机大对象。
 */
typedef struct
{
    control_rate_t current_rate;
    control_rate_t speed_rate;
    control_rate_t position_rate;
    pid_para_t *current_d_pid;
    pid_para_t *current_q_pid;
    pid_para_t *speed_pid;
    pid_para_t *position_pid;
    float voltage_d_v;
    float voltage_q_v;
    float current_q_ref_a;
    float speed_ref_rad_s;
} control_loop_runtime_t;

/**
 * @brief 本周期控制环需要的反馈。
 */
typedef struct
{
    foc_sample_t foc_sample;
    float inv_bus_voltage;
    float rotor_speed_rad_s;
    float rotor_position_rad;
} control_loop_feedback_t;

/**
 * @brief 本周期控制模式和给定值。
 *
 * 只有 mode 对应的字段参与计算。电压单位 V，电流单位 A，速度单位 rad/s，
 * 位置单位 rad；两个 limit 字段表示绝对限制值。
 */
typedef struct
{
    control_loop_mode_t mode;
    float voltage_d_v;
    float voltage_q_v;
    float current_d_ref_a;
    float current_q_ref_a;
    float speed_ref_rad_s;
    float position_ref_rad;
    float current_limit_a;
    float speed_limit_rad_s;
} control_loop_request_t;

/**
 * @brief 本周期 FOC 中间量、级联参考和占空比结果。
 */
typedef struct
{
    foc_frame_t frame;
    foc_duty_t duty;
    float voltage_d_v;
    float voltage_q_v;
    float current_q_ref_a;
    float speed_ref_rad_s;
    bool duty_valid;
} control_loop_output_t;

/**
 * @brief 执行一次指定模式的完整 FOC 主链。
 *
 * @return 三相占空比均有效时返回 true；模式非法或占空比无效时返回 false。
 * @pre runtime、feedback、request、output 均为有效且互不重叠的对象；PID 指针有效。
 *
 * 本函数不访问 HAL、Target、全局电机对象，不分配内存，也不阻塞。
 */
bool control_loop_step(control_loop_runtime_t *runtime,
                       const control_loop_feedback_t *feedback,
                       const control_loop_request_t *request,
                       control_loop_output_t *output);

#endif /* CONTROL_LOOP_H */
