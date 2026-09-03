/**
 * @file control_cycle.h
 * @brief 单个快速控制周期的输入、输出和执行接口。
 */

#ifndef CONTROL_CYCLE_H
#define CONTROL_CYCLE_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_fwd.h"

/**
 * @brief 一个控制周期使用的物理反馈输入。
 *
 * 电流单位 A，电压单位 V，角度和位置单位 rad。Drive 模式、给定值、控制参数和
 * 算法历史状态属于 foc 上下文，不在每个周期重复复制。
 */
typedef struct
{
    uint32_t seq;
    bool i_valid;
    bool vbus_valid;
    bool pos_valid;
    float current_a_a;
    float current_b_a;
    float current_c_a;
    float bus_voltage_v;
    float electrical_angle_rad;
    float rotor_position_rad;
    float output_position_rad;
} control_cycle_input_t;

/**
 * @brief 一个控制周期产生的 Drive 和逻辑三相 PWM 输出。
 *
 * duty_valid 为 false 时，本周期没有产生新的占空比命令，duty_a/b/c 不应被消费。
 * pwm_enabled 是 Drive 软件状态，不等同于功率引脚的硬件回读。
 */
typedef struct
{
    uint32_t seq;
    uint32_t drive_state;
    uint32_t fault_bits;
    bool pwm_enabled;
    bool duty_valid;
    float duty_a;
    float duty_b;
    float duty_c;
} control_cycle_output_t;

/**
 * @brief 使用显式反馈输入执行一次反馈更新和 Drive 控制。
 *
 * @param[in,out] foc 已初始化的控制器上下文和历史状态，本周期独占写入。
 * @param[in] input 本周期物理反馈，所有浮点输入应为有限值，母线电压应不小于 0 V。
 * @param[out] output 本周期 Drive 状态和逻辑 PWM 命令。
 * @pre 三个指针有效且对象互不重叠。
 *
 * 本函数不采集硬件，也不等待外设。真实板卡、离线回放和 HIL 可以用不同方式生成
 * input，但共用相同的控制器上下文和本周期执行入口。
 */
void control_cycle_step(foc_t *foc,
                        const control_cycle_input_t *input,
                        control_cycle_output_t *output);

#endif /* CONTROL_CYCLE_H */
