/**
 * @file drive_io.h
 * @brief Drive PWM 命令与最近一次 Target 提交记录的数据合同。
 */

#ifndef AXDR_DRIVE_IO_H
#define AXDR_DRIVE_IO_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 本控制周期反馈数据的有效状态。
 */
typedef struct
{
    bool i_valid;
    bool vbus_valid;
    bool pos_valid;
} drive_feedback_status_t;

/**
 * @brief Drive 在一个快速周期内生成的逻辑 A、B、C 相 PWM 命令。
 *
 * cmd 是 command 的常用缩写，表示算法希望 Target 写入的占空比。
 */
typedef struct
{
    uint32_t seq;
    bool valid;
    float duty_a;
    float duty_b;
    float duty_c;
} drive_pwm_cmd_t;

/**
 * @brief 最近一次已经写入 Target PWM 比较寄存器的逻辑三相记录。
 *
 * seq 表示完成寄存器写入的快速周期，不宣称该占空比已在引脚生效；
 * 引脚生效拍数仍需示波器确认。
 */
typedef struct
{
    uint32_t seq;
    bool valid;
    float duty_a;
    float duty_b;
    float duty_c;
} drive_pwm_commit_t;

#endif /* AXDR_DRIVE_IO_H */
