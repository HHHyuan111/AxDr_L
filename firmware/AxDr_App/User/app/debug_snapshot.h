/**
 * @file debug_snapshot.h
 * @brief 快速控制周期的只读调试快照。
 */

#ifndef AXDR_DEBUG_SNAPSHOT_H
#define AXDR_DEBUG_SNAPSHOT_H

#include <stdint.h>

#include "common.h"

/**
 * @brief 调试器观察用的单周期关键量镜像。
 *
 * 字段使用固定宽度整数和 float，便于调试器按稳定布局观察。该对象不参与控制，
 * duty_a/b/c 是 FOC 算出的候选占空比，不代表定时器当前实际输出。
 */
typedef struct
{
    /* 发布序号、Drive 请求/状态和模式枚举值。 */
    uint32_t seq;
    uint32_t req;
    uint32_t state;
    uint32_t pwm_on;   /* Drive 的软件输出状态，不是定时器寄存器回读。 */
    uint32_t fault;    /* pmsm_fault_t 的原始故障位图。 */
    uint32_t sys_mode;
    uint32_t op_mode;

    /* 母线电压单位 V；三相电流单位 A；电角度单位 rad。 */
    float v_bus;
    float i_a;
    float i_b;
    float i_c;
    float theta_e;

    /* 转子侧位置单位 rad，转子侧角速度单位 rad/s。 */
    float pos_r_ref;
    float pos_r_fbk;
    float vel_r_ref;
    float vel_r_fbk;

    /* dq 轴电流单位 A；i_q_ref 是原始目标，i_q_lim 是级联环实际限幅结果。 */
    float i_d_ref;
    float i_d_fbk;
    float i_q_ref;
    float i_q_lim;
    float i_q_fbk;

    /* dq 轴电压指令单位 V；三相 duty 是范围通常为 [0, 1] 的候选比例。 */
    float v_d_cmd;
    float v_q_cmd;
    float duty_a;
    float duty_b;
    float duty_c;
} debug_snapshot_t;

/**
 * @brief 当前最新的快速周期调试快照。
 *
 * 生产代码只有 debug_snapshot_publish() 可以写本对象。现阶段消费者是停核后的
 * 调试器观察；运行中异步读取不保证所有字段来自完全相同的瞬间。
 */
extern volatile debug_snapshot_t g_debug_snapshot;

/**
 * @brief 在快速周期结束时发布一份关键控制量快照。
 *
 * @param[in] pm 本周期已经完成采样、反馈更新和 Drive 执行的电机控制对象。
 * @pre pm 指向快速周期独占写入的有效对象；每个快速周期最多调用一次。
 *
 * 本函数只从 pm 复制标量，不修改 pm，不执行通信、动态分配或阻塞操作。
 */
void debug_snapshot_publish(const pmsm_t *pm);

#endif /* AXDR_DEBUG_SNAPSHOT_H */
