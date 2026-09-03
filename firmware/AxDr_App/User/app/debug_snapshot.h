/**
 * @file debug_snapshot.h
 * @brief 快速控制周期的只读调试快照。
 */

#ifndef AXDR_DEBUG_SNAPSHOT_H
#define AXDR_DEBUG_SNAPSHOT_H

#include <stdint.h>

#include "foc_fwd.h"

/**
 * @brief 调试器观察用的单周期关键量镜像。
 *
 * 字段使用固定宽度整数和 float，便于调试器按稳定布局观察。该对象不参与控制，
 * duty_a/b/c 是 FOC 算出的候选占空比，不代表定时器当前实际输出。
 */
typedef struct
{
    /* 快速周期序号、Drive 请求/状态和模式枚举值。 */
    uint32_t seq;
    uint32_t req;
    uint32_t state;
    uint32_t pwm_on;   /* Drive 的软件输出状态，不是定时器寄存器回读。 */
    uint32_t fault;    /* foc_fault_t 的原始故障位图。 */
    uint32_t sys_mode;
    uint32_t op_mode;

    /* 本周期 PWM 命令和最近一次 Target 寄存器提交的关联序号。 */
    uint32_t pwm_cmd_seq;
    uint32_t pwm_commit_seq;
    uint32_t pwm_cmd_valid;
    uint32_t pwm_committed;

    /* 诊断任务、状态、最近结果和本拍电压限幅标记。 */
    uint32_t diag_req;
    uint32_t diag_job;
    uint32_t diag_state;
    uint32_t diag_status;
    uint32_t diag_active;
    uint32_t diag_v_sat;

    /* 在线磁链观测器状态；只用于验证电机模型，不参与位置闭环。 */
    uint32_t obs_status;
    uint32_t obs_samples;

    /* 母线电压单位 V；三相电流单位 A；电角度单位 rad。 */
    float vbus;
    float ia;
    float ib;
    float ic;
    float theta_e;

    /* 转子侧位置单位 rad，转子侧角速度单位 rad/s。 */
    float pos_r_ref;
    float pos_r_fbk;
    float spd_r_ref;
    float spd_r_fbk;

    /* dq 轴电流单位 A；iq_ref 是原始目标，iq_lim 是级联环实际限幅结果。 */
    float id_ref;
    float id_fbk;
    float iq_ref;
    float iq_lim;
    float iq_fbk;

    /* dq 轴电压指令单位 V；三相 duty 是范围通常为 [0, 1] 的候选比例。 */
    float vd;
    float vq;
    float diag_id_ref;
    float diag_iq_ref;
    float diag_vd_ref;
    float diag_vq_ref;
    float diag_freq;
    float flux_wb;
    float duty_a;
    float duty_b;
    float duty_c;

    /* Drive 本周期命令值，以及 Target 最近一次完成寄存器写入的逻辑三相值。 */
    float duty_cmd_a;
    float duty_cmd_b;
    float duty_cmd_c;
    float duty_commit_a;
    float duty_commit_b;
    float duty_commit_c;
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
 * @param[in] foc 本周期已经完成采样、反馈更新和 Drive 执行的电机控制对象。
 * @pre foc 指向快速周期独占写入的有效对象；每个快速周期最多调用一次。
 *
 * 本函数只从 foc 复制标量，不修改 foc，不执行通信、动态分配或阻塞操作。seq 在
 * 最后写入，与本周期 pwm_cmd 和 pwm_commit 的 seq 使用同一快速周期编号。
 */
void debug_snapshot_publish(const foc_t *foc);

#endif /* AXDR_DEBUG_SNAPSHOT_H */
