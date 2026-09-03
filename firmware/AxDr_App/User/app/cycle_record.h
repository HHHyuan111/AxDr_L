/**
 * @file cycle_record.h
 * @brief 快速控制周期的轻量环形记录器。
 *
 * 记录器默认关闭，不参与控制判断。调试时打开后，它保存最近 32 个周期的关键量，
 * 可直接从调试器读取，供波形检查和离线回放使用。
 */

#ifndef CYCLE_RECORD_H
#define CYCLE_RECORD_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_fwd.h"

#define CYCLE_RECORD_CAPACITY (32U)

/** @brief 单个控制周期的关键输入、状态和输出。 */
typedef struct
{
    uint32_t seq;
    uint32_t state;
    uint32_t sys_mode;
    uint32_t op_mode;
    uint32_t fault;
    float vbus;
    float ia;
    float ib;
    float ic;
    float theta;
    float pos;
    float spd;
    float id;
    float iq;
    float vd;
    float vq;
    float duty_a;
    float duty_b;
    float duty_c;
} cycle_sample_t;

/** @brief 调试器可见的固定长度环形缓冲区。 */
typedef struct
{
    bool enabled;
    uint32_t head;
    uint32_t count;
    cycle_sample_t sample[CYCLE_RECORD_CAPACITY];
} cycle_record_t;

extern volatile cycle_record_t g_cycle_record;

/** @brief 清空已有样本，但不改变启停状态。 */
void cycle_record_reset(void);

/** @brief 打开或关闭周期记录；打开时从空缓冲区重新开始。 */
void cycle_record_enable(bool enable);

/** @brief 在本次快速周期结束后保存一条样本。 */
void cycle_record_publish(const foc_t *foc);

#endif /* CYCLE_RECORD_H */
