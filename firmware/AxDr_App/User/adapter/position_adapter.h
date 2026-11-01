/**
 * @file position_adapter.h
 * @brief 编码器角度换算为本拍控制使用的位置。
 */

#ifndef POSITION_ADAPTER_H
#define POSITION_ADAPTER_H

#include <stdbool.h>

#include "foc_fwd.h"

/** @brief 本拍位置换算结果，单位均为 rad。 */
typedef struct
{
    float theta_e; /* 电角度。 */
    float pos_r;   /* 转子多圈机械位置。 */
    float pos_m;   /* 输出轴多圈机械位置。 */
} position_sample_t;

/**
 * @brief 消费新编码器样本，更新跨圈历史，返回本拍位置。
 *
 * 不写入 fb.theta_e、fb.pos_r、fb.pos_m，由控制周期统一装入。
 * 无新样本或配置不支持时返回 false，保留历史和 sample 原值。
 * foc 与 sample 必须指向有效且互不重叠的对象。
 */
bool position_update(foc_t *foc, position_sample_t *sample);

#endif /* POSITION_ADAPTER_H */
