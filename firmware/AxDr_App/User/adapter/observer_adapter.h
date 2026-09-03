/**
 * @file observer_adapter.h
 * @brief FOC 实时量到可移植观测算法的适配接口。
 */

#ifndef OBSERVER_ADAPTER_H
#define OBSERVER_ADAPTER_H

#include "foc_fwd.h"
#include "mc_flux_observer.h"

/**
 * @brief 在线观测器运行对象。
 *
 * 该对象与 foc_t 分开，方便调试器直接观察，也避免观测算法反向依赖 Drive 状态。
 * 当前只接入磁链观测，不参与转子角闭环。
 */
typedef struct
{
    mc_flux_config_t cfg;
    mc_flux_observer_t flux;
    mc_status_t status;
} obs_t;

extern obs_t g_obs;

/** @brief 使用当前电机参数初始化在线磁链观测器。 */
void obs_init(const foc_t *foc);

/** @brief 将本周期 FOC 物理量送入磁链观测器，不修改控制输出。 */
void obs_step(const foc_t *foc);

#endif /* OBSERVER_ADAPTER_H */
