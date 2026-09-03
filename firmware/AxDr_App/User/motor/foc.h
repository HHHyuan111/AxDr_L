/**
 * @file foc.h
 * @brief FOC 总对象和初始化入口。
 */

#ifndef FOC_H
#define FOC_H

#include "foc_fwd.h"

/** @brief 固件唯一的 FOC 总对象，由快速控制周期独占写入。 */
extern foc_t g_foc;

/**
 * @brief 初始化电机、板卡、编码器、控制器和 Drive 初始状态。
 *
 * @param[out] foc 待初始化的 FOC 总对象。
 * @pre foc 指向有效且可写的对象；Target ADC 采样链已经启动。
 */
void foc_init(foc_t *foc);

#endif /* FOC_H */
