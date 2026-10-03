/**
 * @file control_limit.h
 * @brief 与硬件无关的控制量限幅接口。
 */

#ifndef AXDR_CONTROL_LIMIT_H
#define AXDR_CONTROL_LIMIT_H

/**
 * @brief 把输入值限制在指定的闭区间内。
 *
 * @param[in] value 待限制的控制量。
 * @param[in] upper 上限，与 value 使用相同单位。
 * @param[in] lower 下限，与 value 使用相同单位。
 * @return value 高于上限时返回 upper，低于下限时返回 lower，否则原样返回 value。
 * @pre lower 不大于 upper。
 *
 * 函数保持现有比较语义：NaN 会原样返回，正负无穷会被限制到对应边界。
 */
float control_limit(float value, float upper, float lower);

#endif /* AXDR_CONTROL_LIMIT_H */
