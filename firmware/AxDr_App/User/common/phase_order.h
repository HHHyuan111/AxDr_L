/**
 * @file phase_order.h
 * @brief 电机逻辑相序的公共定义。
 */

#ifndef PHASE_ORDER_H
#define PHASE_ORDER_H

/** @brief 当前硬件支持的电机逻辑相序；ACB 表示交换 B、C 两相。 */
typedef enum
{
    PHASE_ORDER_ABC = 0,
    PHASE_ORDER_ACB = 1,
} phase_order_t;

#endif /* PHASE_ORDER_H */
