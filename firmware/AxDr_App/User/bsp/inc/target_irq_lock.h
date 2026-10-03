/**
 * @file target_irq_lock.h
 * @brief 短临界区：保存/恢复 PRIMASK 的就地关中断。
 * @note  保存恢复式（Zephyr irq_lock 同模式）杜绝"嵌套临界区退出时误开中断"。
 *        规则（15 号 R1）：临界区内只允许几条赋值，禁止任何等待与长循环。
 */

#ifndef TARGET_IRQ_LOCK_H
#define TARGET_IRQ_LOCK_H

#include <stdint.h>

#include "main.h"

typedef uint32_t irq_lock_key_t;

static inline irq_lock_key_t irq_lock(void)
{
    irq_lock_key_t key = __get_PRIMASK();
    __disable_irq();
    return key;
}

static inline void irq_unlock(irq_lock_key_t key)
{
    __set_PRIMASK(key);
}

#endif /* TARGET_IRQ_LOCK_H */
