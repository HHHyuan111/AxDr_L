/**
 * @file target_reset.h
 * @brief 复位原因锁存：上电早期读 RCC->CSR 一次并清标志，查询返回锁存副本。
 */

#ifndef TARGET_RESET_H
#define TARGET_RESET_H

#include <stdint.h>

/* 启动早期（HAL_Init 之后）调用一次：锁存复位标志并清除 RCC->CSR。 */
void target_reset_latch(void);

/* 返回锁存的复位标志（RCC_CSR_RESET_FLAGS 位定义），每次查询结果相同。 */
uint32_t target_reset_reason(void);

#endif /* TARGET_RESET_H */
