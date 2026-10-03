/**
 * @file target_time.h
 * @brief 微秒时基：基于内核 DWT->CYCCNT 周期计数器（ARM CMSIS 官方外设）。
 * @note  方案对准：ODrive timestamp / iFOC DelayUs 同款。零定时器占用、零中断。
 *        断点暂停时计数器同停——测得的耗时不含断点停顿，利于性能测量。
 */

#ifndef TARGET_TIME_H
#define TARGET_TIME_H

#include <stdint.h>

/** @brief 使能 DWT 周期计数器。上电调用一次。 */
void target_time_init(void);

/** @brief 当前微秒数。约 71 分钟回绕；调用方按无符号差值使用，勿存绝对值。 */
uint32_t target_time_us(void);

/** @brief 忙等指定微秒（仅用于磁编读取等极短延时，禁止在等待外设时使用）。 */
void target_time_delay_us(uint32_t us);

#endif /* TARGET_TIME_H */
