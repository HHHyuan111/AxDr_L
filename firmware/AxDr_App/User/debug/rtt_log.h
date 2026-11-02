/**
 * @file rtt_log.h
 * @brief RTT 日志薄封装。只允许 MAIN 上下文调用；RT 上下文禁止输出。
 */

#ifndef RTT_LOG_H
#define RTT_LOG_H

#include <stdint.h>

/* 上电调用一次，打印横幅。 */
void rtt_log_init(void);

/* 输出一行文本，末尾自动换行。 */
void rtt_log_str(const char *s);

/* 输出 "name: 值" 一行（十进制整数）。 */
void rtt_log_u32(const char *name, uint32_t value);

/* 输出 "name: 值" 一行（定点 3 位小数，|value| < 2.1e6）。 */
void rtt_log_f32(const char *name, float value);

#endif /* RTT_LOG_H */
