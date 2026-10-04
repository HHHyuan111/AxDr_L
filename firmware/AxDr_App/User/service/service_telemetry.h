/**
 * @file service_telemetry.h
 * @brief S4：AXDR 160B 遥测帧——快速周期抽取组帧 + 主循环取帧发送。
 *
 * B 库 axdr_telemetry 语义原样：capture 在快速上下文按 decimation 抽取组帧
 * （16 槽 seqlock 环，seq 连号 + 覆盖计数），poll 在主循环取最老连号帧、
 * 算 CRC 后经 tx 缝发送（BUSY 保留重试，错误丢弃计 drop）。
 * 传输对象（CDC）由 app/main 绑定，本模块零 USB 依赖。
 */

#ifndef SERVICE_TELEMETRY_H
#define SERVICE_TELEMETRY_H

#include <stdint.h>

#include "axdr_telemetry_contract.h"

#define SERVICE_TELEMETRY_DEFAULT_DECIMATION 1000u

typedef axdr_telemetry_v1_frame_t service_telemetry_frame_t;

/** @brief 发送缝返回值：0=已提交，1=忙（保留重试），其他=错误（丢弃）。 */
typedef int (*service_telemetry_tx_fn)(const uint8_t *data, uint16_t length);

/** @brief 复位序列号/计数/队列。 */
void service_telemetry_init(void);

/** @brief 设置抽取分频（0 恢复默认 1000；变更时清积压防止旧速率帧拥塞）。 */
void service_telemetry_set_decimation(uint32_t decimation);

/** @brief 快速周期调用（20kHz）：分频到点则从 g_foc 组帧入队。 */
void service_telemetry_capture(void);

/** @brief 主循环调用：取帧 → CRC → tx 发送。 */
void service_telemetry_poll(void);

/** @brief 绑定发送缝（未绑定时 poll 静默丢弃积压帧）。 */
void service_telemetry_bind_tx(service_telemetry_tx_fn tx);

/* 诊断计数（GetRuntimeState telemetry_* 字段的数据家） */
uint32_t service_telemetry_decimation(void);
uint32_t service_telemetry_capture_count(void);
uint32_t service_telemetry_sent_count(void);
uint32_t service_telemetry_drop_count(void);

#endif /* SERVICE_TELEMETRY_H */
