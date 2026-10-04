/**
 * @file service_command.h
 * @brief S3：14 opcode 分发器 + 安全会话（对接 drive/motor 的桥）。
 * @note  依赖方向：service → drive/motor/config（向下）。协议编解码在
 *        service_usb/command_core；本文件只做"opcode → 框架动作"的翻译。
 */

#ifndef SERVICE_COMMAND_H
#define SERVICE_COMMAND_H

#include <stdint.h>

#include "axdr_command_contract.h"
#include "axdr_safety_runtime.h"

/* ---- 安全会话（单例，与 drive 共享生命周期） ---- */

/** @brief 每快速周期调用：观察互锁（fault/输出关断）+ 心跳租约超时检查。 */
void service_safety_tick(uint32_t now_ms);

/** @brief 主循环低频调用（now_ms 来自 target_time_us/1000）。 */
void service_safety_poll(uint32_t now_ms);

/** @brief 当前 run 是否被安全链授权（SetSpeed/SetCurrent 的门卫）。 */
uint8_t service_safety_run_authorized(void);

/** @brief 取安全运行时（测试/快照用）。 */
axdr_safety_runtime_t *service_safety_runtime(void);

/* ---- 分发器（service_usb 注入） ---- */

/**
 * @brief 命令分发入口（command_core 的 payload handler）。
 * @return reason 码（0=成功）；ack_state 写 AXDR_ACK_ACCEPTED/APPLIED/REJECTED。
 */
uint8_t service_command_dispatch(void *context, uint16_t opcode,
                                 const uint8_t *request_payload,
                                 uint16_t request_payload_length,
                                 uint8_t *response_payload,
                                 uint16_t response_payload_capacity,
                                 uint16_t *response_payload_length,
                                 uint8_t *ack_state);

#endif /* SERVICE_COMMAND_H */
