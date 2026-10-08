/**
 * @file service_scope.h
 * @brief 扫频测量会话协议层（0x1B-0x1F 号段）。
 *
 * 0x1B-0x1F 原示波器契约语义重定义为电流扫频测量会话（先例同
 * IDENTIFY 段 0x13/0x16），payload 结构见 axdr_scope_contract.h。
 * 本层是纯协议胶水：诊断任务本体在 diag_runtime/mc_current_sweep。
 */

#ifndef SERVICE_SCOPE_H
#define SERVICE_SCOPE_H

#include <stdint.h>

#include "axdr_command_core.h"

/** @brief USB 命令路由缝（service_usb 的 handler 指针位）。
 *
 * 0x1B-0x1F 由本层处理；其余原样转发 service_command_dispatch，
 * 并在 GET_PROTOCOL_INFO 应答上补扫频 capability/mask 位（拦截在
 * 组帧前，CRC 由 core 按改后 payload 重算）。
 * 用户 WIP（axdr_command_contract.h/service_command.c）合入后，
 * 路由与 capability 补位应迁回 service_command.c 统一分发。
 */
uint8_t service_scope_route(void *context, uint16_t opcode,
                            const uint8_t *request_payload,
                            uint16_t request_payload_length,
                            uint8_t *response_payload,
                            uint16_t response_payload_capacity,
                            uint16_t *response_payload_length,
                            uint8_t *ack_state);

#endif /* SERVICE_SCOPE_H */
