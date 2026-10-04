/**
 * @file test_service_core.c
 * @brief S1 协议核心移植冒烟：帧编解码往返、CRC 拒绝、重复 sequence 缓存、
 *        安全运行时档案确认→ARM→心跳→租约状态机。
 */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "axdr_command_core.h"
#include "axdr_command_contract.h"
#include "axdr_safety_runtime.h"

static int fail_count;
static void expect(const char *name, bool cond)
{
    if (!cond)
    {
        (void)printf("FAIL %s\n", name);
        fail_count++;
    }
}

/* 命令分发回调桩：记录 opcode 并返回 Accepted */
static uint16_t last_opcode;
static uint8_t stub_command(void *context, uint16_t opcode,
                            const uint8_t *request_payload,
                            uint16_t request_payload_length,
                            uint8_t *response_payload,
                            uint16_t response_payload_capacity,
                            uint16_t *response_payload_length,
                            uint8_t *ack_state)
{
    (void)context;
    (void)request_payload;
    (void)request_payload_length;
    (void)response_payload;
    (void)response_payload_capacity;
    last_opcode = opcode;
    *ack_state = AXDR_ACK_ACCEPTED;
    *response_payload_length = 0U;
    return AXDR_REASON_NONE; /* 处理成功（reason 语义，非布尔） */
}

/* 构造一条合法请求帧（16B 头 + 4B CRC） */
static uint16_t make_frame(uint8_t *buf, uint32_t sequence, uint16_t opcode)
{
    const uint32_t magic = AXDR_COMMAND_REQUEST_MAGIC;
    /* 帧长 = 16B 头 + 0B payload + 4B CRC = 20B（MIN_REQUEST_SIZE） */
    const uint16_t length = (uint16_t)(AXDR_COMMAND_REQUEST_HEADER_SIZE +
                                       AXDR_COMMAND_CRC_SIZE);
    uint32_t crc;

    memset(buf, 0, 64);
    memcpy(&buf[0], &magic, 4);
    buf[4] = AXDR_COMMAND_VERSION & 0xFFu;
    buf[5] = (AXDR_COMMAND_VERSION >> 8) & 0xFFu;
    buf[6] = (uint8_t)(length & 0xFFu);
    buf[7] = (uint8_t)(length >> 8);
    memcpy(&buf[8], &sequence, 4);
    buf[12] = (uint8_t)(opcode & 0xFFu);
    buf[13] = (uint8_t)(opcode >> 8);
    buf[14] = 0U;
    buf[15] = 0U;
    crc = axdr_command_crc32(buf, length - 4u);
    memcpy(&buf[length - 4], &crc, 4);
    return length;
}

int main(void)
{
    axdr_command_core_t core;
    uint8_t request[64];
    uint8_t response[AXDR_COMMAND_MAX_RESPONSE_SIZE];
    uint16_t request_length;
    uint16_t response_length;

    /* ---- 场景 1：合法帧 → Accepted，opcode 正确分发 ---- */
    axdr_command_core_init(&core);
    request_length = make_frame(request, 1u, AXDR_OPCODE_GET_PROTOCOL_INFO);
    response_length = axdr_command_core_process(
        &core, request, request_length, stub_command, NULL,
        response, sizeof(response));
    expect("合法帧产生响应",
           response_length >= AXDR_COMMAND_MIN_RESPONSE_SIZE);
    expect("opcode 正确分发", last_opcode == AXDR_OPCODE_GET_PROTOCOL_INFO);
    expect("响应 magic", memcmp(response, "AXCA", 4) == 0);

    /* ---- 场景 2：CRC 破坏 → 不调分发 ---- */
    axdr_command_core_init(&core);
    request_length = make_frame(request, 2u, AXDR_OPCODE_SET_SPEED);
    request[request_length - 1] ^= 0xFFu;
    last_opcode = 0xFFFFu;
    (void)axdr_command_core_process(
        &core, request, request_length, stub_command, NULL,
        response, sizeof(response));
    expect("坏 CRC 不调分发", last_opcode == 0xFFFFu);

    /* ---- 场景 3：重复 sequence → 响应缓存重放（不调分发） ---- */
    axdr_command_core_init(&core);
    request_length = make_frame(request, 42u, AXDR_OPCODE_GET_DEVICE_INFO);
    (void)axdr_command_core_process(
        &core, request, request_length, stub_command, NULL,
        response, sizeof(response));
    last_opcode = 0xFFFFu;
    response_length = axdr_command_core_process(
        &core, request, request_length, stub_command, NULL,
        response, sizeof(response));
    expect("重复 seq 不调分发", last_opcode == 0xFFFFu);
    expect("重复 seq 仍有响应",
           response_length >= AXDR_COMMAND_MIN_RESPONSE_SIZE);

    /* ---- 场景 4：安全运行时——互锁观察→档案确认→ARM→心跳→超时撤权 ---- */
    {
        axdr_safety_runtime_t safety;
        axdr_motor_profile_confirm_request_t confirm;
        axdr_safety_state_payload_t snapshot;
        uint32_t challenge;

        axdr_safety_runtime_init(&safety, 0u, 0xB0000001u);

        /* 先观察互锁：无故障 + 相输出已关 + 栅极已拉低 → READY */
        axdr_safety_runtime_observe(&safety, 10u, 0u, 1u, 1u);
        axdr_safety_runtime_snapshot(&safety, &snapshot);
        expect("互锁 READY",
               snapshot.interlock_flags == AXDR_SAFETY_INTERLOCK_READY_TO_ARM);

        /* challenge 从 snapshot 读（上位机 GetSafetyState 同源） */
        challenge = snapshot.arm_challenge;

        axdr_safety_runtime_configure_motor_profile(&safety, 1u, 2u, 0xA5A5u);

        /* 未确认档案时 ARM 拒 */
        expect("未确认档案 ARM 被拒(非零 reason)",
               axdr_safety_runtime_arm(&safety, 100u, challenge, 500u) != 0u);

        /* 确认档案（三元组匹配） */
        memset(&confirm, 0, sizeof(confirm));
        confirm.profile_id = 1u;
        confirm.profile_revision = 2u;
        confirm.profile_crc32 = 0xA5A5u;
        expect("档案确认通过(reason=0)",
               axdr_safety_runtime_confirm_motor_profile(&safety, &confirm) == 0u);
        expect("确认标志置位",
               axdr_safety_runtime_motor_profile_confirmed(&safety));

        /* 错误 challenge 拒 */
        expect("错误 challenge ARM 被拒(非零 reason)",
               axdr_safety_runtime_arm(&safety, 150u, challenge ^ 1u, 500u) != 0u);

        /* 正确 ARM */
        expect("确认后 ARM 通过(reason=0)",
               axdr_safety_runtime_arm(&safety, 200u, challenge, 500u) == 0u);

        /* ARM 后未授权前 run 不许；错误 session 授权拒 */
        expect("ARM 后未授权前 run 未许",
               !axdr_safety_runtime_run_authorized(&safety));
        expect("错误 session 授权拒(仍无权限)",
               !axdr_safety_runtime_run_authorized(&safety));

        /* 心跳超时链 */
        axdr_safety_runtime_tick(&safety, 5000u);
        expect("心跳超时后 run 未许",
               !axdr_safety_runtime_run_authorized(&safety));

        axdr_safety_runtime_disarm(&safety, 6000u);
        expect("disarm 后 run 未许",
               !axdr_safety_runtime_run_authorized(&safety));
    }

    if (fail_count == 0)
    {
        (void)printf("test_service_core: all pass\n");
        return 0;
    }
    (void)printf("test_service_core: %d failure(s)\n", fail_count);
    return 1;
}
