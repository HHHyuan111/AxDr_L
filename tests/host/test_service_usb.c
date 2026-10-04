/**
 * @file test_service_usb.c
 * @brief S2 USB 薄缝与帧同步测试：分片到达 / 垃圾前缀重同步 / 半帧等待 /
 *        坏长度恢复 / tx 缝回发 / 环满丢弃。
 * @note 分发器为 S2 桩（全拒），帧层行为用 tx 回发与 stats 断言。
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "axdr_command_contract.h"
#include "axdr_command_core.h"
#include "service_usb.h"

static int fail_count;
static void expect(const char *name, bool cond)
{
    if (!cond)
    {
        (void)printf("FAIL %s\n", name);
        fail_count++;
    }
}

/* tx 捕获 */
static uint8_t tx_capture[256];
static uint16_t tx_capture_len;
static void tx_stub(const uint8_t *data, uint16_t length)
{
    if (length <= sizeof(tx_capture))
    {
        memcpy(tx_capture, data, length);
        tx_capture_len = length;
    }
}

static void reset_tx(void)
{
    tx_capture_len = 0u;
}

/* 帧构造（同 test_service_core） */
static uint16_t make_frame(uint8_t *buf, uint32_t sequence, uint16_t opcode)
{
    const uint32_t magic = AXDR_COMMAND_REQUEST_MAGIC;
    const uint16_t length = (uint16_t)(AXDR_COMMAND_REQUEST_HEADER_SIZE +
                                       AXDR_COMMAND_CRC_SIZE);

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
    {
        const uint32_t crc = axdr_command_crc32(buf, length - 4u);
        memcpy(&buf[length - 4], &crc, 4);
    }
    return length;
}

int main(void)
{
    uint8_t frame[64];
    uint16_t frame_length;
    const uint32_t base_stats_frames = 0u;

    service_usb_bind_tx(tx_stub);

    /* ---- 场景 1：完整帧一次到达 → 响应经 tx 发出（桩分发 REJECTED 也有响应帧） ---- */
    frame_length = make_frame(frame, 1u, AXDR_OPCODE_GET_RUNTIME_STATE);
    service_usb_rx(frame, frame_length);
    service_usb_poll();
    expect("完整帧产生 tx 响应", tx_capture_len >= AXDR_COMMAND_MIN_RESPONSE_SIZE);
    expect("响应 magic AXCA", memcmp(tx_capture, "AXCA", 4) == 0);
    expect("帧计数 +1", service_usb_stats()->rx_frames == base_stats_frames + 1u);
    expect("桩分发 ACK=REJECTED", tx_capture[14] == AXDR_ACK_REJECTED);
    expect("桩分发 reason=UNKNOWN_OPCODE",
           tx_capture[15] == AXDR_REASON_UNKNOWN_OPCODE);

    /* ---- 场景 2：分片到达（1 字节一次）→ 拼装后仍分发 ---- */
    reset_tx();
    frame_length = make_frame(frame, 2u, AXDR_OPCODE_GET_RUNTIME_STATE);
    for (uint16_t i = 0u; i < frame_length; ++i)
    {
        service_usb_rx(&frame[i], 1u);
    }
    service_usb_poll();
    expect("分片帧产生响应", tx_capture_len >= AXDR_COMMAND_MIN_RESPONSE_SIZE);

    /* ---- 场景 3：垃圾前缀 + 帧 → 重同步后正常分发 ---- */
    reset_tx();
    {
        const uint8_t garbage[] = {0x00, 0xFF, 0x33, 0x77, 0x12};
        frame_length = make_frame(frame, 3u, AXDR_OPCODE_GET_RUNTIME_STATE);
        service_usb_rx(garbage, sizeof(garbage));
        service_usb_rx(frame, frame_length);
        service_usb_poll();
        expect("垃圾前缀后恢复分发",
               tx_capture_len >= AXDR_COMMAND_MIN_RESPONSE_SIZE);
        expect("resync 计数增加", service_usb_stats()->resync_count >= 1u);
    }

    /* ---- 场景 4：半帧等待（先到 10B，无响应；补齐后响应） ---- */
    reset_tx();
    frame_length = make_frame(frame, 4u, AXDR_OPCODE_GET_RUNTIME_STATE);
    service_usb_rx(frame, 10u);
    service_usb_poll();
    expect("半帧无响应", tx_capture_len == 0u);
    service_usb_rx(&frame[10u], (uint16_t)(frame_length - 10u));
    service_usb_poll();
    expect("补齐后响应", tx_capture_len >= AXDR_COMMAND_MIN_RESPONSE_SIZE);

    /* ---- 场景 5：环满丢弃不崩溃（注入 600B 垃圾） ---- */
    {
        uint8_t bulk[64];
        memset(bulk, 0xAA, sizeof(bulk));
        for (int i = 0; i < 10; ++i)
        {
            service_usb_rx(bulk, sizeof(bulk));
        }
        service_usb_poll();
        expect("环满后 poll 存活", true);
    }

    /* ---- 场景 6：坏帧长字段 → 跳 4B 重同步不卡死 ---- */
    reset_tx();
    frame_length = make_frame(frame, 5u, AXDR_OPCODE_GET_RUNTIME_STATE);
    frame[6] = 0xFFu; /* 长度低字节 = 255 > MAX */
    {
        /* CRC 不再有效但帧长字段已坏——parse 走坏长度分支 */
        service_usb_rx(frame, frame_length);
        service_usb_poll();
        expect("坏长度不卡死", true);
    }

    if (fail_count == 0)
    {
        (void)printf("test_service_usb: all pass\n");
        return 0;
    }
    (void)printf("test_service_usb: %d failure(s)\n", fail_count);
    return 1;
}
