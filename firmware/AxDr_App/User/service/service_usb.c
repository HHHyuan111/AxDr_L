/**
 * @file service_usb.c
 * @brief USB 薄缝实现：rx 环形队列 + 帧同步（B 库 parse_one_request 语义原样
 *        移植）+ command_core 分发调用 + tx 回调。协议逻辑零 USB 依赖。
 */

#include "service_usb.h"

#include <string.h>

#include "axdr_command_core.h"
#include "axdr_command_contract.h"

#define SVC_RX_RING_SIZE 512u
#define SVC_RX_RING_MASK (SVC_RX_RING_SIZE - 1u)
_Static_assert((SVC_RX_RING_SIZE & SVC_RX_RING_MASK) == 0u,
               "rx ring must be power of two");

static uint8_t rx_ring[SVC_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

static uint8_t parser_buffer[AXDR_COMMAND_MAX_REQUEST_SIZE];
static uint16_t parser_length;

static axdr_command_core_t command_core;
static service_usb_tx_fn tx_fn;
static service_usb_stats_t stats;

/* ---- S3 分发器的外部入口（当前 S2 阶段为桩，S3 填充 14 opcode） ---- */
uint8_t service_command_dispatch(void *context, uint16_t opcode,
                                 const uint8_t *request_payload,
                                 uint16_t request_payload_length,
                                 uint8_t *response_payload,
                                 uint16_t response_payload_capacity,
                                 uint16_t *response_payload_length,
                                 uint8_t *ack_state);

static uint8_t rx_pop(uint8_t *value)
{
    const uint16_t tail = rx_tail;
    if (tail == rx_head)
    {
        return 0u;
    }
    *value = rx_ring[tail];
    rx_tail = (uint16_t)((tail + 1u) & SVC_RX_RING_MASK);
    return 1u;
}

static void discard_parser_prefix(uint16_t count)
{
    if (count >= parser_length)
    {
        parser_length = 0u;
        return;
    }
    memmove(parser_buffer, &parser_buffer[count], parser_length - count);
    parser_length = (uint16_t)(parser_length - count);
}

static uint32_t read_u32(const uint8_t *buf)
{
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

static uint16_t read_u16(const uint8_t *buf)
{
    return (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

/* B 库 find_request_magic 语义原样 */
static uint16_t find_request_magic(void)
{
    if (parser_length < 4u)
    {
        return parser_length;
    }
    for (uint16_t i = 0u; i <= (uint16_t)(parser_length - 4u); ++i)
    {
        if (read_u32(&parser_buffer[i]) == AXDR_COMMAND_REQUEST_MAGIC)
        {
            return i;
        }
    }
    return parser_length;
}

/* B 库 process_request 语义（CRC 走 core；诊断计数留在本层） */
static void process_request(uint16_t request_length)
{
    uint8_t response[AXDR_COMMAND_MAX_RESPONSE_SIZE];
    const uint16_t response_length = axdr_command_core_process(
        &command_core, parser_buffer, request_length,
        service_command_dispatch, NULL,
        response, sizeof(response));

    if (response_length >= AXDR_COMMAND_MIN_RESPONSE_SIZE)
    {
        stats.rx_frames++;
        if (response[15] != AXDR_REASON_NONE &&
            response[14] == AXDR_ACK_REJECTED)
        {
            stats.rejected_count++;
        }
        if (tx_fn != 0)
        {
            tx_fn(response, response_length);
            stats.tx_frames++;
        }
    }
}

/* B 库 parse_one_request 语义原样移植 */
static void parse_one_request(void)
{
    uint16_t magic_offset;
    uint16_t frame_size;

    magic_offset = find_request_magic();
    if (magic_offset == parser_length)
    {
        if (parser_length > 3u)
        {
            stats.resync_count++;
            discard_parser_prefix((uint16_t)(parser_length - 3u));
        }
        return;
    }
    if (magic_offset != 0u)
    {
        stats.resync_count++;
        discard_parser_prefix(magic_offset);
    }
    if (parser_length < AXDR_COMMAND_REQUEST_HEADER_SIZE)
    {
        return;
    }

    frame_size = read_u16(&parser_buffer[6]);
    if ((frame_size < AXDR_COMMAND_MIN_REQUEST_SIZE) ||
        (frame_size > AXDR_COMMAND_MAX_REQUEST_SIZE))
    {
        stats.resync_count++;
        process_request(AXDR_COMMAND_REQUEST_HEADER_SIZE);
        discard_parser_prefix(4u);
        return;
    }
    if (parser_length < frame_size)
    {
        return;
    }

    process_request(frame_size);
    discard_parser_prefix(frame_size);
}

void service_usb_rx(const uint8_t *data, uint16_t length)
{
    if (data == 0)
    {
        return;
    }
    for (uint16_t i = 0u; i < length; ++i)
    {
        const uint16_t next = (uint16_t)((rx_head + 1u) & SVC_RX_RING_MASK);
        if (next == rx_tail)
        {
            break; /* 环满丢弃（上位机重发机制兜底） */
        }
        rx_ring[rx_head] = data[i];
        rx_head = next;
        stats.rx_bytes++;
    }
}

void service_usb_bind_tx(service_usb_tx_fn tx)
{
    tx_fn = tx;
}

void service_usb_poll(void)
{
    uint8_t byte;
    while (rx_pop(&byte) != 0u)
    {
        if (parser_length < sizeof(parser_buffer))
        {
            parser_buffer[parser_length++] = byte;
        }
        else
        {
            /* 溢出防护：缓冲满时丢弃前半，保住最新字节 */
            stats.resync_count++;
            discard_parser_prefix(parser_length / 2u);
            parser_buffer[parser_length++] = byte;
        }
        parse_one_request();
    }
}

const service_usb_stats_t *service_usb_stats(void)
{
    return &stats;
}

/* S3 真身分发器在 service_command.c（本文件只引用） */
extern uint8_t service_command_dispatch(
    void *context, uint16_t opcode,
    const uint8_t *request_payload,
    uint16_t request_payload_length,
    uint8_t *response_payload,
    uint16_t response_payload_capacity,
    uint16_t *response_payload_length,
    uint8_t *ack_state);
