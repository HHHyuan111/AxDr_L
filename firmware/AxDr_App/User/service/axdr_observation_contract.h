#ifndef AXDR_OBSERVATION_CONTRACT_H
#define AXDR_OBSERVATION_CONTRACT_H

#include <stdint.h>
#include "axdr_scope_contract.h"

/* 普通版被动观测协议。与 0x1B～0x1F 的主动扫频隔离；不改变控制状态。 */
#define AXDR_CAPABILITY_PASSIVE_OBSERVATION (1u << 25)
#define AXDR_CAPABILITY_OBSERVATION_20K_STREAM (1u << 26)
#define AXDR_CAPABILITY_OBSERVATION_BLOCK_STREAM (1u << 27)
#define AXDR_OBSERVATION_BLOCK_MAGIC 0x42535841u /* AXSB */
#define AXDR_OBSERVATION_BLOCK_SAMPLES 32u
#define AXDR_OBSERVATION_SCHEMA_VERSION 1u
#define AXDR_OBSERVATION_MAX_CHANNELS 4u
#define AXDR_OBSERVATION_CAPTURE_SAMPLES 1024u
#define AXDR_OBSERVATION_SAMPLE_FLOAT32 1u
#define AXDR_OBSERVATION_UNKNOWN_COUNT UINT32_MAX

#define AXDR_OPCODE_OBSERVATION_CONFIGURE 0x0030u
#define AXDR_OPCODE_OBSERVATION_CAPTURE 0x0031u
#define AXDR_OPCODE_OBSERVATION_STATE 0x0032u
#define AXDR_OPCODE_OBSERVATION_READ 0x0033u
#define AXDR_OPCODE_OBSERVATION_DIRECTORY 0x0034u
#define AXDR_OPCODE_OBSERVATION_DIAGNOSTICS 0x0035u
#define AXDR_OPCODE_OBSERVATION_CANCEL 0x0036u
#define AXDR_OPCODE_OBSERVATION_BLOCK_CONFIGURE 0x0037u

/* 仅 0x37 显式请求产生此格式；四路 20kHz、32 点、564B。旧 AXSF 不变。 */
typedef struct __attribute__((packed, aligned(4)))
{
    uint32_t magic;
    uint16_t version;
    uint16_t frame_size;
    uint32_t stream_id;
    uint32_t packet_sequence;
    uint32_t first_control_tick;
    uint16_t control_rate_hz;
    uint16_t sample_period_ticks;
    uint8_t sample_count;
    uint8_t channel_count;
    uint16_t flags;
    uint32_t dropped_samples;
    uint16_t channel_ids[8];
    float values[AXDR_OBSERVATION_BLOCK_SAMPLES * 4u];
    uint32_t crc32;
} axdr_observation_block_t;

/* Configure 请求/响应与 AXSF v1 数据包复用 axdr_scope_contract.h。
 * 本版只接受 1～4 路、500/1000/2000/5000/20000 Hz（整除控制率），
 * 20000 Hz 需 bit26；表示可请求，不代表当前链路吞吐已经通过实机验收。
 * 未用 channel_ids 必须为 0。
 * Capture 复用 arm 请求形状，仅支持 MANUAL、pretrigger=0、20000 Hz。
 * READ 复用 chunk 请求/响应；data 为逐样本逐通道的小端 IEEE754 float。
 * DIRECTORY 复用目录分页请求/响应，只报告实际实现的信号。
 * STATE、DIAGNOSTICS、CANCEL 请求为空；CANCEL 保留已完整冻结的数据。
 */
typedef struct __attribute__((packed))
{
    uint16_t schema_version;       /* 0 */
    uint16_t payload_size;         /* 2：52 */
    uint32_t capture_id;           /* 4 */
    uint8_t state;                 /* 8：沿用 AXDR_SCOPE_CAPTURE_* */
    uint8_t channel_count;         /* 9 */
    uint8_t trigger_type;          /* 10：MANUAL */
    uint8_t reserved;              /* 11 */
    uint16_t sample_rate_hz;       /* 12 */
    uint16_t sample_period_ticks;  /* 14 */
    uint32_t total_samples;        /* 16 */
    uint32_t captured_samples;     /* 20 */
    uint32_t total_bytes;          /* 24 */
    uint32_t first_control_tick;   /* 28 */
    uint32_t dropped_samples;      /* 32：时序异常无法确定缺样数时为 UINT32_MAX */
    uint16_t channel_ids[AXDR_OBSERVATION_MAX_CHANNELS]; /* 36 */
    uint32_t control_rate_hz;      /* 44 */
    uint16_t sample_format;        /* 48：FLOAT32 */
    uint16_t reserved1;            /* 50 */
} axdr_observation_capture_state_t;

/* 流计数自连接起，IRQ/TX 计数自启动起。tx_completed 含命令及慢遥测，
 * 不能解释成观测样本数；未绑定的硬件诊断为 UINT32_MAX，不表示通过。 */
typedef struct __attribute__((packed))
{
    uint16_t schema_version;
    uint16_t payload_size;
    uint32_t control_rate_hz;
    uint32_t stream_id;
    uint32_t produced_samples;
    uint32_t dropped_samples;
    uint32_t queued_packets;
    uint32_t capture_id;
    uint32_t capture_gaps;
    uint32_t irq_last_cycles;
    uint32_t irq_max_cycles;
    uint32_t irq_overruns;
    uint32_t core_clock_hz;
    uint32_t tx_completed;
    uint32_t tx_errors;
    uint32_t tx_aborted;
    uint32_t tx_queue_peak;
    uint32_t irq_count;
    uint32_t irq_gap_count;
    uint32_t irq_max_interval_cycles;
    uint32_t adc_overruns;
} axdr_observation_diagnostics_t;

#endif
