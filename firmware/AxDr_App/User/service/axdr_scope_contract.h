#ifndef AXDR_SCOPE_CONTRACT_H
#define AXDR_SCOPE_CONTRACT_H

#include <stdint.h>

#define AXDR_SCOPE_STREAM_MAGIC                 0x46535841u /* AXSF */
#define AXDR_SCOPE_STREAM_VERSION               1u
#define AXDR_SCOPE_CONTROL_RATE_HZ              20000u
#define AXDR_SCOPE_STREAM_MAX_CHANNELS          8u
#define AXDR_SCOPE_STREAM_SAMPLES_PER_PACKET    5u
#define AXDR_SCOPE_STREAM_VALUE_CAPACITY        40u
#define AXDR_SCOPE_STREAM_FRAME_SIZE            212u
#define AXDR_SCOPE_CAPTURE_MAX_CHANNELS         4u
#define AXDR_SCOPE_CAPTURE_STORAGE_BYTES        16384u
#define AXDR_SCOPE_CAPTURE_STORAGE_VALUES       8192u
#define AXDR_SCOPE_CAPTURE_CHUNK_DATA_SIZE       120u
#define AXDR_SCOPE_SIGNAL_DIRECTORY_VERSION      1u
#define AXDR_SCOPE_SIGNAL_KEY_SIZE               28u
#define AXDR_SCOPE_SIGNAL_NAME_SIZE              20u
#define AXDR_SCOPE_SIGNAL_DIRECTORY_PAGE_ENTRIES 2u
#define AXDR_SCOPE_SIGNAL_COUNT                  58u

#define AXDR_SCOPE_SIGNAL_MODE_CONTINUOUS        (1u << 0)
#define AXDR_SCOPE_SIGNAL_MODE_CAPTURE           (1u << 1)

typedef enum
{
    AXDR_SCOPE_UNIT_NONE = 0u,
    AXDR_SCOPE_UNIT_AMPERE = 1u,
    AXDR_SCOPE_UNIT_VOLT = 2u,
    AXDR_SCOPE_UNIT_RADIAN = 3u,
    AXDR_SCOPE_UNIT_RADIAN_PER_SECOND = 4u,
    AXDR_SCOPE_UNIT_DUTY = 5u,
    AXDR_SCOPE_UNIT_SECOND = 6u,
    AXDR_SCOPE_UNIT_HERTZ = 7u,   /* 扫频结果目录（0x1F）新增 */
    AXDR_SCOPE_UNIT_DECIBEL = 8u,
    AXDR_SCOPE_UNIT_DEGREE = 9u
} axdr_scope_unit_t;

typedef enum
{
    AXDR_SCOPE_CHANNEL_PHASE_CURRENT_A = 1u,
    AXDR_SCOPE_CHANNEL_PHASE_CURRENT_B = 2u,
    AXDR_SCOPE_CHANNEL_PHASE_CURRENT_C = 3u,
    AXDR_SCOPE_CHANNEL_CURRENT_D = 4u,
    AXDR_SCOPE_CHANNEL_CURRENT_Q = 5u,
    AXDR_SCOPE_CHANNEL_VOLTAGE_D = 6u,
    AXDR_SCOPE_CHANNEL_VOLTAGE_Q = 7u,
    AXDR_SCOPE_CHANNEL_BUS_VOLTAGE = 8u,
    AXDR_SCOPE_CHANNEL_ROTOR_SPEED_FILTERED = 9u,
    AXDR_SCOPE_CHANNEL_ELECTRICAL_ANGLE = 10u,
    AXDR_SCOPE_CHANNEL_OUTPUT_POSITION = 11u,
    AXDR_SCOPE_CHANNEL_SPEED_REFERENCE = 12u,
    AXDR_SCOPE_CHANNEL_IQ_REFERENCE_APPLIED = 13u,
    AXDR_SCOPE_CHANNEL_DUTY_A = 14u,
    AXDR_SCOPE_CHANNEL_DUTY_B = 15u,
    AXDR_SCOPE_CHANNEL_DUTY_C = 16u,
    AXDR_SCOPE_CHANNEL_VDEAD_DISTORTION_D = 17u,
    AXDR_SCOPE_CHANNEL_VDEAD_D_VOLTAGE_HF = 18u,
    AXDR_SCOPE_CHANNEL_VDEAD_ESTIMATE = 19u,
    AXDR_SCOPE_CHANNEL_VDEAD_RESIDUAL = 20u,
    AXDR_SCOPE_CHANNEL_VDEAD_ELECTRICAL_SPEED = 21u,
    AXDR_SCOPE_CHANNEL_VDEAD_STATUS = 22u,
    AXDR_SCOPE_CHANNEL_VDEAD_REASON = 23u,
    AXDR_SCOPE_CHANNEL_VDEAD_ACCEPTED_SAMPLES = 24u,
    AXDR_SCOPE_CHANNEL_VDEAD_DISTORTION_Q = 25u,
    AXDR_SCOPE_CHANNEL_VDEAD_BASIS_CONFIDENT = 26u,
    AXDR_SCOPE_CHANNEL_VDEAD_SAMPLE_USED = 27u,
    AXDR_SCOPE_CHANNEL_ADC_PHASE_A_RAW = 28u,
    AXDR_SCOPE_CHANNEL_ADC_PHASE_B_RAW = 29u,
    AXDR_SCOPE_CHANNEL_ADC_PHASE_C_RAW = 30u,
    AXDR_SCOPE_CHANNEL_ADC_SEQUENCE_MOD16384 = 31u,
    AXDR_SCOPE_CHANNEL_ID_REFERENCE_APPLIED = 32u,
    AXDR_SCOPE_CHANNEL_PI_VOLTAGE_D = 33u,
    AXDR_SCOPE_CHANNEL_SPEED_REFERENCE_APPLIED = 34u,
    AXDR_SCOPE_CHANNEL_SPEED_PI_P = 35u,
    AXDR_SCOPE_CHANNEL_SPEED_PI_I = 36u,
    AXDR_SCOPE_CHANNEL_POSITION_REFERENCE_APPLIED = 37u,
    AXDR_SCOPE_CHANNEL_POSITION_GUARD_FLAGS = 38u,
    AXDR_SCOPE_CHANNEL_POSITION_INTEGRAL_RELEASED = 39u,
    AXDR_SCOPE_CHANNEL_PERIODIC_CURRENT = 40u,
    AXDR_SCOPE_CHANNEL_PWM_VOLTAGE_Q = 41u,
    AXDR_SCOPE_CHANNEL_OBSERVER_SPEED = 42u,
    AXDR_SCOPE_CHANNEL_OBSERVER_ANGLE = 43u,
    AXDR_SCOPE_CHANNEL_OBSERVER_ANGLE_ERROR = 44u,
    AXDR_SCOPE_CHANNEL_OBSERVER_VALID = 45u,
    AXDR_SCOPE_CHANNEL_OBSERVER_VOLTAGE_ALPHA = 46u,
    AXDR_SCOPE_CHANNEL_OBSERVER_VOLTAGE_BETA = 47u,
    AXDR_SCOPE_CHANNEL_OBSERVER_CURRENT_ALPHA = 48u,
    AXDR_SCOPE_CHANNEL_OBSERVER_CURRENT_BETA = 49u,
    AXDR_SCOPE_CHANNEL_OBSERVER_STEP_US = 50u,
    AXDR_SCOPE_CHANNEL_HANDOVER_STATE = 51u,
    AXDR_SCOPE_CHANNEL_HANDOVER_WEIGHT = 52u,
    AXDR_SCOPE_CHANNEL_CONTROL_ANGLE = 53u,
    AXDR_SCOPE_CHANNEL_CURRENT_PI_D_INTEGRAL = 54u,
    AXDR_SCOPE_CHANNEL_CURRENT_PI_Q_INTEGRAL = 55u,
    AXDR_SCOPE_CHANNEL_HANDOVER_REASON = 56u,
    AXDR_SCOPE_CHANNEL_CONTROL_ANGLE_ERROR = 57u,
    AXDR_SCOPE_CHANNEL_CONTROL_SPEED = 58u
} axdr_scope_channel_id_t;

typedef enum
{
    AXDR_SCOPE_TRIGGER_MANUAL = 1u,
    AXDR_SCOPE_TRIGGER_RISING = 2u,
    AXDR_SCOPE_TRIGGER_FALLING = 3u,
    AXDR_SCOPE_TRIGGER_FAULT = 4u,
    AXDR_SCOPE_TRIGGER_MODE_TRANSITION = 5u
} axdr_scope_trigger_t;

typedef enum
{
    AXDR_SCOPE_CAPTURE_IDLE = 0u,
    AXDR_SCOPE_CAPTURE_ARMED = 1u,
    AXDR_SCOPE_CAPTURE_TRIGGERED = 2u,
    AXDR_SCOPE_CAPTURE_COMPLETE = 3u,
    AXDR_SCOPE_CAPTURE_ERROR = 4u
} axdr_scope_capture_state_t;

typedef struct __attribute__((packed))
{
    uint8_t enabled;
    uint8_t channel_count;
    uint16_t sample_rate_hz;
    uint16_t channel_ids[AXDR_SCOPE_STREAM_MAX_CHANNELS];
} axdr_scope_stream_config_request_t;

typedef struct __attribute__((packed))
{
    uint32_t stream_id;
    uint16_t sample_rate_hz;
    uint8_t channel_count;
    uint8_t samples_per_packet;
    uint16_t channel_ids[AXDR_SCOPE_STREAM_MAX_CHANNELS];
    uint32_t dropped_samples;
} axdr_scope_stream_config_payload_t;

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
    uint16_t channel_ids[AXDR_SCOPE_STREAM_MAX_CHANNELS];
    float values[AXDR_SCOPE_STREAM_VALUE_CAPACITY];
    uint32_t crc32;
} axdr_scope_stream_frame_v1_t;

typedef struct __attribute__((packed))
{
    uint8_t channel_count;
    uint8_t trigger_type;
    uint8_t trigger_channel_index;
    uint8_t pretrigger_percent;
    uint16_t sample_rate_hz;
    uint16_t reserved;
    float trigger_level;
    uint16_t channel_ids[AXDR_SCOPE_CAPTURE_MAX_CHANNELS];
} axdr_scope_capture_arm_request_t;

typedef struct __attribute__((packed))
{
    uint32_t capture_id;
    uint32_t total_samples;
    uint32_t captured_samples;
    uint32_t total_bytes;
    uint32_t dropped_samples;
    uint16_t sample_rate_hz;
    uint8_t state;
    uint8_t channel_count;
    uint8_t pretrigger_percent;
    uint8_t trigger_type;
    uint16_t reserved;
    uint16_t channel_ids[AXDR_SCOPE_CAPTURE_MAX_CHANNELS];
    float scale_per_lsb[AXDR_SCOPE_CAPTURE_MAX_CHANNELS];
} axdr_scope_capture_state_payload_t;

typedef struct __attribute__((packed))
{
    uint32_t capture_id;
    uint32_t byte_offset;
} axdr_scope_capture_read_request_t;

typedef struct __attribute__((packed))
{
    uint32_t capture_id;
    uint32_t total_bytes;
    uint32_t byte_offset;
    uint16_t payload_length;
    uint16_t flags;
    uint8_t data[AXDR_SCOPE_CAPTURE_CHUNK_DATA_SIZE];
} axdr_scope_capture_chunk_payload_t;

typedef struct __attribute__((packed))
{
    uint16_t start_index;
    uint8_t max_entries;
    uint8_t reserved;
} axdr_scope_signal_directory_request_t;

typedef struct __attribute__((packed))
{
    uint16_t channel_id;
    uint16_t sampling_modes;
    uint8_t decimals;
    uint8_t unit;
    uint16_t reserved;
    float suggested_minimum;
    float suggested_maximum;
    char key[AXDR_SCOPE_SIGNAL_KEY_SIZE];
    char name[AXDR_SCOPE_SIGNAL_NAME_SIZE];
} axdr_scope_signal_descriptor_t;

typedef struct __attribute__((packed))
{
    uint16_t schema_version;
    uint16_t total_entries;
    uint16_t start_index;
    uint8_t returned_entries;
    uint8_t descriptor_size;
    axdr_scope_signal_descriptor_t
        entries[AXDR_SCOPE_SIGNAL_DIRECTORY_PAGE_ENTRIES];
} axdr_scope_signal_directory_payload_t;

/* Shared by the USB command handler and host tests so pagination semantics
 * cannot drift between the wire contract and the implementation. */
static inline uint8_t axdr_scope_signal_directory_page_entries(
    uint16_t start_index,
    uint8_t requested_entries)
{
    uint16_t remaining;
    uint8_t returned;

    if ((start_index > AXDR_SCOPE_SIGNAL_COUNT) ||
        (requested_entries == 0u))
    {
        return 0u;
    }

    remaining = AXDR_SCOPE_SIGNAL_COUNT - start_index;
    returned = requested_entries;
    if (returned > AXDR_SCOPE_SIGNAL_DIRECTORY_PAGE_ENTRIES)
    {
        returned = AXDR_SCOPE_SIGNAL_DIRECTORY_PAGE_ENTRIES;
    }
    if (returned > remaining)
    {
        returned = (uint8_t)remaining;
    }
    return returned;
}

/* ---- 扫频测量会话（0x1B-0x1F 段先行承载） ----
 * 原示波器时序捕获语义（16KB 捕获/流模式）尚无消费者，本段重定义为
 * 电流扫频测量（先例同 IDENTIFY 段 0x13/0x16）；示波器将来启用 0x30+
 * 新段（>0x1F 靠专属 capability 位广播）。频域结果按每点 24B 字节流
 * 布局由 0x1E chunk 复用读出，以下尺寸均受 136B 应答上限约束。 */

#define AXDR_SCOPE_SWEEP_POINT_STRIDE      24u  /* 5 float + valid + pad3 */
#define AXDR_SCOPE_SWEEP_AXIS_D            0u
#define AXDR_SCOPE_SWEEP_AXIS_Q            1u

/* 扫频会话状态（axdr_scope_sweep_state_payload_t.state） */
#define AXDR_SCOPE_SWEEP_IDLE              0u
#define AXDR_SCOPE_SWEEP_RUNNING           1u
#define AXDR_SCOPE_SWEEP_DONE              2u
#define AXDR_SCOPE_SWEEP_ABORTED           3u

typedef struct __attribute__((packed))
{
    uint8_t axis;                  /* AXDR_SCOPE_SWEEP_AXIS_D/Q */
    float start_frequency_hz;
    float end_frequency_hz;
    uint16_t requested_points;     /* 1..128；=1 即单点 */
    float amplitude_a;
    float offset_a;
} axdr_scope_sweep_config_request_t;

typedef struct __attribute__((packed))
{
    uint32_t sweep_id;             /* 固定 0x53574550，会话句柄占位 */
    float start_frequency_hz;
    float end_frequency_hz;
    uint16_t requested_points;
    float amplitude_a;
    float offset_a;
    uint8_t axis;
} axdr_scope_sweep_config_payload_t;

typedef struct __attribute__((packed))
{
    uint32_t session_id;           /* 预留：安全会话接入后校验 */
} axdr_scope_sweep_arm_request_t;

typedef struct __attribute__((packed))
{
    uint32_t sweep_id;
    uint8_t accepted;
} axdr_scope_sweep_arm_payload_t;

typedef struct __attribute__((packed))
{
    uint8_t state;                 /* IDLE/RUNNING/DONE/ABORTED */
    uint8_t axis;
    uint16_t completed_points;
    uint16_t valid_points;
    uint16_t reserved;
    float active_frequency_hz;     /* 运行中当前频点；完成后=末点 */
    float closed_magnitude_db;
    float closed_phase_deg;
    float bandwidth_hz;            /* -3dB（SOP 9.11）；无结果=0 */
    float kp_d;                    /* 带宽->PI 建议（diag_current_pi） */
    float ki_d_per_s;
    float kp_q;
    float ki_q_per_s;
} axdr_scope_sweep_state_payload_t;

/* 扫频结果信号目录（0x1F，共 6 条：每点 24B 字节流布局逐字段） */
#define AXDR_SCOPE_SWEEP_SIGNAL_COUNT 6u

#endif /* AXDR_SCOPE_CONTRACT_H */
