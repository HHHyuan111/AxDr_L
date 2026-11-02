/**
 * @file service_observation.c
 * @brief 被动观测：ISR 只取同拍数据，主循环负责协议、CRC 和发送。
 *
 * 配置单槽邮箱由主循环发布，ADC ISR 消费后才允许复用。流队列为 SPSC：
 * ISR 独占 head，主循环独占 tail；已发布槽在 tail 推进前禁止覆盖。
 * 短窗仅 ISR 写，完成后冻结。状态通过 guard 发布，不关闭全局中断。
 */
#include "service_observation.h"

#include <stddef.h>
#include <string.h>

#include "axdr_command_contract.h"
#include "axdr_command_core.h"
#include "axdr_observation_contract.h"
#include "common.h"

/* 32 包 = 160 样本：5 kHz 档下发满一环 32ms，覆盖参数页擦除 ~22ms 的
 * 主循环停顿（8 包只有 8ms 会丢样）。RAM 代价 32*212B ≈ 6.6KB。 */
#define OBS_RING_CAPACITY 32u
#define OBS_RING_MASK (OBS_RING_CAPACITY - 1u)
#define OBS_BLOCK_CAPACITY 4u
#define OBS_SIGNAL_COUNT 17u
#define OBS_ACTION_NONE 0u
#define OBS_ACTION_STREAM 1u
#define OBS_ACTION_CAPTURE 2u
#define OBS_FLAG_DISCONTINUITY 1u
#define OBS_FLAG_INVALID_FEEDBACK 2u

_Static_assert(sizeof(float) == 4u, "observation requires float32");
_Static_assert(sizeof(axdr_observation_block_t) == 564u, "block wire size");
_Static_assert(offsetof(axdr_observation_block_t, values) == 48u, "block value offset");
_Static_assert(offsetof(axdr_observation_block_t, crc32) == 560u, "block CRC offset");
_Static_assert(sizeof(axdr_scope_stream_frame_v1_t) == 212u, "stream wire size");
_Static_assert(sizeof(axdr_observation_capture_state_t) == 52u, "capture wire size");
_Static_assert(sizeof(axdr_observation_diagnostics_t) == 80u, "diagnostic wire size");
_Static_assert(offsetof(axdr_scope_stream_frame_v1_t, crc32) == 208u, "CRC offset");

/* 同核 Cortex-M SRAM 发布；memory clobber 同时阻止编译器移动数据访问。 */
static void observation_barrier(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("dmb 0xF" ::: "memory");
#else
    __asm__ volatile("" ::: "memory");
#endif
}

typedef struct
{
    uint32_t id;
    uint16_t rate;
    uint8_t enabled;
    uint8_t count;
    uint8_t compact;
    uint16_t ids[AXDR_OBSERVATION_MAX_CHANNELS];
} observation_config_t;

static volatile uint8_t initialized;
static observation_config_t mailbox;
static volatile uint32_t mailbox_action;
static volatile uint32_t reset_requested;
static volatile uint32_t reset_consumed;
static volatile uint32_t cancel_requested;
static volatile uint32_t cancel_consumed;
static uint32_t next_stream_id;
static uint32_t next_capture_id;
static uint32_t requested_stream_id; /* 主循环独占；0 禁止排队发送。 */

static volatile observation_config_t stream_config; /* 下列生产状态由 ISR 独占。 */
static axdr_scope_stream_frame_v1_t building_frame;
static axdr_scope_stream_frame_v1_t stream_ring[OBS_RING_CAPACITY];
static volatile uint32_t ring_head;
static volatile uint32_t ring_tail;
static axdr_observation_block_t building_block;
static axdr_observation_block_t block_ring[OBS_BLOCK_CAPACITY];
static axdr_observation_block_t pending_block;
static volatile uint32_t block_head;
static volatile uint32_t block_tail;
static uint8_t pending_block_valid;
static uint32_t stream_countdown;
static uint32_t stream_packet_sequence;
static volatile uint32_t produced_samples;
static volatile uint32_t dropped_samples;
static uint32_t main_dropped_samples;
static uint32_t last_control_tick;
static uint8_t last_tick_valid;
static uint16_t next_packet_flags;
static uint8_t timing_gap_pending;
static volatile uint32_t capture_gap_events;
/* 保存 float32 原始位，避免控制工程的有限浮点优化把 NaN 判定折叠。 */
static uint32_t capture_values[AXDR_OBSERVATION_CAPTURE_SAMPLES *
                               AXDR_OBSERVATION_MAX_CHANNELS];
static volatile uint32_t capture_guard;
static volatile axdr_observation_capture_state_t capture_state;
static service_observation_tx_fn transmit;
static axdr_scope_stream_frame_v1_t pending_tx[4]; /* 主循环独占；攒批发送。 */
static uint8_t pending_tx_count;
static uint8_t pending_tx_valid;
static uint8_t tx_batch[4u * sizeof(axdr_scope_stream_frame_v1_t)];
static axdr_observation_diagnostics_t external_diagnostics;

static uint32_t allocate_id(uint32_t *id)
{
    ++*id;
    if (*id == 0u)
    {
        ++*id;
    }
    return *id;
}

static uint8_t mailbox_busy(void)
{
    return ((mailbox_action != OBS_ACTION_NONE) ||
            (reset_requested != reset_consumed) ||
            (cancel_requested != cancel_consumed)) ? 1u : 0u;
}

static void publish_action(uint32_t action)
{
    observation_barrier();
    mailbox_action = action;
}

static uint8_t signal_supported(uint16_t id)
{
    return (((id >= 1u) && (id <= 16u)) || (id == 32u)) ? 1u : 0u;
}

static uint8_t channels_valid(const uint16_t *ids, uint8_t count, uint8_t capacity)
{
    uint8_t i;
    uint8_t j;
    if ((count == 0u) || (count > AXDR_OBSERVATION_MAX_CHANNELS))
    {
        return 0u;
    }
    for (i = 0u; i < capacity; ++i)
    {
        if (i >= count)
        {
            if (ids[i] != 0u) return 0u;
        }
        else
        {
            if (signal_supported(ids[i]) == 0u) return 0u;
            for (j = 0u; j < i; ++j)
            {
                if (ids[j] == ids[i]) return 0u;
            }
        }
    }
    return 1u;
}

static uint32_t signal_bits(uint16_t id)
{
    const float *value;
    uint32_t bits;
    const uint8_t cascade = (g_foc.mode.debug == spd_curr_cl) ||
                            (g_foc.mode.debug == pos_spd_curr_cl);
    const uint8_t output_current = g_foc.pwm_active && g_foc.pwm_commit.valid &&
                                   (g_foc.pwm_commit.seq == g_foc.fast_seq);
    const uint8_t alignment_current = (g_foc.state == DRIVE_STATE_ALIGN);
    const uint8_t control_current = output_current &&
        ((g_foc.state == DRIVE_STATE_RUN) || (g_foc.state == DRIVE_STATE_DIAG) ||
         (alignment_current != 0u));
    if ((id <= 5u) && !g_foc.fb.i_valid) return 0x7FC00000u;
    /* STOP/首次中性占空比无 FOC 计算，缓存 dq/Vdq 作废。 */
    if ((id >= 4u) && (id <= 7u) && (control_current == 0u)) return 0x7FC00000u;
    if ((id >= 14u) && (id <= 16u) && (output_current == 0u)) return 0x7FC00000u;
    if (((id == 13u) || (id == 32u)) && (control_current == 0u)) return 0x7FC00000u;
    if ((id == 8u) && !g_foc.fb.vbus_valid) return 0x7FC00000u;
    if ((id >= 9u) && (id <= 11u) && !g_foc.fb.pos_valid) return 0x7FC00000u;
    switch (id)
    {
    case 1u: value = &g_foc.fb.ia; break;
    case 2u: value = &g_foc.fb.ib; break;
    case 3u: value = &g_foc.fb.ic; break;
    case 4u: value = &g_foc.fb.id; break;
    case 5u: value = &g_foc.fb.iq; break;
    case 6u: value = &g_foc.out.vd; break;
    case 7u: value = &g_foc.out.vq; break;
    case 8u: value = &g_foc.fb.vbus; break;
    case 9u: value = &g_foc.fb.spd_r; break;
    case 10u: value = &g_foc.fb.theta_e; break;
    case 11u: value = &g_foc.fb.pos_m; break;
    case 12u: value = &g_foc.ref.spd_r; break;
    case 13u:
        if (alignment_current != 0u) return 0u;
        value = cascade ? &g_foc.ref.iq_lim : &g_foc.ref.iq;
        break;
    case 14u: value = &g_foc.pwm_commit.duty_a; break;
    case 15u: value = &g_foc.pwm_commit.duty_b; break;
    case 16u: value = &g_foc.pwm_commit.duty_c; break;
    case 32u: value = alignment_current ? &g_foc.motor.align_current_a : &g_foc.ref.id; break;
    default: return 0x7FC00000u;
    }
    memcpy(&bits, value, sizeof(bits));
    return bits;
}

static void capture_begin_write(void)
{
    ++capture_guard;
    observation_barrier();
}

static void capture_end_write(void)
{
    observation_barrier();
    ++capture_guard;
}

static void capture_clear(void)
{
    axdr_observation_capture_state_t empty;
    memset(&empty, 0, sizeof(empty));
    empty.schema_version = AXDR_OBSERVATION_SCHEMA_VERSION;
    empty.payload_size = sizeof(empty);
    empty.control_rate_hz = AXDR_SCOPE_CONTROL_RATE_HZ;
    empty.sample_format = AXDR_OBSERVATION_SAMPLE_FLOAT32;
    capture_begin_write();
    capture_state = empty;
    capture_end_write();
}

static uint8_t capture_snapshot(axdr_observation_capture_state_t *out)
{
    uint8_t attempt;
    uint32_t before;
    for (attempt = 0u; attempt < 3u; ++attempt)
    {
        before = capture_guard;
        observation_barrier();
        if ((before & 1u) != 0u) continue;
        *out = capture_state;
        observation_barrier();
        if (capture_guard == before) return 1u;
    }
    return 0u;
}

void service_observation_init(void)
{
    initialized = 0u;
    memset(&mailbox, 0, sizeof(mailbox));
    memset((void *)&stream_config, 0, sizeof(stream_config));
    memset(&building_frame, 0, sizeof(building_frame));
    memset(&external_diagnostics, 0xFF, sizeof(external_diagnostics));
    mailbox_action = OBS_ACTION_NONE;
    reset_requested = 0u;
    reset_consumed = 0u;
    cancel_requested = 0u;
    cancel_consumed = 0u;
    next_stream_id = 0u;
    next_capture_id = 0u;
    requested_stream_id = 0u;
    ring_head = 0u;
    ring_tail = 0u;
    block_head = block_tail = 0u;
    pending_block_valid = 0u;
    memset(&building_block, 0, sizeof(building_block));
    stream_countdown = 0u;
    stream_packet_sequence = 0u;
    produced_samples = 0u;
    dropped_samples = 0u;
    main_dropped_samples = 0u;
    pending_tx_valid = 0u;
    last_tick_valid = 0u;
    next_packet_flags = 0u;
    timing_gap_pending = 0u;
    capture_gap_events = 0u;
    capture_guard = 0u;
    capture_clear();
    observation_barrier();
    initialized = 1u;
}

void service_observation_bind_tx(service_observation_tx_fn tx)
{
    transmit = tx;
}

void service_observation_reset_link(void)
{
    requested_stream_id = 0u;
    pending_tx_valid = 0u;
    pending_block_valid = 0u;
    main_dropped_samples = 0u;
    ++reset_requested;
    observation_barrier();
    /* 此处不写 head；ISR 可能在下一拍完成旧包，poll 会按会话丢弃。 */
    ring_tail = ring_head;
    block_tail = block_head;
}

static void consume_action(void)
{
    uint32_t action;
    uint8_t i;
    if (reset_requested != reset_consumed)
    {
        stream_config.enabled = 0u;
        building_frame.sample_count = 0u;
        building_block.sample_count = 0u;
        produced_samples = 0u;
        dropped_samples = 0u;
        capture_gap_events = 0u;
        capture_clear();
        mailbox_action = OBS_ACTION_NONE;
        cancel_consumed = cancel_requested;
        observation_barrier();
        reset_consumed = reset_requested;
        return;
    }
    if (cancel_requested != cancel_consumed)
    {
        stream_config.enabled = 0u;
        building_frame.sample_count = 0u;
        building_block.sample_count = 0u;
        if (capture_state.state != AXDR_SCOPE_CAPTURE_COMPLETE) capture_clear();
        mailbox_action = OBS_ACTION_NONE;
        observation_barrier();
        cancel_consumed = cancel_requested;
        return;
    }
    action = mailbox_action;
    if (action == OBS_ACTION_NONE) return;
    observation_barrier();
    if (action == OBS_ACTION_STREAM)
    {
        stream_config = mailbox;
        building_frame.sample_count = 0u;
        building_block.sample_count = 0u;
        stream_packet_sequence = 0u;
        stream_countdown = 0u;
        next_packet_flags = 0u;
    }
    else if (action == OBS_ACTION_CAPTURE)
    {
        capture_clear();
        capture_begin_write();
        capture_state.capture_id = mailbox.id;
        capture_state.state = AXDR_SCOPE_CAPTURE_ARMED;
        capture_state.channel_count = mailbox.count;
        capture_state.trigger_type = AXDR_SCOPE_TRIGGER_MANUAL;
        capture_state.sample_rate_hz = AXDR_SCOPE_CONTROL_RATE_HZ;
        capture_state.sample_period_ticks = 1u;
        capture_state.total_samples = AXDR_OBSERVATION_CAPTURE_SAMPLES;
        capture_state.total_bytes = AXDR_OBSERVATION_CAPTURE_SAMPLES *
                                    mailbox.count * sizeof(float);
        for (i = 0u; i < AXDR_OBSERVATION_MAX_CHANNELS; ++i)
        {
            capture_state.channel_ids[i] = mailbox.ids[i];
        }
        capture_end_write();
    }
    observation_barrier();
    mailbox_action = OBS_ACTION_NONE;
}

static void capture_one(uint32_t tick, uint32_t elapsed, uint8_t timing_gap)
{
    uint32_t index;
    uint8_t i;
    if ((capture_state.state != AXDR_SCOPE_CAPTURE_ARMED) &&
        (capture_state.state != AXDR_SCOPE_CAPTURE_TRIGGERED)) return;
    capture_begin_write();
    index = capture_state.captured_samples;
    if ((index != 0u) && ((elapsed != 1u) || (timing_gap != 0u)))
    {
        /* 间隔异常只能证明时间不连续，未知缺样数用 UINT32_MAX 表示。 */
        capture_state.dropped_samples = (timing_gap != 0u) ? UINT32_MAX : elapsed - 1u;
        ++capture_gap_events;
        capture_state.state = AXDR_SCOPE_CAPTURE_ERROR;
        capture_end_write();
        return;
    }
    if (index == 0u)
    {
        capture_state.first_control_tick = tick;
        capture_state.state = AXDR_SCOPE_CAPTURE_TRIGGERED;
    }
    for (i = 0u; i < capture_state.channel_count; ++i)
    {
        capture_values[index * capture_state.channel_count + i] =
            signal_bits(capture_state.channel_ids[i]);
    }
    capture_state.captured_samples = index + 1u;
    if ((index + 1u) == AXDR_OBSERVATION_CAPTURE_SAMPLES)
    {
        capture_state.state = AXDR_SCOPE_CAPTURE_COMPLETE;
    }
    /* 完成状态发布前，所有原始 float 必须已经写完。 */
    capture_end_write();
}

static void block_one(uint32_t tick, uint32_t elapsed, uint8_t timing_gap)
{
    uint8_t i;
    uint32_t head;
    uint16_t period;
    if (stream_config.enabled == 0u) return;
    period = (uint16_t)(AXDR_SCOPE_CONTROL_RATE_HZ / stream_config.rate);
    if ((elapsed != 1u) || (timing_gap != 0u))
    {
        if ((elapsed - 1u) > stream_countdown)
        {
            dropped_samples += 1u + (elapsed - 2u - stream_countdown) / period;
        }
        dropped_samples += building_block.sample_count;
        building_block.sample_count = 0u;
        stream_countdown = 0u;
        next_packet_flags |= OBS_FLAG_DISCONTINUITY;
    }
    if (stream_countdown != 0u)
    {
        --stream_countdown;
        return;
    }
    stream_countdown = period - 1u;
    if (building_block.sample_count == 0u)
    {
        memset(&building_block, 0, sizeof(building_block));
        building_block.magic = AXDR_OBSERVATION_BLOCK_MAGIC;
        building_block.version = AXDR_SCOPE_STREAM_VERSION;
        building_block.frame_size = sizeof(building_block);
        building_block.stream_id = stream_config.id;
        building_block.first_control_tick = tick;
        building_block.control_rate_hz = AXDR_SCOPE_CONTROL_RATE_HZ;
        building_block.sample_period_ticks = period;
        building_block.channel_count = stream_config.count;
        building_block.flags = next_packet_flags;
        next_packet_flags = 0u;
        for (i = 0u; i < stream_config.count; ++i)
        {
            building_block.channel_ids[i] = stream_config.ids[i];
        }
    }
    for (i = 0u; i < stream_config.count; ++i)
    {
        const uint32_t bits = signal_bits(stream_config.ids[i]);
        const uint32_t index = building_block.sample_count * stream_config.count + i;
        memcpy((uint8_t *)&building_block + offsetof(axdr_observation_block_t, values) +
               index * sizeof(bits), &bits, sizeof(bits));
        if ((bits & 0x7F800000u) == 0x7F800000u) building_block.flags |= OBS_FLAG_INVALID_FEEDBACK;
    }
    ++building_block.sample_count;
    ++produced_samples;
    if (building_block.sample_count != AXDR_OBSERVATION_BLOCK_SAMPLES) return;
    building_block.packet_sequence = ++stream_packet_sequence;
    head = block_head;
    if ((head - block_tail) < OBS_BLOCK_CAPACITY)
    {
        block_ring[head & (OBS_BLOCK_CAPACITY - 1u)] = building_block;
        observation_barrier();
        block_head = head + 1u;
    }
    else
    {
        dropped_samples += AXDR_OBSERVATION_BLOCK_SAMPLES;
        next_packet_flags |= OBS_FLAG_DISCONTINUITY;
    }
    building_block.sample_count = 0u;
}


static void stream_one(uint32_t tick, uint32_t elapsed, uint8_t timing_gap)
{
    uint8_t i;
    uint32_t head;
    uint16_t period;
    if (stream_config.enabled == 0u) return;
    if (stream_config.compact != 0u)
    {
        block_one(tick, elapsed, timing_gap);
        return;
    }
    period = (uint16_t)(AXDR_SCOPE_CONTROL_RATE_HZ / stream_config.rate);
    if ((elapsed != 1u) || (timing_gap != 0u))
    {
        if ((elapsed - 1u) > stream_countdown)
        {
            dropped_samples += 1u + (elapsed - 2u - stream_countdown) / period;
        }
        dropped_samples += building_frame.sample_count;
        building_frame.sample_count = 0u;
        stream_countdown = 0u;
        next_packet_flags |= OBS_FLAG_DISCONTINUITY;
    }
    if (stream_countdown != 0u)
    {
        --stream_countdown;
        return;
    }
    stream_countdown = period - 1u;
    if (building_frame.sample_count == 0u)
    {
        memset(&building_frame, 0, sizeof(building_frame));
        building_frame.magic = AXDR_SCOPE_STREAM_MAGIC;
        building_frame.version = AXDR_SCOPE_STREAM_VERSION;
        building_frame.frame_size = sizeof(building_frame);
        building_frame.stream_id = stream_config.id;
        building_frame.first_control_tick = tick;
        building_frame.control_rate_hz = AXDR_SCOPE_CONTROL_RATE_HZ;
        building_frame.sample_period_ticks = period;
        building_frame.channel_count = stream_config.count;
        building_frame.flags = next_packet_flags;
        next_packet_flags = 0u;
        for (i = 0u; i < stream_config.count; ++i)
        {
            building_frame.channel_ids[i] = stream_config.ids[i];
        }
    }
    for (i = 0u; i < stream_config.count; ++i)
    {
        const uint32_t bits = signal_bits(stream_config.ids[i]);
        const uint32_t index = building_frame.sample_count * stream_config.count + i;
        memcpy((uint8_t *)&building_frame + offsetof(axdr_scope_stream_frame_v1_t, values) +
               index * sizeof(bits), &bits, sizeof(bits));
        if ((bits & 0x7F800000u) == 0x7F800000u) building_frame.flags |= OBS_FLAG_INVALID_FEEDBACK;
    }
    ++building_frame.sample_count;
    ++produced_samples;
    if (building_frame.sample_count != AXDR_SCOPE_STREAM_SAMPLES_PER_PACKET) return;
    building_frame.packet_sequence = ++stream_packet_sequence;
    head = ring_head;
    if ((head - ring_tail) < OBS_RING_CAPACITY)
    {
        stream_ring[head & OBS_RING_MASK] = building_frame;
        observation_barrier();
        ring_head = head + 1u;
    }
    else
    {
        dropped_samples += AXDR_SCOPE_STREAM_SAMPLES_PER_PACKET;
        next_packet_flags |= OBS_FLAG_DISCONTINUITY;
    }
    building_frame.sample_count = 0u;
}

void service_observation_capture(void)
{
    uint32_t tick;
    uint32_t elapsed;
    uint8_t timing_gap;
    if (initialized == 0u) return;
    tick = g_foc.fast_seq;
    elapsed = (last_tick_valid != 0u) ? tick - last_control_tick : 1u;
    if (elapsed == 0u) return;
    last_tick_valid = 1u;
    last_control_tick = tick;
    timing_gap = timing_gap_pending;
    timing_gap_pending = 0u;
    consume_action();
    capture_one(tick, elapsed, timing_gap);
    stream_one(tick, elapsed, timing_gap);
}

void service_observation_note_timing_gap(void)
{
    if (initialized != 0u) timing_gap_pending = 1u;
}

static void poll_block(void)
{
    const uint32_t tail = block_tail;
    int result;
    if (tail == block_head) return;
    observation_barrier();
    if ((requested_stream_id == 0u) ||
        (block_ring[tail & (OBS_BLOCK_CAPACITY - 1u)].stream_id != requested_stream_id))
    {
        observation_barrier();
        block_tail = tail + 1u;
        pending_block_valid = 0u;
        return;
    }
    if (pending_block_valid == 0u)
    {
        pending_block = block_ring[tail & (OBS_BLOCK_CAPACITY - 1u)];
        pending_block.dropped_samples = dropped_samples + main_dropped_samples;
        pending_block.crc32 = axdr_command_crc32((const uint8_t *)&pending_block,
                                                offsetof(axdr_observation_block_t, crc32));
        pending_block_valid = 1u;
    }
    if (transmit == NULL) return;
    result = transmit((const uint8_t *)&pending_block, sizeof(pending_block));
    if (result == 1) return;
    if (result != 0) main_dropped_samples += pending_block.sample_count;
    observation_barrier();
    block_tail = tail + 1u;
    pending_block_valid = 0u;
}

void service_observation_poll(void)
{
    uint32_t tail;
    uint16_t bytes;
    uint8_t i;
    int result;
    if (initialized == 0u) return;
    tail = ring_tail;
    if (tail == ring_head)
    {
        poll_block();
        return;
    }
    if (pending_tx_valid == 0u)
    {
        uint8_t limit;
        uint32_t head;
        /* 20 kHz 至多四帧 848B；旧档仍至多两帧。不等待凑批，不阻塞 ISR。
         * 保存一次 head 快照；本批只读已发布的槽，且只属于本流。 */
        pending_tx_count = 1u;
        head = ring_head;
        observation_barrier();
        pending_tx[0] = stream_ring[tail & OBS_RING_MASK];
        limit = pending_tx[0].sample_period_ticks == 1u ? 4u : 2u;
        while ((pending_tx_count < limit) &&
               ((uint32_t)(tail + pending_tx_count) != head))
        {
            const axdr_scope_stream_frame_v1_t *next =
                &stream_ring[(tail + pending_tx_count) & OBS_RING_MASK];
            if (next->stream_id != pending_tx[0].stream_id) break;
            pending_tx[pending_tx_count++] = *next;
        }
        for (i = 0u; i < pending_tx_count; ++i)
        {
            pending_tx[i].dropped_samples =
                dropped_samples + main_dropped_samples;
            pending_tx[i].crc32 = axdr_command_crc32(
                (const uint8_t *)&pending_tx[i],
                offsetof(axdr_scope_stream_frame_v1_t, crc32));
        }
        pending_tx_valid = 1u;
    }
    if ((requested_stream_id == 0u) ||
        (pending_tx[0].stream_id != requested_stream_id))
    {
        observation_barrier();
        ring_tail = tail + 1u;
        pending_tx_valid = 0u;
        return;
    }
    if (transmit == NULL) return;
    bytes = (uint16_t)(pending_tx_count * sizeof(pending_tx[0]));
    memcpy(tx_batch, pending_tx, bytes);
    result = transmit(tx_batch, bytes);
    if (result == 1) return;
    if (result != 0)
    {
        for (i = 0u; i < pending_tx_count; ++i)
        {
            main_dropped_samples += pending_tx[i].sample_count;
        }
    }
    observation_barrier();
    ring_tail = tail + pending_tx_count;
    pending_tx_valid = 0u;
}

void service_observation_set_timing_stats(uint32_t last_cycles,
                                         uint32_t max_cycles,
                                         uint32_t overruns,
                                         uint32_t core_clock_hz)
{
    external_diagnostics.irq_last_cycles = last_cycles;
    external_diagnostics.irq_max_cycles = max_cycles;
    external_diagnostics.irq_overruns = overruns;
    external_diagnostics.core_clock_hz = core_clock_hz;
}

void service_observation_set_tx_stats(uint32_t completed, uint32_t errors,
                                     uint32_t aborted, uint32_t queue_peak)
{
    external_diagnostics.tx_completed = completed;
    external_diagnostics.tx_errors = errors;
    external_diagnostics.tx_aborted = aborted;
    external_diagnostics.tx_queue_peak = queue_peak;
}

void service_observation_set_irq_stats(uint32_t count, uint32_t gaps,
                                      uint32_t max_interval_cycles,
                                      uint32_t adc_overruns)
{
    external_diagnostics.irq_count = count;
    external_diagnostics.irq_gap_count = gaps;
    external_diagnostics.irq_max_interval_cycles = max_interval_cycles;
    external_diagnostics.adc_overruns = adc_overruns;
}

static uint8_t payload_out(const void *data, uint16_t size, uint8_t *out,
                           uint16_t capacity, uint16_t *length)
{
    if (size > capacity) return AXDR_REASON_INVALID_LENGTH;
    memcpy(out, data, size);
    *length = size;
    return AXDR_REASON_NONE;
}

static uint8_t configure_stream(uint8_t compact, const uint8_t *request, uint16_t length,
                                uint8_t *response, uint16_t capacity,
                                uint16_t *out_length)
{
    axdr_scope_stream_config_request_t req;
    axdr_scope_stream_config_payload_t out;
    axdr_observation_capture_state_t state;
    uint16_t ids[AXDR_SCOPE_STREAM_MAX_CHANNELS];
    if (length != sizeof(req)) return AXDR_REASON_INVALID_PAYLOAD;
    if (capacity < sizeof(out)) return AXDR_REASON_INVALID_LENGTH;
    memcpy(&req, request, sizeof(req));
    memcpy(ids, req.channel_ids, sizeof(ids));
    if (req.enabled > 1u) return AXDR_REASON_INVALID_PAYLOAD;
    if ((req.enabled != 0u) &&
        ((channels_valid(ids, req.channel_count, AXDR_SCOPE_STREAM_MAX_CHANNELS) == 0u) ||
         ((req.sample_rate_hz != 500u) && (req.sample_rate_hz != 1000u) &&
          (req.sample_rate_hz != 2000u) && (req.sample_rate_hz != 5000u) &&
          (req.sample_rate_hz != AXDR_SCOPE_CONTROL_RATE_HZ))))
        return AXDR_REASON_INVALID_PAYLOAD;
    if (compact && req.enabled &&
        (req.channel_count != 4u || req.sample_rate_hz != AXDR_SCOPE_CONTROL_RATE_HZ))
        return AXDR_REASON_INVALID_PAYLOAD;
    if (mailbox_busy() != 0u) return AXDR_REASON_BUSY;
    if (capture_snapshot(&state) == 0u) return AXDR_REASON_BUSY;
    if ((state.state == AXDR_SCOPE_CAPTURE_ARMED) ||
        (state.state == AXDR_SCOPE_CAPTURE_TRIGGERED)) return AXDR_REASON_BUSY;
    memset(&mailbox, 0, sizeof(mailbox));
    mailbox.enabled = req.enabled;
    mailbox.compact = compact;
    if (req.enabled != 0u)
    {
        mailbox.id = allocate_id(&next_stream_id);
        mailbox.rate = req.sample_rate_hz;
        mailbox.count = req.channel_count;
        memcpy(mailbox.ids, ids, sizeof(mailbox.ids));
    }
    requested_stream_id = mailbox.id;
    pending_tx_valid = 0u;
    pending_block_valid = 0u;
    memset(&out, 0, sizeof(out));
    out.stream_id = mailbox.id;
    out.sample_rate_hz = mailbox.rate;
    out.channel_count = mailbox.count;
    out.samples_per_packet = compact ? AXDR_OBSERVATION_BLOCK_SAMPLES : AXDR_SCOPE_STREAM_SAMPLES_PER_PACKET;
    memcpy(out.channel_ids, mailbox.ids, sizeof(mailbox.ids));
    out.dropped_samples = dropped_samples + main_dropped_samples;
    publish_action(OBS_ACTION_STREAM);
    return payload_out(&out, sizeof(out), response, capacity, out_length);
}

static uint8_t arm_capture(const uint8_t *request, uint16_t length,
                           uint8_t *response, uint16_t capacity,
                           uint16_t *out_length)
{
    axdr_scope_capture_arm_request_t req;
    axdr_observation_capture_state_t out;
    uint16_t ids[AXDR_OBSERVATION_MAX_CHANNELS];
    uint32_t trigger_bits;
    if (length != sizeof(req)) return AXDR_REASON_INVALID_PAYLOAD;
    if (capacity < sizeof(out)) return AXDR_REASON_INVALID_LENGTH;
    memcpy(&req, request, sizeof(req));
    memcpy(ids, req.channel_ids, sizeof(ids));
    memcpy(&trigger_bits, &req.trigger_level, sizeof(trigger_bits));
    if ((channels_valid(ids, req.channel_count, AXDR_OBSERVATION_MAX_CHANNELS) == 0u) ||
        (req.trigger_type != AXDR_SCOPE_TRIGGER_MANUAL) ||
        (req.pretrigger_percent != 0u) || (req.trigger_channel_index != 0u) ||
        (req.reserved != 0u) || ((trigger_bits & 0x7FFFFFFFu) != 0u) ||
        (req.sample_rate_hz != AXDR_SCOPE_CONTROL_RATE_HZ))
        return AXDR_REASON_INVALID_PAYLOAD;
    if ((mailbox_busy() != 0u) || (stream_config.enabled != 0u)) return AXDR_REASON_BUSY;
    if (capture_snapshot(&out) == 0u) return AXDR_REASON_BUSY;
    if ((out.state == AXDR_SCOPE_CAPTURE_ARMED) ||
        (out.state == AXDR_SCOPE_CAPTURE_TRIGGERED)) return AXDR_REASON_BUSY;
    memset(&mailbox, 0, sizeof(mailbox));
    mailbox.id = allocate_id(&next_capture_id);
    mailbox.count = req.channel_count;
    memcpy(mailbox.ids, ids, sizeof(mailbox.ids));
    memset(&out, 0, sizeof(out));
    out.schema_version = AXDR_OBSERVATION_SCHEMA_VERSION;
    out.payload_size = sizeof(out);
    out.capture_id = mailbox.id;
    out.state = AXDR_SCOPE_CAPTURE_ARMED;
    out.channel_count = req.channel_count;
    out.trigger_type = AXDR_SCOPE_TRIGGER_MANUAL;
    out.sample_rate_hz = AXDR_SCOPE_CONTROL_RATE_HZ;
    out.sample_period_ticks = 1u;
    out.total_samples = AXDR_OBSERVATION_CAPTURE_SAMPLES;
    out.total_bytes = AXDR_OBSERVATION_CAPTURE_SAMPLES * req.channel_count * sizeof(float);
    memcpy(out.channel_ids, ids, sizeof(ids));
    out.control_rate_hz = AXDR_SCOPE_CONTROL_RATE_HZ;
    out.sample_format = AXDR_OBSERVATION_SAMPLE_FLOAT32;
    publish_action(OBS_ACTION_CAPTURE);
    return payload_out(&out, sizeof(out), response, capacity, out_length);
}

static uint8_t read_capture(const uint8_t *request, uint16_t length,
                            uint8_t *response, uint16_t capacity,
                            uint16_t *out_length)
{
    axdr_scope_capture_read_request_t req;
    axdr_scope_capture_chunk_payload_t out;
    axdr_observation_capture_state_t state;
    uint32_t remaining;
    if (length != sizeof(req)) return AXDR_REASON_INVALID_PAYLOAD;
    if (mailbox_busy() != 0u) return AXDR_REASON_BUSY;
    memcpy(&req, request, sizeof(req));
    if (capture_snapshot(&state) == 0u) return AXDR_REASON_BUSY;
    if ((state.capture_id == 0u) || (req.capture_id != state.capture_id))
        return AXDR_REASON_INVALID_PAYLOAD;
    if (state.state != AXDR_SCOPE_CAPTURE_COMPLETE) return AXDR_REASON_INVALID_STATE;
    if (req.byte_offset >= state.total_bytes) return AXDR_REASON_OUT_OF_RANGE;
    remaining = state.total_bytes - req.byte_offset;
    memset(&out, 0, sizeof(out));
    out.capture_id = state.capture_id;
    out.total_bytes = state.total_bytes;
    out.byte_offset = req.byte_offset;
    out.payload_length = (remaining < sizeof(out.data)) ? (uint16_t)remaining : sizeof(out.data);
    out.flags = (remaining <= sizeof(out.data)) ? 1u : 0u;
    memcpy(out.data, ((const uint8_t *)capture_values) + req.byte_offset, out.payload_length);
    return payload_out(&out, sizeof(out), response, capacity, out_length);
}

static uint8_t directory(const uint8_t *request, uint16_t length,
                         uint8_t *response, uint16_t capacity, uint16_t *out_length)
{
    static const struct
    {
        const char *key;
        const char *name;
        uint8_t unit;
        float minimum;
        float maximum;
    } signals[OBS_SIGNAL_COUNT] = {
        {"phase_current_a", "Ia", AXDR_SCOPE_UNIT_AMPERE, -10.0f, 10.0f},
        {"phase_current_b", "Ib", AXDR_SCOPE_UNIT_AMPERE, -10.0f, 10.0f},
        {"phase_current_c", "Ic", AXDR_SCOPE_UNIT_AMPERE, -10.0f, 10.0f},
        {"current_d", "Id", AXDR_SCOPE_UNIT_AMPERE, -10.0f, 10.0f},
        {"current_q", "Iq", AXDR_SCOPE_UNIT_AMPERE, -10.0f, 10.0f},
        {"voltage_d", "Vd", AXDR_SCOPE_UNIT_VOLT, -30.0f, 30.0f},
        {"voltage_q", "Vq", AXDR_SCOPE_UNIT_VOLT, -30.0f, 30.0f},
        {"bus_voltage", "Vbus", AXDR_SCOPE_UNIT_VOLT, 0.0f, 60.0f},
        {"rotor_speed_filtered", "Speed", AXDR_SCOPE_UNIT_RADIAN_PER_SECOND, -100.0f, 100.0f},
        {"electrical_angle", "Angle", AXDR_SCOPE_UNIT_RADIAN, 0.0f, 6.2831853f},
        {"output_position", "Position", AXDR_SCOPE_UNIT_RADIAN, -6.2831853f, 6.2831853f},
        {"speed_reference", "Speed ref", AXDR_SCOPE_UNIT_RADIAN_PER_SECOND, -100.0f, 100.0f},
        {"iq_reference_applied", "Iq ref", AXDR_SCOPE_UNIT_AMPERE, -10.0f, 10.0f},
        {"duty_a", "Duty A", AXDR_SCOPE_UNIT_DUTY, 0.0f, 1.0f},
        {"duty_b", "Duty B", AXDR_SCOPE_UNIT_DUTY, 0.0f, 1.0f},
        {"duty_c", "Duty C", AXDR_SCOPE_UNIT_DUTY, 0.0f, 1.0f},
        {"id_reference_applied", "Id ref", AXDR_SCOPE_UNIT_AMPERE, -10.0f, 10.0f}
    };
    axdr_scope_signal_directory_request_t req;
    axdr_scope_signal_directory_payload_t out;
    uint8_t i;
    uint32_t index;
    if (length != sizeof(req)) return AXDR_REASON_INVALID_PAYLOAD;
    memcpy(&req, request, sizeof(req));
    if (req.reserved != 0u) return AXDR_REASON_INVALID_PAYLOAD;
    memset(&out, 0, sizeof(out));
    out.schema_version = AXDR_SCOPE_SIGNAL_DIRECTORY_VERSION;
    out.total_entries = OBS_SIGNAL_COUNT;
    out.start_index = req.start_index;
    out.descriptor_size = sizeof(axdr_scope_signal_descriptor_t);
    for (i = 0u; i < AXDR_SCOPE_SIGNAL_DIRECTORY_PAGE_ENTRIES && i < req.max_entries; ++i)
    {
        index = req.start_index + i;
        if (index >= OBS_SIGNAL_COUNT) break;
        out.entries[i].channel_id = (index < 16u) ? index + 1u : 32u;
        out.entries[i].sampling_modes = AXDR_SCOPE_SIGNAL_MODE_CONTINUOUS | AXDR_SCOPE_SIGNAL_MODE_CAPTURE;
        out.entries[i].decimals = 3u;
        out.entries[i].unit = signals[index].unit;
        out.entries[i].suggested_minimum = signals[index].minimum;
        out.entries[i].suggested_maximum = signals[index].maximum;
        memcpy(out.entries[i].key, signals[index].key, strlen(signals[index].key));
        memcpy(out.entries[i].name, signals[index].name, strlen(signals[index].name));
        ++out.returned_entries;
    }
    return payload_out(&out, sizeof(out), response, capacity, out_length);
}

uint8_t service_observation_route(void *context, uint16_t opcode,
                                 const uint8_t *request_payload,
                                 uint16_t request_payload_length,
                                 uint8_t *response_payload,
                                 uint16_t response_payload_capacity,
                                 uint16_t *response_payload_length,
                                 uint8_t *ack_state)
{
    axdr_observation_capture_state_t state;
    axdr_observation_diagnostics_t diag;
    (void)context;
    *response_payload_length = 0u;
    *ack_state = AXDR_ACK_ACCEPTED;
    if (initialized == 0u) return AXDR_REASON_INVALID_STATE;
    switch (opcode)
    {
    case AXDR_OPCODE_OBSERVATION_CONFIGURE:
        return configure_stream(0u, request_payload, request_payload_length,
                                response_payload, response_payload_capacity, response_payload_length);
    case AXDR_OPCODE_OBSERVATION_BLOCK_CONFIGURE:
        return configure_stream(1u, request_payload, request_payload_length,
                                response_payload, response_payload_capacity, response_payload_length);
    case AXDR_OPCODE_OBSERVATION_CAPTURE:
        return arm_capture(request_payload, request_payload_length,
                           response_payload, response_payload_capacity, response_payload_length);
    case AXDR_OPCODE_OBSERVATION_READ:
        return read_capture(request_payload, request_payload_length,
                            response_payload, response_payload_capacity, response_payload_length);
    case AXDR_OPCODE_OBSERVATION_DIRECTORY:
        return directory(request_payload, request_payload_length,
                         response_payload, response_payload_capacity, response_payload_length);
    case AXDR_OPCODE_OBSERVATION_STATE:
        if (request_payload_length != 0u) return AXDR_REASON_INVALID_PAYLOAD;
        if (mailbox_busy() != 0u) return AXDR_REASON_BUSY;
        if (capture_snapshot(&state) == 0u) return AXDR_REASON_BUSY;
        return payload_out(&state, sizeof(state), response_payload,
                           response_payload_capacity, response_payload_length);
    case AXDR_OPCODE_OBSERVATION_DIAGNOSTICS:
        if (request_payload_length != 0u) return AXDR_REASON_INVALID_PAYLOAD;
        if (capture_snapshot(&state) == 0u) return AXDR_REASON_BUSY;
        diag = external_diagnostics;
        diag.schema_version = AXDR_OBSERVATION_SCHEMA_VERSION;
        diag.payload_size = sizeof(diag);
        diag.control_rate_hz = AXDR_SCOPE_CONTROL_RATE_HZ;
        diag.stream_id = requested_stream_id;
        diag.produced_samples = produced_samples;
        diag.dropped_samples = dropped_samples + main_dropped_samples;
        diag.queued_packets = (ring_head - ring_tail) + (block_head - block_tail);
        diag.capture_id = state.capture_id;
        diag.capture_gaps = capture_gap_events;
        return payload_out(&diag, sizeof(diag), response_payload,
                           response_payload_capacity, response_payload_length);
    case AXDR_OPCODE_OBSERVATION_CANCEL:
        if (request_payload_length != 0u) return AXDR_REASON_INVALID_PAYLOAD;
        requested_stream_id = 0u;
        pending_tx_valid = 0u;
        pending_block_valid = 0u;
        ++cancel_requested;
        observation_barrier();
        /* 空 ACK 表示取消请求已接收；下一拍生效，随后可查询 STATE。 */
        return AXDR_REASON_NONE;
    default:
        return AXDR_REASON_UNKNOWN_OPCODE;
    }
}
