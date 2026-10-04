/**
 * @file service_telemetry.c
 * @brief S4：AXDR 160B 遥测帧实现（B 库 axdr_telemetry 语义原样移植）。
 *
 * capture（快速上下文）按 decimation 抽取，从 g_foc 组帧写入 16 槽
 * seqlock 环（guard 奇偶 + seq 连号 + 覆盖计数）；poll（主循环）取最老
 * 连号帧（guard 重试防撕裂）→ 回填 dropped_frames → CRC32（前 156B）
 * → tx 缝发送。BUSY 保留本帧下轮重试，其他错误丢弃计 drop。
 *
 * B pm.* → g_foc 字段映射（对照 B axdr_telemetry.c L125-L186）：
 *   ctrl/state/mode/encoder_type/phase_order/fault → 同名直映
 *   encoder_raw_count  ← enc.abz.raw
 *   cmd_vd/vq          ← out.vd/vq（已应用电压指令）
 *   cmd_iq             ← 级联模式 ref.iq_lim（速度环 PI 实际输出，
 *                         B ctrl.iq_lim 同语义），直流模式 ref.iq
 *   cmd_wr/wm          ← ref.spd_r / ref.spd_m
 *   cmd_posr/posm/torm ← ref.pos_r / 0（本版无独立输出位置参考） / 0
 *   encoder_pos        ← fb.pos_r_1t（单圈机械角，B pos_1 语义）
 *   raw 电气角         ← fb.theta_e - motor.e_off（还原校准前，B e_pr 语义）
 *   electrical_angle   ← fb.theta_e（Park 消费值，B p_e 语义）
 *   forced angle       ← 0（本版无 drag/vf）
 *   多圈位置 rotor/out ← fb.pos_r / fb.pos_m
 *   速度 raw/滤波/out  ← fb.spd_r_raw / fb.spd_r（速度环消费值） / fb.spd_m
 *   电流/电压/母线/占空比 ← fb.ia/ib/ic,id,iq / out.vd,vq / fb.vbus / out.duty_*
 *   tick_ms            ← fast_seq/20（20kHz 统一时钟，与服务命令一致）
 */

#include "service_telemetry.h"

#include <stddef.h>
#include <string.h>

#include "axdr_command_core.h"
#include "common.h"

_Static_assert(sizeof(float) == 4u, "AxDr telemetry requires 32-bit IEEE-754 float");
_Static_assert(sizeof(service_telemetry_frame_t) == AXDR_TELEMETRY_V1_FRAME_SIZE,
               "Unexpected AxDr telemetry frame size");
_Static_assert(offsetof(service_telemetry_frame_t, version) == 4u,
               "Unexpected AxDr telemetry version offset");
_Static_assert(offsetof(service_telemetry_frame_t, sequence) == 8u,
               "Unexpected AxDr telemetry sequence offset");
_Static_assert(offsetof(service_telemetry_frame_t, ctrl_bit) == 16u,
               "Unexpected AxDr telemetry state offset");
_Static_assert(offsetof(service_telemetry_frame_t, encoder_raw_count) == 28u,
               "Unexpected AxDr telemetry encoder offset");
_Static_assert(offsetof(service_telemetry_frame_t, cmd_vd_V) == 36u,
               "Unexpected AxDr telemetry command offset");
_Static_assert(offsetof(service_telemetry_frame_t, encoder_pos_rad) == 72u,
               "Unexpected AxDr telemetry position offset");
_Static_assert(offsetof(service_telemetry_frame_t, phase_current_a_A) == 108u,
               "Unexpected AxDr telemetry current offset");
_Static_assert(offsetof(service_telemetry_frame_t, dropped_frames) == 152u,
               "Unexpected AxDr telemetry counter offset");
_Static_assert(offsetof(service_telemetry_frame_t, crc32) == 156u,
               "Unexpected AxDr telemetry CRC offset");

#define SVC_TLM_QUEUE_CAPACITY 16u
#define SVC_TLM_QUEUE_MASK     (SVC_TLM_QUEUE_CAPACITY - 1u)

_Static_assert((SVC_TLM_QUEUE_CAPACITY & (SVC_TLM_QUEUE_CAPACITY - 1u)) == 0u,
               "telemetry queue capacity must be a power of two");

typedef struct
{
    volatile uint32_t guard;
    volatile service_telemetry_frame_t frame;
} svc_tlm_slot_t;

static svc_tlm_slot_t telemetry_queue[SVC_TLM_QUEUE_CAPACITY];
static uint32_t telemetry_divider;
static volatile uint32_t telemetry_sequence;
static uint32_t telemetry_last_processed_sequence;
static service_telemetry_frame_t telemetry_pending_frame;
static uint8_t telemetry_pending_valid;
static service_telemetry_tx_fn tx_fn;

static volatile uint32_t telemetry_decimation =
    SERVICE_TELEMETRY_DEFAULT_DECIMATION;
static volatile uint32_t capture_count;
static volatile uint32_t sent_count;
static volatile uint32_t drop_count;

void service_telemetry_init(void)
{
    telemetry_divider = 0u;
    telemetry_sequence = 0u;
    telemetry_last_processed_sequence = 0u;
    telemetry_pending_valid = 0u;
    telemetry_decimation = SERVICE_TELEMETRY_DEFAULT_DECIMATION;
    capture_count = 0u;
    sent_count = 0u;
    drop_count = 0u;
    memset(telemetry_queue, 0, sizeof(telemetry_queue));
    memset(&telemetry_pending_frame, 0, sizeof(telemetry_pending_frame));
}

void service_telemetry_set_decimation(uint32_t decimation)
{
    if (decimation == 0u)
    {
        decimation = SERVICE_TELEMETRY_DEFAULT_DECIMATION;
    }
    /* 变更时清积压：旧速率的排队帧不得拥塞新速率（B 原语义） */
    telemetry_decimation = decimation;
    telemetry_divider = 0u;
    telemetry_last_processed_sequence = telemetry_sequence;
    telemetry_pending_valid = 0u;
}

void service_telemetry_bind_tx(service_telemetry_tx_fn tx)
{
    tx_fn = tx;
}

/* 帧填充（快速上下文，与 20kHz ISR 同上下文读 g_foc，无撕裂） */
static void fill_frame(volatile service_telemetry_frame_t *frame,
                       uint32_t sequence)
{
    const uint8_t cascaded =
        (g_foc.mode.debug == spd_curr_cl) ||
        (g_foc.mode.debug == pos_spd_curr_cl);

    frame->magic = AXDR_WIRE_MAGIC;
    frame->version = AXDR_TELEMETRY_V1_VERSION;
    frame->frame_size = (uint16_t)sizeof(service_telemetry_frame_t);
    frame->sequence = sequence;
    frame->tick_ms = g_foc.fast_seq / 20u;

    frame->ctrl_bit = (uint8_t)g_foc.req;
    frame->state_bit = (uint8_t)g_foc.state;
    frame->sys_mode = (uint8_t)g_foc.mode.sys;
    frame->debug_mode = (uint8_t)g_foc.mode.debug;
    frame->release_mode = (uint8_t)g_foc.mode.release;
    frame->calibration_mode = (uint8_t)g_foc.mode.calibrat;
    frame->halt_mode = (uint8_t)g_foc.mode.halt;
    frame->position_mode = 0u; /* 本版无独立位置释放模式 */
    frame->encoder_type = (uint8_t)g_foc.enc.primary;
    frame->phase_order = (uint8_t)g_foc.motor.phase_order;
    frame->reserved0 = 0u;
    frame->reserved1 = 0u;

    frame->encoder_raw_count = g_foc.enc.abz.raw;
    frame->fault_bits = g_foc.fault.all;

    frame->cmd_vd_V = g_foc.out.vd;
    frame->cmd_vq_V = g_foc.out.vq;
    frame->cmd_id_A = g_foc.ref.id;
    /* 级联模式下 iq 参考字段是速度环 PI 实际输出，不是配置限幅（B 原语义） */
    frame->cmd_iq_A = cascaded ? g_foc.ref.iq_lim : g_foc.ref.iq;
    frame->cmd_wr_rad_s = g_foc.ref.spd_r;
    frame->cmd_wm_rad_s = g_foc.ref.spd_m;
    frame->cmd_posr_rad = g_foc.ref.pos_r;
    frame->cmd_posm_rad = 0.0f; /* 本版无独立输出位置参考 */
    frame->cmd_torm_Nm = 0.0f;  /* 本版无转矩模式 */

    frame->encoder_pos_rad = g_foc.fb.pos_r_1t;
    frame->encoder_raw_angle_rad = g_foc.fb.theta_e - g_foc.motor.e_off;
    frame->electrical_angle_rad = g_foc.fb.theta_e;
    frame->forced_electrical_angle_rad = 0.0f; /* 本版无 drag/vf */
    frame->rotor_multi_turn_pos_rad = g_foc.fb.pos_r;
    frame->output_multi_turn_pos_rad = g_foc.fb.pos_m;
    frame->rotor_speed_rad_s = g_foc.fb.spd_r_raw;
    frame->rotor_speed_filtered_rad_s = g_foc.fb.spd_r;
    frame->output_speed_rad_s = g_foc.fb.spd_m;

    frame->phase_current_a_A = g_foc.fb.ia;
    frame->phase_current_b_A = g_foc.fb.ib;
    frame->phase_current_c_A = g_foc.fb.ic;
    frame->current_d_A = g_foc.fb.id;
    frame->current_q_A = g_foc.fb.iq;
    frame->voltage_d_V = g_foc.out.vd;
    frame->voltage_q_V = g_foc.out.vq;
    frame->bus_voltage_V = g_foc.fb.vbus;
    frame->duty_a = g_foc.out.duty_a;
    frame->duty_b = g_foc.out.duty_b;
    frame->duty_c = g_foc.out.duty_c;

    frame->dropped_frames = drop_count;
    frame->crc32 = 0u;
}

void service_telemetry_capture(void)
{
    uint32_t decimation = telemetry_decimation;

    ++capture_count;
    if (decimation == 0u)
    {
        decimation = SERVICE_TELEMETRY_DEFAULT_DECIMATION;
    }
    if (++telemetry_divider < decimation)
    {
        return;
    }
    telemetry_divider = 0u;

    const uint32_t sequence = ++telemetry_sequence;
    svc_tlm_slot_t *slot =
        &telemetry_queue[sequence & SVC_TLM_QUEUE_MASK];
    volatile service_telemetry_frame_t *frame = &slot->frame;

    ++slot->guard; /* 进入临界：guard 变奇 */
    fill_frame(frame, sequence);
    ++slot->guard; /* 退出临界：guard 变偶 */
}

void service_telemetry_poll(void)
{
    uint32_t available_frames;
    uint32_t next_sequence;
    uint32_t overwritten_frames;
    uint32_t guard_before;
    uint32_t guard_after;
    int transmit_status;

    if (telemetry_pending_valid == 0u)
    {
        const uint32_t latest_sequence = telemetry_sequence;
        svc_tlm_slot_t *slot;

        available_frames = latest_sequence - telemetry_last_processed_sequence;
        if (available_frames == 0u)
        {
            return;
        }

        /* 队列已翻覆时跳到最老存活帧，跳过的计入 drop（B 原语义） */
        overwritten_frames = 0u;
        next_sequence = telemetry_last_processed_sequence + 1u;
        if (available_frames > SVC_TLM_QUEUE_CAPACITY)
        {
            overwritten_frames = available_frames - SVC_TLM_QUEUE_CAPACITY;
            next_sequence += overwritten_frames;
        }

        slot = &telemetry_queue[next_sequence & SVC_TLM_QUEUE_MASK];
        do
        {
            guard_before = slot->guard;
            if ((guard_before & 1u) != 0u)
            {
                return; /* capture 正在写，下一轮再取 */
            }
            memcpy(&telemetry_pending_frame,
                   (const void *)&slot->frame,
                   sizeof(telemetry_pending_frame));
            guard_after = slot->guard;
        } while ((guard_before != guard_after) || ((guard_after & 1u) != 0u));

        if (telemetry_pending_frame.sequence != next_sequence)
        {
            return; /* 槽内还不是目标帧，下一轮再取 */
        }

        drop_count += overwritten_frames;
        telemetry_pending_frame.dropped_frames = drop_count;
        telemetry_pending_frame.crc32 = axdr_command_crc32(
            (const uint8_t *)&telemetry_pending_frame,
            (uint32_t)sizeof(telemetry_pending_frame) -
                sizeof(telemetry_pending_frame.crc32));
        telemetry_pending_valid = 1u;
    }

    if (tx_fn == 0)
    {
        /* 发送缝未绑定（板级 CDC 未接）：静默丢弃，不占计数 */
        telemetry_pending_valid = 0u;
        telemetry_last_processed_sequence = telemetry_pending_frame.sequence;
        return;
    }

    transmit_status = tx_fn((const uint8_t *)&telemetry_pending_frame,
                            (uint16_t)sizeof(telemetry_pending_frame));
    if (transmit_status == 0)
    {
        telemetry_last_processed_sequence = telemetry_pending_frame.sequence;
        telemetry_pending_valid = 0u;
        ++sent_count;
    }
    else if (transmit_status != 1) /* 1=BUSY：保留本帧下轮重试 */
    {
        ++drop_count;
        telemetry_last_processed_sequence = telemetry_pending_frame.sequence;
        telemetry_pending_valid = 0u;
    }
}

uint32_t service_telemetry_decimation(void)
{
    return telemetry_decimation;
}

uint32_t service_telemetry_capture_count(void)
{
    return capture_count;
}

uint32_t service_telemetry_sent_count(void)
{
    return sent_count;
}

uint32_t service_telemetry_drop_count(void)
{
    return drop_count;
}
