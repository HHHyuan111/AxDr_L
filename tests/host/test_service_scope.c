/**
 * @file test_service_scope.c
 * @brief 扫频测量会话协议层全链路测试（经 service_usb 缝注入帧）。
 *
 * service_command.c 是用户 WIP（引用 foc_drv 的 cur_pi_init，host 不链
 * 电机源），本测试用替身分发器：非 0x1B-0x1F 段记录转发并回最小
 * protocol_info（capability/mask 置 0），专验 route 拦截补位；扫频段
 * 走真 g_diag 门面与 mc_current_sweep 结果结构。
 *
 * 覆盖：0x01 capability/mask 补位 → 0x1B 配置（合法/非法矩阵/忙拒）
 * → 0x1C 受理（请求槽断言）→ 0x1D 状态三态+带宽插值+PI 建议
 * → 0x1E 字节流切片（末块标志/越界/错 id）→ 0x1F 目录分页与空页。
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "algorithm_config.h"
#include "axdr_command_contract.h"
#include "axdr_command_core.h"
#include "axdr_scope_contract.h"
#include "common.h"
#include "diag_runtime.h"
#include "drive_diag.h"
#include "service_command.h"
#include "service_scope.h"
#include "service_usb.h"

#define SWEEP_ID 0x53574550u /* 与 service_scope.c 的 SCOPE_SWEEP_ID 同值 */

foc_t g_foc;
diag_runtime_t g_diag;

/* ---- service_command.c 替身（WIP 不入链）：记录转发 + 空应答 ---- */
static uint16_t forwarded_opcode;
static uint16_t forwarded_len;
uint8_t service_command_dispatch(void *context, uint16_t opcode,
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
    (void)response_payload_capacity;
    (void)ack_state;
    forwarded_opcode = opcode;
    forwarded_len = request_payload_length;
    if (opcode == AXDR_OPCODE_GET_PROTOCOL_INFO)
    {
        /* 最小 protocol_info：capability/mask 全 0，专验 route 补位 */
        memset(response_payload, 0, sizeof(axdr_protocol_info_payload_t));
        *response_payload_length =
            (uint16_t)sizeof(axdr_protocol_info_payload_t);
    }
    else
    {
        *response_payload_length = 0u;
    }
    return AXDR_REASON_NONE;
}

static int fail_count;
static void expect(const char *name, int cond)
{
    if (!cond)
    {
        (void)printf("FAIL %s\n", name);
        fail_count++;
    }
}

static int nearf(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

static uint8_t tx_buf[256];
static uint16_t tx_len;
static void tx_stub(const uint8_t *data, uint16_t length)
{
    if (length <= sizeof(tx_buf))
    {
        memcpy(tx_buf, data, length);
        tx_len = length;
    }
}

static void send_frame(const uint8_t *payload, uint16_t payload_length,
                       uint32_t sequence, uint16_t opcode)
{
    uint8_t frame[128];
    const uint32_t magic = AXDR_COMMAND_REQUEST_MAGIC;
    const uint16_t length = (uint16_t)(AXDR_COMMAND_REQUEST_HEADER_SIZE +
                                       payload_length +
                                       AXDR_COMMAND_CRC_SIZE);
    uint32_t crc;

    memset(frame, 0, sizeof(frame));
    memcpy(&frame[0], &magic, 4);
    frame[4] = AXDR_COMMAND_VERSION & 0xFFu;
    frame[5] = (AXDR_COMMAND_VERSION >> 8) & 0xFFu;
    frame[6] = (uint8_t)(length & 0xFFu);
    frame[7] = (uint8_t)(length >> 8);
    memcpy(&frame[8], &sequence, 4);
    frame[12] = (uint8_t)(opcode & 0xFFu);
    frame[13] = (uint8_t)(opcode >> 8);
    frame[14] = (uint8_t)(payload_length & 0xFFu);
    frame[15] = (uint8_t)(payload_length >> 8);
    if (payload_length > 0u)
    {
        memcpy(&frame[16], payload, payload_length);
    }
    crc = axdr_command_crc32(frame, length - 4u);
    memcpy(&frame[length - 4], &crc, 4);

    tx_len = 0u;
    service_usb_rx(frame, length);
    service_usb_poll();
}

#define ack() tx_buf[14]
#define reason() tx_buf[15]
#define resp_payload() (&tx_buf[20])

static uint32_t rd_u32(const uint8_t *base, uint16_t offset)
{
    uint32_t value;
    memcpy(&value, &base[offset], 4);
    return value;
}
static float rd_f32(const uint8_t *base, uint16_t offset)
{
    float value;
    memcpy(&value, &base[offset], 4);
    return value;
}
static uint16_t rd_u16(const uint8_t *base, uint16_t offset)
{
    uint16_t value;
    memcpy(&value, &base[offset], 2);
    return value;
}

/* 结果样本填充：5 点对数频点，-3dB 跨越在 100→200Hz 之间（插值 150Hz） */
static void fill_done_result(void)
{
    static const float freq[5] = {20.0f, 50.0f, 100.0f, 200.0f, 400.0f};
    static const float gain[5] = {0.0f, 0.0f, -2.0f, -4.0f, -10.0f};
    uint32_t i;

    memset(&g_diag.sweep.result, 0, sizeof(g_diag.sweep.result));
    g_diag.sweep.result.completed_points = 5u;
    for (i = 0u; i < 5u; ++i)
    {
        g_diag.sweep.result.frequency_hz[i] = freq[i];
        g_diag.sweep.result.closed_magnitude_db[i] = gain[i];
        g_diag.sweep.result.closed_phase_deg[i] = -5.0f * (float)i;
        g_diag.sweep.result.open_magnitude_db[i] = gain[i] - 3.0f;
        g_diag.sweep.result.open_phase_deg[i] = -10.0f * (float)i;
        g_diag.sweep.result.valid[i] = 1u;
    }
}

int main(void)
{
    const diag_seed_t seed = {
        .control_period_s = 50.0e-6f,
        .phase_resistance_ohm = 0.162977806f,
        .d_axis_inductance_h = 0.000108778855f,
        .q_axis_inductance_h = 0.000112416135f,
        .flux_linkage_wb = 0.00498822471f,
        .pole_pairs = 10U,
        .encoder_full_scale = 16384U,
        .encoder_direction = 1,
    };
    axdr_scope_sweep_config_request_t cfg;
    axdr_scope_sweep_arm_request_t arm;
    uint8_t chunk_req[8];
    uint8_t dir_req[4];

    memset(&g_foc, 0, sizeof(g_foc));
    memset(&g_diag, 0, sizeof(g_diag));
    diag_runtime_init(&g_diag, &seed);
    g_diag.profile.current_limit_a = 1.0f;
    g_diag.profile.voltage_limit_v = 2.0f;
    g_diag.profile.minimum_vbus_v = 10.0f;
    g_foc.motor.Rs = 0.12f;
    g_foc.motor.Ld = 100.0e-6f;
    g_foc.motor.Lq = 110.0e-6f;
    service_usb_bind_tx(tx_stub);

    /* 1) 0x01：capability/mask 补位广播（转发后 OR，CRC 已由 core 重算） */
    send_frame(NULL, 0u, 1u, AXDR_OPCODE_GET_PROTOCOL_INFO);
    expect("proto reason", reason() == AXDR_REASON_NONE);
    expect("proto forwarded", forwarded_opcode == AXDR_OPCODE_GET_PROTOCOL_INFO);
    expect("proto cap17/18",
           (rd_u32(resp_payload(), 12u) &
            (AXDR_CAPABILITY_HIGH_RATE_USB_SCOPE |
             AXDR_CAPABILITY_SCOPE_SIGNAL_DIRECTORY)) ==
               (AXDR_CAPABILITY_HIGH_RATE_USB_SCOPE |
                AXDR_CAPABILITY_SCOPE_SIGNAL_DIRECTORY));
    expect("proto mask bit27-31",
           (rd_u32(resp_payload(), 16u) & 0xF8000000u) == 0xF8000000u);

    /* 2) 0x1B 合法配置：q 轴 20-1000Hz 11 点 0.2A（真门面暂存） */
    memset(&cfg, 0, sizeof(cfg));
    cfg.axis = AXDR_SCOPE_SWEEP_AXIS_Q;
    cfg.start_frequency_hz = 20.0f;
    cfg.end_frequency_hz = 1000.0f;
    cfg.requested_points = 11u;
    cfg.amplitude_a = 0.2f;
    cfg.offset_a = 0.0f;
    send_frame((const uint8_t *)&cfg, (uint16_t)sizeof(cfg), 2u,
               AXDR_OPCODE_CONFIGURE_SCOPE_STREAM);
    expect("cfg reason", reason() == AXDR_REASON_NONE);
    expect("cfg ack accepted", ack() == AXDR_ACK_ACCEPTED);
    expect("cfg resp len", tx_len == 20u + 23u + 4u); /* 头+payload+CRC */
    expect("cfg sweep_id", rd_u32(resp_payload(), 0u) == SWEEP_ID);
    expect("cfg echo end", nearf(rd_f32(resp_payload(), 8u), 1000.0f, 1e-6f));
    expect("cfg pending valid", g_diag.sweep_pending_valid == 1u);
    expect("cfg pending axis", g_diag.sweep_pending.axis == MC_AXIS_Q);
    expect("cfg pending points", g_diag.sweep_pending.requested_points == 11u);
    expect("cfg profile untouched",
           g_diag.profile.sweep.start_frequency_hz == DIAG_SWEEP_START_FREQ_HZ);

    /* 3) 0x1B 非法矩阵：坏轴 / 坏点数 / 超奈奎斯特 / 超限流 / 坏长度 */
    cfg.axis = 2u;
    send_frame((const uint8_t *)&cfg, (uint16_t)sizeof(cfg), 3u,
               AXDR_OPCODE_CONFIGURE_SCOPE_STREAM);
    expect("cfg bad axis", reason() == AXDR_REASON_INVALID_PAYLOAD);
    cfg.axis = AXDR_SCOPE_SWEEP_AXIS_Q;
    cfg.requested_points = 0u;
    send_frame((const uint8_t *)&cfg, (uint16_t)sizeof(cfg), 4u,
               AXDR_OPCODE_CONFIGURE_SCOPE_STREAM);
    expect("cfg zero points", reason() == AXDR_REASON_INVALID_PAYLOAD);
    cfg.requested_points = 11u;
    cfg.end_frequency_hz = 12000.0f; /* fs=20kHz，奈奎斯特 10kHz */
    send_frame((const uint8_t *)&cfg, (uint16_t)sizeof(cfg), 5u,
               AXDR_OPCODE_CONFIGURE_SCOPE_STREAM);
    expect("cfg beyond nyquist", reason() == AXDR_REASON_INVALID_PAYLOAD);
    cfg.end_frequency_hz = 1000.0f;
    cfg.offset_a = 0.9f; /* 0.9+0.2 > 限流 1.0 */
    send_frame((const uint8_t *)&cfg, (uint16_t)sizeof(cfg), 6u,
               AXDR_OPCODE_CONFIGURE_SCOPE_STREAM);
    expect("cfg over limit", reason() == AXDR_REASON_OUT_OF_RANGE);
    cfg.offset_a = 0.0f;
    send_frame((const uint8_t *)&cfg, (uint16_t)(sizeof(cfg) - 1u), 7u,
               AXDR_OPCODE_CONFIGURE_SCOPE_STREAM);
    expect("cfg bad length", reason() == AXDR_REASON_INVALID_PAYLOAD);
    expect("cfg failure keeps pending",
           (g_diag.sweep_pending_valid == 1u) &&
               (g_diag.sweep_pending.requested_points == 11u));

    /* 4) 0x1C ARM：门面受理（请求槽置位），applied 语义 */
    memset(&arm, 0, sizeof(arm));
    arm.session_id = 0u; /* 会话校验待 WIP 合入统一接，先透传 */
    send_frame((const uint8_t *)&arm, (uint16_t)sizeof(arm), 8u,
               AXDR_OPCODE_ARM_SCOPE_CAPTURE);
    expect("arm reason", reason() == AXDR_REASON_NONE);
    expect("arm ack applied", ack() == AXDR_ACK_APPLIED);
    expect("arm resp id", rd_u32(resp_payload(), 0u) == SWEEP_ID);
    expect("arm accepted", resp_payload()[4] == 1u);
    expect("arm request slot", g_diag.request == DIAG_REQUEST_START);
    expect("arm job", g_diag.requested_job == (uint32_t)DIAG_JOB_CURRENT_SWEEP);
    send_frame((const uint8_t *)&arm, (uint16_t)(sizeof(arm) - 1u), 9u,
               AXDR_OPCODE_ARM_SCOPE_CAPTURE);
    expect("arm bad length", reason() == AXDR_REASON_INVALID_PAYLOAD);

    /* 5) 请求槽占用期间 0x1B 忙拒；清槽后恢复 */
    send_frame((const uint8_t *)&cfg, (uint16_t)sizeof(cfg), 10u,
               AXDR_OPCODE_CONFIGURE_SCOPE_STREAM);
    expect("cfg busy while armed", reason() == AXDR_REASON_BUSY);
    g_diag.request = DIAG_REQUEST_NONE;

    /* 6) 0x1D 运行态：state=RUNNING、axis 回显 q（模拟 start 已消费
     * pending——快环把 config 拷进 sweep） */
    g_diag.active = true;
    g_diag.active_job = DIAG_JOB_CURRENT_SWEEP;
    g_diag.sweep.config.axis = MC_AXIS_Q;
    g_diag.sweep.active_frequency_hz = 123.4f;
    send_frame(NULL, 0u, 11u, AXDR_OPCODE_GET_SCOPE_CAPTURE_STATE);
    expect("run reason", reason() == AXDR_REASON_NONE);
    expect("run state", resp_payload()[0] == AXDR_SCOPE_SWEEP_RUNNING);
    expect("run axis", resp_payload()[1] == AXDR_SCOPE_SWEEP_AXIS_Q);
    g_diag.active = false;

    /* 7) 0x1D 完成态：带宽插值 150Hz + PI 建议（kp=Ld·ωc、ki=Rs·ωc） */
    g_diag.sweep.status = MC_DONE;
    g_diag.sweep.active_frequency_hz = 400.0f;
    fill_done_result();
    send_frame(NULL, 0u, 12u, AXDR_OPCODE_GET_SCOPE_CAPTURE_STATE);
    expect("done state", resp_payload()[0] == AXDR_SCOPE_SWEEP_DONE);
    expect("done completed", rd_u16(resp_payload(), 2u) == 5u);
    expect("done valid", rd_u16(resp_payload(), 4u) == 5u);
    /* 偏移表：active_freq@8 closed_db@12 closed_deg@16 bandwidth@20
     * kp_d@24 ki_d@28 kp_q@32 ki_q@36（40B packed） */
    expect("done bandwidth",
           nearf(rd_f32(resp_payload(), 20u), 150.0f, 0.1f));
    expect("done kp_d",
           nearf(rd_f32(resp_payload(), 24u),
                 100.0e-6f * MC_TWO_PI_F * 150.0f, 1e-3f));
    expect("done ki_d",
           nearf(rd_f32(resp_payload(), 28u), 0.12f * MC_TWO_PI_F * 150.0f,
                 1e-2f));
    expect("done kp_q>kp_d",
           rd_f32(resp_payload(), 32u) > rd_f32(resp_payload(), 24u));

    /* 8) 0x1E chunk：错 id / 未完成 BUSY / 越界 / 全量 / 末块 */
    memset(chunk_req, 0, sizeof(chunk_req));
    memcpy(chunk_req, &(uint32_t){0xDEADu}, 4);
    send_frame(chunk_req, (uint16_t)sizeof(chunk_req), 13u,
               AXDR_OPCODE_READ_SCOPE_CAPTURE_CHUNK);
    expect("chunk bad id", reason() == AXDR_REASON_INVALID_PAYLOAD);
    memcpy(chunk_req, &(uint32_t){SWEEP_ID}, 4);
    g_diag.sweep.status = MC_BUSY;
    send_frame(chunk_req, (uint16_t)sizeof(chunk_req), 14u,
               AXDR_OPCODE_READ_SCOPE_CAPTURE_CHUNK);
    expect("chunk busy", reason() == AXDR_REASON_BUSY);
    g_diag.sweep.status = MC_DONE;
    memcpy(&chunk_req[4], &(uint32_t){999u * 24u}, 4);
    send_frame(chunk_req, (uint16_t)sizeof(chunk_req), 15u,
               AXDR_OPCODE_READ_SCOPE_CAPTURE_CHUNK);
    expect("chunk offset oob", reason() == AXDR_REASON_OUT_OF_RANGE);

    /* 8 点样本：total=192，两块读出（120B+72B），末块 flags=1 */
    g_diag.sweep.result.completed_points = 8u;
    memcpy(&chunk_req[4], &(uint32_t){0u}, 4);
    send_frame(chunk_req, (uint16_t)sizeof(chunk_req), 16u,
               AXDR_OPCODE_READ_SCOPE_CAPTURE_CHUNK);
    expect("chunk0 reason", reason() == AXDR_REASON_NONE);
    expect("chunk0 total", rd_u32(resp_payload(), 4u) == 192u);
    expect("chunk0 offset", rd_u32(resp_payload(), 8u) == 0u);
    expect("chunk0 len", rd_u16(resp_payload(), 12u) == 120u);
    expect("chunk0 flags", rd_u16(resp_payload(), 14u) == 0u);
    expect("chunk0 point0 freq",
           nearf(rd_f32(resp_payload(), 16u), 20.0f, 1e-6f));
    expect("chunk0 point0 valid", resp_payload()[16u + 20u] == 1u);
    expect("chunk0 point4 gain",
           nearf(rd_f32(resp_payload(), 16u + 4u * 24u + 4u), -10.0f, 1e-6f));
    memcpy(&chunk_req[4], &(uint32_t){120u}, 4);
    send_frame(chunk_req, (uint16_t)sizeof(chunk_req), 17u,
               AXDR_OPCODE_READ_SCOPE_CAPTURE_CHUNK);
    expect("chunk1 reason", reason() == AXDR_REASON_NONE);
    expect("chunk1 offset", rd_u32(resp_payload(), 8u) == 120u);
    expect("chunk1 len", rd_u16(resp_payload(), 12u) == 72u);
    expect("chunk1 last flags", rd_u16(resp_payload(), 14u) == 1u);
    /* 跨块一致性：offset 120 = 点 5 起始，频点仍取样本表（5 点样本外为 0） */
    expect("chunk1 point5 zero",
           nearf(rd_f32(resp_payload(), 16u), 0.0f, 1e-8f));

    /* 9) 0x1F 目录：首页 2 条 / 尾页 2 条 / 越界空页 */
    memset(dir_req, 0, sizeof(dir_req));
    dir_req[2] = 2u; /* max_entries=2（页上限） */
    send_frame(dir_req, (uint16_t)sizeof(dir_req), 18u,
               AXDR_OPCODE_GET_SCOPE_SIGNAL_DIRECTORY);
    expect("dir reason", reason() == AXDR_REASON_NONE);
    expect("dir schema", rd_u16(resp_payload(), 0u) ==
                             AXDR_SCOPE_SIGNAL_DIRECTORY_VERSION);
    expect("dir total", rd_u16(resp_payload(), 2u) ==
                            AXDR_SCOPE_SWEEP_SIGNAL_COUNT);
    /* 目录头：schema@0 total@2 start@4 returned@6 desc_size@7 */
    expect("dir returned", resp_payload()[6] == 2u);
    expect("dir desc size", resp_payload()[7] == 64u);
    expect("dir ch1", rd_u16(resp_payload(), 8u) == 1u);
    expect("dir key1",
           memcmp(&resp_payload()[8u + 16u], "freq_hz", 8u) == 0);
    expect("dir key2",
           memcmp(&resp_payload()[8u + 64u + 16u], "closed_db", 10u) == 0);
    dir_req[0] = 4u; /* start_index=4 */
    dir_req[2] = 8u; /* max=8 → 截到页上限 2 */
    send_frame(dir_req, (uint16_t)sizeof(dir_req), 19u,
               AXDR_OPCODE_GET_SCOPE_SIGNAL_DIRECTORY);
    expect("dir tail reason", reason() == AXDR_REASON_NONE);
    expect("dir tail returned", resp_payload()[6] == 2u);
    expect("dir tail ch5", rd_u16(resp_payload(), 8u) == 5u);
    expect("dir tail ch6", rd_u16(resp_payload(), 8u + 64u) == 6u);
    dir_req[0] = 6u; /* start=6 == COUNT → 空页（翻页终止条件） */
    send_frame(dir_req, (uint16_t)sizeof(dir_req), 20u,
               AXDR_OPCODE_GET_SCOPE_SIGNAL_DIRECTORY);
    expect("dir empty reason", reason() == AXDR_REASON_NONE);
    expect("dir empty returned", resp_payload()[6] == 0u);

    /* 10) 段外 opcode 原样转发（替身记录），reason 透传 */
    send_frame(NULL, 0u, 21u, 0x0040u);
    expect("route forwarded 0x40", forwarded_opcode == 0x0040u);
    expect("route passthrough reason", reason() == AXDR_REASON_NONE);

    if (fail_count != 0)
    {
        (void)printf("service scope tests: %d FAIL\n", fail_count);
        return 1;
    }
    puts("service scope protocol tests: PASS");
    return 0;
}
