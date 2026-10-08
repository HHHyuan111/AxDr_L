/**
 * @file service_scope.c
 * @brief 扫频测量会话协议层：配置 0x1B → 启动 0x1C → 状态 0x1D（含
 *        带宽与 PI 建议）→ 结果分块 0x1E → 信号目录 0x1F。
 *
 * 诊断任务本体在 diag_runtime（门面早拒）与快环 drive_diag_poll_request
 * （STOP/PWM/故障权威门禁）；本层只做 payload 解析/组包与字节流切片。
 */

#include "service_scope.h"

#include <string.h>

#include "axdr_command_contract.h"
#include "axdr_scope_contract.h"
#include "common.h"
#include "diag_runtime.h"
#include "drive_diag.h"
#include "service_command.h"

/* 单会话句柄：无多会话管理，固定值供宿主做请求/应答一致性校验 */
#define SCOPE_SWEEP_ID 0x53574550u /* "PWSE" 小端 */

/* ---- 本地拷入/拷出帮手（与 service_command.c 同款；WIP 合入迁移后去重） */
static uint8_t scope_payload_out(const void *src, uint16_t src_size,
                                 uint8_t *response_payload,
                                 uint16_t response_payload_capacity,
                                 uint16_t *response_payload_length)
{
    if (src_size > response_payload_capacity)
    {
        return AXDR_REASON_INVALID_LENGTH;
    }
    memcpy(response_payload, src, src_size);
    *response_payload_length = src_size;
    return AXDR_REASON_NONE;
}

static const void *scope_payload_in(const uint8_t *request_payload,
                                    uint16_t request_payload_length,
                                    uint16_t expected_size)
{
    if (request_payload_length != expected_size)
    {
        return NULL;
    }
    return (const void *)request_payload;
}

/* mc 门面状态 → 协议 reason */
static uint8_t scope_reason(mc_status_t status)
{
    switch (status)
    {
    case MC_OK:
        return AXDR_REASON_NONE;
    case MC_INVALID_ARGUMENT:
        return AXDR_REASON_INVALID_PAYLOAD;
    case MC_OUT_OF_RANGE:
        return AXDR_REASON_OUT_OF_RANGE;
    case MC_REJECTED:
    case MC_BUSY:
        return AXDR_REASON_BUSY;
    default:
        return AXDR_REASON_INVALID_STATE;
    }
}

static uint8_t sweep_running(const diag_runtime_t *diag)
{
    return ((diag->active != false) &&
            (diag->active_job == DIAG_JOB_CURRENT_SWEEP)) ? 1u : 0u;
}

/* ---- 0x1B CONFIGURE：配置暂存（settle/measure 等沿 profile 预设） */
static uint8_t sweep_handle_configure(const uint8_t *request_payload,
                                      uint16_t request_payload_length,
                                      uint8_t *response_payload,
                                      uint16_t response_payload_capacity,
                                      uint16_t *response_payload_length,
                                      uint8_t *ack_state)
{
    const axdr_scope_sweep_config_request_t *req = scope_payload_in(
        request_payload, request_payload_length,
        (uint16_t)sizeof(axdr_scope_sweep_config_request_t));
    axdr_scope_sweep_config_payload_t out;
    mc_current_sweep_config_t cfg;
    uint8_t reason;

    if (req == NULL)
    {
        return AXDR_REASON_INVALID_PAYLOAD;
    }
    if ((req->axis != AXDR_SCOPE_SWEEP_AXIS_D) &&
        (req->axis != AXDR_SCOPE_SWEEP_AXIS_Q))
    {
        return AXDR_REASON_INVALID_PAYLOAD;
    }

    cfg = g_diag.profile.sweep;
    cfg.single_point = (req->requested_points == 1u) ? true : false;
    cfg.axis = (req->axis == AXDR_SCOPE_SWEEP_AXIS_Q) ? MC_AXIS_Q : MC_AXIS_D;
    cfg.single_frequency_hz = req->start_frequency_hz;
    cfg.start_frequency_hz = req->start_frequency_hz;
    cfg.end_frequency_hz = (cfg.single_point != false)
                               ? req->start_frequency_hz
                               : req->end_frequency_hz;
    cfg.requested_points = req->requested_points;
    cfg.amplitude_a = req->amplitude_a;
    cfg.offset_a = req->offset_a;
    /* safe_current_limit_a 不经协议：start 消费时强制取 profile 限值 */

    reason = scope_reason(diag_runtime_configure_sweep(&g_diag, &cfg));
    if (reason != AXDR_REASON_NONE)
    {
        return reason;
    }

    memset(&out, 0, sizeof(out));
    out.sweep_id = SCOPE_SWEEP_ID;
    out.start_frequency_hz = req->start_frequency_hz;
    out.end_frequency_hz = cfg.end_frequency_hz;
    out.requested_points = req->requested_points;
    out.amplitude_a = req->amplitude_a;
    out.offset_a = req->offset_a;
    out.axis = req->axis;
    *ack_state = AXDR_ACK_ACCEPTED;
    return scope_payload_out(&out, (uint16_t)sizeof(out), response_payload,
                             response_payload_capacity,
                             response_payload_length);
}

/* ---- 0x1C ARM：经门面写启动请求，快环在 STOP/无故障时消费。
 * session_id 的安全会话校验待 WIP 合入后与 0x13 IDENTIFY 统一接
 * axdr_safety_runtime_authorize_run（同 0x12 先例），包结构已预留。 */
static uint8_t sweep_handle_arm(const uint8_t *request_payload,
                                uint16_t request_payload_length,
                                uint8_t *response_payload,
                                uint16_t response_payload_capacity,
                                uint16_t *response_payload_length,
                                uint8_t *ack_state)
{
    const axdr_scope_sweep_arm_request_t *req = scope_payload_in(
        request_payload, request_payload_length,
        (uint16_t)sizeof(axdr_scope_sweep_arm_request_t));
    axdr_scope_sweep_arm_payload_t out;
    uint8_t reason;

    if (req == NULL)
    {
        return AXDR_REASON_INVALID_PAYLOAD;
    }
    reason = scope_reason(
        diag_runtime_request_start(&g_diag, DIAG_JOB_CURRENT_SWEEP));
    if (reason != AXDR_REASON_NONE)
    {
        return reason;
    }

    memset(&out, 0, sizeof(out));
    out.sweep_id = SCOPE_SWEEP_ID;
    out.accepted = 1u;
    *ack_state = AXDR_ACK_APPLIED;
    return scope_payload_out(&out, (uint16_t)sizeof(out), response_payload,
                             response_payload_capacity,
                             response_payload_length);
}

/* ---- 0x1D STATE：状态+进度+末点读数+带宽+PI 建议（一次轮询全带出） */
static uint8_t sweep_handle_state(const uint8_t *request_payload,
                                  uint16_t request_payload_length,
                                  uint8_t *response_payload,
                                  uint16_t response_payload_capacity,
                                  uint16_t *response_payload_length,
                                  uint8_t *ack_state)
{
    const mc_current_sweep_t *sw = &g_diag.sweep;
    axdr_scope_sweep_state_payload_t out;
    uint32_t i;
    uint32_t valid_points = 0u;
    uint32_t last_valid = MC_SWEEP_MAX_POINTS; /* 哨兵：暂无有效点 */
    float bandwidth;

    if (request_payload_length != 0u)
    {
        return AXDR_REASON_INVALID_PAYLOAD;
    }

    memset(&out, 0, sizeof(out));
    if (sweep_running(&g_diag) != 0u)
    {
        out.state = AXDR_SCOPE_SWEEP_RUNNING;
    }
    else if (sw->status == MC_DONE)
    {
        out.state = AXDR_SCOPE_SWEEP_DONE;
    }
    else if (sw->status == MC_ABORTED)
    {
        out.state = AXDR_SCOPE_SWEEP_ABORTED;
    }
    else
    {
        out.state = AXDR_SCOPE_SWEEP_IDLE;
    }

    for (i = 0u; i < sw->result.completed_points; ++i)
    {
        if (sw->result.valid[i] == 0u)
        {
            continue;
        }
        valid_points++;
        last_valid = i;
    }

    out.axis = (sw->config.axis == MC_AXIS_Q) ? AXDR_SCOPE_SWEEP_AXIS_Q
                                              : AXDR_SCOPE_SWEEP_AXIS_D;
    out.completed_points = (uint16_t)sw->result.completed_points;
    out.valid_points = (uint16_t)valid_points;
    out.active_frequency_hz = sw->active_frequency_hz;
    if (last_valid < MC_SWEEP_MAX_POINTS)
    {
        out.closed_magnitude_db = sw->result.closed_magnitude_db[last_valid];
        out.closed_phase_deg = sw->result.closed_phase_deg[last_valid];
    }

    bandwidth = mc_sweep_bandwidth_hz(&sw->result);
    out.bandwidth_hz = bandwidth;
    /* PI 建议仅在完成且带宽有效时给出；电机参数取运行档案。
     * packed 成员不可取址，先落局部再回填。 */
    if ((out.state == AXDR_SCOPE_SWEEP_DONE) && (bandwidth > 0.0f))
    {
        float kp_d;
        float ki_d;
        float kp_q;
        float ki_q;

        (void)diag_current_pi_for_project(
            bandwidth, g_diag.profile.control_period_s, g_foc.motor.Rs,
            g_foc.motor.Ld, g_foc.motor.Lq, &kp_d, &ki_d, &kp_q, &ki_q);
        out.kp_d = kp_d;
        out.ki_d_per_s = ki_d;
        out.kp_q = kp_q;
        out.ki_q_per_s = ki_q;
    }
    (void)request_payload;
    *ack_state = AXDR_ACK_ACCEPTED;
    return scope_payload_out(&out, (uint16_t)sizeof(out), response_payload,
                             response_payload_capacity,
                             response_payload_length);
}

/* ---- 0x1E CHUNK：结果按 24B/点字节流在线切片，不落中间缓冲。
 * 布局 = 5 float 小端（freq/closed_db/closed_deg/open_db/open_deg）
 * + valid(1) + 保留(3)。仅 MC_DONE 后可读——结果零撕裂，无 seqlock。 */
static uint8_t sweep_stream_byte(const mc_current_sweep_result_t *result,
                                 uint32_t index)
{
    const float *fields[5];
    const uint32_t point = index / AXDR_SCOPE_SWEEP_POINT_STRIDE;
    const uint32_t intra = index % AXDR_SCOPE_SWEEP_POINT_STRIDE;
    float value;
    uint32_t bits;

    if (point >= result->completed_points)
    {
        return 0u;
    }
    if (intra < 20u)
    {
        fields[0] = result->frequency_hz;
        fields[1] = result->closed_magnitude_db;
        fields[2] = result->closed_phase_deg;
        fields[3] = result->open_magnitude_db;
        fields[4] = result->open_phase_deg;
        value = fields[intra >> 2][point];
        memcpy(&bits, &value, sizeof(bits));
        return (uint8_t)(bits >> ((intra & 3u) * 8u));
    }
    return (intra == 20u) ? result->valid[point] : 0u;
}

static uint8_t sweep_handle_chunk(const uint8_t *request_payload,
                                  uint16_t request_payload_length,
                                  uint8_t *response_payload,
                                  uint16_t response_payload_capacity,
                                  uint16_t *response_payload_length,
                                  uint8_t *ack_state)
{
    const axdr_scope_capture_read_request_t *req = scope_payload_in(
        request_payload, request_payload_length,
        (uint16_t)sizeof(axdr_scope_capture_read_request_t));
    axdr_scope_capture_chunk_payload_t out;
    const mc_current_sweep_result_t *result = &g_diag.sweep.result;
    uint32_t total_bytes;
    uint32_t remain;
    uint16_t copy_len;
    uint32_t i;

    if (req == NULL)
    {
        return AXDR_REASON_INVALID_PAYLOAD;
    }
    if (req->capture_id != SCOPE_SWEEP_ID)
    {
        return AXDR_REASON_INVALID_PAYLOAD;
    }
    if (g_diag.sweep.status != MC_DONE)
    {
        return AXDR_REASON_BUSY;
    }

    total_bytes = (uint32_t)result->completed_points *
                  AXDR_SCOPE_SWEEP_POINT_STRIDE;
    if ((req->byte_offset >= total_bytes) || (total_bytes == 0u))
    {
        return AXDR_REASON_OUT_OF_RANGE;
    }

    remain = total_bytes - req->byte_offset;
    copy_len = (remain < AXDR_SCOPE_CAPTURE_CHUNK_DATA_SIZE)
                   ? (uint16_t)remain
                   : (uint16_t)AXDR_SCOPE_CAPTURE_CHUNK_DATA_SIZE;

    memset(&out, 0, sizeof(out));
    out.capture_id = SCOPE_SWEEP_ID;
    out.total_bytes = total_bytes;
    out.byte_offset = req->byte_offset;
    out.payload_length = copy_len;
    out.flags = (((uint32_t)req->byte_offset + (uint32_t)copy_len) >=
                 total_bytes) ? 1u : 0u; /* bit0=末块 */
    for (i = 0u; i < (uint32_t)copy_len; ++i)
    {
        out.data[i] = sweep_stream_byte(result, req->byte_offset + i);
    }
    *ack_state = AXDR_ACK_ACCEPTED;
    return scope_payload_out(&out, (uint16_t)sizeof(out), response_payload,
                             response_payload_capacity,
                             response_payload_length);
}

/* ---- 0x1F DIRECTORY：扫频结果信号目录（6 条，2 条/页；空页=翻页终止） */
static uint8_t sweep_handle_directory(const uint8_t *request_payload,
                                      uint16_t request_payload_length,
                                      uint8_t *response_payload,
                                      uint16_t response_payload_capacity,
                                      uint16_t *response_payload_length,
                                      uint8_t *ack_state)
{
    static const struct
    {
        const char *key;
        const char *name;
        axdr_scope_unit_t unit;
        uint8_t decimals;
        float minimum;
        float maximum;
    } signals[AXDR_SCOPE_SWEEP_SIGNAL_COUNT] = {
        { "freq_hz",    "frequency",     AXDR_SCOPE_UNIT_HERTZ,   1u,
          0.0f,    10000.0f },
        { "closed_db",  "closed gain",   AXDR_SCOPE_UNIT_DECIBEL, 2u,
          -60.0f,  20.0f },
        { "closed_deg", "closed phase",  AXDR_SCOPE_UNIT_DEGREE,  1u,
          -180.0f, 180.0f },
        { "open_db",    "open gain est", AXDR_SCOPE_UNIT_DECIBEL, 2u,
          -60.0f,  20.0f },
        { "open_deg",   "open phase est", AXDR_SCOPE_UNIT_DEGREE, 1u,
          -180.0f, 180.0f },
        { "valid",      "point valid",   AXDR_SCOPE_UNIT_NONE,    0u,
          0.0f,    1.0f },
    };
    const axdr_scope_signal_directory_request_t *req = scope_payload_in(
        request_payload, request_payload_length,
        (uint16_t)sizeof(axdr_scope_signal_directory_request_t));
    axdr_scope_signal_directory_payload_t out;
    uint16_t start;
    uint8_t returned;
    uint8_t i;

    if (req == NULL)
    {
        return AXDR_REASON_INVALID_PAYLOAD;
    }

    memset(&out, 0, sizeof(out));
    out.schema_version = AXDR_SCOPE_SIGNAL_DIRECTORY_VERSION;
    out.total_entries = AXDR_SCOPE_SWEEP_SIGNAL_COUNT;
    out.descriptor_size = (uint8_t)sizeof(axdr_scope_signal_descriptor_t);

    start = req->start_index;
    out.start_index = start;
    returned = 0u;
    if ((start < AXDR_SCOPE_SWEEP_SIGNAL_COUNT) && (req->max_entries != 0u))
    {
        returned = req->max_entries;
        if (returned > AXDR_SCOPE_SIGNAL_DIRECTORY_PAGE_ENTRIES)
        {
            returned = AXDR_SCOPE_SIGNAL_DIRECTORY_PAGE_ENTRIES;
        }
        if (returned > (uint8_t)(AXDR_SCOPE_SWEEP_SIGNAL_COUNT - start))
        {
            returned = (uint8_t)(AXDR_SCOPE_SWEEP_SIGNAL_COUNT - start);
        }
    }
    for (i = 0u; i < returned; ++i)
    {
        axdr_scope_signal_descriptor_t *d = &out.entries[i];
        const uint16_t index = (uint16_t)(start + i);
        const size_t key_len = strlen(signals[index].key);
        const size_t name_len = strlen(signals[index].name);

        d->channel_id = (uint16_t)(index + 1u);
        d->sampling_modes = 0u; /* 扫频目录非示波器通道 */
        d->decimals = signals[index].decimals;
        d->unit = (uint8_t)signals[index].unit;
        d->suggested_minimum = signals[index].minimum;
        d->suggested_maximum = signals[index].maximum;
        if (key_len < sizeof(d->key))
        {
            memcpy(d->key, signals[index].key, key_len);
        }
        if (name_len < sizeof(d->name))
        {
            memcpy(d->name, signals[index].name, name_len);
        }
    }
    out.returned_entries = returned;
    *ack_state = AXDR_ACK_ACCEPTED;
    return scope_payload_out(&out, (uint16_t)sizeof(out), response_payload,
                             response_payload_capacity,
                             response_payload_length);
}

static uint8_t sweep_dispatch(uint16_t opcode, const uint8_t *request_payload,
                              uint16_t request_payload_length,
                              uint8_t *response_payload,
                              uint16_t response_payload_capacity,
                              uint16_t *response_payload_length,
                              uint8_t *ack_state)
{
    switch (opcode)
    {
    case AXDR_OPCODE_CONFIGURE_SCOPE_STREAM:
        return sweep_handle_configure(request_payload,
                                      request_payload_length,
                                      response_payload,
                                      response_payload_capacity,
                                      response_payload_length, ack_state);
    case AXDR_OPCODE_ARM_SCOPE_CAPTURE:
        return sweep_handle_arm(request_payload, request_payload_length,
                                response_payload, response_payload_capacity,
                                response_payload_length, ack_state);
    case AXDR_OPCODE_GET_SCOPE_CAPTURE_STATE:
        return sweep_handle_state(request_payload, request_payload_length,
                                  response_payload,
                                  response_payload_capacity,
                                  response_payload_length, ack_state);
    case AXDR_OPCODE_READ_SCOPE_CAPTURE_CHUNK:
        return sweep_handle_chunk(request_payload, request_payload_length,
                                  response_payload,
                                  response_payload_capacity,
                                  response_payload_length, ack_state);
    case AXDR_OPCODE_GET_SCOPE_SIGNAL_DIRECTORY:
        return sweep_handle_directory(request_payload, request_payload_length,
                                      response_payload,
                                      response_payload_capacity,
                                      response_payload_length, ack_state);
    default:
        return AXDR_REASON_UNKNOWN_OPCODE;
    }
}

uint8_t service_scope_route(void *context, uint16_t opcode,
                            const uint8_t *request_payload,
                            uint16_t request_payload_length,
                            uint8_t *response_payload,
                            uint16_t response_payload_capacity,
                            uint16_t *response_payload_length,
                            uint8_t *ack_state)
{
    uint8_t reason;

    if ((opcode >= AXDR_OPCODE_CONFIGURE_SCOPE_STREAM) &&
        (opcode <= AXDR_OPCODE_GET_SCOPE_SIGNAL_DIRECTORY))
    {
        return sweep_dispatch(opcode, request_payload, request_payload_length,
                              response_payload, response_payload_capacity,
                              response_payload_length, ack_state);
    }

    reason = service_command_dispatch(context, opcode, request_payload,
                                      request_payload_length,
                                      response_payload,
                                      response_payload_capacity,
                                      response_payload_length, ack_state);

    /* GET_PROTOCOL_INFO 应答补扫频 capability/mask：拦截发生在组帧前，
     * CRC 由 core 按改后 payload 重算。WIP 合入后迁回 service_command.c。 */
    if ((opcode == AXDR_OPCODE_GET_PROTOCOL_INFO) &&
        (reason == AXDR_REASON_NONE) &&
        (*response_payload_length >= sizeof(axdr_protocol_info_payload_t)))
    {
        axdr_protocol_info_payload_t info;
        memcpy(&info, response_payload, sizeof(info));
        info.capability_flags |= AXDR_CAPABILITY_HIGH_RATE_USB_SCOPE |
                                 AXDR_CAPABILITY_SCOPE_SIGNAL_DIRECTORY;
        info.supported_opcode_mask |= 0xF8000000u; /* bit27-31 = 0x1B..0x1F */
        memcpy(response_payload, &info, sizeof(info));
    }
    return reason;
}
