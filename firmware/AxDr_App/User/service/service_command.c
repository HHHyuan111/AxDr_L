/**
 * @file service_command.c
 * @brief S3/S3.5：15 opcode 分发器实现——连通/使能链/控制/诊断四组。
 *
 * 数据来源（值的家）：协议常量→axdr_command_contract.h；电机档案→motor_config.h；
 * 运行时状态→g_foc；链路计数→service_usb_stats。安全链会话状态在
 * service_safety_runtime() 单例。
 *
 * S3.5 修复（审查 F1-F5）：
 *  F1 运动授权链——SetSpeed/SetCurrent 先 authorize_run 验会话再应用，并记录
 *    命令租约（lease_ms）；poll 租约超时 → revoke+停机（fail-closed），
 *    stop_reason 按 B 契约 axdr_control_stop_reason_t 填写；
 *  F2 档案 CRC——档案 payload 只构造一次，CRC32 指纹（crc 字段清零、
 *    status_flags 排除）同时喂给 configure 注册与 GetMotorProfile 应答；
 *  F3 会话门卫（方案A）——首次 USB ARM 成功前 poll 不执法（台架模式，
 *    调试器手动运行不会被 observe 判成 FAULT_LOCKED）；首次 ARM 后全程执法；
 *  F4 删除 service_safety_tick（防快环误用）；
 *  F5 负值/范围校验补齐。
 *
 * 时钟：全服务统一用 fast_seq/20（20kHz→ms，单调；回绕用减法比较）。
 */

#include "service_command.h"

#include <string.h>

#include "axdr_command_core.h"
#include "common.h"
#include "motor_config.h"
#include "encoder_type.h"

/* 档案版本（B 库 chensha_config L28=rev2；主线宏暂未定义，本地别名，S5 收编） */
#define CHENSHA_ARCHIVE_REVISION 2u
#include "service_usb.h"

/* ---- 板级/固件身份（上位机 GetDeviceInfo/GetProtocolInfo 匹配用） ---- */
#define SVC_FW_MAJOR 0u
#define SVC_FW_MINOR 7u
#define SVC_FW_PATCH 0u
#define SVC_DEVICE_FAMILY 0x00000001u /* 沉沙/AxDrive-L 驱动 */
#define SVC_BOARD_PROFILE 0x00000003u /* hw_rev v1.3 */
#define SVC_PRODUCT_NAME "AxDrService"

/* 本版支持的 opcode 掩码（15 个，契约 bit 位 = opcode 值，最大 0x1A < 32） */
#define SVC_SUPPORTED_MASK                                                  \
    ((1u << AXDR_OPCODE_GET_PROTOCOL_INFO) |                               \
     (1u << AXDR_OPCODE_GET_DEVICE_INFO) |                                 \
     (1u << AXDR_OPCODE_GET_RUNTIME_STATE) |                               \
     (1u << AXDR_OPCODE_GET_SAFETY_STATE) |                                \
     (1u << AXDR_OPCODE_SAFETY_ARM) |                                      \
     (1u << AXDR_OPCODE_SAFETY_DISARM) |                                   \
     (1u << AXDR_OPCODE_SAFETY_HEARTBEAT) |                                \
     (1u << AXDR_OPCODE_GET_MOTOR_PROFILE) |                               \
     (1u << AXDR_OPCODE_GET_CONTROL_LIMITS) |                              \
     (1u << AXDR_OPCODE_CONFIRM_MOTOR_PROFILE) |                            \
     (1u << AXDR_OPCODE_GET_LINK_DIAGNOSTICS) |                            \
     (1u << AXDR_OPCODE_GET_CONTROL_STATE) |                                \
     (1u << AXDR_OPCODE_CONTROL_STOP) |                                     \
     (1u << AXDR_OPCODE_SET_SPEED) |                                        \
     (1u << AXDR_OPCODE_SET_CURRENT))

/* ---- 安全会话单例 ---- */
static axdr_safety_runtime_t safety;

/* F3 会话门卫：首次 ARM 成功后置位，poll 由此开始执法 */
static uint8_t session_ever_active;

/* F1 命令租约（B 契约 control_state 的 command_deadline/applied/timeout） */
static uint32_t command_deadline_ms;
static uint8_t command_active;
static uint32_t applied_count;
static uint32_t lease_timeout_count;
static uint8_t last_stop_reason = AXDR_CONTROL_STOP_NONE;

/* F2 档案 payload：只构造一次，CRC 注册与应答同源 */
static axdr_motor_profile_payload_t profile_payload;
static uint8_t profile_built;
static uint8_t profile_configured;

/* 档案版本（B 库 chensha_config rev2）与档案 id */
#define SVC_PROFILE_ID 1u

/* ---- 内部工具 ---- */

/* 全服务统一时钟：fast_seq @20kHz → ms */
static uint32_t now_ms(void)
{
    return g_foc.fast_seq / 20u;
}

static void build_profile_once(void)
{
    axdr_motor_profile_payload_t tmp;

    if (profile_built != 0u)
    {
        return;
    }
    memset(&profile_payload, 0, sizeof(profile_payload));
    profile_payload.schema_version = 1u;
    profile_payload.payload_size = (uint16_t)sizeof(profile_payload);
    profile_payload.profile_id = SVC_PROFILE_ID;
    profile_payload.revision = CHENSHA_ARCHIVE_REVISION;
    profile_payload.encoder_type = ENCODER_TYPE_ABZ;
    profile_payload.pole_pairs = (uint8_t)CHENSHA_POLE_PAIRS;
    profile_payload.name_length = 10u;
    memcpy(profile_payload.name, "chensha200", 10u);
    profile_payload.phase_resistance_ohm = CHENSHA_PHASE_RESISTANCE_OHM;
    profile_payload.d_axis_inductance_H = CHENSHA_D_AXIS_INDUCTANCE_H;
    profile_payload.q_axis_inductance_H = CHENSHA_Q_AXIS_INDUCTANCE_H;
    profile_payload.flux_linkage_Wb = CHENSHA_FLUX_LINKAGE_WB;
    profile_payload.continuous_current_A = CHENSHA_RATED_CURRENT_A;
    profile_payload.peak_current_A = CHENSHA_PEAK_CURRENT_A;
    profile_payload.maximum_speed_rad_s = CHENSHA_PEAK_SPEED_RAD_S;
    profile_payload.maximum_acceleration_rad_s2 = CHENSHA_ACCEL_LIMIT_RAD_S2;
    profile_payload.minimum_bus_voltage_V = CHENSHA_UNDER_VOLTAGE_V;
    profile_payload.maximum_bus_voltage_V = CHENSHA_OVER_VOLTAGE_V;

    /* CRC 指纹（契约 §motor_profile）：profile_crc32 字段清零参与计算；
     * status_flags 为信息字段不参与——本字段恒为 0，临时置零后整体计算
     * 与逐段排除等价，且对未来 status_flags 变化免疫。 */
    tmp = profile_payload;
    tmp.profile_crc32 = 0u;
    tmp.status_flags = 0u;
    profile_payload.profile_crc32 =
        axdr_command_crc32((const uint8_t *)&tmp, (uint32_t)sizeof(tmp));
    profile_built = 1u;
}

/* 首次调用时把固件档案三元组注册到安全运行时（上位机 Confirm 比对的基准） */
static void ensure_profile_configured(void)
{
    if (profile_configured == 0u)
    {
        build_profile_once();
        axdr_safety_runtime_init(&safety, 0u, 0x20261004u);
        axdr_safety_runtime_configure_motor_profile(
            &safety, SVC_PROFILE_ID, CHENSHA_ARCHIVE_REVISION,
            profile_payload.profile_crc32);
        profile_configured = 1u;
    }
}

void service_safety_poll(void)
{
    const uint32_t now = now_ms();

    if (session_ever_active == 0u)
    {
        return; /* F3 台架模式：USB 会话从未建立，不执法（调试器手动运行不受影响） */
    }
    ensure_profile_configured();

    /* 互锁观察：fault 全零 + PWM 未激活（相输出关断）+ 栅极拉低（PWM 停=低） */
    {
        const uint8_t outputs_disabled = (g_foc.pwm_active == 0u) ? 1u : 0u;
        axdr_safety_runtime_observe(&safety, now, g_foc.fault.all,
                                    outputs_disabled, outputs_disabled);
    }
    axdr_safety_runtime_tick(&safety, now);

    /* F1 命令租约超时：撤销运行授权（随后由下方 fail-closed 停机） */
    if ((command_active != 0u) &&
        ((int32_t)(now - command_deadline_ms) >= 0))
    {
        lease_timeout_count++;
        command_active = 0u;
        last_stop_reason = AXDR_CONTROL_STOP_LEASE_TIMEOUT;
        axdr_safety_runtime_revoke_run(&safety, now);
    }

    /* 撤权后运动请求残留 → 强制停机（fail-closed） */
    if (!axdr_safety_runtime_run_authorized(&safety) &&
        ((g_foc.req == DRIVE_REQ_RUN) || (g_foc.req == DRIVE_REQ_START)))
    {
        g_foc.req = DRIVE_REQ_STOP;
    }
}

uint8_t service_safety_run_authorized(void)
{
    ensure_profile_configured();
    return axdr_safety_runtime_run_authorized(&safety);
}

axdr_safety_runtime_t *service_safety_runtime(void)
{
    return &safety;
}

/* ---- 分发器内部：安全地拷入/拷出 packed 结构 ---- */
static uint8_t payload_out(const void *src, uint16_t src_size,
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

static const void *payload_in(const uint8_t *request_payload,
                              uint16_t request_payload_length,
                              uint16_t expected_size)
{
    if (request_payload_length != expected_size)
    {
        return NULL;
    }
    return (const void *)request_payload;
}

/* F1：运动命令授权——租约范围校验 → 会话授权 → 记录租约。
 * 返回 reason（0=通过）；通过后调用方应用命令。
 * 注意顺序：先做无副作用的 payload 校验，authorize_run 有副作用
 * （置 run_authorized），非法请求不得消耗授权。 */
static uint8_t authorize_motion(uint32_t lease_ms, uint32_t session_id)
{
    const uint32_t now = now_ms();
    uint8_t reason;

    if ((lease_ms < AXDR_CONTROL_LEASE_MIN_MS) ||
        (lease_ms > AXDR_CONTROL_LEASE_MAX_MS))
    {
        return AXDR_REASON_INVALID_PAYLOAD;
    }
    reason = axdr_safety_runtime_authorize_run(&safety, now, session_id);
    if (reason != AXDR_REASON_NONE)
    {
        return reason;
    }
    command_deadline_ms = now + lease_ms;
    applied_count++;
    command_active = 1u;
    return AXDR_REASON_NONE;
}

/* ---- 各 opcode 响应构造 ---- */

static uint8_t provide_protocol_info(uint8_t *resp, uint16_t cap,
                                     uint16_t *len)
{
    axdr_protocol_info_payload_t p;
    memset(&p, 0, sizeof(p));
    p.command_version = AXDR_COMMAND_VERSION;
    p.telemetry_version = 1u; /* S4 对齐 */
    p.telemetry_frame_size = 160u;
    p.max_request_size = AXDR_COMMAND_MAX_REQUEST_SIZE;
    p.max_response_size = AXDR_COMMAND_MAX_RESPONSE_SIZE;
    p.parameter_schema_version = 0u;
    p.capability_flags = AXDR_CAPABILITY_READ_ONLY_QUERIES |
                         AXDR_CAPABILITY_SAFETY_SESSION |
                         AXDR_CAPABILITY_USB_MOTION_CONTROL |
                         AXDR_CAPABILITY_CONTROL_LEASE |
                         AXDR_CAPABILITY_MOTOR_PROFILE |
                         AXDR_CAPABILITY_MOTOR_PROFILE_CONFIRMATION |
                         AXDR_CAPABILITY_LINK_DIAGNOSTICS;
    p.supported_opcode_mask = SVC_SUPPORTED_MASK;
    p.parameter_count = 0u;
    return payload_out(&p, sizeof(p), resp, cap, len);
}

static uint8_t provide_device_info(uint8_t *resp, uint16_t cap,
                                   uint16_t *len)
{
    axdr_device_info_payload_t p;
    memset(&p, 0, sizeof(p));
    p.device_family = SVC_DEVICE_FAMILY;
    p.board_profile = SVC_BOARD_PROFILE;
    p.firmware_major = SVC_FW_MAJOR;
    p.firmware_minor = SVC_FW_MINOR;
    p.firmware_patch = SVC_FW_PATCH;
    p.telemetry_version = 1u;
    p.command_version = AXDR_COMMAND_VERSION;
    p.build_id = 0x20261004u;
    /* strncpy 语义：源短于目标时补零（packed char[16]） */
    { const char *src = SVC_PRODUCT_NAME; uint16_t i = 0u;
      while (src[i] != 0 && i < sizeof(p.product_name)) { p.product_name[i] = src[i]; i++; } }
    return payload_out(&p, sizeof(p), resp, cap, len);
}

static uint8_t provide_runtime_state(uint8_t *resp, uint16_t cap,
                                     uint16_t *len)
{
    axdr_runtime_state_payload_t p;
    memset(&p, 0, sizeof(p));
    p.tick_ms = now_ms();
    p.ctrl_bit = (uint8_t)g_foc.req;
    p.state_bit = (uint8_t)g_foc.state;
    p.sys_mode = (uint8_t)g_foc.mode.sys;
    p.debug_mode = (uint8_t)g_foc.mode.debug;
    p.release_mode = (uint8_t)g_foc.mode.release;
    p.encoder_type = (uint8_t)g_foc.enc.primary;
    p.phase_order = (uint8_t)g_foc.motor.phase_order;
    p.fault_bits = g_foc.fault.all;
    return payload_out(&p, sizeof(p), resp, cap, len);
}

static uint8_t provide_safety_state(uint8_t *resp, uint16_t cap,
                                    uint16_t *len)
{
    axdr_safety_state_payload_t p;
    ensure_profile_configured();
    axdr_safety_runtime_snapshot(&safety, &p);
    return payload_out(&p, sizeof(p), resp, cap, len);
}

static uint8_t provide_motor_profile(uint8_t *resp, uint16_t cap,
                                     uint16_t *len)
{
    build_profile_once();
    return payload_out(&profile_payload, (uint16_t)sizeof(profile_payload),
                       resp, cap, len);
}

static uint8_t provide_control_limits(uint8_t *resp, uint16_t cap,
                                      uint16_t *len)
{
    axdr_control_limits_payload_t p;
    memset(&p, 0, sizeof(p));
    p.schema_version = 1u;
    p.payload_size = sizeof(p);
    p.profile_id = SVC_PROFILE_ID;
    p.profile_revision = CHENSHA_ARCHIVE_REVISION;
    p.maximum_current_A = CHENSHA_COMMAND_CURRENT_LIMIT_A;
    p.maximum_speed_rad_s = CHENSHA_PEAK_SPEED_RAD_S;
    p.maximum_speed_iq_A = CHENSHA_COMMAND_CURRENT_LIMIT_A;
    p.maximum_alignment_current_A = CHENSHA_ALIGN_CURRENT_A;
    return payload_out(&p, sizeof(p), resp, cap, len);
}

static uint8_t provide_link_diagnostics(uint8_t *resp, uint16_t cap,
                                        uint16_t *len)
{
    const service_usb_stats_t *s = service_usb_stats();
    axdr_link_diagnostics_payload_t p;
    memset(&p, 0, sizeof(p));
    p.version = 1u;
    p.payload_size = sizeof(p);
    p.rx_frame_count = s->rx_frames;
    p.tx_frame_count = s->tx_frames;
    p.bad_crc_count = s->bad_crc_count;
    p.resync_count = s->resync_count;
    p.parser_reject_count = s->rejected_count;
    return payload_out(&p, sizeof(p), resp, cap, len);
}

static uint8_t provide_control_state(uint8_t *resp, uint16_t cap,
                                     uint16_t *len)
{
    axdr_control_state_payload_t p;
    memset(&p, 0, sizeof(p));
    p.tick_ms = now_ms();
    p.session_id = safety.session_id;
    p.command_deadline_ms = command_deadline_ms;
    p.applied_count = applied_count;
    p.timeout_count = lease_timeout_count;
    p.mode = (uint8_t)g_foc.mode.debug;
    p.active = (g_foc.state == DRIVE_STATE_RUN) ? 1u : 0u;
    p.stop_reason = last_stop_reason;
    p.alignment_valid = (g_foc.enc_aligned != 0u) ? 1u : 0u;
    p.speed_target_rad_s = g_foc.ref.spd_r;
    p.iq_target_A = g_foc.ref.iq;
    return payload_out(&p, sizeof(p), resp, cap, len);
}

/* ---- 分发器主体 ---- */

uint8_t service_command_dispatch(void *context, uint16_t opcode,
                                 const uint8_t *request_payload,
                                 uint16_t request_payload_length,
                                 uint8_t *response_payload,
                                 uint16_t response_payload_capacity,
                                 uint16_t *response_payload_length,
                                 uint8_t *ack_state)
{
    (void)context;

    *response_payload_length = 0u;
    *ack_state = AXDR_ACK_REJECTED;

    switch (opcode)
    {
        /* ---- 连通组（无 payload，只读） ---- */
        case AXDR_OPCODE_GET_PROTOCOL_INFO:
            if (request_payload_length != 0u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            *ack_state = AXDR_ACK_ACCEPTED;
            return provide_protocol_info(response_payload,
                                         response_payload_capacity,
                                         response_payload_length);

        case AXDR_OPCODE_GET_DEVICE_INFO:
            if (request_payload_length != 0u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            *ack_state = AXDR_ACK_ACCEPTED;
            return provide_device_info(response_payload,
                                       response_payload_capacity,
                                       response_payload_length);

        case AXDR_OPCODE_GET_RUNTIME_STATE:
            if (request_payload_length != 0u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            *ack_state = AXDR_ACK_ACCEPTED;
            return provide_runtime_state(response_payload,
                                         response_payload_capacity,
                                         response_payload_length);

        case AXDR_OPCODE_GET_LINK_DIAGNOSTICS:
            if (request_payload_length != 0u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            *ack_state = AXDR_ACK_ACCEPTED;
            return provide_link_diagnostics(response_payload,
                                            response_payload_capacity,
                                            response_payload_length);

        /* ---- 使能链组 ---- */
        case AXDR_OPCODE_GET_SAFETY_STATE:
            if (request_payload_length != 0u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            *ack_state = AXDR_ACK_ACCEPTED;
            return provide_safety_state(response_payload,
                                        response_payload_capacity,
                                        response_payload_length);

        case AXDR_OPCODE_SAFETY_ARM:
        {
            const axdr_safety_arm_request_t *req = payload_in(
                request_payload, request_payload_length, sizeof(*req));
            uint8_t reason;
            if (req == NULL)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            /* challenge 从上次 GetSafetyState 快照来；会话由 arm 内部建立。
             * ARM 前现场 observe 一次：会话门卫模式下 poll 在首次 ARM 前不
             * 执法，互锁位只能在这里刷新——ARM 本就应在"电机静止"瞬间评估
             * 互锁（输出使能中 ARM → INTERLOCK_OPEN/FAULT_LOCKED，B 原语义）。 */
            {
                const uint8_t outputs_disabled =
                    (g_foc.pwm_active == 0u) ? 1u : 0u;
                axdr_safety_runtime_observe(&safety, now_ms(),
                                            g_foc.fault.all, outputs_disabled,
                                            outputs_disabled);
            }
            reason = axdr_safety_runtime_arm(&safety, now_ms(),
                                             req->arm_challenge,
                                             req->heartbeat_timeout_ms);
            if (reason != AXDR_REASON_NONE)
            {
                return reason;
            }
            session_ever_active = 1u; /* F3：从现在起 poll 开始执法 */
            *ack_state = AXDR_ACK_APPLIED;
            return AXDR_REASON_NONE;
        }

        case AXDR_OPCODE_SAFETY_DISARM:
            if (request_payload_length != 0u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            (void)axdr_safety_runtime_disarm(&safety, now_ms());
            command_active = 0u;
            last_stop_reason = AXDR_CONTROL_STOP_SAFETY_SESSION;
            /* 立即停机 + 撤销运行授权（下次运动命令需重新过入口互锁） */
            if ((g_foc.req == DRIVE_REQ_RUN) ||
                (g_foc.req == DRIVE_REQ_START))
            {
                g_foc.req = DRIVE_REQ_STOP;
            }
            *ack_state = AXDR_ACK_APPLIED;
            return AXDR_REASON_NONE;

        case AXDR_OPCODE_SAFETY_HEARTBEAT:
        {
            const axdr_safety_heartbeat_request_t *req = payload_in(
                request_payload, request_payload_length, sizeof(*req));
            uint8_t reason;
            if (req == NULL)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            reason = axdr_safety_runtime_heartbeat(&safety, now_ms(),
                                                   req->session_id);
            if (reason != AXDR_REASON_NONE)
            {
                return reason;
            }
            *ack_state = AXDR_ACK_APPLIED;
            return AXDR_REASON_NONE;
        }

        case AXDR_OPCODE_GET_MOTOR_PROFILE:
            if (request_payload_length != 0u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            *ack_state = AXDR_ACK_ACCEPTED;
            return provide_motor_profile(response_payload,
                                         response_payload_capacity,
                                         response_payload_length);

        case AXDR_OPCODE_GET_CONTROL_LIMITS:
            if (request_payload_length != 0u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            *ack_state = AXDR_ACK_ACCEPTED;
            return provide_control_limits(response_payload,
                                          response_payload_capacity,
                                          response_payload_length);

        case AXDR_OPCODE_CONFIRM_MOTOR_PROFILE:
        {
            const axdr_motor_profile_confirm_request_t *req = payload_in(
                request_payload, request_payload_length, sizeof(*req));
            uint8_t reason;
            if (req == NULL)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            ensure_profile_configured();
            reason = axdr_safety_runtime_confirm_motor_profile(&safety, req);
            if (reason != AXDR_REASON_NONE)
            {
                return reason;
            }
            *ack_state = AXDR_ACK_APPLIED;
            return AXDR_REASON_NONE;
        }

        /* ---- 控制组（需安全链授权 + 命令租约） ---- */
        case AXDR_OPCODE_GET_CONTROL_STATE:
            if (request_payload_length != 0u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            *ack_state = AXDR_ACK_ACCEPTED;
            return provide_control_state(response_payload,
                                         response_payload_capacity,
                                         response_payload_length);

        case AXDR_OPCODE_CONTROL_STOP:
            if (request_payload_length != 0u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            g_foc.req = DRIVE_REQ_STOP;
            command_active = 0u;
            last_stop_reason = AXDR_CONTROL_STOP_HOST_REQUEST;
            /* 撤销运行授权：下次运动命令重新过入口互锁（fail-safe 方向） */
            (void)axdr_safety_runtime_revoke_run(&safety, now_ms());
            *ack_state = AXDR_ACK_APPLIED;
            return AXDR_REASON_NONE;

        case AXDR_OPCODE_SET_SPEED:
        {
            const axdr_speed_request_t *req = payload_in(
                request_payload, request_payload_length, sizeof(*req));
            uint8_t reason;
            if (req == NULL)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            if ((req->target_rad_s > CHENSHA_PEAK_SPEED_RAD_S) ||
                (req->target_rad_s < -CHENSHA_PEAK_SPEED_RAD_S) ||
                (req->iq_limit_A <= 0.0f) ||
                (req->iq_limit_A > CHENSHA_COMMAND_CURRENT_LIMIT_A))
            {
                return AXDR_REASON_OUT_OF_RANGE;
            }
            reason = authorize_motion(req->lease_ms, req->session_id);
            if (reason != AXDR_REASON_NONE)
            {
                return reason;
            }
            g_foc.mode.sys = debug_mode;
            g_foc.mode.debug = spd_curr_cl;
            g_foc.ref.iq = req->iq_limit_A;
            g_foc.ref.spd_r = req->target_rad_s;
            if (g_foc.state != DRIVE_STATE_RUN)
            {
                g_foc.req = DRIVE_REQ_START;
            }
            *ack_state = AXDR_ACK_APPLIED;
            return AXDR_REASON_NONE;
        }

        case AXDR_OPCODE_SET_CURRENT:
        {
            const axdr_current_request_t *req = payload_in(
                request_payload, request_payload_length, sizeof(*req));
            uint8_t reason;
            if (req == NULL)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            if ((req->iq_target_A > CHENSHA_COMMAND_CURRENT_LIMIT_A) ||
                (req->iq_target_A < -CHENSHA_COMMAND_CURRENT_LIMIT_A) ||
                (req->id_target_A > CHENSHA_COMMAND_CURRENT_LIMIT_A) ||
                (req->id_target_A < -CHENSHA_COMMAND_CURRENT_LIMIT_A))
            {
                return AXDR_REASON_OUT_OF_RANGE;
            }
            reason = authorize_motion(req->lease_ms, req->session_id);
            if (reason != AXDR_REASON_NONE)
            {
                return reason;
            }
            g_foc.mode.sys = debug_mode;
            g_foc.mode.debug = curr_cl;
            g_foc.ref.id = req->id_target_A;
            g_foc.ref.iq = req->iq_target_A;
            if (g_foc.state != DRIVE_STATE_RUN)
            {
                g_foc.req = DRIVE_REQ_START;
            }
            *ack_state = AXDR_ACK_APPLIED;
            return AXDR_REASON_NONE;
        }

        default:
            return AXDR_REASON_UNKNOWN_OPCODE;
    }
}
