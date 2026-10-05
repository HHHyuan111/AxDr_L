/**
 * @file test_service_command.c
 * @brief S3/S3.5 分发器与安全链全旅程测试（经 service_usb 缝注入帧，全链路）。
 *
 * 正向旅程（S3.5 补齐，此前只测拒绝路径漏掉 F1 授权链断裂）：
 *   GetMotorProfile(真CRC) → Confirm → Arm → SetSpeed(带session+lease)
 *   → 租约超时 revoke+停机 → 续租重授权 → 心跳超时 disarm
 *   → 完整重使能（confirm→arm→run）→ ControlStop。
 * 负向路径：错 CRC / 未 ARM 的 SetSpeed / 错 session / 未知 opcode。
 *
 * 时钟：dispatch/poll 统一用 g_foc.fast_seq/20（20kHz→ms），测试直接写
 * fast_seq 推进时间。offset 全部用 offsetof，不硬编码偏移。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "axdr_command_contract.h"
#include "axdr_command_core.h"
#include "common.h"
#include "service_command.h"
#include "service_usb.h"

foc_t g_foc;

/* drive.c 替身：service_command 的 DISARM 故障恢复路径调用（真机由
 * drive.c 提供，本测试不链 drive 源）。语义对齐 drive.c——PWM 活动期
 * 拒绝清零，否则清锁存故障并请求 STOP。 */
bool drive_fault_clear(foc_t *foc)
{
    if ((foc == NULL) || (foc->pwm_active != 0u))
    {
        return false;
    }
    foc->fault.all = 0u;
    foc->req = DRIVE_REQ_STOP;
    return true;
}

static int fail_count;
static void expect(const char *name, bool cond)
{
    if (!cond)
    {
        (void)printf("FAIL %s\n", name);
        fail_count++;
    }
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

/* ---- 响应 payload 字段读取（packed，小端） ---- */
static uint32_t rd_u32(const uint8_t *base, uint16_t offset)
{
    uint32_t value;
    memcpy(&value, &base[offset], 4);
    return value;
}
static uint8_t rd_u8(const uint8_t *base, uint16_t offset)
{
    return base[offset];
}

/* ---- 时间推进：fast_seq @20kHz ---- */
static void set_time_ms(uint32_t ms)
{
    g_foc.fast_seq = ms * 20u;
}

int main(void)
{
    uint32_t challenge;
    uint32_t session_id;
    uint32_t profile_id;
    uint32_t profile_revision;
    uint32_t profile_crc32;
    axdr_speed_request_t spd;
    axdr_current_request_t cur;
    axdr_safety_arm_request_t arm_req;
    axdr_safety_heartbeat_request_t hb;
    axdr_motor_profile_confirm_request_t confirm;

    /* 最小 g_foc 初始化 */
    memset(&g_foc, 0, sizeof(g_foc));
    g_foc.motor.phase_order = PHASE_ORDER_ABC;
    g_foc.mode.sys = debug_mode;
    g_foc.mode.debug = curr_cl;
    g_foc.req = DRIVE_REQ_STOP;
    g_foc.state = DRIVE_STATE_STOP;
    g_foc.enc.primary = ENCODER_TYPE_ABZ;
    g_foc.enc_aligned = true;
    g_foc.pwm_active = 0u;

    service_usb_bind_tx(tx_stub);

    /* ---- 0. GUI 预检序首查（S10 回归）----
     * 真实上位机预检顺序是 0x01→0x02→0x17→0x18→0x08，档案确认(0x19)
     * 要到点使能时才发——0x08 常是上电后第一条会触碰安全 runtime 的命令。
     * 曾因 ensure 懒初始化排在 observe 之后，首查互锁被 init 的 memset
     * 清零（恒 0x00/tick=0），上位机"互锁标志未就绪"死锁。下方旅程第 3
     * 步就发 Confirm（其内部先 ensure），恰好绕开该路径，故须单独前置。 */
    set_time_ms(316127u);
    send_frame(NULL, 0u, 0x0100u, AXDR_OPCODE_GET_PROTOCOL_INFO);
    expect("预检 protocol ack", ack() == AXDR_ACK_ACCEPTED);
    send_frame(NULL, 0u, 0x0101u, AXDR_OPCODE_GET_DEVICE_INFO);
    expect("预检 device ack", ack() == AXDR_ACK_ACCEPTED);
    send_frame(NULL, 0u, 0x0102u, AXDR_OPCODE_GET_MOTOR_PROFILE);
    expect("预检 profile ack", ack() == AXDR_ACK_ACCEPTED);
    send_frame(NULL, 0u, 0x0103u, AXDR_OPCODE_GET_CONTROL_LIMITS);
    expect("预检 limits ack", ack() == AXDR_ACK_ACCEPTED);
    send_frame(NULL, 0u, 0x0104u, AXDR_OPCODE_GET_SAFETY_STATE);
    expect("首查 safety ack", ack() == AXDR_ACK_ACCEPTED);
    expect("首查互锁 READY_TO_ARM(S10)",
           rd_u8(resp_payload(),
                 offsetof(axdr_safety_state_payload_t, interlock_flags)) ==
               AXDR_SAFETY_INTERLOCK_READY_TO_ARM);
    expect("首查 tick=上电毫秒(非 init 清零)",
           rd_u32(resp_payload(),
                  offsetof(axdr_safety_state_payload_t, tick_ms)) == 316127u);
    /* 回归时间轴：下方主旅程从 t=0 重新计时（租约/心跳断言依赖它） */
    set_time_ms(0u);

    /* ---- 1. GetProtocolInfo（能力位含运动控制与租约） ---- */
    send_frame(NULL, 0u, 1u, AXDR_OPCODE_GET_PROTOCOL_INFO);
    expect("protocol info ack", ack() == AXDR_ACK_ACCEPTED);
    expect("protocol payload 完整", tx_len >= 40u);

    /* ---- 2. GetMotorProfile：CRC 必须为固件真指纹（非零） ---- */
    send_frame(NULL, 0u, 2u, AXDR_OPCODE_GET_MOTOR_PROFILE);
    expect("motor profile ack", ack() == AXDR_ACK_ACCEPTED);
    profile_id = rd_u32(resp_payload(), offsetof(axdr_motor_profile_payload_t, profile_id));
    profile_revision = rd_u32(resp_payload(), offsetof(axdr_motor_profile_payload_t, revision));
    profile_crc32 = rd_u32(resp_payload(), offsetof(axdr_motor_profile_payload_t, profile_crc32));
    expect("profile id=1", profile_id == 1u);
    expect("profile rev=2", profile_revision == 2u);
    expect("profile crc 非零(F2)", profile_crc32 != 0u);

    /* ---- 3. 错 CRC 确认 → MISMATCH ---- */
    memset(&confirm, 0, sizeof(confirm));
    confirm.profile_id = profile_id;
    confirm.profile_revision = profile_revision;
    confirm.profile_crc32 = profile_crc32 + 1u;
    send_frame((const uint8_t *)&confirm, sizeof(confirm), 3u,
               AXDR_OPCODE_CONFIRM_MOTOR_PROFILE);
    expect("错 crc 确认拒", ack() == AXDR_ACK_REJECTED);
    expect("拒因 PROFILE_MISMATCH",
           reason() == AXDR_REASON_MOTOR_PROFILE_MISMATCH);

    /* ---- 4. 未 ARM 时 SetSpeed → INVALID_STATE（authorize_run 状态门） ---- */
    memset(&spd, 0, sizeof(spd));
    spd.session_id = 0u;
    spd.lease_ms = 300u;
    spd.target_rad_s = 52.36f;
    spd.iq_limit_A = 8.0f;
    send_frame((const uint8_t *)&spd, sizeof(spd), 4u,
               AXDR_OPCODE_SET_SPEED);
    expect("未 ARM SetSpeed 拒", ack() == AXDR_ACK_REJECTED);
    expect("拒因 INVALID_STATE", reason() == AXDR_REASON_INVALID_STATE);

    /* ---- 5. 正确三元组确认 → APPLIED ---- */
    confirm.profile_crc32 = profile_crc32;
    send_frame((const uint8_t *)&confirm, sizeof(confirm), 5u,
               AXDR_OPCODE_CONFIRM_MOTOR_PROFILE);
    expect("正确确认 APPLIED", ack() == AXDR_ACK_APPLIED);

    /* ---- 6. 拿 challenge 并 ARM ---- */
    send_frame(NULL, 0u, 6u, AXDR_OPCODE_GET_SAFETY_STATE);
    expect("safety state ack", ack() == AXDR_ACK_ACCEPTED);
    challenge = rd_u32(resp_payload(),
                       offsetof(axdr_safety_state_payload_t, arm_challenge));
    expect("challenge 非零", challenge != 0u);

    memset(&arm_req, 0, sizeof(arm_req));
    arm_req.arm_challenge = challenge;
    arm_req.heartbeat_timeout_ms = 500u;
    send_frame((const uint8_t *)&arm_req, sizeof(arm_req), 7u,
               AXDR_OPCODE_SAFETY_ARM);
    expect("ARM APPLIED", ack() == AXDR_ACK_APPLIED);

    /* ARM 只建会话，不授权运动 */
    send_frame(NULL, 0u, 8u, AXDR_OPCODE_GET_SAFETY_STATE);
    session_id = rd_u32(resp_payload(),
                        offsetof(axdr_safety_state_payload_t, session_id));
    expect("session 非零", session_id != 0u);
    expect("ARM 后仍未授权",
           rd_u8(resp_payload(), offsetof(axdr_safety_state_payload_t,
                                          run_authorized)) == 0u);

    /* ---- 7. t=0 poll 开启执法（F3 门卫：ARM 后才 observe） ---- */
    service_safety_poll();

    /* ---- 8. SetSpeed（带 session+lease）→ APPLIED，写 mode/ref/req ---- */
    spd.session_id = session_id;
    spd.lease_ms = 100u;
    send_frame((const uint8_t *)&spd, sizeof(spd), 9u,
               AXDR_OPCODE_SET_SPEED);
    expect("SetSpeed APPLIED(F1)", ack() == AXDR_ACK_APPLIED);
    expect("mode=spd_curr_cl", g_foc.mode.debug == spd_curr_cl);
    expect("spd ref 写入", g_foc.ref.spd_r == 52.36f);
    expect("iq limit 写入", g_foc.ref.iq == 8.0f);
    expect("req=START", g_foc.req == DRIVE_REQ_START);

    /* applied_count=1、deadline=100 经 GetControlState 可见 */
    send_frame(NULL, 0u, 10u, AXDR_OPCODE_GET_CONTROL_STATE);
    expect("control state ack", ack() == AXDR_ACK_ACCEPTED);
    expect("applied_count=1",
           rd_u32(resp_payload(),
                  offsetof(axdr_control_state_payload_t, applied_count)) == 1u);
    expect("command_deadline=100",
           rd_u32(resp_payload(), offsetof(axdr_control_state_payload_t,
                                           command_deadline_ms)) == 100u);

    /* ---- 9. SetCurrent 正向 ---- */
    memset(&cur, 0, sizeof(cur));
    cur.session_id = session_id;
    cur.lease_ms = 100u;
    cur.id_target_A = 0.0f;
    cur.iq_target_A = 1.5f;
    send_frame((const uint8_t *)&cur, sizeof(cur), 11u,
               AXDR_OPCODE_SET_CURRENT);
    expect("SetCurrent APPLIED", ack() == AXDR_ACK_APPLIED);
    expect("mode=curr_cl", g_foc.mode.debug == curr_cl);
    expect("iq ref 写入", g_foc.ref.iq == 1.5f);

    /* ---- 10. t=150ms：租约(100ms)超时 → revoke + 强制停机 ---- */
    set_time_ms(150u);
    service_safety_poll();
    expect("租约超时强制 STOP", g_foc.req == DRIVE_REQ_STOP);
    send_frame(NULL, 0u, 12u, AXDR_OPCODE_GET_CONTROL_STATE);
    expect("lease timeout_count=1",
           rd_u32(resp_payload(),
                  offsetof(axdr_control_state_payload_t, timeout_count)) == 1u);
    expect("stop_reason=LEASE_TIMEOUT",
           rd_u8(resp_payload(), offsetof(axdr_control_state_payload_t,
                                          stop_reason)) ==
               AXDR_CONTROL_STOP_LEASE_TIMEOUT);
    send_frame(NULL, 0u, 13u, AXDR_OPCODE_GET_SAFETY_STATE);
    expect("租约超时已撤权",
           rd_u8(resp_payload(), offsetof(axdr_safety_state_payload_t,
                                          run_authorized)) == 0u);

    /* ---- 11. 续租：输出已关断（入口互锁满足）→ 重新授权成功 ---- */
    spd.lease_ms = 100u;
    send_frame((const uint8_t *)&spd, sizeof(spd), 14u,
               AXDR_OPCODE_SET_SPEED);
    expect("续租 SetSpeed APPLIED", ack() == AXDR_ACK_APPLIED);
    expect("req 恢复 START", g_foc.req == DRIVE_REQ_START);

    /* ---- 12. 心跳：错 session 拒 / 对 session 续 ---- */
    memset(&hb, 0, sizeof(hb));
    hb.session_id = session_id + 1u;
    send_frame((const uint8_t *)&hb, sizeof(hb), 15u,
               AXDR_OPCODE_SAFETY_HEARTBEAT);
    expect("错 session 心跳拒", ack() == AXDR_ACK_REJECTED);
    expect("拒因 SESSION_MISMATCH", reason() == AXDR_REASON_SESSION_MISMATCH);
    hb.session_id = session_id;
    send_frame((const uint8_t *)&hb, sizeof(hb), 16u,
               AXDR_OPCODE_SAFETY_HEARTBEAT);
    expect("对 session 心跳 APPLIED", ack() == AXDR_ACK_APPLIED);

    /* ---- 13. t=1000ms：心跳超时（last_hb=150）+ 租约超时 → DISARMED ---- */
    set_time_ms(1000u);
    service_safety_poll();
    send_frame(NULL, 0u, 17u, AXDR_OPCODE_GET_SAFETY_STATE);
    expect("心跳超时后 DISARMED",
           rd_u8(resp_payload(), offsetof(axdr_safety_state_payload_t,
                                          state)) == AXDR_SAFETY_DISARMED);
    expect("disarm_reason=HEARTBEAT_TIMEOUT",
           rd_u8(resp_payload(), offsetof(axdr_safety_state_payload_t,
                                          disarm_reason)) ==
               AXDR_SAFETY_DISARM_HEARTBEAT_TIMEOUT);
    send_frame(NULL, 0u, 171u, AXDR_OPCODE_GET_CONTROL_STATE);
    expect("租约 timeout_count=2",
           rd_u32(resp_payload(),
                  offsetof(axdr_control_state_payload_t, timeout_count)) == 2u);
    expect("强制 STOP", g_foc.req == DRIVE_REQ_STOP);

    /* ---- 14. 完整重使能闭环：confirm → arm → 新 session → SetSpeed ---- */
    send_frame((const uint8_t *)&confirm, sizeof(confirm), 18u,
               AXDR_OPCODE_CONFIRM_MOTOR_PROFILE);
    expect("重确认 APPLIED", ack() == AXDR_ACK_APPLIED);
    send_frame((const uint8_t *)&arm_req, sizeof(arm_req), 19u,
               AXDR_OPCODE_SAFETY_ARM);
    expect("重 ARM APPLIED", ack() == AXDR_ACK_APPLIED);
    send_frame(NULL, 0u, 20u, AXDR_OPCODE_GET_SAFETY_STATE);
    session_id = rd_u32(resp_payload(),
                        offsetof(axdr_safety_state_payload_t, session_id));
    expect("新 session", session_id != 0u);
    spd.session_id = session_id;
    spd.lease_ms = 300u;
    send_frame((const uint8_t *)&spd, sizeof(spd), 21u,
               AXDR_OPCODE_SET_SPEED);
    expect("重使能 SetSpeed APPLIED", ack() == AXDR_ACK_APPLIED);
    expect("req=START", g_foc.req == DRIVE_REQ_START);

    /* ---- 15. ControlStop：无条件可用（安全方向），撤运行授权 ---- */
    send_frame(NULL, 0u, 22u, AXDR_OPCODE_CONTROL_STOP);
    expect("ControlStop APPLIED", ack() == AXDR_ACK_APPLIED);
    expect("req=STOP", g_foc.req == DRIVE_REQ_STOP);
    send_frame(NULL, 0u, 23u, AXDR_OPCODE_GET_SAFETY_STATE);
    expect("stop 已撤权",
           rd_u8(resp_payload(), offsetof(axdr_safety_state_payload_t,
                                          run_authorized)) == 0u);

    /* ---- 16. 未知 opcode ---- */
    send_frame(NULL, 0u, 24u, 0x0027u /* RestoreSpeedPi 不在本版 */);
    expect("未知 opcode 拒", ack() == AXDR_ACK_REJECTED);
    expect("拒因 UNKNOWN", reason() == AXDR_REASON_UNKNOWN_OPCODE);

    /* ---- 17. 超范围校验（F5）：负 iq_limit / 超限转速 ---- */
    spd.session_id = session_id;
    spd.iq_limit_A = -1.0f;
    send_frame((const uint8_t *)&spd, sizeof(spd), 25u,
               AXDR_OPCODE_SET_SPEED);
    expect("负 iq_limit 拒", ack() == AXDR_ACK_REJECTED);
    expect("拒因 OUT_OF_RANGE", reason() == AXDR_REASON_OUT_OF_RANGE);
    spd.iq_limit_A = 8.0f;
    spd.target_rad_s = 9999.0f;
    send_frame((const uint8_t *)&spd, sizeof(spd), 26u,
               AXDR_OPCODE_SET_SPEED);
    expect("超限转速拒", ack() == AXDR_ACK_REJECTED);
    expect("拒因 OUT_OF_RANGE", reason() == AXDR_REASON_OUT_OF_RANGE);
    /* 非法 lease（低于 100ms）拒且不消耗授权 */
    spd.target_rad_s = 52.36f;
    spd.lease_ms = 50u;
    send_frame((const uint8_t *)&spd, sizeof(spd), 27u,
               AXDR_OPCODE_SET_SPEED);
    expect("非法 lease 拒", ack() == AXDR_ACK_REJECTED);
    expect("拒因 INVALID_PAYLOAD", reason() == AXDR_REASON_INVALID_PAYLOAD);

    /* ---- 18. NaN 校验（0.7.6）：NaN 与任何数比较恒假，纯范围检查
     * 挡不住，必须 isfinite 先行（14 号数值政策；ALIGN_ENCODER 同款）。
     * volatile 防编译器常量折叠，确保运行时真 NaN。 ---- */
    {
        volatile float zero = 0.0f;
        const float nan_v = zero / zero;

        spd.lease_ms = 300u;
        spd.iq_limit_A = 8.0f;
        spd.target_rad_s = nan_v;
        send_frame((const uint8_t *)&spd, sizeof(spd), 28u,
                   AXDR_OPCODE_SET_SPEED);
        expect("NaN 转速目标拒", ack() == AXDR_ACK_REJECTED);
        expect("拒因 OUT_OF_RANGE", reason() == AXDR_REASON_OUT_OF_RANGE);
        spd.target_rad_s = 52.36f;
        spd.iq_limit_A = nan_v;
        send_frame((const uint8_t *)&spd, sizeof(spd), 29u,
                   AXDR_OPCODE_SET_SPEED);
        expect("NaN iq_limit 拒", ack() == AXDR_ACK_REJECTED);
        expect("拒因 OUT_OF_RANGE", reason() == AXDR_REASON_OUT_OF_RANGE);
        memset(&cur, 0, sizeof(cur));
        cur.session_id = session_id;
        cur.lease_ms = 300u;
        cur.iq_target_A = nan_v;
        cur.id_target_A = 1.0f;
        send_frame((const uint8_t *)&cur, sizeof(cur), 30u,
                   AXDR_OPCODE_SET_CURRENT);
        expect("NaN 电流目标拒", ack() == AXDR_ACK_REJECTED);
        expect("拒因 OUT_OF_RANGE", reason() == AXDR_REASON_OUT_OF_RANGE);
    }

    if (fail_count == 0)
    {
        (void)printf("test_service_command: all pass\n");
        return 0;
    }
    (void)printf("test_service_command: %d failure(s)\n", fail_count);
    return 1;
}
