/**
 * @file usb_acceptance.c
 * @brief P7 上位机基准验收工具——协议级命令旅程自动化（61 号手册第 3-7 节）。
 *
 * 与固件同源编译 User/service 的协议核心与契约头（CRC/结构布局零漂移）。
 * 串口层抽象为 transport：真实 Win32 CDC 与 --selftest 内存回环（模拟
 * 固件应答器）二选一，整条旅程逻辑可在无硬件主机上回归。
 *
 * 用法:
 *   usb_acceptance COMx                 全旅程（探针→查询→ARM→运动→超时负路径）
 *   usb_acceptance COMx --reboot-check  断电重启后免对齐验证
 *   usb_acceptance COMx --probe-only    只跑遥测探针（不发运动指令）
 *   usb_acceptance --selftest           无硬件全逻辑回归（模拟固件端）
 * 选项: --speed X (rad/s, 默认 8) --iq Y (A, 默认 2) --out file.csv
 *
 * 退出码 0=全 PASS；ARM 失败或探针不过即终止（不发运动指令）。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include "../firmware/AxDr_App/User/service/axdr_command_contract.h"
#include "../firmware/AxDr_App/User/service/axdr_command_core.h"
#include "../firmware/AxDr_App/User/service/axdr_telemetry_contract.h"

/* ---- 遥测帧字段偏移（S4 主机测试验证过的线值） ---- */
#define TL_OFF_SEQ 8u
#define TL_OFF_TICK 12u
#define TL_OFF_STATE 17u
#define TL_OFF_FAULT 32u
#define TL_OFF_CMD_WR 52u
#define TL_OFF_SPD_M 104u
#define TL_OFF_VBUS 136u

#define DRIVE_STATE_STOP 0u
#define DRIVE_STATE_STARTING 1u
#define DRIVE_STATE_RUN 2u
#define DRIVE_STATE_FAULT 3u

/* safety_state payload 内偏移（40B） */
#define SS_OFF_CHALLENGE 8u
#define SS_OFF_SESSION 12u
#define SS_OFF_TIMEOUT_COUNT 32u
#define SS_OFF_STATE 36u
#define SS_OFF_DISARM_REASON 37u
#define SS_OFF_INTERLOCK 38u
#define SS_OFF_RUN_AUTHORIZED 39u

/* control_state payload 内偏移（40B） */
#define CS_OFF_STOP_REASON 22u

static int g_fails;

static void expect(const char *name, int cond)
{
    printf("[%s] %s\n", cond ? "PASS" : "FAIL", name);
    if (!cond)
    {
        g_fails++;
    }
}

static uint32_t rd_u32(const uint8_t *p)
{
    uint32_t v;
    memcpy(&v, p, 4);
    return v;
}

static uint16_t rd_u16(const uint8_t *p)
{
    uint16_t v;
    memcpy(&v, p, 2);
    return v;
}

static float rd_f32(const uint8_t *p)
{
    float v;
    memcpy(&v, p, 4);
    return v;
}

static void wr_u32(uint8_t *p, uint32_t v)
{
    memcpy(p, &v, 4);
}

static void wr_u16(uint8_t *p, uint16_t v)
{
    memcpy(p, &v, 2);
}

static void wr_f32(uint8_t *p, float v)
{
    memcpy(p, &v, 4);
}

static uint32_t now_ms(void)
{
#ifdef _WIN32
    return (uint32_t)GetTickCount(); /* 墙钟：mingw 的 clock() 是 CPU 时间 */
#else
    return (uint32_t)(clock() * 1000u / CLOCKS_PER_SEC);
#endif
}

/* ============================================================
 * transport 抽象：真实串口与 selftest 回环二选一
 * ============================================================ */
typedef struct
{
    int (*open_port)(const char *name);
    int (*write_bytes)(const uint8_t *data, unsigned len);
    int (*read_bytes)(uint8_t *data, unsigned cap); /* 实读数，0=暂无 */
    void (*close_port)(void);
} transport_t;

static const transport_t *tr;

/* ---- 真实 Win32 串口 ---- */
#ifdef _WIN32
static HANDLE g_serial;

static int ser_open(const char *name)
{
    char path[16];
    DCB dcb;
    COMMTIMEOUTS to;

    if (strncmp(name, "COM", 3) != 0)
    {
        return 0;
    }
    (void)snprintf(path, sizeof(path), "\\\\.\\%s", name);
    g_serial = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                           OPEN_EXISTING, 0, NULL);
    if (g_serial == INVALID_HANDLE_VALUE)
    {
        return 0;
    }
    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    if (!SetCommState(g_serial, &dcb))
    {
        goto fail;
    }
    /* 轮询读：10ms 总超时，无数据返回 0 */
    memset(&to, 0, sizeof(to));
    to.ReadTotalTimeoutConstant = 10;
    to.WriteTotalTimeoutConstant = 100;
    if (!SetCommTimeouts(g_serial, &to))
    {
        goto fail;
    }
    (void)PurgeComm(g_serial, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return 1;

fail:
    CloseHandle(g_serial);
    g_serial = INVALID_HANDLE_VALUE;
    return 0;
}

static int ser_write(const uint8_t *data, unsigned len)
{
    DWORD written = 0;
    if ((g_serial == INVALID_HANDLE_VALUE) ||
        (!WriteFile(g_serial, data, len, &written, NULL)))
    {
        return -1;
    }
    return (written == len) ? (int)len : -1;
}

static int ser_read(uint8_t *data, unsigned cap)
{
    DWORD got = 0;
    if ((g_serial == INVALID_HANDLE_VALUE) ||
        (!ReadFile(g_serial, data, cap, &got, NULL)))
    {
        return -1;
    }
    return (int)got;
}

static void ser_close(void)
{
    if (g_serial != INVALID_HANDLE_VALUE)
    {
        CloseHandle(g_serial);
        g_serial = INVALID_HANDLE_VALUE;
    }
}

static const transport_t serial_transport = {
    ser_open, ser_write, ser_read, ser_close};
#endif /* _WIN32 */

/* ---- selftest 回环：内存管道 + 模拟固件端 ---- */
static uint8_t st_rx[4096];
static unsigned st_rx_len;
static axdr_command_core_t st_core;
static uint32_t st_tel_seq;
static int st_armed;
static uint32_t st_session;
static const uint32_t st_challenge = 0xA5A51234u;
static uint32_t st_tel_state;
static uint32_t st_start_ms; /* SetSpeed/校准时刻：模拟 1.6s 对齐到 RUN */
static int st_authorized;    /* 模拟 run_authorized：SetSpeed 置位/停机清零 */
static int st_profile_ok;    /* ConfirmProfile 已通过 */
static int st_aligned;       /* enc_aligned：对齐完成（STARTING→RUN 置位） */
static int st_stop_reason;   /* control_state.stop_reason 快照 */
static int st_lease_count;   /* control_state.timeout_count：租约超时计数 */
static int st_applied_count; /* control_state.applied_count */
static uint32_t st_lease_deadline; /* 授权到期时刻（模拟命令租约） */

/* safety/控制状态只有被测固件是权威；模拟端按旅程需要给出最小状态 */

/* 模拟命令租约执法：授权到期未续 → 撤权 + LEASE_TIMEOUT 停机
 * （与固件 service_safety_poll 的 revoke+强制 STOP 同语义） */
static void st_lease_poll(void)
{
    if ((st_authorized != 0) &&
        ((int32_t)(now_ms() - st_lease_deadline) >= 0))
    {
        st_authorized = 0;
        st_stop_reason = (int)AXDR_CONTROL_STOP_LEASE_TIMEOUT;
        st_tel_state = DRIVE_STATE_STOP;
        st_lease_count++;
    }
}

/* control_state 40B 快照（与固件 provide_control_state 同布局） */
static uint16_t st_control_state(uint8_t *resp)
{
    uint8_t p[40];
    memset(p, 0, sizeof(p));
    wr_u32(&p[0], now_ms());
    wr_u32(&p[4], st_session);
    wr_u32(&p[8], (st_authorized != 0) ? st_lease_deadline : 0u);
    wr_u32(&p[12], (uint32_t)st_applied_count);
    wr_u32(&p[16], (uint32_t)st_lease_count);
    p[20] = 2u; /* mode=速度 */
    p[21] = ((st_tel_state == DRIVE_STATE_RUN) ||
             (st_tel_state == DRIVE_STATE_STARTING)) ? 1u : 0u;
    p[22] = (uint8_t)st_stop_reason;
    p[23] = (st_aligned != 0) ? 1u : 0u;
    memcpy(resp, p, sizeof(p));
    return (uint16_t)sizeof(p);
}

static uint8_t st_handler(void *ctx, uint16_t opcode,
                          const uint8_t *req_payload, uint16_t req_len,
                          uint8_t *resp, uint16_t cap, uint16_t *resp_len,
                          uint8_t *ack)
{
    (void)ctx;
    (void)cap;
    *resp_len = 0u;
    switch (opcode)
    {
        case AXDR_OPCODE_GET_SAFETY_STATE:
        {
            uint8_t p[40];
            memset(p, 0, sizeof(p));
            wr_u32(&p[SS_OFF_CHALLENGE], st_armed ? 0u : st_challenge);
            wr_u32(&p[SS_OFF_SESSION], st_session);
            p[SS_OFF_STATE] = st_armed ? AXDR_SAFETY_ARMED
                                       : AXDR_SAFETY_DISARMED;
            /* 与修复后固件同语义：查询即评估互锁——静止(相输出关断+无
             * 故障)三位全齐；运行中相输出使能位清零 */
            p[SS_OFF_INTERLOCK] =
                (st_authorized != 0) ? AXDR_SAFETY_INTERLOCK_FAULTS_CLEAR
                                     : AXDR_SAFETY_INTERLOCK_READY_TO_ARM;
            p[SS_OFF_RUN_AUTHORIZED] = st_authorized ? 1u : 0u;
            memcpy(resp, p, sizeof(p));
            *resp_len = (uint16_t)sizeof(p);
            *ack = AXDR_ACK_ACCEPTED;
            return AXDR_REASON_NONE;
        }
        case AXDR_OPCODE_SAFETY_ARM:
            if (req_len != 8u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            if (rd_u32(req_payload) != st_challenge)
            {
                return AXDR_REASON_CHALLENGE_MISMATCH;
            }
            st_armed = 1;
            st_session = 0x13572468u;
            *ack = AXDR_ACK_APPLIED;
            return AXDR_REASON_NONE;
        case AXDR_OPCODE_SAFETY_HEARTBEAT:
            if (req_len != 4u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            if ((!st_armed) || (rd_u32(req_payload) != st_session))
            {
                return AXDR_REASON_SESSION_MISMATCH;
            }
            *ack = AXDR_ACK_APPLIED;
            return AXDR_REASON_NONE;
        case AXDR_OPCODE_GET_MOTOR_PROFILE:
        {
            uint8_t p[80];
            memset(p, 0, sizeof(p));
            wr_u16(&p[0], 1u);
            wr_u16(&p[2], (uint16_t)sizeof(p));
            wr_u32(&p[4], 1u);           /* profile_id */
            wr_u32(&p[8], 2u);           /* revision */
            wr_u32(&p[12], 0xCAFEBABEu); /* profile_crc32 */
            memcpy(resp, p, sizeof(p));
            *resp_len = (uint16_t)sizeof(p);
            *ack = AXDR_ACK_ACCEPTED;
            return AXDR_REASON_NONE;
        }
        case AXDR_OPCODE_CONFIRM_MOTOR_PROFILE:
            if (req_len != 12u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            if (rd_u32(req_payload) != 1u || rd_u32(&req_payload[4]) != 2u ||
                rd_u32(&req_payload[8]) != 0xCAFEBABEu)
            {
                return AXDR_REASON_MOTOR_PROFILE_MISMATCH;
            }
            st_profile_ok = 1;
            *ack = AXDR_ACK_APPLIED;
            return AXDR_REASON_NONE;
        case AXDR_OPCODE_SET_SPEED:
            if (req_len != 16u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            if ((!st_armed) || (rd_u32(req_payload) != st_session))
            {
                return AXDR_REASON_SESSION_MISMATCH;
            }
            if ((rd_f32(&req_payload[12]) <= 0.0f) ||
                (rd_f32(&req_payload[12]) > 8.0f))
            {
                return AXDR_REASON_OUT_OF_RANGE;
            }
            st_authorized = 1; /* 模拟 authorize_run 副作用 */
            st_lease_deadline = now_ms() + rd_u16(&req_payload[4]);
            st_applied_count++;
            st_stop_reason = 0;
            if (st_tel_state == DRIVE_STATE_STOP)
            {
                /* 未对齐首启动走 STARTING（1.6s 对齐）；已对齐直进 RUN */
                st_tel_state = (st_aligned != 0) ? DRIVE_STATE_RUN
                                                : DRIVE_STATE_STARTING;
                st_start_ms = now_ms();
            }
            *ack = AXDR_ACK_APPLIED;
            *resp_len = st_control_state(resp);
            return AXDR_REASON_NONE;
        case AXDR_OPCODE_ALIGN_ENCODER:
            /* 宿主布局（makeEncoderAlignmentRequest）：
             * session/lease/ramp/id_target_A@8/forced_angle@12(=0) */
            if (req_len != 16u)
            {
                return AXDR_REASON_INVALID_PAYLOAD;
            }
            if ((!st_armed) || (rd_u32(req_payload) != st_session))
            {
                return AXDR_REASON_SESSION_MISMATCH;
            }
            if ((rd_f32(&req_payload[8]) <= 0.0f) ||
                (rd_f32(&req_payload[8]) > 2.0f) ||
                (rd_f32(&req_payload[12]) != 0.0f))
            {
                return AXDR_REASON_OUT_OF_RANGE;
            }
            /* 与固件同语义：授权 + 固定 2s 租约（覆盖 1.6s 对齐） */
            st_authorized = 1;
            st_lease_deadline = now_ms() + 2000u;
            if (st_aligned == 0)
            {
                st_tel_state = DRIVE_STATE_STARTING;
                st_start_ms = now_ms();
            }
            *ack = AXDR_ACK_APPLIED;
            *resp_len = st_control_state(resp);
            return AXDR_REASON_NONE;
        case AXDR_OPCODE_GET_CONTROL_STATE:
            *ack = AXDR_ACK_ACCEPTED;
            *resp_len = st_control_state(resp);
            return AXDR_REASON_NONE;
        case AXDR_OPCODE_GET_PROTOCOL_INFO:
        {
            uint8_t p[24];
            memset(p, 0, sizeof(p));
            wr_u16(&p[0], 1u);  /* command_version */
            wr_u16(&p[2], 1u);  /* telemetry_version */
            wr_u16(&p[4], 160u); /* telemetry_frame_size */
            wr_u32(&p[12], AXDR_CAPABILITY_READ_ONLY_QUERIES |
                              AXDR_CAPABILITY_SAFETY_SESSION |
                              AXDR_CAPABILITY_USB_MOTION_CONTROL |
                              AXDR_CAPABILITY_CONTROL_LEASE |
                              AXDR_CAPABILITY_ENCODER_ALIGNMENT |
                              AXDR_CAPABILITY_MOTOR_PROFILE |
                              AXDR_CAPABILITY_MOTOR_PROFILE_CONFIRMATION |
                              AXDR_CAPABILITY_LINK_DIAGNOSTICS);
            wr_u32(&p[16], (1u << AXDR_OPCODE_SET_SPEED) |
                              (1u << AXDR_OPCODE_SET_CURRENT) |
                              (1u << AXDR_OPCODE_ALIGN_ENCODER) |
                              (1u << AXDR_OPCODE_CONTROL_STOP));
            memcpy(resp, p, sizeof(p));
            *resp_len = (uint16_t)sizeof(p);
            *ack = AXDR_ACK_ACCEPTED;
            return AXDR_REASON_NONE;
        }
        case AXDR_OPCODE_CONTROL_STOP:
            /* 只撤运行授权不停会话（与固件同语义：session 归 ARM 层） */
            st_stop_reason = (int)AXDR_CONTROL_STOP_HOST_REQUEST;
            st_authorized = 0;
            st_tel_state = DRIVE_STATE_STOP;
            *ack = AXDR_ACK_APPLIED;
            return AXDR_REASON_NONE;
        case AXDR_OPCODE_SAFETY_DISARM:
            st_armed = 0;
            st_session = 0u;
            st_profile_ok = 0;
            st_stop_reason = 3; /* SAFETY_SESSION */
            st_authorized = 0;
            st_tel_state = DRIVE_STATE_STOP;
            *ack = AXDR_ACK_APPLIED;
            return AXDR_REASON_NONE;
        default:
            *ack = AXDR_ACK_ACCEPTED;
            return AXDR_REASON_NONE;
    }
}

static void st_inject_telemetry(void)
{
    uint8_t f[AXDR_TELEMETRY_V1_FRAME_SIZE];
    uint8_t state = (uint8_t)st_tel_state;

    st_lease_poll();
    /* 模拟对齐时序：STARTING 停留 1.6s 再转 RUN（对齐完成置位） */
    if ((st_tel_state == DRIVE_STATE_STARTING) &&
        ((now_ms() - st_start_ms) > 1600u))
    {
        st_tel_state = DRIVE_STATE_RUN;
        st_aligned = 1; /* 与固件 drive 对齐完成语义一致 */
    }
    state = (uint8_t)st_tel_state;
    memset(f, 0, sizeof(f));
    wr_u32(&f[0], AXDR_WIRE_MAGIC);
    f[4] = 1u;
    wr_u16(&f[6], (uint16_t)AXDR_TELEMETRY_V1_FRAME_SIZE);
    wr_u32(&f[TL_OFF_SEQ], ++st_tel_seq);
    f[TL_OFF_STATE] = state;
    wr_u32(&f[AXDR_TELEMETRY_V1_FRAME_SIZE - 4u],
           axdr_command_crc32(f, AXDR_TELEMETRY_V1_FRAME_SIZE - 4u));
    if ((st_rx_len + sizeof(f)) <= sizeof(st_rx))
    {
        memcpy(&st_rx[st_rx_len], f, sizeof(f));
        st_rx_len += (unsigned)sizeof(f);
    }
}

static int st_open(const char *name)
{
    (void)name;
    st_rx_len = 0u;
    st_tel_seq = 0u;
    st_armed = 0;
    st_session = 0u;
    st_tel_state = DRIVE_STATE_STOP;
    st_authorized = 0;
    st_profile_ok = 0;
    st_aligned = 0;
    st_stop_reason = 0;
    st_lease_count = 0;
    st_applied_count = 0;
    st_lease_deadline = 0u;
    axdr_command_core_init(&st_core);
    return 1;
}

static int st_write(const uint8_t *data, unsigned len)
{
    uint8_t resp[AXDR_COMMAND_MAX_RESPONSE_SIZE];
    const uint16_t framesz = axdr_command_core_process(
        &st_core, data, (uint16_t)len, st_handler, NULL, resp,
        (uint16_t)sizeof(resp));

    /* 状态推进（STARTING 起步/租约计时）全在 st_handler 各 case 内，
     * 与真固件"命令处理即生效"同构 */
    if (framesz > 0u)
    {
        if ((st_rx_len + framesz) <= sizeof(st_rx))
        {
            memcpy(&st_rx[st_rx_len], resp, framesz);
            st_rx_len += framesz;
        }
    }
    st_inject_telemetry();
    return (int)len;
}

static int st_read(uint8_t *data, unsigned cap)
{
    /* 每次读取附带注入一帧遥测：模拟 20Hz 流（读周期 ~10ms → 密集注入，
     * 序列连号；真硬件上此路径不存在） */
    st_inject_telemetry();
    {
        const unsigned n = (st_rx_len < cap) ? st_rx_len : cap;
        if (n == 0u)
        {
            return 0;
        }
        memcpy(data, st_rx, n);
        memmove(st_rx, &st_rx[n], st_rx_len - n);
        st_rx_len -= n;
        return (int)n;
    }
}

static void st_close(void)
{
    st_rx_len = 0u;
}

static const transport_t selftest_transport = {
    st_open, st_write, st_read, st_close};

/* ============================================================
 * 帧层 + 流分割 + 事务
 * ============================================================ */
static uint32_t g_seq;

static unsigned build_request(uint8_t *buf, uint16_t opcode,
                              const uint8_t *payload, uint16_t plen)
{
    wr_u32(&buf[0], AXDR_COMMAND_REQUEST_MAGIC);
    wr_u16(&buf[4], (uint16_t)AXDR_COMMAND_VERSION);
    wr_u16(&buf[6], (uint16_t)(AXDR_COMMAND_REQUEST_HEADER_SIZE + plen +
                               AXDR_COMMAND_CRC_SIZE));
    ++g_seq;
    wr_u32(&buf[8], g_seq);
    wr_u16(&buf[12], opcode);
    wr_u16(&buf[14], plen);
    if (plen > 0u)
    {
        memcpy(&buf[AXDR_COMMAND_REQUEST_HEADER_SIZE], payload, plen);
    }
    wr_u32(&buf[AXDR_COMMAND_REQUEST_HEADER_SIZE + plen],
           axdr_command_crc32(buf, AXDR_COMMAND_REQUEST_HEADER_SIZE + plen));
    return AXDR_COMMAND_REQUEST_HEADER_SIZE + plen + AXDR_COMMAND_CRC_SIZE;
}

typedef struct
{
    uint32_t sequence;
    uint16_t opcode;
    uint8_t ack_state;
    uint8_t reason;
    uint16_t payload_length;
    uint8_t payload[AXDR_COMMAND_MAX_RESPONSE_SIZE];
} response_t;

typedef struct
{
    uint32_t sequence;
    uint32_t tick_ms;
    uint8_t state;
    uint32_t fault;
    float spd_m;
    float cmd_wr;
    float vbus;
} telem_t;

typedef struct
{
    uint8_t buf[2048];
    unsigned len;
    unsigned bad_crc;
    unsigned resp_count;
    unsigned tel_count;
    unsigned seq_gap;
} splitter_t;

static splitter_t g_sp;
static unsigned long g_raw_bytes; /* 诊断：transport 层实际收到的原始字节 */
static response_t g_last_resp;
static telem_t g_last_tel;
static uint32_t g_tel_first_seq;
static uint32_t g_tel_last_seq;
static int g_tel_frames;

/* 从 g_sp.buf 头部分割一帧。返回 1=应答 2=遥测 0=不完整 -1=丢弃滑动 */
static int split_one(void)
{
    unsigned frame;
    const uint32_t magic = rd_u32(g_sp.buf);

    if ((magic != AXDR_COMMAND_RESPONSE_MAGIC) && (magic != AXDR_WIRE_MAGIC))
    {
        memmove(g_sp.buf, &g_sp.buf[1], g_sp.len - 1u);
        g_sp.len--;
        return -1;
    }
    if (magic == AXDR_COMMAND_RESPONSE_MAGIC)
    {
        /* 应答头布局（packed 累加）：ack@14 / reason@15 / plen@16，
         * payload 从 20（RESPONSE_HEADER_SIZE）起。 */
        frame = AXDR_COMMAND_RESPONSE_HEADER_SIZE + rd_u16(&g_sp.buf[16]) +
                AXDR_COMMAND_CRC_SIZE;
    }
    else
    {
        frame = AXDR_TELEMETRY_V1_FRAME_SIZE;
    }
    if (g_sp.len < frame)
    {
        return 0;
    }
    if (axdr_command_crc32(g_sp.buf, frame - AXDR_COMMAND_CRC_SIZE) !=
        rd_u32(&g_sp.buf[frame - AXDR_COMMAND_CRC_SIZE]))
    {
        g_sp.bad_crc++;
        memmove(g_sp.buf, &g_sp.buf[1], g_sp.len - 1u);
        g_sp.len--;
        return -1;
    }

    if (magic == AXDR_COMMAND_RESPONSE_MAGIC)
    {
        const uint16_t plen = rd_u16(&g_sp.buf[16]);
        g_last_resp.sequence = rd_u32(&g_sp.buf[8]);
        g_last_resp.opcode = rd_u16(&g_sp.buf[12]);
        g_last_resp.ack_state = g_sp.buf[14];
        g_last_resp.reason = g_sp.buf[15];
        g_last_resp.payload_length = plen;
        if ((plen > 0u) && (plen <= sizeof(g_last_resp.payload)))
        {
            memcpy(g_last_resp.payload, &g_sp.buf
                       [AXDR_COMMAND_RESPONSE_HEADER_SIZE],
                   plen);
        }
        g_sp.resp_count++;
        memmove(g_sp.buf, &g_sp.buf[frame], g_sp.len - frame);
        g_sp.len -= frame;
        return 1;
    }

    /* 遥测帧 */
    {
        telem_t t;
        t.sequence = rd_u32(&g_sp.buf[TL_OFF_SEQ]);
        t.tick_ms = rd_u32(&g_sp.buf[TL_OFF_TICK]);
        t.state = g_sp.buf[TL_OFF_STATE];
        t.fault = rd_u32(&g_sp.buf[TL_OFF_FAULT]);
        t.cmd_wr = rd_f32(&g_sp.buf[TL_OFF_CMD_WR]);
        t.spd_m = rd_f32(&g_sp.buf[TL_OFF_SPD_M]);
        t.vbus = rd_f32(&g_sp.buf[TL_OFF_VBUS]);
        if (g_tel_frames == 0)
        {
            g_tel_first_seq = t.sequence;
        }
        else if (t.sequence != (g_tel_last_seq + 1u))
        {
            g_sp.seq_gap++;
        }
        g_tel_last_seq = t.sequence;
        g_last_tel = t;
        g_tel_frames++;
        g_sp.tel_count++;
    }
    memmove(g_sp.buf, &g_sp.buf[frame], g_sp.len - frame);
    g_sp.len -= frame;
    return 2;
}

/* 采集至多 ms 毫秒字节流并分割。返回 1=期间收到新应答（提前返回——
 * 否则续租周期被拉满，会越过 300ms 租约）。探针场景无请求在飞，
 * 行为等同固定时长采集。 */
static int pump(unsigned ms)
{
    uint8_t buf[512];
    const uint32_t t0 = now_ms();
    int got_resp = 0;

    while ((now_ms() - t0) < ms)
    {
        const int n = tr->read_bytes(buf, sizeof(buf));
        if (n < 0)
        {
            return -1;
        }
        if (n > 0)
        {
            unsigned i;
            g_raw_bytes += (unsigned long)n;
            for (i = 0; i < (unsigned)n; i++)
            {
                if (g_sp.len < sizeof(g_sp.buf))
                {
                    g_sp.buf[g_sp.len++] = buf[i];
                }
            }
        }
        while (g_sp.len >= AXDR_COMMAND_MIN_RESPONSE_SIZE)
        {
            const int r = split_one();
            if (r == 1)
            {
                got_resp = 1;
            }
            if (r == 0)
            {
                break; /* 帧未凑满：等下一轮字节 */
            }
            /* r==1 出一帧；r==-1 滑动 1 字节重新对齐帧头——必须继续
             * 循环，否则每轮 ReadFile 只滑 1 字节，真机字节流一旦有
             * 偏移就永远追不上帧头（2026-10-05 上机 frames=0 根因）。 */
        }
        if (got_resp != 0)
        {
            return got_resp;
        }
    }
    return got_resp;
}

/* 发请求 + 等应答（300ms 无应答重发一次）。返回 0=拿到匹配应答 */
static int transact(uint16_t opcode, const uint8_t *payload, uint16_t plen)
{
    uint8_t buf[AXDR_COMMAND_MAX_REQUEST_SIZE];
    int attempt;

    for (attempt = 0; attempt < 2; attempt++)
    {
        const unsigned len = build_request(buf, opcode, payload, plen);
        const uint32_t want_seq = g_seq;
        if (tr->write_bytes(buf, len) < 0)
        {
            return -1;
        }
        pump(300);
        if ((g_last_resp.sequence == want_seq) &&
            (g_last_resp.opcode == opcode))
        {
            return 0;
        }
    }
    return -1;
}

/* want_reason 用 0xFF 作"任意"哨兵（合法枚举最大 17）。name 允许 NULL
 * （静默续租场景：只有失败才打印）。返回 1=符合预期。 */
static int tx_expect(uint16_t opcode, const uint8_t *payload, uint16_t plen,
                     uint8_t want_ack, uint8_t want_reason, const char *name)
{
    if (transact(opcode, payload, plen) != 0)
    {
        printf("[FAIL] %s: 无应答\n", (name != NULL) ? name : "tx");
        g_fails++;
        return 0;
    }
    if ((g_last_resp.ack_state != want_ack) ||
        ((want_reason != 0xFFu) && (g_last_resp.reason != want_reason)))
    {
        printf("[FAIL] %s: ack=%u reason=%u（期望 ack=%u reason=%s）\n",
               (name != NULL) ? name : "tx", g_last_resp.ack_state,
               g_last_resp.reason, want_ack,
               (want_reason == 0xFFu) ? "任意" : "指定值");
        g_fails++;
        return 0;
    }
    if ((name != NULL) && (want_ack == AXDR_ACK_APPLIED))
    {
        printf("[PASS] %s\n", name);
    }
    return 1;
}

/* ============================================================
 * 验收旅程
 * ============================================================ */
static int g_session;
static uint32_t g_challenge;
static uint32_t g_last_hb_ms;

/* 心跳保活：运动旅程长于任何心跳阈值，须在循环中穿插（间隔取
 * 阈值/2 以下；arm 时配 2000ms 档）。 */
static void heartbeat_keepalive(unsigned period_ms)
{
    uint8_t hb[4];
    if ((now_ms() - g_last_hb_ms) < period_ms)
    {
        return;
    }
    g_last_hb_ms = now_ms();
    wr_u32(&hb[0], (uint32_t)g_session);
    (void)tx_expect(AXDR_OPCODE_SAFETY_HEARTBEAT, hb, 4, AXDR_ACK_APPLIED,
                    0xFFu, NULL);
}

static int run_probe(unsigned ms, const char *tag)
{
    g_tel_frames = 0;
    g_sp.bad_crc = 0u;
    g_sp.seq_gap = 0u;
    g_raw_bytes = 0u;
    /* 真机行为（2026-10-05 上机定位）：固件 CDC 遥测流在主机长时间
     * 纯读（无命令活动）时吐完 16 槽队列积压即静默；任意主机写
     * （哪怕 4 字节）即恢复 20Hz 全速推流。GUI 连接即发预检所以
     * 从不复现。探针先发一个只读查询唤醒推流，再计时采样。 */
    (void)transact(AXDR_OPCODE_GET_PROTOCOL_INFO, NULL, 0);
    g_tel_frames = 0;
    g_sp.bad_crc = 0u;
    pump(ms);
    printf("--- %s: frames=%u bad_crc=%u seq_gaps=%u state=%u fault=0x%x "
           "vbus=%.2f raw=%lu residual=%u\n",
           tag, g_tel_frames, g_sp.bad_crc, g_sp.seq_gap, g_last_tel.state,
           g_last_tel.fault, (double)g_last_tel.vbus, g_raw_bytes,
           (unsigned)g_sp.len);
    expect("遥测帧数达标", (g_tel_frames * 1000u) >= (15u * ms));
    expect("遥测 CRC 零错", g_sp.bad_crc == 0u);
    expect("遥测序列零跳号", g_sp.seq_gap == 0u);
    expect("无活动故障位", g_last_tel.fault == 0u);
    return (g_tel_frames >= 10) ? 0 : 1;
}

/* P1：查询组 */
static void run_queries(void)
{
    uint32_t profile_crc = 0u;
    uint8_t req[12];

    (void)tx_expect(AXDR_OPCODE_GET_PROTOCOL_INFO, NULL, 0, AXDR_ACK_ACCEPTED,
                    0xFFu, "GetProtocolInfo");
    if ((g_last_resp.opcode == AXDR_OPCODE_GET_PROTOCOL_INFO) &&
        (g_last_resp.payload_length >= 24u))
    {
        expect("协议版本=1", rd_u16(g_last_resp.payload) == 1u);
        expect("遥测版本=1/帧长=160",
               (rd_u16(&g_last_resp.payload[2]) == 1u) &&
                   (rd_u16(&g_last_resp.payload[4]) == 160u));
        /* Mit_Tool 校准按钮同款双闸（supportsEncoderAlignment）：能力位
         * caps bit11 与掩码位 mask 0x12 必须同时在——2026-10-05 上机
         * "按钮灰"根因：只加了掩码位漏能力位。协议层从此处拦住同类回归。 */
        expect("对齐能力双闸就绪（caps bit11 + mask 0x12）",
               ((rd_u32(&g_last_resp.payload[12]) &
                 AXDR_CAPABILITY_ENCODER_ALIGNMENT) != 0u) &&
                   ((rd_u32(&g_last_resp.payload[16]) &
                     (1u << AXDR_OPCODE_ALIGN_ENCODER)) != 0u));
    }

    (void)tx_expect(AXDR_OPCODE_GET_DEVICE_INFO, NULL, 0, AXDR_ACK_ACCEPTED,
                    0xFFu, NULL);

    if (tx_expect(AXDR_OPCODE_GET_MOTOR_PROFILE, NULL, 0, AXDR_ACK_ACCEPTED,
                  0xFFu, "GetMotorProfile") &&
        (g_last_resp.payload_length >= 80u))
    {
        expect("档案 id=1/rev=2",
               (rd_u32(&g_last_resp.payload[4]) == 1u) &&
                   (rd_u32(&g_last_resp.payload[8]) == 2u));
        profile_crc = rd_u32(&g_last_resp.payload[12]);
        expect("档案 CRC 非零", profile_crc != 0u);
    }

    if (profile_crc != 0u)
    {
        wr_u32(&req[0], 1u);
        wr_u32(&req[4], 2u);
        wr_u32(&req[8], profile_crc ^ 0xFFFFFFFFu);
        (void)tx_expect(AXDR_OPCODE_CONFIRM_MOTOR_PROFILE, req, 12,
                        AXDR_ACK_REJECTED, AXDR_REASON_MOTOR_PROFILE_MISMATCH,
                        "错档案 CRC 拒");
        wr_u32(&req[8], profile_crc);
        (void)tx_expect(AXDR_OPCODE_CONFIRM_MOTOR_PROFILE, req, 12,
                        AXDR_ACK_APPLIED, 0xFFu, "ConfirmProfile 确认");
    }
}

/* P2：ARM 链（with_negative=先错 challenge）。返回 1=已 ARM */
static int run_arm(int with_negative)
{
    uint8_t req[8];

    if (!tx_expect(AXDR_OPCODE_GET_SAFETY_STATE, NULL, 0, AXDR_ACK_ACCEPTED,
                   0xFFu, "GetSafetyState") ||
        (g_last_resp.payload_length < 40u))
    {
        expect("safety_state payload>=40B",
               g_last_resp.payload_length >= 40u);
        return 0;
    }
    g_challenge = rd_u32(&g_last_resp.payload[SS_OFF_CHALLENGE]);

    /* 上位机闸门模拟（Mit_Tool requestArm 同款前置）：interlock 不到位
     * 就不发起 ARM——固件若把互锁刷新只留在 ARM/poll 内部，此处即红，
     * 死锁类回归在回环里直接暴露，不等上机。 */
    {
        const uint8_t interlock = g_last_resp.payload[SS_OFF_INTERLOCK];
        expect("ARM 前置互锁就绪（interlock==READY_TO_ARM）",
               interlock == (uint8_t)AXDR_SAFETY_INTERLOCK_READY_TO_ARM);
        if (interlock != (uint8_t)AXDR_SAFETY_INTERLOCK_READY_TO_ARM)
        {
            printf("[FAIL] 上位机闸门拦截：interlock=0x%02X ≠ READY_TO_ARM，"
                   "不发 ARM（固件未在 GetSafetyState 刷新互锁？）\n",
                   interlock);
            return 0;
        }
    }

    if (with_negative)
    {
        wr_u32(&req[0], g_challenge ^ 0x12345678u);
        wr_u32(&req[4], 1000u);
        (void)tx_expect(AXDR_OPCODE_SAFETY_ARM, req, 8, AXDR_ACK_REJECTED,
                        AXDR_REASON_CHALLENGE_MISMATCH, "错 challenge ARM 拒");
    }

    wr_u32(&req[0], g_challenge);
    wr_u32(&req[4], 2000u); /* 最大档：给长运动旅程留心跳余量 */
    if (!tx_expect(AXDR_OPCODE_SAFETY_ARM, req, 8, AXDR_ACK_APPLIED, 0xFFu,
                   "ARM 正确 challenge"))
    {
        return 0;
    }
    g_last_hb_ms = now_ms();

    if (tx_expect(AXDR_OPCODE_GET_SAFETY_STATE, NULL, 0, AXDR_ACK_ACCEPTED,
                  0xFFu, "GetSafetyState(ARMED)") &&
        (g_last_resp.payload_length >= 40u))
    {
        g_session = (int)rd_u32(&g_last_resp.payload[SS_OFF_SESSION]);
        expect("会话非零且 ARMED 且未授权运动",
               (g_session != 0) &&
                   (g_last_resp.payload[SS_OFF_STATE] == AXDR_SAFETY_ARMED) &&
                   (g_last_resp.payload[SS_OFF_RUN_AUTHORIZED] == 0u));
    }
    return g_session != 0;
}

/* P2b：错会话运动拒（固定 14B 合法 payload，拒绝应来自会话而非长度） */
static void run_motion_negative(void)
{
    uint8_t req[16];
    wr_u32(&req[0], 0xDEADu);
    wr_u16(&req[4], 300u);
    wr_u16(&req[6], 0u);
    wr_f32(&req[8], 0.0f);
    wr_f32(&req[12], 2.0f);
    (void)tx_expect(AXDR_OPCODE_SET_SPEED, req, 16, AXDR_ACK_REJECTED, 0xFFu,
                    "错会话 SetSpeed 拒");
}

static int refresh_session(void)
{
    if (transact(AXDR_OPCODE_GET_SAFETY_STATE, NULL, 0) != 0)
    {
        return 0;
    }
    if (g_last_resp.payload_length < 40u)
    {
        return 0;
    }
    g_session = (int)rd_u32(&g_last_resp.payload[SS_OFF_SESSION]);
    return g_session != 0;
}

/* P2.5：编码器校准旅程（Mit_Tool"开始对齐"同款命令链）。返回 1=已对齐 */
static int run_align(void)
{
    uint8_t req[16];
    uint32_t t0;
    uint32_t t_done = 0u;
    int had_starting = 0;

    /* 负 path：错会话拒（宿主布局 id@8/forced_angle@12） */
    wr_u32(&req[0], 0xDEADu);
    wr_u16(&req[4], 1000u);
    wr_u16(&req[6], 300u);
    wr_f32(&req[8], 2.0f);
    wr_f32(&req[12], 0.0f);
    (void)tx_expect(AXDR_OPCODE_ALIGN_ENCODER, req, 16, AXDR_ACK_REJECTED,
                    0xFFu, "错会话校准拒");

    if (!refresh_session())
    {
        expect("校准前会话有效", 0);
        return 0;
    }
    wr_u32(&req[0], (uint32_t)g_session);
    if (!tx_expect(AXDR_OPCODE_ALIGN_ENCODER, req, 16, AXDR_ACK_APPLIED, 0xFFu,
                   "校准命令 Applied"))
    {
        return 0;
    }

    /* 轮询 control_state.alignment_valid：固件对齐 1.5s + 轮询余量 */
    t0 = now_ms();
    while ((now_ms() - t0) < 3500u)
    {
        pump(100);
        heartbeat_keepalive(800u); /* 真机保活：授权态虽归租约，ARM 钟别断 */
        if (g_last_tel.state == DRIVE_STATE_STARTING)
        {
            had_starting = 1;
        }
        if (tx_expect(AXDR_OPCODE_GET_CONTROL_STATE, NULL, 0,
                      AXDR_ACK_ACCEPTED, 0xFFu, NULL) &&
            (g_last_resp.payload_length >= 24u) &&
            (g_last_resp.payload[23] == 1u))
        {
            t_done = now_ms() - t0;
            break;
        }
    }
    if (t_done != 0u)
    {
        printf("[INFO] 校准完成耗时 %u ms（had_starting=%d）\n", t_done,
               had_starting);
        expect("校准：alignment_valid 置位", 1);
    }
    else
    {
        expect("校准：3.5s 内 alignment_valid 置位", 0);
        return 0;
    }

    (void)tx_expect(AXDR_OPCODE_CONTROL_STOP, NULL, 0, AXDR_ACK_APPLIED, 0xFFu,
                    "校准收尾 ControlStop");
    pump(300);
    expect("校准后回到 STOP", g_last_tel.state == DRIVE_STATE_STOP);
    return 1;
}

/* P3：运动 + 曲线 CSV + 停止。expect_slow_start=1 验首上电对齐时序，
 * =0 验免对齐直进 RUN。 */
static void run_motion(const char *csv_path, float speed, float iq,
                       int expect_slow_start)
{
    uint8_t req[16];
    FILE *csv = NULL;
    uint32_t t0;
    uint32_t t_run = 0u;
    int had_starting = 0;

    if (!refresh_session())
    {
        expect("运动前会话有效", 0);
        return;
    }

    wr_u32(&req[0], (uint32_t)g_session);
    wr_u16(&req[4], 300u);
    wr_u16(&req[6], 0u);
    wr_f32(&req[8], speed);
    wr_f32(&req[12], iq);
    if (!tx_expect(AXDR_OPCODE_SET_SPEED, req, 16, AXDR_ACK_APPLIED, 0xFFu,
                   "SetSpeed 启动"))
    {
        return;
    }

    if (csv_path != NULL)
    {
        csv = fopen(csv_path, "w");
        if (csv != NULL)
        {
            fprintf(csv, "tick_ms,seq,state,spd_m,cmd_wr,vbus\n");
        }
    }

    /* 启动观察：到 RUN 的耗时。首上电对齐 ~1.5s；免对齐 <800ms。 */
    t0 = now_ms();
    while ((now_ms() - t0) < 5000u)
    {
        pump(50);
        if (g_last_tel.state == DRIVE_STATE_RUN)
        {
            t_run = now_ms() - t0;
            break;
        }
        if (g_last_tel.state == DRIVE_STATE_STARTING)
        {
            had_starting = 1;
        }
        if ((now_ms() - t0) > 200u)
        {
            /* 保持租约（静默失败也要继续观察状态） */
            (void)tx_expect(AXDR_OPCODE_SET_SPEED, req, 16,
                            AXDR_ACK_APPLIED, 0xFFu, NULL);
        }
        heartbeat_keepalive(800u);
    }
    if (t_run != 0u)
    {
        printf("[INFO] 到 RUN 耗时 %u ms（had_starting=%d）\n", t_run,
               had_starting);
        if (expect_slow_start != 0)
        {
            expect("首上电：STARTING 对齐后进 RUN",
                   (had_starting != 0) && (t_run >= 1000u));
        }
        else
        {
            expect("免对齐：无 STARTING 且 <800ms 直进 RUN",
                   (had_starting == 0) && (t_run < 800u));
        }
    }
    else
    {
        expect("5s 内到达 RUN", 0);
    }

    /* 3s 曲线：250ms 续租节奏 */
    t0 = now_ms();
    while ((now_ms() - t0) < 3000u)
    {
        wr_u32(&req[0], (uint32_t)g_session);
        if (tx_expect(AXDR_OPCODE_SET_SPEED, req, 16, AXDR_ACK_APPLIED, 0xFFu,
                      NULL) == 0)
        {
            expect("运动中续租连续", 0);
            break;
        }
        pump(200);
        heartbeat_keepalive(800u);
        if (csv != NULL)
        {
            fprintf(csv, "%u,%u,%u,%.3f,%.3f,%.2f\n", g_last_tel.tick_ms,
                    g_last_tel.sequence, g_last_tel.state,
                    (double)g_last_tel.spd_m, (double)g_last_tel.cmd_wr,
                    (double)g_last_tel.vbus);
        }
    }
    if (csv != NULL)
    {
        (void)fclose(csv);
        printf("[INFO] 曲线已存 %s\n", csv_path);
    }

    expect("运行中零故障", g_last_tel.fault == 0u);

    (void)tx_expect(AXDR_OPCODE_CONTROL_STOP, NULL, 0, AXDR_ACK_APPLIED, 0xFFu,
                    "ControlStop");
    if (tx_expect(AXDR_OPCODE_GET_CONTROL_STATE, NULL, 0, AXDR_ACK_ACCEPTED,
                  0xFFu, NULL) &&
        (g_last_resp.payload_length >= CS_OFF_STOP_REASON + 1u))
    {
        expect("stop_reason=HOST_REQUEST(1)",
               g_last_resp.payload[CS_OFF_STOP_REASON] ==
                   AXDR_CONTROL_STOP_HOST_REQUEST);
    }
    pump(300);
    expect("遥测回到 STOP", g_last_tel.state == DRIVE_STATE_STOP);
}

/* P4：租约超时（发完不再续租，零速授权） */
static void run_lease_timeout(void)
{
    uint8_t req[16];

    if (!refresh_session())
    {
        expect("租约场景会话有效", 0);
        return;
    }
    wr_u32(&req[0], (uint32_t)g_session);
    wr_u16(&req[4], 300u);
    wr_u16(&req[6], 0u);
    wr_f32(&req[8], 0.0f);
    wr_f32(&req[12], 2.0f);
    if (!tx_expect(AXDR_OPCODE_SET_SPEED, req, 16, AXDR_ACK_APPLIED, 0xFFu,
                   "SetSpeed(租约 300ms)"))
    {
        return;
    }
    printf("[INFO] 越过租约等待 600ms...\n");
    {
        const uint32_t t0 = now_ms();
        while ((now_ms() - t0) < 600u)
        {
            (void)pump(100); /* 小步消化遥测，防 splitter 积压 */
        }
    }

    if (tx_expect(AXDR_OPCODE_GET_SAFETY_STATE, NULL, 0, AXDR_ACK_ACCEPTED,
                  0xFFu, NULL) &&
        (g_last_resp.payload_length >= 40u))
    {
        expect("租约超时已撤权",
               g_last_resp.payload[SS_OFF_RUN_AUTHORIZED] == 0u);
    }
    /* control_state.timeout_count 是租约超时计数（safety_state 的是心跳） */
    if (tx_expect(AXDR_OPCODE_GET_CONTROL_STATE, NULL, 0, AXDR_ACK_ACCEPTED,
                  0xFFu, NULL) &&
        (g_last_resp.payload_length >= CS_OFF_STOP_REASON + 1u))
    {
        expect("stop_reason=LEASE_TIMEOUT(2)",
               g_last_resp.payload[CS_OFF_STOP_REASON] ==
                   AXDR_CONTROL_STOP_LEASE_TIMEOUT);
        expect("租约超时计数递增",
               rd_u32(&g_last_resp.payload[16]) >= 1u);
    }
    (void)pump(200);
    expect("租约超时后电机已停", g_last_tel.state == DRIVE_STATE_STOP);
}

/* P5：心跳超时（先退出 P4 遗留会话，再 ARM 后不耐心跳） */
static void run_heartbeat_timeout(void)
{
    uint8_t req[8];
    uint32_t prev_timeouts = 0u;

    /* ARMED 态重 ARM 会被拒（INVALID_STATE）——先正常退出上一会话 */
    (void)tx_expect(AXDR_OPCODE_SAFETY_DISARM, NULL, 0, AXDR_ACK_APPLIED,
                    0xFFu, "P5 前退出旧会话");

    if (tx_expect(AXDR_OPCODE_GET_SAFETY_STATE, NULL, 0, AXDR_ACK_ACCEPTED,
                  0xFFu, NULL) &&
        (g_last_resp.payload_length >= 40u))
    {
        prev_timeouts = rd_u32(&g_last_resp.payload[SS_OFF_TIMEOUT_COUNT]);
        g_challenge = rd_u32(&g_last_resp.payload[SS_OFF_CHALLENGE]);
    }
    wr_u32(&req[0], g_challenge);
    wr_u32(&req[4], 1000u);
    if (!tx_expect(AXDR_OPCODE_SAFETY_ARM, req, 8, AXDR_ACK_APPLIED, 0xFFu,
                   "重新 ARM"))
    {
        return;
    }
    (void)refresh_session();
    printf("[INFO] 等待心跳超时（1.3s，阈值 1.0s）...\n");
    {
        const uint32_t t0 = now_ms();
        while ((now_ms() - t0) < 1300u)
        {
            (void)pump(100);
        }
    }

    if (tx_expect(AXDR_OPCODE_GET_SAFETY_STATE, NULL, 0, AXDR_ACK_ACCEPTED,
                  0xFFu, NULL) &&
        (g_last_resp.payload_length >= 40u))
    {
        expect("会话已 DISARMED",
               g_last_resp.payload[SS_OFF_STATE] == AXDR_SAFETY_DISARMED);
        expect("disarm_reason=HEARTBEAT_TIMEOUT(3)",
               g_last_resp.payload[SS_OFF_DISARM_REASON] ==
                   3u /* AXDR_SAFETY_DISARM_HEARTBEAT_TIMEOUT */);
        expect("超时计数 +1",
               rd_u32(&g_last_resp.payload[SS_OFF_TIMEOUT_COUNT]) ==
                   (prev_timeouts + 1u));
    }
    /* 失效会话再运动 → 拒（完整合法 payload，拒因是会话不是长度） */
    {
        uint8_t sp[16];
        wr_u32(&sp[0], (uint32_t)g_session);
        wr_u16(&sp[4], 300u);
        wr_u16(&sp[6], 0u);
        wr_f32(&sp[8], 0.0f);
        wr_f32(&sp[12], 2.0f);
        (void)tx_expect(AXDR_OPCODE_SET_SPEED, sp, 16, AXDR_ACK_REJECTED,
                        0xFFu, "超时会话 SetSpeed 拒");
    }
}

/* P6：收尾 */
static void run_finalize(void)
{
    (void)tx_expect(AXDR_OPCODE_SAFETY_DISARM, NULL, 0, AXDR_ACK_APPLIED,
                    0xFFu, "SafetyDisarm 收尾");
    (void)pump(300);
    expect("最终 state=STOP 且 fault=0",
           (g_last_tel.state == DRIVE_STATE_STOP) &&
               (g_last_tel.fault == 0u));
}

static void emergency_stop(void)
{
    uint8_t buf[AXDR_COMMAND_MAX_REQUEST_SIZE];
    unsigned len = build_request(buf, AXDR_OPCODE_CONTROL_STOP, NULL, 0);
    (void)tr->write_bytes(buf, len);
    (void)pump(200);
    len = build_request(buf, AXDR_OPCODE_SAFETY_DISARM, NULL, 0);
    (void)tr->write_bytes(buf, len);
    (void)pump(200);
    printf("[INFO] 已尝试紧急 STOP+DISARM\n");
}

static void print_usage(void)
{
    printf("用法: usb_acceptance COMx [--reboot-check] [--probe-only] "
           "[--speed X] [--iq Y] [--out file.csv]\n"
           "      usb_acceptance --selftest\n");
}

int main(int argc, char **argv)
{
    const char *port = NULL;
    const char *csv = NULL;
    float speed = 8.0f;
    float iq = 2.0f;
    int reboot_check = 0;
    int probe_only = 0;
    int selftest = 0;
    int i;

    for (i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--selftest") == 0)
        {
            selftest = 1;
        }
        else if (strcmp(argv[i], "--reboot-check") == 0)
        {
            reboot_check = 1;
        }
        else if (strcmp(argv[i], "--probe-only") == 0)
        {
            probe_only = 1;
        }
        else if ((strcmp(argv[i], "--speed") == 0) && (i + 1 < argc))
        {
            speed = (float)atof(argv[++i]);
        }
        else if ((strcmp(argv[i], "--iq") == 0) && (i + 1 < argc))
        {
            iq = (float)atof(argv[++i]);
        }
        else if ((strcmp(argv[i], "--out") == 0) && (i + 1 < argc))
        {
            csv = argv[++i];
        }
        else if (argv[i][0] != '-')
        {
            port = argv[i];
        }
    }

    if (selftest != 0)
    {
        tr = &selftest_transport;
    }
    else
    {
        if (port == NULL)
        {
            print_usage();
            return 2;
        }
#ifdef _WIN32
        tr = &serial_transport;
        g_serial = INVALID_HANDLE_VALUE;
#else
        printf("非 Windows 平台暂无串口实现（用 --selftest）\n");
        return 2;
#endif
    }

    if (tr->open_port(port != NULL ? port : "SELFTEST") == 0)
    {
        printf("串口打开失败: %s\n", (port != NULL) ? port : "-");
        return 3;
    }
    memset(&g_sp, 0, sizeof(g_sp));
    memset(&g_last_resp, 0, sizeof(g_last_resp));
    memset(&g_last_tel, 0, sizeof(g_last_tel));

    printf("=== P7 上位机基准验收（%s%s）===\n", selftest ? "selftest" : port,
           (reboot_check != 0) ? " reboot-check" : "");

    if (run_probe(2000u, "P0 遥测探针") != 0)
    {
        printf("探针不过，终止（不发命令）\n");
        tr->close_port();
        return 4;
    }
    if (probe_only != 0)
    {
        tr->close_port();
        printf("=== %s（probe-only）===\n", (g_fails == 0) ? "全部 PASS" : "FAIL");
        return (g_fails == 0) ? 0 : 5;
    }

    run_queries();
    if (run_arm(1) == 0)
    {
        emergency_stop();
        tr->close_port();
        return 5;
    }
    /* 校准旅程（对齐已完成则幂等）→ 之后的运动按"已对齐直进 RUN"断言 */
    if (run_align() == 0)
    {
        emergency_stop();
        tr->close_port();
        return 5;
    }
    run_motion_negative();

    if (reboot_check != 0)
    {
        run_motion(csv, speed, iq, 0);
        run_finalize();
    }
    else if (selftest != 0)
    {
        /* 模拟端已具备租约撤权语义（P4 回环覆盖）；心跳超时是被测
         * safety runtime 的职责，selftest 不伪造——P5 只在真机上跑。 */
        run_motion(csv, speed, iq, 0);
        run_lease_timeout();
        run_finalize();
    }
    else
    {
        run_motion(csv, speed, iq, 0);
        run_lease_timeout();
        run_heartbeat_timeout();
        run_finalize();
    }

    tr->close_port();
    printf("=== %s（%d 项失败）===\n", (g_fails == 0) ? "全部 PASS" : "FAIL",
           g_fails);
    return (g_fails == 0) ? 0 : 5;
}
