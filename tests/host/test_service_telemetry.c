/**
 * @file test_service_telemetry.c
 * @brief S4 遥测测试：decimation 抽取 / 帧内容映射 / CRC / seq 连号与
 *        覆盖计数 / BUSY 保留重试 / 发送错误丢弃 / 变速清积压。
 * @note 帧填充读 g_foc，测试直接预填已知值后断言线上字节。
 *       场景链的 seq 全程手推：init=0 → S1 发1 → S6 发2 → S7 发12,13
 *       （25帧翻覆9帧）→ S8 发28,29,29,29 → S9 丢30 → S10 清积压。
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "axdr_command_core.h"
#include "axdr_telemetry_contract.h"
#include "common.h"
#include "service_telemetry.h"

foc_t g_foc;

static int fail_count;
static void expect(const char *name, bool cond)
{
    if (!cond)
    {
        (void)printf("FAIL %s\n", name);
        fail_count++;
    }
}

/* tx 捕获：tx_result 注入下一次发送的返回值（0=OK 1=BUSY 其他=错误） */
static uint8_t tx_buf[256];
static uint16_t tx_len;
static int tx_result;
static int tx_with_result(const uint8_t *data, uint16_t length)
{
    if (length <= sizeof(tx_buf))
    {
        memcpy(tx_buf, data, length);
        tx_len = length;
    }
    return tx_result;
}

static void reset_tx(void)
{
    tx_len = 0u;
    tx_result = 0;
}

static float rd_f(uint16_t offset)
{
    float value;
    memcpy(&value, &tx_buf[offset], 4);
    return value;
}

static uint32_t rd_u32(uint16_t offset)
{
    uint32_t value;
    memcpy(&value, &tx_buf[offset], 4);
    return value;
}

static uint16_t rd_u16(uint16_t offset)
{
    uint16_t value;
    memcpy(&value, &tx_buf[offset], 2);
    return value;
}

static void fill_g_foc(void)
{
    memset(&g_foc, 0, sizeof(g_foc));
    g_foc.req = DRIVE_REQ_RUN;
    g_foc.state = DRIVE_STATE_RUN;
    g_foc.mode.sys = debug_mode;
    g_foc.mode.debug = spd_curr_cl;
    g_foc.mode.release = 2u;
    g_foc.mode.calibrat = 3u;
    g_foc.mode.halt = 1u;
    g_foc.enc.primary = ENCODER_TYPE_ABZ;
    g_foc.motor.phase_order = PHASE_ORDER_ABC;
    g_foc.enc.abz.raw = 1234567u;
    g_foc.fault.all = 0u;
    g_foc.out.vd = -0.5f;
    g_foc.out.vq = 2.25f;
    g_foc.ref.id = 0.0f;
    g_foc.ref.iq = 8.0f;     /* 用户限幅 */
    g_foc.ref.iq_lim = 3.5f; /* 速度环 PI 实际输出 */
    g_foc.ref.spd_r = 52.36f;
    g_foc.ref.spd_m = 52.36f;
    g_foc.ref.pos_r = 1.25f;
    g_foc.fb.pos_r_1t = 0.75f;
    g_foc.fb.theta_e = 2.0f;
    g_foc.motor.e_off = 0.5f;
    g_foc.fb.pos_r = 12.5f;
    g_foc.fb.pos_m = 12.5f;
    g_foc.fb.spd_r_raw = 51.9f;
    g_foc.fb.spd_r = 52.1f;
    g_foc.fb.spd_m = 52.1f;
    g_foc.fb.ia = 0.11f;
    g_foc.fb.ib = -0.22f;
    g_foc.fb.ic = 0.11f;
    g_foc.fb.id = 0.01f;
    g_foc.fb.iq = 3.49f;
    g_foc.fb.vbus = 24.1f;
    g_foc.out.duty_a = 0.4f;
    g_foc.out.duty_b = 0.3f;
    g_foc.out.duty_c = 0.3f;
    g_foc.fast_seq = 20u * 5000u; /* t=5s */
}

int main(void)
{
    uint32_t i;
    uint32_t drops_before;

    service_telemetry_init();
    service_telemetry_bind_tx(tx_with_result);
    fill_g_foc();

    /* ---- 1. decimation 抽取：默认 1000 次 capture 触发一帧 ---- */
    for (i = 0u; i < 999u; ++i)
    {
        service_telemetry_capture();
    }
    reset_tx();
    service_telemetry_capture(); /* 第 1000 次 */
    expect("抽取计数 1000", service_telemetry_capture_count() == 1000u);
    service_telemetry_poll();
    expect("抽取到点产生一帧", tx_len == AXDR_TELEMETRY_V1_FRAME_SIZE);
    reset_tx();
    service_telemetry_poll();
    expect("未到点无新帧", tx_len == 0u);

    /* ---- 2. 帧头与模式区 ---- */
    expect("magic AXDR", rd_u32(0) == AXDR_WIRE_MAGIC);
    expect("version=1", tx_buf[4] == 1u);
    expect("frame_size=160", rd_u16(6) == 160u);
    expect("sequence=1", rd_u32(8) == 1u);
    expect("tick_ms=5000", rd_u32(12) == 5000u);
    expect("ctrl=req=RUN", tx_buf[16] == (uint8_t)DRIVE_REQ_RUN);
    expect("state=RUN", tx_buf[17] == (uint8_t)DRIVE_STATE_RUN);
    expect("sys_mode", tx_buf[18] == (uint8_t)debug_mode);
    expect("debug_mode", tx_buf[19] == (uint8_t)spd_curr_cl);
    expect("release_mode", tx_buf[20] == 2u);
    expect("calibration_mode", tx_buf[21] == 3u);
    expect("halt_mode", tx_buf[22] == 1u);
    expect("position_mode=0", tx_buf[23] == 0u);
    expect("encoder_type", tx_buf[24] == ENCODER_TYPE_ABZ);
    expect("phase_order", tx_buf[25] == PHASE_ORDER_ABC);
    expect("encoder_raw_count", rd_u32(28) == 1234567u);
    expect("fault_bits=0", rd_u32(32) == 0u);

    /* ---- 3. 指令区：级联模式 cmd_iq 取 PI 实际输出（B 原语义） ---- */
    expect("cmd_vd", rd_f(36) == -0.5f);
    expect("cmd_vq", rd_f(40) == 2.25f);
    expect("cmd_id", rd_f(44) == 0.0f);
    expect("cmd_iq=iq_lim(级联)", rd_f(48) == 3.5f);
    expect("cmd_wr", rd_f(52) == 52.36f);
    expect("cmd_wm", rd_f(56) == 52.36f);
    expect("cmd_posr", rd_f(60) == 1.25f);
    expect("cmd_posm=0", rd_f(64) == 0.0f);
    expect("cmd_torm=0", rd_f(68) == 0.0f);

    /* ---- 4. 位置/速度区 ---- */
    expect("encoder_pos_1t", rd_f(72) == 0.75f);
    expect("raw_angle=theta_e-e_off", fabsf(rd_f(76) - 1.5f) < 1e-6f);
    expect("electrical_angle", rd_f(80) == 2.0f);
    expect("forced_angle=0", rd_f(84) == 0.0f);
    expect("rotor_multi_turn", rd_f(88) == 12.5f);
    expect("output_multi_turn", rd_f(92) == 12.5f);
    expect("rotor_speed_raw", rd_f(96) == 51.9f);
    expect("rotor_speed_filtered", rd_f(100) == 52.1f);
    expect("output_speed", rd_f(104) == 52.1f);

    /* ---- 5. 电流/电压/占空比区 + CRC ---- */
    expect("ia", rd_f(108) == 0.11f);
    expect("ib", rd_f(112) == -0.22f);
    expect("ic", rd_f(116) == 0.11f);
    expect("id", rd_f(120) == 0.01f);
    expect("iq", rd_f(124) == 3.49f);
    expect("vd", rd_f(128) == -0.5f);
    expect("vq", rd_f(132) == 2.25f);
    expect("vbus", rd_f(136) == 24.1f);
    expect("duty_a", rd_f(140) == 0.4f);
    expect("duty_b", rd_f(144) == 0.3f);
    expect("duty_c", rd_f(148) == 0.3f);
    expect("dropped_frames=0", rd_u32(152) == 0u);
    expect("CRC 校验通过",
           axdr_command_crc32(tx_buf, AXDR_TELEMETRY_V1_FRAME_SIZE - 4u) ==
               rd_u32(156));

    /* ---- 6. 直流电流模式：cmd_iq 取 ref.iq ---- */
    service_telemetry_set_decimation(1u);
    g_foc.mode.debug = curr_cl;
    reset_tx();
    service_telemetry_capture(); /* seq=2 */
    service_telemetry_poll();
    expect("直流模式帧发出", rd_u32(8) == 2u);
    expect("cmd_iq=ref.iq(直流)", rd_f(48) == 8.0f);

    /* ---- 7. 队列翻覆：25 帧 > 16 槽 → 跳 9 帧取最老存活 seq=12 ---- */
    service_telemetry_set_decimation(1u);
    reset_tx();
    for (i = 0u; i < 25u; ++i)
    {
        service_telemetry_capture(); /* seq 3..27 */
    }
    service_telemetry_poll();
    expect("翻覆后取到最老存活帧", tx_len == AXDR_TELEMETRY_V1_FRAME_SIZE);
    expect("最老存活 seq=12", rd_u32(8) == 12u);
    expect("dropped_frames=9", rd_u32(152) == 9u);
    expect("CRC 仍通过",
           axdr_command_crc32(tx_buf, AXDR_TELEMETRY_V1_FRAME_SIZE - 4u) ==
               rd_u32(156));
    service_telemetry_poll();
    expect("连号 seq=13", rd_u32(8) == 13u);

    /* ---- 8. BUSY 保留重试：同一帧反复尝试直至成功 ---- */
    reset_tx();
    service_telemetry_set_decimation(1u); /* 丢弃 14..27 积压 */
    service_telemetry_capture(); /* seq=28 */
    service_telemetry_poll();
    expect("发到 seq=28", rd_u32(8) == 28u);
    tx_result = 1; /* BUSY */
    service_telemetry_capture(); /* seq=29 */
    service_telemetry_poll(); /* 取 29 → BUSY 保留 */
    expect("BUSY 保留 seq=29", rd_u32(8) == 29u);
    reset_tx();
    service_telemetry_poll(); /* 重试仍 BUSY */
    expect("重试后仍是 seq=29", tx_len == AXDR_TELEMETRY_V1_FRAME_SIZE &&
                                    rd_u32(8) == 29u);
    tx_result = 0;
    service_telemetry_poll();
    expect("恢复后发出 seq=29", rd_u32(8) == 29u);
    expect("sent 计数推进", service_telemetry_sent_count() == 6u);

    /* ---- 9. 发送错误（非 BUSY）→ 丢弃 + drop 计数 ---- */
    drops_before = service_telemetry_drop_count();
    tx_result = 2; /* 错误 */
    service_telemetry_capture(); /* seq=30 */
    service_telemetry_poll();
    expect("错误帧计入 drop",
           service_telemetry_drop_count() == drops_before + 1u);
    expect("错误后 seq 前进", (service_telemetry_capture(),
                              service_telemetry_poll(), rd_u32(8)) == 31u);
    tx_result = 0;

    /* ---- 10. 变速清积压：旧帧作废不发送 ---- */
    reset_tx();
    service_telemetry_set_decimation(1u);
    for (i = 0u; i < 5u; ++i)
    {
        service_telemetry_capture(); /* seq 33..37 */
    }
    service_telemetry_set_decimation(1000u); /* 清积压 */
    service_telemetry_poll();
    expect("变速后积压作废", tx_len == 0u);

    if (fail_count == 0)
    {
        (void)printf("test_service_telemetry: all pass\n");
        return 0;
    }
    (void)printf("test_service_telemetry: %d failure(s)\n", fail_count);
    return 1;
}
