/**
 * @file test_service_param_store.c
 * @brief S5 参数持久化测试：装载校验矩阵（magic/版本/档案 id/revision/
 *        编码器/值域+CRC 七重门）、有效装载、装载后不重存、空页对齐边沿
 *        保存、保存内容线上核对、擦写失败只试一次。
 * @note  记录 32B 手排进 fake flash（不经被测内部结构），纯线上语义断言。
 *        场景链利用"boot=重新上电"复位 loaded；save_tried 场景置位后不可
 *        复位，放进程末尾。
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "axdr_command_core.h"
#include "common.h"
#include "encoder_type.h"
#include "fake_target_flash.h"
#include "motor_config.h"
#include "ret.h"
#include "service_param_store.h"
#include "target_flash.h"

foc_t g_foc;

#define STORE_ADDR 0x08040000U
#define REC_E_OFF 1.234f

static int fail_count;
static void expect(const char *name, int cond)
{
    if (!cond)
    {
        (void)printf("FAIL %s\n", name);
        fail_count++;
    }
}

/* ---- 手排 32B 记录（与被测布局一致：magic/version/size/pid/prev/enc/
 *       e_off/reserved/crc；CRC=32B 全量、crc 字段清零，与 service 同法） ---- */
static uint8_t rec_buf[32];

static void rec_build(uint32_t pid, uint32_t prev, uint32_t enc, float e_off)
{
    uint32_t magic = 0x53585041U;
    uint16_t version = 1U;
    uint16_t size = 32U;
    uint32_t crc;

    memset(rec_buf, 0, sizeof(rec_buf));
    memcpy(&rec_buf[0], &magic, 4);
    memcpy(&rec_buf[4], &version, 2);
    memcpy(&rec_buf[6], &size, 2);
    memcpy(&rec_buf[8], &pid, 4);
    memcpy(&rec_buf[12], &prev, 4);
    memcpy(&rec_buf[16], &enc, 4);
    memcpy(&rec_buf[20], &e_off, 4);
    /* [24..27] reserved 恒 0；[28..31] crc 位置零参与计算 */
    crc = axdr_command_crc32(rec_buf, sizeof(rec_buf));
    memcpy(&rec_buf[28], &crc, 4);
}

/* 铺页：erase + write 真桩路径 */
static void rec_store(void)
{
    (void)target_flash_erase_page(STORE_ADDR);
    (void)target_flash_write(STORE_ADDR, rec_buf, sizeof(rec_buf));
}

/* 上电复位：fake 页保持当前内容，foc 复位为沉沙 ABZ 初态 */
static void power_on(void)
{
    memset(&g_foc, 0, sizeof(g_foc));
    g_foc.enc.primary = ENCODER_TYPE_ABZ;
    service_param_store_boot();
}

static uint32_t rd_u32(uint32_t offset)
{
    return fake_flash_raw(offset);
}

static float rd_f(uint32_t offset)
{
    float value;
    uint32_t raw = fake_flash_raw(offset);
    memcpy(&value, &raw, 4);
    return value;
}

int main(void)
{
    /* ---- 1. 拒载矩阵：七重门逐一击破 ---- */
    fake_flash_reset();

    rec_build(MOTOR_PROFILE_ID, MOTOR_PROFILE_REVISION, ENCODER_TYPE_ABZ, REC_E_OFF);
    rec_buf[28] ^= 0xFFU; /* CRC 坏 */
    rec_store();
    power_on();
    expect("CRC 坏拒载", (service_param_store_loaded() == 0U) &&
                            (g_foc.enc_aligned == false));

    rec_build(MOTOR_PROFILE_ID, MOTOR_PROFILE_REVISION, ENCODER_TYPE_ABZ, REC_E_OFF);
    rec_buf[0] ^= 0x01U; /* magic 坏（CRC 随之不匹配，双门同测） */
    rec_store();
    power_on();
    expect("magic 错拒载", service_param_store_loaded() == 0U);

    rec_build(MOTOR_PROFILE_ID, MOTOR_PROFILE_REVISION + 1U, ENCODER_TYPE_ABZ, REC_E_OFF);
    rec_store();
    power_on();
    expect("revision 不匹配拒载", service_param_store_loaded() == 0U);

    rec_build(MOTOR_PROFILE_ID + 1U, MOTOR_PROFILE_REVISION, ENCODER_TYPE_ABZ, REC_E_OFF);
    rec_store();
    power_on();
    expect("profile_id 不匹配拒载", service_param_store_loaded() == 0U);

    rec_build(MOTOR_PROFILE_ID, MOTOR_PROFILE_REVISION, ENCODER_TYPE_MA732, REC_E_OFF);
    rec_store();
    power_on();
    expect("编码器类型不匹配拒载", service_param_store_loaded() == 0U);

    rec_build(MOTOR_PROFILE_ID, MOTOR_PROFILE_REVISION, ENCODER_TYPE_ABZ, -0.5f);
    rec_store();
    power_on();
    expect("e_off 负值拒载", service_param_store_loaded() == 0U);

    rec_build(MOTOR_PROFILE_ID, MOTOR_PROFILE_REVISION, ENCODER_TYPE_ABZ, 7.0f);
    rec_store();
    power_on();
    expect("e_off 越域拒载", service_param_store_loaded() == 0U);

    rec_build(MOTOR_PROFILE_ID, MOTOR_PROFILE_REVISION, ENCODER_TYPE_ABZ, 0.0f / 0.0f);
    rec_store();
    power_on();
    expect("e_off NaN 拒载", service_param_store_loaded() == 0U);

    /* ---- 2. ABZ 有效装载：e_off 仅作对齐初值，不免对齐 ---- */
    fake_flash_reset();
    rec_build(MOTOR_PROFILE_ID, MOTOR_PROFILE_REVISION, ENCODER_TYPE_ABZ, REC_E_OFF);
    rec_store();
    power_on();
    expect("ABZ 装载后保持未对齐", (g_foc.enc_aligned == false) &&
                                      (service_param_store_loaded() == 0U));
    expect("e_off 作对齐初值装载", g_foc.motor.e_off == REC_E_OFF);

    /* ---- 3. ABZ 装载后未对齐：poll 不写页（无 enc_aligned 边沿） ---- */
    service_param_store_poll();
    expect("未对齐 poll 无写动作", rd_u32(0) == 0x53585041U &&
                                     rd_u32(28) == fake_flash_raw(28U) &&
                                     rd_f(20) == REC_E_OFF);

    /* ---- 4. 空页：boot 不应用 → 对齐边沿 → poll 保存 ---- */
    fake_flash_reset();
    power_on();
    expect("空页 boot 不应用", (service_param_store_loaded() == 0U) &&
                                  (g_foc.enc_aligned == false));

    g_foc.enc_aligned = true; /* 模拟中断里对齐完成 */
    g_foc.motor.e_off = 2.5f;
    service_param_store_poll();
    expect("边沿保存成功", service_param_store_loaded() == 1U);

    /* 保存内容线上核对 */
    expect("落盘 magic", rd_u32(0) == 0x53585041U);
    expect("落盘 version/size", rd_u32(4) == ((32U << 16) | 1U));
    expect("落盘 profile_id", rd_u32(8) == MOTOR_PROFILE_ID);
    expect("落盘 revision", rd_u32(12) == MOTOR_PROFILE_REVISION);
    expect("落盘 encoder", rd_u32(16) == (uint32_t)ENCODER_TYPE_ABZ);
    expect("落盘 e_off", rd_f(20) == 2.5f);
    expect("落盘 reserved=0", rd_u32(24) == 0U);
    {
        uint8_t page_img[32];
        target_flash_read(page_img, STORE_ADDR, sizeof(page_img));
        memset(&page_img[28], 0, 4);
        expect("落盘 CRC 自洽",
               rd_u32(28) == axdr_command_crc32(page_img, sizeof(page_img)));
    }

    /* ---- 5. 保存后"重新上电"（ABZ）：e_off 复载作初值，仍需重对齐 ---- */
    power_on();
    expect("保存后复载（初值语义）", (service_param_store_loaded() == 0U) &&
                                       (g_foc.motor.e_off == 2.5f) &&
                                       (g_foc.enc_aligned == false));

    /* ---- 5b. 绝对值编码器：装载即免对齐（旧行为保持） ---- */
    fake_flash_reset();
    rec_build(MOTOR_PROFILE_ID, MOTOR_PROFILE_REVISION, ENCODER_TYPE_MA732, 1.25f);
    rec_store();
    memset(&g_foc, 0, sizeof(g_foc));
    g_foc.enc.primary = ENCODER_TYPE_MA732;
    service_param_store_boot();
    expect("绝对编码器装载免对齐", (service_param_store_loaded() == 1U) &&
                                     (g_foc.motor.e_off == 1.25f) &&
                                     (g_foc.enc_aligned == true));

    /* ---- 6. 擦写失败只试一次（save_tried 不可逆，放最后） ---- */
    fake_flash_reset();
    rec_build(MOTOR_PROFILE_ID, MOTOR_PROFILE_REVISION, ENCODER_TYPE_ABZ, REC_E_OFF);
    rec_buf[28] ^= 0xFFU; /* 坏记录：boot 拒载，loaded 回 0 */
    rec_store();
    power_on();
    expect("坏记录 boot 拒载", service_param_store_loaded() == 0U);

    g_foc.enc_aligned = true; /* 对齐完成边沿 */
    fake_flash_inject_erase_fail(1);
    service_param_store_poll();
    expect("擦除失败未装载", service_param_store_loaded() == 0U);
    expect("擦除失败页未被写", rd_u32(0) == 0x53585041U); /* 坏记录原样 */

    fake_flash_inject_erase_fail(0);
    service_param_store_poll(); /* 同周期第二次 poll：不得再试（若重试，
                                  * erase 成功会清页——magic 消失即为反证） */
    expect("失败后不再重试", rd_u32(0) == 0x53585041U &&
                                 service_param_store_loaded() == 0U);

    if (fail_count == 0)
    {
        (void)printf("test_service_param_store: all pass\n");
        return 0;
    }
    (void)printf("test_service_param_store: %d failure(s)\n", fail_count);
    return 1;
}
