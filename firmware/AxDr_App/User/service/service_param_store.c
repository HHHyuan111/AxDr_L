/**
 * @file service_param_store.c
 * @brief S5 参数持久化实现：对齐零位 e_off 的 Flash 存取。
 *
 * 存储区选 Bank2 首页（0x08040000）：代码在 Bank1——擦写期间 Bank1 取指
 * 零停顿（G4 双 Bank 独立读），20kHz 控制环不受 2KB 页擦除（约 22ms）影响；
 * 代码现占 66KB，物理隔离不可能越界。
 *
 * 记录 32B（8 字节对齐，匹配双字编程）。CRC 覆盖除 crc 字段外的全记录
 * （算前清零，与档案 CRC 同法）。e_off 合法域 [0,2π)——drive 对齐输出
 * atan2 后归一化到该区间，NaN 比较恒假自动拒绝。
 *
 * 保存时机：主循环 poll 检测 enc_aligned 上升沿（对齐在中断里完成）。
 * 每上电周期至多尝试一次——坏 Flash 反复擦写是折损，失败即放弃。
 */

#include "service_param_store.h"

#include <math.h>
#include <string.h>

#include "axdr_command_core.h"
#include "common.h" /* foc_t 完整定义（foc.h/foc_fwd.h 均为前置声明壳） */
#include "encoder_type.h"
#include "motor_config.h"
#include "ret.h"
#include "target_flash.h"

#define PS_STORE_ADDR 0x08040000U /* Bank2 首页 */
#define PS_MAGIC 0x53585041U      /* "APXS" 倒序读 */
#define PS_VERSION 1U

/* 辨识档案页（P8 A1 svc 先行段）：独立第 2 页——本模块的写策略是
 * 整页擦除重写，同页双记录会互删；两页互不影响各自的有效性。 */
#define PS_IDENT_ADDR 0x08040800U   /* Bank2 第 2 页 */
#define PS_IDENT_MAGIC 0x49585041U  /* "APXI" 倒序读 */
#define PS_IDENT_VERSION 1U

/* 32B 记录：字段序刻意凑 8 对齐，无填充洞；reserved 恒 0。
 * （无保存计数字段——旧记录有效即蕴含 boot 已装载，本周期不会再保存，
 *  计数递增逻辑不可达，不留伪诊断字段。） */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t profile_id;
    uint32_t profile_revision;
    uint32_t encoder_type;
    float e_off;
    uint32_t reserved;
    uint32_t crc32;
} ps_record_t;

_Static_assert(sizeof(ps_record_t) == 32U, "ps_record_t 须为 32B 无填充");

/* 64B 辨识档案：头字段与 e_off 记录同构（档案绑定作废机制共用），
 * 数据体 40B 全 float，8 字节对齐无填充洞。 */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t profile_id;
    uint32_t profile_revision;
    uint32_t encoder_type;
    service_ident_data_t data;
    uint32_t crc32;
} ps_ident_record_t;

_Static_assert(sizeof(service_ident_data_t) == 40U,
               "service_ident_data_t 须为 40B");
_Static_assert(sizeof(ps_ident_record_t) == 64U,
               "ps_ident_record_t 须为 64B 无填充");

static uint8_t loaded;    /* 零位已就位（装载或保存成功） */
static uint8_t save_tried; /* 本周期保存已尝试（成败都不再来） */

static service_ident_data_t ident_pending; /* ISR 暂存快照 */
static volatile uint8_t ident_stage_flag;  /* 1=poll 待落盘 */

/* 记录校验：标识/版本/长度/档案绑定/编码器一致 + e_off 值域 + CRC */
static uint8_t ps_valid(const ps_record_t *rec)
{
    ps_record_t tmp = *rec;
    tmp.crc32 = 0U;
    return (rec->magic == PS_MAGIC) &&
           (rec->version == PS_VERSION) &&
           (rec->size == (uint16_t)sizeof(ps_record_t)) &&
           (rec->profile_id == MOTOR_PROFILE_ID) &&
           (rec->profile_revision == MOTOR_PROFILE_REVISION) &&
           (rec->encoder_type == (uint32_t)g_foc.enc.primary) &&
           (rec->e_off >= 0.0f) && (rec->e_off < 6.28318530718f) &&
           (axdr_command_crc32((const uint8_t *)&tmp, sizeof(tmp)) == rec->crc32);
}

void service_param_store_boot(void)
{
    ps_record_t rec;

    /* 幂等：boot 即重新上电，先清本周期状态（拒绝装载不得沿用旧判定） */
    loaded = 0U;
    save_tried = 0U;

    target_flash_read(&rec, PS_STORE_ADDR, sizeof(rec));
    if (ps_valid(&rec))
    {
        /* 与 drive 对齐成功同效：START 见 enc_aligned 即免对齐直进 RUN
         * （drive_exec_action START 分支的既有门控，drive.c 零改动）。 */
        g_foc.motor.e_off = rec.e_off;
        g_foc.enc_aligned = true;
        loaded = 1U;
    }
    /* 无效记录：保持 foc_init 默认（e_off=0、未对齐），走对齐后 poll 保存 */
}

void service_param_store_poll(void)
{
    ps_record_t rec;

    if (loaded || save_tried || !g_foc.enc_aligned)
    {
        return;
    }
    save_tried = 1U; /* 无论成败只试一次 */

    memset(&rec, 0, sizeof(rec));
    rec.magic = PS_MAGIC;
    rec.version = PS_VERSION;
    rec.size = (uint16_t)sizeof(ps_record_t);
    rec.profile_id = MOTOR_PROFILE_ID;
    rec.profile_revision = MOTOR_PROFILE_REVISION;
    rec.encoder_type = (uint32_t)g_foc.enc.primary;
    rec.e_off = g_foc.motor.e_off;
    rec.crc32 = axdr_command_crc32((const uint8_t *)&rec, sizeof(rec));

    if ((target_flash_erase_page(PS_STORE_ADDR) != RET_OK) ||
        (target_flash_write(PS_STORE_ADDR, &rec, sizeof(rec)) != RET_OK))
    {
        return; /* 硬件失败：下次上电重新对齐，功能照常 */
    }
    loaded = 1U;
}

uint8_t service_param_store_loaded(void)
{
    return loaded;
}

/* 档案校验：与 e_off 记录同四重（标识/版本/长度/档案绑定/编码器）+ CRC，
 * 另加数据体合理性（Rs/Ld/Lq 恒正；flux 允许 0=从未测过磁链的合并档案）。 */
static uint8_t ps_ident_valid(const ps_ident_record_t *rec)
{
    ps_ident_record_t tmp = *rec;
    const service_ident_data_t *d = &rec->data;

    tmp.crc32 = 0U;
    return (rec->magic == PS_IDENT_MAGIC)
        && (rec->version == PS_IDENT_VERSION)
        && (rec->size == (uint16_t)sizeof(ps_ident_record_t))
        && (rec->profile_id == MOTOR_PROFILE_ID)
        && (rec->profile_revision == MOTOR_PROFILE_REVISION)
        && (rec->encoder_type == (uint32_t)g_foc.enc.primary)
        && isfinite(d->rs_ohm) && (d->rs_ohm > 0.0f)
        && isfinite(d->ld_h) && (d->ld_h > 0.0f)
        && isfinite(d->lq_h) && (d->lq_h > 0.0f)
        && isfinite(d->flux_wb) && (d->flux_wb >= 0.0f)
        && (axdr_command_crc32((const uint8_t *)&tmp, sizeof(tmp))
            == rec->crc32);
}

uint8_t service_param_store_ident_read(service_ident_data_t *out)
{
    ps_ident_record_t rec;

    if (out == NULL)
    {
        return 0U;
    }
    target_flash_read(&rec, PS_IDENT_ADDR, sizeof(rec));
    if (!ps_ident_valid(&rec))
    {
        return 0U;
    }
    *out = rec.data;
    return 1U;
}

void service_param_store_ident_stage(const service_ident_data_t *data)
{
    if (data == NULL)
    {
        return;
    }
    /* 40B 结构拷贝，ISR 可承受；落盘（页擦 22ms）归主循环 poll。 */
    ident_pending = *data;
    ident_stage_flag = 1U;
}

/* 合并落盘：逐字段 >0 者胜——LQ 复测（flux/Ke/Kt=0）不冲掉 FULL 档案。 */
static uint8_t ps_ident_save_merged(const service_ident_data_t *data)
{
    ps_ident_record_t rec;
    ps_ident_record_t old;
    service_ident_data_t merged;
    float *dst = (float *)&merged;
    const float *src = (const float *)data;
    uint16_t i;

    target_flash_read(&old, PS_IDENT_ADDR, sizeof(old));
    merged = ps_ident_valid(&old) ? old.data
                                  : (service_ident_data_t){0};

    for (i = 0U; i < (uint16_t)(sizeof(service_ident_data_t)
                               / sizeof(float)); ++i)
    {
        if (src[i] > 0.0f)
        {
            dst[i] = src[i];
        }
    }

    memset(&rec, 0, sizeof(rec));
    rec.magic = PS_IDENT_MAGIC;
    rec.version = PS_IDENT_VERSION;
    rec.size = (uint16_t)sizeof(ps_ident_record_t);
    rec.profile_id = MOTOR_PROFILE_ID;
    rec.profile_revision = MOTOR_PROFILE_REVISION;
    rec.encoder_type = (uint32_t)g_foc.enc.primary;
    rec.data = merged;
    rec.crc32 = axdr_command_crc32((const uint8_t *)&rec, sizeof(rec));

    return ((target_flash_erase_page(PS_IDENT_ADDR) == RET_OK)
            && (target_flash_write(PS_IDENT_ADDR, &rec, sizeof(rec))
                == RET_OK)) ? 1U : 0U;
}

void service_param_store_ident_poll(void)
{
    service_ident_data_t local;

    if (!ident_stage_flag)
    {
        return;
    }
    /* 先取本地副本再清标志：若辨识在写 Flash 期间再次完成（秒级间隔，
     * 实际不可能），标志重置、下轮 poll 再落。与 e_off 同策失败不重试。 */
    local = ident_pending;
    ident_stage_flag = 0U;
    (void)ps_ident_save_merged(&local);
}
