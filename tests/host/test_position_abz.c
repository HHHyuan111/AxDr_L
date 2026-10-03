/**
 * @file test_position_abz.c
 * @brief ABZ 位置链主机验证：原始角换算、方向、多圈回绕、影子测速、配置一致性。
 *
 * 覆盖 P3 新链中所有可离线验证的纯逻辑；TIM3 硬件读取用桩替代，
 * 真硬件行为由关口①上板验收（18 号盘点 §2.1 的三坑清单）。
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "common.h"
#include "encoder_config.h"
#include "math_const.h"
#include "speed_adapter.h"
#include "target_encoder.h"

/* 配置一致性：与 B 库沉沙定版值逐项锁定。 */
_Static_assert(ENCODER_SELECTED_TYPE == ENCODER_TYPE_ABZ, "primary must be ABZ");
_Static_assert(ABZ_DIRECTION == -1, "ABZ direction is -1 (B-lib dual-point aligned)");
_Static_assert(ABZ_COUNTS_PER_REV == 10000U, "2500 lpi x4 = 10000");
_Static_assert(ABZ_RESOLUTION_BITS == 14U, "14-bit guard (B-lib pit 3)");

foc_t g_foc;

/* ---- target_qenc 桩：模拟 TIM3 计数，可由测试设定 ---- */
static int32_t qenc_stub_count;

int32_t target_qenc_read(void)
{
    return qenc_stub_count;
}

void target_qenc_init(void)
{
}

uint32_t target_qenc_z_count(void)
{
    return 0U;
}

/* SPI 磁编桩：encoder_adapter 引用其符号，主机测试不驱动 SPI 路径。 */
bool target_encoder_read_ma732_raw(uint16_t *raw_count)
{
    (void)raw_count;
    return false;
}

bool target_encoder_read_mt6816_raw(uint16_t *raw_count)
{
    (void)raw_count;
    return false;
}

static int expect_f(const char *name, float actual, float expected, float tol)
{
    if (fabsf(actual - expected) > tol)
    {
        (void)printf("FAIL %s: actual=%.6f expected=%.6f\n", name, (double)actual, (double)expected);
        return 1;
    }
    return 0;
}

static int expect_i(const char *name, long actual, long expected)
{
    if (actual != expected)
    {
        (void)printf("FAIL %s: actual=%ld expected=%ld\n", name, actual, expected);
        return 1;
    }
    return 0;
}

int main(void)
{
    int fail = 0;
    encoder_data_t *abz = &g_foc.enc.abz;

    /* 场景 1：编码器参数与换算（encoder_init 的 ABZ 分支）。 */
    encoder_init(&g_foc.enc);
    fail += expect_f("factor", abz->factor, MATH_2PI / 10000.0f, 1e-9f);
    fail += expect_i("dir", abz->dir, ABZ_DIRECTION);

    /* 场景 2：读数 + 方向应用：TIM3=2500（正转 1/4 圈），dir=-1 → 角度 3π/2。 */
    qenc_stub_count = 2500;
    (void)read_abz_raw(abz);
    encoder_update_angle(abz);
    fail += expect_f("pos dir applied", abz->pos, 1.5f * MATH_PI, 1e-4f);

    /* 场景 3：position_update 全链（pn=5, e_off=r_off=0, Gr=1）。
     * raw 9990 → 5 跨越计数回绕：单圈角从 ~6.22 跳回 ~0.003，rev 应 +1，
     * 多圈位置连续（增量 ≈ 15 counts × 2π/10000）。 */
    g_foc.enc.source = POSITION_SOURCE_ENCODER;
    g_foc.enc.primary = ENCODER_TYPE_ABZ;
    g_foc.motor.pn = 5.0f;
    g_foc.motor.pnd_2pi = 5.0f / MATH_2PI;
    g_foc.motor.e_off = 0.0f;
    g_foc.motor.r_off = 0.0f;
    g_foc.motor.div_Gr = 1.0f;
    g_foc.motor.m_off = 0.0f;

    qenc_stub_count = 9990;
    (void)read_abz_raw(abz);
    (void)position_update(&g_foc);
    float pos_r_before = g_foc.fb.pos_r;
    int rev_before = g_foc.fb.rev;

    qenc_stub_count = 5; /* 跨回绕 */
    (void)read_abz_raw(abz);
    (void)position_update(&g_foc);

    /* dir=-1 在读数源头应用：TIM3 计数 9990→5 对应角度反转跨绕，rev 减、增量为负。 */
    fail += expect_i("rev decrement on reverse wrap", (long)g_foc.fb.rev, (long)(rev_before - 1));
    fail += expect_f("pos continuity",
                     g_foc.fb.pos_r - pos_r_before,
                     -15.0f * MATH_2PI / 10000.0f,
                     1e-3f);

    /* 场景 4：电角度换算：单圈角 θ，电角度 = θ×pn mod 2π。 */
    fail += expect_f("theta_e scaled",
                     g_foc.fb.theta_e,
                     fmodf(g_foc.fb.enc_pos_r * 5.0f, MATH_2PI),
                     1e-4f);

    /* 场景 5：影子测速。窗口 2、fs 20000、每拍 +0.001 rad → Δ2拍=0.002，
     * 速度 = 0.002 × 20000 / 2 = 20 rad/s。 */
    speed_est_init();
    float pos = 1.0f;
    for (int i = 0; i < 4; i++)
    {
        speed_est_step(pos, 20000.0f);
        pos += 0.001f;
    }
    fail += expect_f("speed window", speed_est_get(), 20.0f, 1e-3f);

    /* 场景 6：测速跨零回绕：6.2820 → 0.0005 → 0.0015，速度仍应为 +10 rad/s。 */
    speed_est_init();
    speed_est_step(6.2820f, 20000.0f);
    speed_est_step(0.0005f, 20000.0f); /* Δ 实际 +0.0025？不：2π=6.28319，6.2820→0.0005 绕回 Δ=+0.00131 */
    speed_est_step(0.0015f, 20000.0f); /* 此拍与 6.2820 差 2 拍窗：Δ=0.0015-6.2820+2π=+0.00299 */
    /* 窗口 2：第 3 拍对比第 1 拍 */
    fail += expect_f("speed wrap", speed_est_get(),
                     (0.0015f - 6.2820f + MATH_2PI) * 20000.0f / 2.0f, 1e-2f);

    if (fail == 0)
    {
        (void)printf("test_position_abz: all pass\n");
        return 0;
    }
    (void)printf("test_position_abz: %d failure(s)\n", fail);
    return 1;
}
