/**
 * @file test_motor_chensha.c
 * @brief 沉沙档案装载验收：B 库定版值逐项锁定（浮点字面量按位比对）。
 *
 * 值出处：motor_config.h 沉沙档案块注释（B 库 axdr_chensha_config.h /
 * axdr_chensha_servo_profile.h / target_config 沉沙段 / foc_drv.c 实测零位）。
 * 本测试是关口②/③参数不漂移的第一道防线：任何人改档案值，这里立刻红。
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "common.h"
#include "motor_config.h"
#include "motor_profile.h"

foc_t g_foc;

static int fail_count;

static void expect_bits(const char *name, float actual, float expected)
{
    if (memcmp(&actual, &expected, sizeof(float)) != 0)
    {
        (void)printf("FAIL %s: actual=%.9g expected=%.9g\n",
                     name, (double)actual, (double)expected);
        fail_count++;
    }
}

static void expect_close(const char *name, float actual, float expected, float tol)
{
    if (fabsf(actual - expected) > tol)
    {
        (void)printf("FAIL %s: actual=%.9g expected=%.9g\n",
                     name, (double)actual, (double)expected);
        fail_count++;
    }
}

static void expect_int(const char *name, long actual, long expected)
{
    if (actual != expected)
    {
        (void)printf("FAIL %s: actual=%ld expected=%ld\n", name, actual, expected);
        fail_count++;
    }
}

int main(void)
{
    const float flux_expected = 0.06f / (1.41421356237f * 1.5f * 5.0f);
    memset(&g_foc, 0, sizeof(g_foc));

    /* ---- 物理档案（B 库 chensha_config 逐项） ---- */
    motor_profile_load(&g_foc);

    expect_bits("Rs", g_foc.motor.Rs, 0.105125f);
    expect_bits("Ld", g_foc.motor.Ld, 0.0001037615f);
    expect_bits("Lq", g_foc.motor.Lq, 0.0001037615f);
    expect_bits("Ls=Ld", g_foc.motor.Ls, g_foc.motor.Ld);
    expect_bits("Ldif=0", g_foc.motor.Ldif, 0.0f);
    expect_bits("flux", g_foc.motor.flux, flux_expected);
    expect_bits("pn=5", g_foc.motor.pn, 5.0f);
    expect_bits("Js", g_foc.motor.Js, 0.000032f);
    expect_bits("Gr=1", g_foc.motor.Gr, 1.0f);
    expect_bits("e_off=0(上电对齐)", g_foc.motor.e_off, 0.0f);
    expect_bits("r_off(B实测)", g_foc.motor.r_off, 0.0240995f);
    expect_int("phase=ABC", (long)g_foc.motor.phase_order, (long)PHASE_ORDER_ABC);
    expect_close("div_pn", g_foc.motor.div_pn, 0.2f, 1e-9f);
    expect_close("Kt=1.5*pn*flux", g_foc.motor.Kt,
                 1.5f * 5.0f * flux_expected, 1e-9f);
    expect_bits("ibw=4000", g_foc.motor.ibw, 4000.0f);

    /* ---- 台架限幅（B 库 target_config 沉沙段） ---- */
    expect_close("iq_max=8A", g_foc.ref.iq_max, 8.0f, 1e-6f);
    expect_close("iq_min=-8A", g_foc.ref.iq_min, -8.0f, 1e-6f);
    expect_bits("spd_max=335", g_foc.ref.spd_max, 335.0f);
    expect_bits("spd_min=-335", g_foc.ref.spd_min, -335.0f);
    expect_bits("acc=1200", g_foc.ref.acc_m, 1200.0f);

    /* ---- 速度/位置整定参数（早绑定部分） ---- */
    expect_bits("kfp=0.5", g_foc.spd_pi.kfp, 0.5f);
    expect_bits("kf_damp=0", g_foc.spd_pi.kf_damp, 0.0f);
    expect_bits("pos_kp", g_foc.pos_pi.kp, 6.28318530718f);
    expect_bits("pos_ki=0", g_foc.pos_pi.ki, 0.0f);
    expect_bits("pos_kd=0", g_foc.pos_pi.kd, 0.0f);

    /* ---- 晚绑定：保护覆盖 + 速度环固化值（模拟 prot/spd 先行初始化再定版） ---- */
    g_foc.prot_cfg.over_current_a = 999.0f;
    g_foc.prot_cfg.over_voltage_v = 999.0f;
    g_foc.prot_cfg.under_voltage_v = 999.0f;
    g_foc.spd_pi.kp = 123.0f;
    g_foc.spd_pi.ki = 456.0f;

    motor_profile_control_load(&g_foc);

    expect_bits("prot OC=10A", g_foc.prot_cfg.over_current_a, 10.0f);
    expect_bits("prot OV=30V", g_foc.prot_cfg.over_voltage_v, 30.0f);
    expect_bits("prot UV=10V", g_foc.prot_cfg.under_voltage_v, 10.0f);
    expect_bits("spd Kp 固化值", g_foc.spd_pi.kp, 0.332909408f);
    expect_bits("spd Ki 固化值", g_foc.spd_pi.ki, 10.45865749f);

    /* ---- 电流环公式核对（Kp=Ls*ibw / Ki=Rs*ibw，B 库同式） ---- */
    expect_close("cur Kp=Ls*ibw",
                 g_foc.motor.Ls * g_foc.motor.ibw,
                 0.0001037615f * 4000.0f, 1e-12f);
    expect_close("cur Ki=Rs*ibw",
                 g_foc.motor.Rs * g_foc.motor.ibw,
                 0.105125f * 4000.0f, 1e-12f);

    if (fail_count == 0)
    {
        (void)printf("test_motor_chensha: all pass\n");
        return 0;
    }
    (void)printf("test_motor_chensha: %d failure(s)\n", fail_count);
    return 1;
}
