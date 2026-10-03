/**
 * @file test_pid_conditional.c
 * @brief PDFF 条件积分变体（B 库语义）的行为验证。
 *
 * B 库 pdff_ctrl_conditional 的三条契约：
 * 1. 试探输出已越过限幅且积分增量同向 → 放弃本拍增量（积分冻结）；
 * 2. 未饱和、或增量方向与饱和侧相反（帮助脱离饱和）→ 正常积分；
 * 3. abs_output_limit 非正 → 退化为普通 PDFF（与 control_pid_pdff_step 一致）。
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "control_pid.h"

static int fail_count;

static pid_para_t make_cond_pid(void)
{
    pid_para_t pid;
    memset(&pid, 0, sizeof(pid));
    pid.kp = 2.0f;
    pid.ki = 1.0f;
    pid.kd = 0.0f;
    pid.kfp = 1.0f;   /* 前馈权重 1：p_term = kp*(ref - fdb*(1+damp)) */
    pid.kf_damp = 0.0f;
    pid.ts = 0.001f;
    pid.out_max = 10.0f;
    pid.out_min = -10.0f;
    return pid;
}

static void expect(const char *name, int cond)
{
    if (!cond)
    {
        (void)printf("FAIL %s\n", name);
        fail_count++;
    }
}

int main(void)
{
    /* 契约 1：饱和冻结。kp=2、err=5 → p=10 已达限幅，正向增量必须被丢弃。 */
    {
        pid_para_t pid = make_cond_pid();
        float before = pid.i_term;
        (void)control_pid_pdff_conditional_step(&pid, 5.0f, 0.0f, 4.0f); /* 限幅4 < p=10 */
        expect("saturated integral frozen", pid.i_term == before);
        expect("output clamped to limit", pid.out_value == 4.0f);
    }

    /* 契约 2：未饱和正常积分（单拍手算：p=2*0.5=1, inc=1*0.5*0.001=0.0005）。 */
    {
        pid_para_t pid = make_cond_pid();
        float out = control_pid_pdff_conditional_step(&pid, 0.5f, 0.0f, 10.0f);
        expect("normal integration applied",
               fabsf(pid.i_term - 0.0005f) < 1e-6f);
        expect("unsaturated output", fabsf(out - 1.0005f) < 1e-5f);
    }

    /* 契约 2b：输出负向饱和但增量为正（帮助脱离饱和）→ 增量被采纳。
     * kfp=0/kf_damp=1、fdb=3、ref=4：p 项 = 2*(0 - 3*2) = -12 负向饱和（限幅 4），
     * 而 error = +1 → 增量 +0.001 与饱和侧相反，必须被采纳。 */
    {
        pid_para_t pid = make_cond_pid();
        pid.kfp = 0.0f;
        pid.kf_damp = 1.0f;
        float before = pid.i_term;
        (void)control_pid_pdff_conditional_step(&pid, 4.0f, 3.0f, 4.0f);
        expect("opposite-direction increment accepted",
               fabsf(pid.i_term - (before + 0.001f)) < 1e-6f);
        expect("output clamped to -limit", pid.out_value == -4.0f);
    }

    /* 契约 3：abs_output_limit=0 → 与普通 PDFF 输出一致。 */
    {
        pid_para_t a = make_cond_pid();
        pid_para_t b = make_cond_pid();
        float oa = control_pid_pdff_conditional_step(&a, 0.5f, 0.0f, 0.0f);
        float ob = control_pid_pdff_step(&b, 0.5f, 0.0f);
        expect("zero limit falls back to plain pdff",
               memcmp(&oa, &ob, sizeof(float)) == 0);
        expect("fallback context equal", memcmp(&a, &b, sizeof(a)) == 0);
    }

    /* 多拍一致性：持续大误差下积分冻结，输出恒为限幅值。 */
    {
        pid_para_t pid = make_cond_pid();
        float before = pid.i_term;
        for (int i = 0; i < 100; i++)
        {
            (void)control_pid_pdff_conditional_step(&pid, 5.0f, 0.0f, 4.0f);
        }
        expect("integral stays frozen over 100 ticks", pid.i_term == before);
        expect("output stays at limit", pid.out_value == 4.0f);
    }

    if (fail_count == 0)
    {
        (void)printf("test_pid_conditional: all pass\n");
        return 0;
    }
    (void)printf("test_pid_conditional: %d failure(s)\n", fail_count);
    return 1;
}
