/**
 * @file test_cycle_record.c
 * @brief 验证周期记录器默认关闭、字段映射和环形覆盖行为。
 */

#include <stdio.h>

#include "common.h"
#include "cycle_record.h"

static bool expect_true(bool condition, const char *message)
{
    if (condition)
    {
        return true;
    }

    fprintf(stderr, "%s\n", message);
    return false;
}

int main(void)
{
    foc_t foc = {0};
    uint32_t index;

    _Static_assert(sizeof(cycle_sample_t) == 76U,
                   "周期样本布局发生了变化");

    cycle_record_reset();
    cycle_record_publish(&foc);
    if (!expect_true(g_cycle_record.count == 0U,
                     "记录器默认关闭时不应保存样本。"))
    {
        return 1;
    }

    cycle_record_enable(true);
    foc.mode.sys = release_mode;
    foc.mode.release = csv_mode;
    foc.state = DRIVE_STATE_RUN;
    foc.fault.all = 0x12U;
    foc.sig.vbus = 24.0f;
    foc.sig.ia = 1.0f;
    foc.sig.ib = -0.4f;
    foc.sig.ic = -0.6f;
    foc.sig.theta_e = 0.7f;
    foc.sig.pos_r = 1.2f;
    foc.sig.spd_r = 3.4f;
    foc.sig.id = 0.1f;
    foc.sig.iq = 0.8f;
    foc.sig.vd = 2.0f;
    foc.sig.vq = 3.0f;
    foc.sig.duty_a = 0.4f;
    foc.sig.duty_b = 0.5f;
    foc.sig.duty_c = 0.6f;

    for (index = 1U; index <= (CYCLE_RECORD_CAPACITY + 3U); index++)
    {
        foc.fast_seq = index;
        cycle_record_publish(&foc);
    }

    if (!expect_true(g_cycle_record.count == CYCLE_RECORD_CAPACITY,
                     "缓冲区装满后样本数应保持为固定容量。") ||
        !expect_true(g_cycle_record.head == 3U,
                     "环形缓冲区写指针没有正确回绕。") ||
        !expect_true(g_cycle_record.sample[2].seq == 35U,
                     "最新样本没有写入预期位置。") ||
        !expect_true(g_cycle_record.sample[3].seq == 4U,
                     "最旧的有效样本位置不正确。") ||
        !expect_true(g_cycle_record.sample[2].op_mode == (uint32_t)csv_mode,
                     "运行模式没有正确记录。") ||
        !expect_true(g_cycle_record.sample[2].vbus == 24.0f,
                     "母线电压没有正确记录。") ||
        !expect_true(g_cycle_record.sample[2].duty_c == 0.6f,
                     "PWM 占空比没有正确记录。"))
    {
        return 2;
    }

    cycle_record_enable(false);
    foc.fast_seq++;
    cycle_record_publish(&foc);
    if (!expect_true(g_cycle_record.count == CYCLE_RECORD_CAPACITY,
                     "关闭记录器后不应再改动样本数。"))
    {
        return 3;
    }

    return 0;
}
