/**
 * @file cycle_record.c
 * @brief 快速控制周期环形记录器实现。
 */

#include "cycle_record.h"

#include "common.h"
#include "compiler.h"

volatile cycle_record_t g_cycle_record;

static PLATFORM_FAST_CODE uint32_t cycle_record_op_mode(const foc_t *foc)
{
    switch (foc->mode.sys)
    {
        case debug_mode:
            return (uint32_t)foc->mode.debug;

        case release_mode:
            return (uint32_t)foc->mode.release;

        case calibrat_mode:
            return (uint32_t)foc->mode.calibrat;

        case halt_mode:
            return (uint32_t)foc->mode.halt;

        default:
            return UINT32_MAX;
    }
}

void cycle_record_reset(void)
{
    g_cycle_record.head = 0U;
    g_cycle_record.count = 0U;
}

void cycle_record_enable(bool enable)
{
    if (enable)
    {
        cycle_record_reset();
    }
    g_cycle_record.enabled = enable;
}

PLATFORM_FAST_CODE void cycle_record_publish(const foc_t *foc)
{
    volatile cycle_sample_t *dst;
    uint32_t next;

    if (!g_cycle_record.enabled)
    {
        return;
    }

    dst = &g_cycle_record.sample[g_cycle_record.head];
    dst->seq = foc->fast_seq;
    dst->state = (uint32_t)foc->state;
    dst->sys_mode = (uint32_t)foc->mode.sys;
    dst->op_mode = cycle_record_op_mode(foc);
    dst->fault = foc->fault.all;
    dst->vbus = foc->fb.vbus;
    dst->ia = foc->fb.ia;
    dst->ib = foc->fb.ib;
    dst->ic = foc->fb.ic;
    dst->theta = foc->fb.theta_e;
    dst->pos = foc->fb.pos_r;
    dst->spd = foc->fb.spd_r;
    dst->id = foc->fb.id;
    dst->iq = foc->fb.iq;
    dst->vd = foc->out.vd;
    dst->vq = foc->out.vq;
    dst->duty_a = foc->out.duty_a;
    dst->duty_b = foc->out.duty_b;
    dst->duty_c = foc->out.duty_c;

    next = g_cycle_record.head + 1U;
    if (next >= CYCLE_RECORD_CAPACITY)
    {
        next = 0U;
    }
    g_cycle_record.head = next;
    if (g_cycle_record.count < CYCLE_RECORD_CAPACITY)
    {
        g_cycle_record.count++;
    }
}
