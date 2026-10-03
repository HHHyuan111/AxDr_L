/**
 * @file control_traj.c
 * @brief 速度斜坡和在线梯形位置轨迹实现。
 */

#include "control_traj.h"

#include <math.h>

#include "compiler.h"

static PLATFORM_FAST_CODE float traj_move_toward(float value,
                                                  float target,
                                                  float step)
{
    const float error = target - value;

    if (fabsf(error) <= step)
    {
        return target;
    }
    return value + copysignf(step, error);
}

void traj_spd_reset(traj_spd_t *traj, float spd)
{
    traj->ref = spd;
}

PLATFORM_FAST_CODE float traj_spd_step(traj_spd_t *traj,
                                       float target,
                                       float acc,
                                       float dec,
                                       float dt)
{
    const bool reversing = (traj->ref * target) < 0.0f;
    float step;

    if (reversing)
    {
        step = dec * dt;
        traj->ref = (fabsf(traj->ref) <= step)
            ? 0.0f
            : traj->ref - copysignf(step, traj->ref);
        return traj->ref;
    }

    step = ((fabsf(target) > fabsf(traj->ref)) ? acc : dec) * dt;
    traj->ref = traj_move_toward(traj->ref, target, step);
    return traj->ref;
}

void traj_pos_reset(traj_pos_t *traj, float pos, float spd)
{
    traj->pos = pos;
    traj->spd = spd;
    traj->done = false;
}

PLATFORM_FAST_CODE float traj_pos_step(traj_pos_t *traj,
                                       float target,
                                       float spd_lim,
                                       float acc,
                                       float dec,
                                       float dt)
{
    const float old_spd = traj->spd;
    const float error = target - traj->pos;
    const float stop_spd = sqrtf(2.0f * dec * fabsf(error));
    const float ref_abs = fminf(spd_lim, stop_spd);
    const float spd_ref = copysignf(ref_abs, error);
    const bool accelerating = (old_spd * spd_ref >= 0.0f)
        && (fabsf(spd_ref) > fabsf(old_spd));
    const float rate = accelerating ? acc : dec;

    traj->spd = traj_move_toward(old_spd, spd_ref, rate * dt);
    traj->pos += 0.5f * (old_spd + traj->spd) * dt;

    if (((error >= 0.0f) && (traj->pos >= target))
        || ((error < 0.0f) && (traj->pos <= target)))
    {
        traj->pos = target;
        traj->spd = 0.0f;
        traj->done = true;
    }
    else
    {
        traj->done = false;
    }

    return traj->pos;
}
