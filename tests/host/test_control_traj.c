/**
 * @file test_control_traj.c
 * @brief 验证速度斜坡和在线梯形位置轨迹。
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "control_traj.h"

static bool expect_true(bool condition, const char *message)
{
    if (condition)
    {
        return true;
    }

    fprintf(stderr, "%s\n", message);
    return false;
}

static bool test_speed_ramp(void)
{
    traj_spd_t traj;

    traj_spd_reset(&traj, 0.0f);
    if (!expect_true(fabsf(traj_spd_step(&traj, 2.0f, 1.0f, 2.0f, 0.1f)
                           - 0.1f) < 1.0e-6f,
                     "速度上升应使用加速度限制。"))
    {
        return false;
    }

    traj_spd_reset(&traj, 1.0f);
    if (!expect_true(fabsf(traj_spd_step(&traj, 0.0f, 1.0f, 2.0f, 0.1f)
                           - 0.8f) < 1.0e-6f,
                     "速度下降应使用减速度限制。"))
    {
        return false;
    }

    traj_spd_reset(&traj, 0.1f);
    return expect_true(fabsf(traj_spd_step(&traj, -1.0f, 1.0f, 2.0f, 0.1f))
                       < 1.0e-6f,
                       "速度反向时应先按减速度降到零。 ");
}

static bool test_position_profile(float start, float target)
{
    traj_pos_t traj;
    unsigned int tick;

    traj_pos_reset(&traj, start, 0.0f);
    for (tick = 0U; tick < 5000U && !traj.done; tick++)
    {
        const float last_pos = traj.pos;

        (void)traj_pos_step(&traj, target, 1.0f, 2.0f, 2.0f, 0.001f);
        if (!expect_true(fabsf(traj.spd) <= 1.000001f,
                         "位置轨迹不能超过速度限制。") ||
            !expect_true(((target >= start) && (traj.pos >= last_pos)) ||
                         ((target < start) && (traj.pos <= last_pos)),
                         "位置轨迹方向不正确。"))
        {
            return false;
        }
    }

    return expect_true(traj.done, "位置轨迹应在有限周期内完成。") &&
           expect_true(traj.pos == target, "位置轨迹结束值应等于目标位置。") &&
           expect_true(traj.spd == 0.0f, "位置轨迹结束速度应为零。");
}

int main(void)
{
    if (!test_speed_ramp() ||
        !test_position_profile(0.0f, 1.0f) ||
        !test_position_profile(1.0f, -0.5f))
    {
        return 1;
    }

    return 0;
}
