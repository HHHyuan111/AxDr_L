/**
 * @file test_drive_command.c
 * @brief 验证发布命令的校验、限幅、极性转换和失败原子性。
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "common.h"
#include "drive_command.h"

static bool expect_true(bool condition, const char *message)
{
    if (condition)
    {
        return true;
    }

    fprintf(stderr, "%s\n", message);
    return false;
}

static pmsm_t test_motor(void)
{
    pmsm_t motor = {0};

    motor.app_ctrl.polarity = motor_polarity_p;
    motor.app_ctrl.nmax_torm = -2.0f;
    motor.app_ctrl.pmax_torm = 2.0f;
    motor.app_ctrl.nmax_velm = -100.0f;
    motor.app_ctrl.pmax_velm = 100.0f;
    motor.app_ctrl.nmax_posm = -10.0f;
    motor.app_ctrl.pmax_posm = 10.0f;
    return motor;
}

static bool test_positive_polarity_and_limits(void)
{
    pmsm_t motor = test_motor();

    motor.cmd.torm_set = 3.0f;
    motor.cmd.wm_set = -120.0f;
    motor.cmd.posm_set = 4.0f;

    return expect_true(drive_cmd_apply(&motor),
                       "有限命令和有效限幅应被接受。") &&
           expect_true(motor.ctrl.torm_set == 2.0f,
                       "转矩命令必须限制到正向上限。") &&
           expect_true(motor.ctrl.wm_set == -100.0f,
                       "速度命令必须限制到反向下限。") &&
           expect_true(motor.ctrl.posm_set == 4.0f,
                       "区间内位置命令必须保持不变。");
}

static bool test_negative_polarity(void)
{
    pmsm_t motor = test_motor();

    motor.app_ctrl.polarity = motor_polarity_n;
    motor.cmd.torm_set = 1.0f;
    motor.cmd.wm_set = -20.0f;
    motor.cmd.posm_set = 3.0f;

    return expect_true(drive_cmd_apply(&motor),
                       "反向极性下的有效命令应被接受。") &&
           expect_true(motor.ctrl.torm_set == -1.0f,
                       "反向极性必须翻转转矩给定。") &&
           expect_true(motor.ctrl.wm_set == 20.0f,
                       "反向极性必须翻转速度给定。") &&
           expect_true(motor.ctrl.posm_set == -3.0f,
                       "反向极性必须翻转位置给定。");
}

static bool test_invalid_input_does_not_update_setpoints(void)
{
    pmsm_t motor = test_motor();

    motor.ctrl.torm_set = 0.1f;
    motor.ctrl.wm_set = 0.2f;
    motor.ctrl.posm_set = 0.3f;
    motor.cmd.wm_set = NAN;

    if (!expect_true(!drive_cmd_apply(&motor),
                     "NaN 命令必须被拒绝。") ||
        !expect_true((motor.ctrl.torm_set == 0.1f) &&
                     (motor.ctrl.wm_set == 0.2f) &&
                     (motor.ctrl.posm_set == 0.3f),
                     "命令失败时不能只更新一部分给定。"))
    {
        return false;
    }

    motor.cmd.wm_set = 0.0f;
    motor.app_ctrl.nmax_velm = 10.0f;
    motor.app_ctrl.pmax_velm = -10.0f;
    if (!expect_true(!drive_cmd_apply(&motor),
                     "上下限颠倒时必须拒绝命令。"))
    {
        return false;
    }

    motor.app_ctrl.nmax_velm = -100.0f;
    motor.app_ctrl.pmax_velm = 100.0f;
    motor.app_ctrl.polarity = (motor_polarity_e)99;
    return expect_true(!drive_cmd_apply(&motor),
                       "未知极性必须拒绝命令。");
}

int main(void)
{
    if (!test_positive_polarity_and_limits())
    {
        return 1;
    }

    if (!test_negative_polarity())
    {
        return 2;
    }

    if (!test_invalid_input_does_not_update_setpoints())
    {
        return 3;
    }

    return 0;
}
