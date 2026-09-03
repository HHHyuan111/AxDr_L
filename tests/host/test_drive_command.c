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

static foc_t test_motor(void)
{
    foc_t motor = {0};

    motor.app.polarity = motor_polarity_p;
    motor.app.nmax_torm = -2.0f;
    motor.app.pmax_torm = 2.0f;
    motor.app.nmax_velm = -100.0f;
    motor.app.pmax_velm = 100.0f;
    motor.app.nmax_posm = -10.0f;
    motor.app.pmax_posm = 10.0f;
    return motor;
}

static bool test_positive_polarity_and_limits(void)
{
    foc_t motor = test_motor();

    motor.mode.release = csp_mode;
    motor.cmd.torq = 3.0f;
    motor.cmd.spd = -120.0f;
    motor.cmd.pos = 4.0f;

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
    foc_t motor = test_motor();

    motor.mode.release = csp_mode;
    motor.app.polarity = motor_polarity_n;
    motor.cmd.torq = 1.0f;
    motor.cmd.spd = -20.0f;
    motor.cmd.pos = 3.0f;

    return expect_true(drive_cmd_apply(&motor),
                       "反向极性下的有效命令应被接受。") &&
           expect_true(motor.ctrl.torm_set == -1.0f,
                       "反向极性必须翻转转矩给定。") &&
           expect_true(motor.ctrl.wm_set == 20.0f,
                       "反向极性必须翻转速度给定。") &&
           expect_true(motor.ctrl.posm_set == -3.0f,
                       "反向极性必须翻转位置给定。");
}

static bool test_mit_command(void)
{
    foc_t motor = test_motor();

    motor.mode.release = mit_mode;
    motor.app.polarity = motor_polarity_n;
    motor.cmd.mit_ff = 3.0f;
    motor.cmd.spd = 20.0f;
    motor.cmd.pos = 3.0f;
    motor.cmd.kp = 2.0f;
    motor.cmd.kd = 0.5f;

    return expect_true(drive_cmd_apply(&motor),
                       "有效 MIT 命令应被接受。") &&
           expect_true(motor.ctrl.mit_tor_set == -2.0f,
                       "MIT 前馈转矩应先限幅再按极性翻转。") &&
           expect_true((motor.ctrl.wm_set == -20.0f) &&
                       (motor.ctrl.posm_set == -3.0f),
                       "MIT 位置和速度应使用统一极性。") &&
           expect_true((motor.ctrl.kp == 2.0f) && (motor.ctrl.kd == 0.5f),
                       "MIT 增益必须完整传入控制状态。");
}

static bool test_invalid_input_does_not_update_setpoints(void)
{
    foc_t motor = test_motor();

    motor.ctrl.torm_set = 0.1f;
    motor.ctrl.wm_set = 0.2f;
    motor.ctrl.posm_set = 0.3f;
    motor.cmd.spd = NAN;

    if (!expect_true(!drive_cmd_apply(&motor),
                     "NaN 命令必须被拒绝。") ||
        !expect_true((motor.ctrl.torm_set == 0.1f) &&
                     (motor.ctrl.wm_set == 0.2f) &&
                     (motor.ctrl.posm_set == 0.3f),
                     "命令失败时不能只更新一部分给定。"))
    {
        return false;
    }

    motor.cmd.spd = 0.0f;
    motor.app.nmax_velm = 10.0f;
    motor.app.pmax_velm = -10.0f;
    if (!expect_true(!drive_cmd_apply(&motor),
                     "上下限颠倒时必须拒绝命令。"))
    {
        return false;
    }

    motor.app.nmax_velm = -100.0f;
    motor.app.pmax_velm = 100.0f;
    motor.app.polarity = (motor_polarity_e)99;
    if (!expect_true(!drive_cmd_apply(&motor),
                     "未知极性必须拒绝命令。"))
    {
        return false;
    }

    motor.app.polarity = motor_polarity_p;
    motor.cmd.kp = -1.0f;
    return expect_true(!drive_cmd_apply(&motor),
                       "MIT 增益不能为负值。");
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

    if (!test_mit_command())
    {
        return 3;
    }

    if (!test_invalid_input_does_not_update_setpoints())
    {
        return 4;
    }

    return 0;
}
