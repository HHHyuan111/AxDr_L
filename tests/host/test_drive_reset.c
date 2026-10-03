/**
 * @file test_drive_reset.c
 * @brief 验证 Drive 复位清除运行历史但保留配置和反馈。
 */

#include <stdbool.h>
#include <stdio.h>

#include "common.h"
#include "drive_reset.h"

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
    foc_t motor = {
        .rate = {
            .cur_pid_cnt = 3U,
            .spd_pid_cnt = 4U,
            .pos_pid_cnt = 5U,
            .foc_ts = 0.00005f
        },
        .motor = {.pn = 7.0f},
        .ref = {
            .iq = 2.0f,
            .spd_r = 3.0f,
            .pos_r = 4.0f,
            .acc_m = 20.0f,
            .iq_max = 6.0f
        },
        .fb = {
            .theta_e = 1.2f,
            .pos_r = 8.0f,
            .pos_m = 4.0f,
            .spd_m = 1.5f,
            .ia = 0.3f
        },
        .out = {
            .vd = 2.0f,
            .vq = 3.0f,
            .duty_a = 0.2f,
            .duty_b = 0.4f,
            .duty_c = 0.6f
        },
        .id_pi = {
            .kp = 0.5f,
            .i_term = 2.0f,
            .out_max = 10.0f
        },
    };

    drive_control_reset(&motor);

    return expect_true((motor.rate.cur_pid_cnt == 0U) &&
                       (motor.rate.spd_pid_cnt == 0U) &&
                       (motor.rate.pos_pid_cnt == 0U),
                       "复位必须清除三级控制环分频计数。") &&
           expect_true((motor.ref.iq == 0.0f) &&
                       (motor.ref.spd_r == 0.0f) &&
                       (motor.ref.pos_r == 0.0f),
                       "复位必须清除内部电流、速度和位置给定。") &&
           expect_true((motor.out.vd == 0.0f) &&
                       (motor.out.vq == 0.0f),
                       "复位必须清除上一拍 dq 电压。") &&
           expect_true((motor.out.duty_a == 0.5f) &&
                       (motor.out.duty_b == 0.5f) &&
                       (motor.out.duty_c == 0.5f),
                       "复位后的候选占空比应回到中性值。") &&
           expect_true((motor.id_pi.kp == 0.5f) &&
                       (motor.id_pi.out_max == 10.0f) &&
                       (motor.id_pi.i_term == 0.0f),
                       "PID 复位必须保留配置并清除运行历史。") &&
           expect_true((motor.spd_traj.ref == 1.5f) &&
                       (motor.pos_traj.pos == 4.0f) &&
                       (motor.pos_traj.spd == 1.5f),
                       "轨迹复位必须从当前输出轴位置和速度继续。") &&
           expect_true((motor.rate.foc_ts == 0.00005f) &&
                       (motor.motor.pn == 7.0f) &&
                       (motor.ref.acc_m == 20.0f) &&
                       (motor.ref.iq_max == 6.0f),
                       "复位不能覆盖周期、电机参数和限幅配置。") &&
           expect_true((motor.fb.pos_r == 8.0f) &&
                       (motor.fb.ia == 0.3f),
                       "复位不能抹掉当前物理位置和电流反馈。")
        ? 0
        : 1;
}
