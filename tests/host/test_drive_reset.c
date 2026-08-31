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
    pmsm_t motor = {
        .period = {
            .cur_pid_cnt = 3U,
            .spd_pid_cnt = 4U,
            .pos_pid_cnt = 5U,
            .foc_ts = 0.00005f
        },
        .para = {.pn = 7.0f},
        .ctrl = {
            .iq_set = 2.0f,
            .wr_set = 3.0f,
            .posr_set = 4.0f,
            .wm_acc = 20.0f,
            .pmax_iq = 6.0f
        },
        .foc = {
            .p_e = 1.2f,
            .mp_r = 8.0f,
            .i_a = 0.3f,
            .v_d = 2.0f,
            .v_q = 3.0f,
            .dtc_a = 0.2f,
            .dtc_b = 0.4f,
            .dtc_c = 0.6f
        },
        .id_pi = {
            .kp = 0.5f,
            .i_term = 2.0f,
            .out_max = 10.0f
        },
        .elec_speed_diff = {
            .delta_angle_rad = 0.7f,
            .previous_angle_rad = 0.8f
        }
    };

    drive_control_reset(&motor);

    return expect_true((motor.period.cur_pid_cnt == 0U) &&
                       (motor.period.spd_pid_cnt == 0U) &&
                       (motor.period.pos_pid_cnt == 0U),
                       "复位必须清除三级控制环分频计数。") &&
           expect_true((motor.ctrl.iq_set == 0.0f) &&
                       (motor.ctrl.wr_set == 0.0f) &&
                       (motor.ctrl.posr_set == 0.0f),
                       "复位必须清除内部电流、速度和位置给定。") &&
           expect_true((motor.foc.v_d == 0.0f) &&
                       (motor.foc.v_q == 0.0f),
                       "复位必须清除上一拍 dq 电压。") &&
           expect_true((motor.foc.dtc_a == 0.5f) &&
                       (motor.foc.dtc_b == 0.5f) &&
                       (motor.foc.dtc_c == 0.5f),
                       "复位后的候选占空比应回到中性值。") &&
           expect_true((motor.id_pi.kp == 0.5f) &&
                       (motor.id_pi.out_max == 10.0f) &&
                       (motor.id_pi.i_term == 0.0f),
                       "PID 复位必须保留配置并清除运行历史。") &&
           expect_true((motor.elec_speed_diff.delta_angle_rad == 0.0f) &&
                       (motor.elec_speed_diff.previous_angle_rad == 1.2f),
                       "测速复位必须用当前电角度建立下一拍基准。") &&
           expect_true((motor.period.foc_ts == 0.00005f) &&
                       (motor.para.pn == 7.0f) &&
                       (motor.ctrl.wm_acc == 20.0f) &&
                       (motor.ctrl.pmax_iq == 6.0f),
                       "复位不能覆盖周期、电机参数和限幅配置。") &&
           expect_true((motor.foc.mp_r == 8.0f) &&
                       (motor.foc.i_a == 0.3f),
                       "复位不能抹掉当前物理位置和电流反馈。")
        ? 0
        : 1;
}
