/**
 * @file test_public_headers.c
 * @brief 验证 App 与 Drive 对外头文件不依赖完整 foc_t 定义。
 */

#include "fast_loop.h"
#include "control_cycle.h"
#include "debug_snapshot.h"
#include "drive.h"
#include "drive_diag.h"
#include "drive_io.h"
#include "drive_mode.h"
#include "drive_pwm.h"
#include "drive_reset.h"

int main(void)
{
    foc_t *motor = (foc_t *)0;
    control_cycle_input_t input = {0};
    control_cycle_output_t output = {0};
    drive_pwm_cmd_t cmd = {0};

    (void)motor;
    (void)input;
    (void)output;
    (void)cmd;
    return 0;
}
