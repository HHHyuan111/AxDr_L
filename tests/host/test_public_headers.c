/**
 * @file test_public_headers.c
 * @brief 验证 App 与 Drive 对外头文件不依赖完整 pmsm_t 定义。
 */

#include "axdr_app.h"
#include "debug_snapshot.h"
#include "drive.h"
#include "drive_io.h"
#include "drive_mode.h"
#include "drive_pwm.h"

int main(void)
{
    pmsm_t *motor = (pmsm_t *)0;
    drive_pwm_cmd_t cmd = {0};

    (void)motor;
    (void)cmd;
    return 0;
}
