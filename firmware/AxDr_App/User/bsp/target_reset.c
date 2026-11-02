/**
 * @file target_reset.c
 * @brief 复位原因锁存实现。主链位置：主程序初始化（仅锁存一次）。
 */

#include "target_reset.h"

#include "main.h"

/* G4 HAL 未提供复位标志汇总掩码：RM0440 §RCC_CSR 位 25~31
 * （LPWRR/WWDG/IWDG/SFTRST/BOR/PIN/OBL）。 */
#define TARGET_RESET_FLAGS 0xFE000000u

static uint32_t latched_reason;

void target_reset_latch(void)
{
    latched_reason = RCC->CSR & TARGET_RESET_FLAGS;
    /* 清掉标志：下一次复位的原因才能被正确锁存，不会被旧标志污染。 */
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

uint32_t target_reset_reason(void)
{
    return latched_reason;
}
