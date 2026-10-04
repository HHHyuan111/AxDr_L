/**
 * @file fake_target_flash.h
 * @brief target_flash 主机测试桩：内存模拟一页 + 失败注入 + 原始直读。
 */

#ifndef FAKE_TARGET_FLASH_H
#define FAKE_TARGET_FLASH_H

#include <stdint.h>

void fake_flash_reset(void);            /* 恢复全 0xFF 擦除态，清除注入 */
void fake_flash_inject_erase_fail(int on); /* erase 返回 RET_IO */
void fake_flash_inject_write_fail(int on); /* write 返回 RET_IO */
uint32_t fake_flash_raw(uint32_t offset);  /* 页内偏移读 4 字节（验证落盘） */

#endif /* FAKE_TARGET_FLASH_H */
