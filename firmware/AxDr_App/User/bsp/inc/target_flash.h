/**
 * @file target_flash.h
 * @brief 内部 Flash 擦写读取（P7 配置持久化的地基，本阶段无调用者）。
 * @note  流程对准 ST RM0440 第 3 章 / HAL 官方序列：Unlock → 页擦除 → 双字编程 → Lock。
 *        存储策略（magic+版本+CRC）属于 service 层，不在此处。
 */

#ifndef TARGET_FLASH_H
#define TARGET_FLASH_H

#include <stdint.h>

#include "ret.h"

/** @brief 擦除地址所在页（G474 每页 2KB，双 Bank 自动识别）。 */
ret_e target_flash_erase_page(uint32_t addr);

/**
 * @brief 向 Flash 写入数据（自动按 8 字节对齐编程，尾部用 0xFF 填充）。
 * @note  目标区域必须已擦除（FF）；addr 必须 8 字节对齐。
 */
ret_e target_flash_write(uint32_t addr, const void *data, uint32_t len);

/** @brief 读取 Flash 内容（内存映射，等同 memcpy）。 */
void target_flash_read(void *dst, uint32_t addr, uint32_t len);

/** @brief 页大小（字节）。 */
uint32_t target_flash_page_size(void);

#endif /* TARGET_FLASH_H */
