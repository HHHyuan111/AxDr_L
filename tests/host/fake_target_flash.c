/**
 * @file fake_target_flash.c
 * @brief target_flash 主机测试桩实现：单页后备数组，语义对齐真实现
 *        （erase → 全 0xFF；write 落后备；read 从后备取）。
 * @note  真实现的 addr→后备映射本桩不校验——param_store 只用固定页首，
 *        错误地址路径由真实现的目标机测试覆盖。
 */

#include "fake_target_flash.h"

#include <string.h>

#include "ret.h"

#define FAKE_PAGE_SIZE 2048U

static uint8_t flash_mem[FAKE_PAGE_SIZE];
static int fail_erase;
static int fail_write;

ret_e target_flash_erase_page(uint32_t addr);
ret_e target_flash_write(uint32_t addr, const void *data, uint32_t len);
void target_flash_read(void *dst, uint32_t addr, uint32_t len);
uint32_t target_flash_page_size(void);

void fake_flash_reset(void)
{
    memset(flash_mem, 0xFF, sizeof(flash_mem));
    fail_erase = 0;
    fail_write = 0;
}

void fake_flash_inject_erase_fail(int on)
{
    fail_erase = on;
}

void fake_flash_inject_write_fail(int on)
{
    fail_write = on;
}

uint32_t fake_flash_raw(uint32_t offset)
{
    uint32_t value;
    memcpy(&value, &flash_mem[offset], 4);
    return value;
}

ret_e target_flash_erase_page(uint32_t addr)
{
    (void)addr;
    if (fail_erase)
    {
        return RET_IO;
    }
    memset(flash_mem, 0xFF, sizeof(flash_mem));
    return RET_OK;
}

ret_e target_flash_write(uint32_t addr, const void *data, uint32_t len)
{
    (void)addr;
    if (fail_write)
    {
        return RET_IO;
    }
    if ((data == 0) || (len == 0U) || (len > FAKE_PAGE_SIZE))
    {
        return RET_PARAM;
    }
    memcpy(flash_mem, data, len);
    return RET_OK;
}

void target_flash_read(void *dst, uint32_t addr, uint32_t len)
{
    (void)addr;
    memcpy(dst, flash_mem, len);
}

uint32_t target_flash_page_size(void)
{
    return FAKE_PAGE_SIZE;
}
