/**
 * @file fake_target_flash.c
 * @brief target_flash 主机测试桩实现：按地址映射的 3 页后备数组，
 *        语义对齐真实现（erase → 整页 0xFF；write 落后备；read 从后备取）。
 * @note  映射基址 0x08040000（Bank2）：页1=e_off、页2=辨识档案、页3=boot_id。
 *        越界地址回退页 0，保持旧桩宽松语义；错误地址路径由真实现覆盖。
 */

#include "fake_target_flash.h"

#include <string.h>

#include "ret.h"

#define FAKE_PAGE_SIZE 2048U
#define FAKE_FLASH_BASE 0x08040000U
#define FAKE_PAGE_COUNT 3U

static uint8_t flash_mem[FAKE_PAGE_SIZE * FAKE_PAGE_COUNT];
static int fail_erase;
static int fail_write;

ret_e target_flash_erase_page(uint32_t addr);
ret_e target_flash_write(uint32_t addr, const void *data, uint32_t len);
void target_flash_read(void *dst, uint32_t addr, uint32_t len);
uint32_t target_flash_page_size(void);

static uint32_t fake_offset(uint32_t addr)
{
    uint32_t off = addr - FAKE_FLASH_BASE;
    if (off >= (uint32_t)sizeof(flash_mem))
    {
        off = 0U;
    }
    return off;
}

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
    const uint32_t off = fake_offset(addr);

    if (fail_erase)
    {
        return RET_IO;
    }
    memset(&flash_mem[(off / FAKE_PAGE_SIZE) * FAKE_PAGE_SIZE],
           0xFF, FAKE_PAGE_SIZE);
    return RET_OK;
}

ret_e target_flash_write(uint32_t addr, const void *data, uint32_t len)
{
    const uint32_t off = fake_offset(addr);

    if (fail_write)
    {
        return RET_IO;
    }
    if ((data == 0) || (len == 0U) ||
        (off + len > (uint32_t)sizeof(flash_mem)))
    {
        return RET_PARAM;
    }
    memcpy(&flash_mem[off], data, len);
    return RET_OK;
}

void target_flash_read(void *dst, uint32_t addr, uint32_t len)
{
    const uint32_t off = fake_offset(addr);
    memcpy(dst, &flash_mem[off], len);
}

uint32_t target_flash_page_size(void)
{
    return FAKE_PAGE_SIZE;
}
