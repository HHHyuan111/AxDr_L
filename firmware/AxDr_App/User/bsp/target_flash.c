/**
 * @file target_flash.c
 * @brief 内部 Flash 擦写读取实现（STM32G474 双 Bank，每页 2KB）。
 */

#include "target_flash.h"

#include <string.h>

#include "main.h"

#define FLASH_END_ADDR 0x08080000U /* 512KB 末尾 */
#define WORD_SIZE 8U

static ret_e flash_addr_to_page(uint32_t addr, uint32_t *page, uint32_t *bank)
{
    if ((addr < FLASH_BASE) || (addr >= FLASH_END_ADDR) || ((addr % WORD_SIZE) != 0U))
    {
        return RET_PARAM;
    }

    if (addr < (FLASH_BASE + 0x00040000U)) /* Bank1: 0x08000000 + 256KB */
    {
        *bank = FLASH_BANK_1;
        *page = (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
    }
    else /* Bank2: 0x08040000 + 256KB */
    {
        *bank = FLASH_BANK_2;
        *page = (addr - FLASH_BASE - 0x00040000U) / FLASH_PAGE_SIZE;
    }
    return RET_OK;
}

ret_e target_flash_erase_page(uint32_t addr)
{
    uint32_t page = 0U;
    uint32_t bank = 0U;
    uint32_t page_error = 0U;
    FLASH_EraseInitTypeDef erase;
    ret_e ret = flash_addr_to_page(addr, &page, &bank);

    if (ret != RET_OK)
    {
        return ret;
    }

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = bank; /* G4 HAL 字段为复数：单 Bank 时即该 Bank 位 */
    erase.Page = page;
    erase.NbPages = 1U;

    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
    {
        ret = RET_IO;
    }
    HAL_FLASH_Lock();
    return ret;
}

ret_e target_flash_write(uint32_t addr, const void *data, uint32_t len)
{
    const uint8_t *src = (const uint8_t *)data;
    uint32_t word;
    uint32_t i;

    if ((data == 0) || (len == 0U) || ((addr % WORD_SIZE) != 0U)
        || (addr < FLASH_BASE) || ((addr + len) > FLASH_END_ADDR))
    {
        return RET_PARAM;
    }

    HAL_FLASH_Unlock();
    for (i = 0U; i < len; i += WORD_SIZE)
    {
        uint32_t j;
        word = 0xFFFFFFFFU; /* 尾部不足 8 字节时用 0xFF 填充（Flash 擦除态） */
        for (j = 0U; (j < WORD_SIZE) && ((i + j) < len); j++)
        {
            word |= ((uint32_t)src[i + j]) << (8U * j);
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, word) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return RET_IO;
        }
    }
    HAL_FLASH_Lock();
    return RET_OK;
}

void target_flash_read(void *dst, uint32_t addr, uint32_t len)
{
    (void)memcpy(dst, (const void *)addr, len);
}

uint32_t target_flash_page_size(void)
{
    return FLASH_PAGE_SIZE;
}
