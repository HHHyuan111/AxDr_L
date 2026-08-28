/**
 * @file target_encoder.c
 * @brief STM32G474 驱动板上的 MT6816 原始位置读取。
 *
 * 数据流：MT6816 -> SPI1 两帧响应 -> 14 位原始计数 -> 电机位置计算层。
 * 本文件只处理硬件通信和数据拼接，不处理弧度换算、零位和极对数。
 */

#include "target_encoder.h"

#include "spi.h"
#include "spi_bsp.h"

#define MT6816_READ_REG_03_COMMAND (0x8300U)
#define MT6816_READ_REG_04_COMMAND (0x8400U)
#define MT6816_SPI_TIMEOUT_COUNT   (200U)

/**
 * @brief 通过 SPI1 完成一次带独立片选的 16 位 MT6816 通信。
 *
 * @param[in] tx_word 本帧发送的 16 位命令。
 * @param[out] rx_word 接收 MT6816 返回的 16 位数据。
 *
 * 当前保留原工程行为：底层传输状态暂不向位置算法层传播。
 */
static __attribute__((section(".RamFunc")))
void target_encoder_transfer_word(uint16_t tx_word, uint16_t *rx_word)
{
    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);
    (void)spi_transmit_receive_sync(
        &hspi1,
        tx_word,
        rx_word,
        MT6816_SPI_TIMEOUT_COUNT);
    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);
}

__attribute__((section(".RamFunc")))
uint16_t target_encoder_read_mt6816_raw(void)
{
    uint16_t reg_03_response = 0U;
    uint16_t reg_04_response = 0U;

    /* 第 1 步：分别读取保存位置高位和低位的两个寄存器。 */
    target_encoder_transfer_word(MT6816_READ_REG_03_COMMAND, &reg_03_response);
    target_encoder_transfer_word(MT6816_READ_REG_04_COMMAND, &reg_04_response);

    /* 第 2 步：取两帧响应的低 8 位，拼接后右移 2 位，得到 14 位位置值。 */
    return (uint16_t)((
        ((uint32_t)(reg_03_response & 0x00FFU) << 8U)
        | (uint32_t)(reg_04_response & 0x00FFU))
        >> 2U);
}
