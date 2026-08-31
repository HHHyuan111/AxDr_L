/**
 * @file spi_bsp.h
 * @brief 快速周期使用的有界同步 SPI 单字传输接口。
 */

#ifndef SPI_BSP_H
#define SPI_BSP_H

#include <stdint.h>

#include "main.h"

static inline uint32_t spi_bsp_tx_ready(const SPI_TypeDef *spi)
{
    return (READ_BIT(spi->SR, SPI_SR_TXE) == SPI_SR_TXE) ? 1UL : 0UL;
}

static inline uint32_t spi_bsp_busy(const SPI_TypeDef *spi)
{
    return (READ_BIT(spi->SR, SPI_SR_BSY) == SPI_SR_BSY) ? 1UL : 0UL;
}

static inline uint32_t spi_bsp_rx_ready(const SPI_TypeDef *spi)
{
    return (READ_BIT(spi->SR, SPI_SR_RXNE) == SPI_SR_RXNE) ? 1UL : 0UL;
}

/**
 * @brief 在限定轮询次数内完成一次 16 位 SPI 收发。
 *
 * @param[in,out] hspi SPI 外设句柄。
 * @param[in] tx_word 本次发送的 16 位数据。
 * @param[out] rx_word 成功时写入收到的 16 位数据。
 * @param[in] timeout_count 每个等待阶段允许的最大轮询次数。
 * @return 成功返回 0；等待超时返回 -1。
 *
 * 这是 ADC 快速周期里的同步接口，因此超时参数表示轮询次数，不表示毫秒。
 * 任一阶段超时后立即退出，不再继续发送或读取无效数据。
 */
static inline int8_t spi_transmit_receive_sync(
    SPI_HandleTypeDef *hspi,
    uint16_t tx_word,
    uint16_t *rx_word,
    uint32_t timeout_count)
{
    uint32_t count = 0U;

    if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
    {
        __HAL_SPI_ENABLE(hspi);
    }

    while (spi_bsp_tx_ready(hspi->Instance) == 0U)
    {
        if (++count > timeout_count)
        {
            return -1;
        }
    }

    hspi->Instance->DR = tx_word;

    count = 0U;
    while (spi_bsp_rx_ready(hspi->Instance) == 0U)
    {
        if (++count > timeout_count)
        {
            return -1;
        }
    }

    *rx_word = (uint16_t)READ_REG(hspi->Instance->DR);

    count = 0U;
    while (spi_bsp_busy(hspi->Instance) != 0U)
    {
        if (++count > timeout_count)
        {
            return -1;
        }
    }

    return 0;
}

#endif /* SPI_BSP_H */
