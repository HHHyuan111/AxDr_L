#include "common.h"
#include "spi_bsp.h"
#include "target_encoder.h"
//#include "spi.h"

/**
 * @brief 初始化各类编码器的软件参数。
 *
 * @param[in,out] pos_box 接收编码器方向、分辨率和计数到弧度的换算系数。
 *
 * 本函数只配置软件参数，不访问 SPI，也不读取编码器。
 */
void encoder_init(encoder_state_t *pos_box)
{
    pos_box->ma732.dir = 1;
    pos_box->ma732.bit = 14;
    pos_box->ma732.cpr = 16384;

    pos_box->ma732.shift_bit = log2f(pos_box->ma732.cpr / 256);
    pos_box->ma732.factor = M_2PI / pos_box->ma732.cpr;

    pos_box->dm485enc.dir = -1;
    pos_box->dm485enc.bit = 17;
    pos_box->dm485enc.cpr = 131072;

    pos_box->dm485enc.shift_bit = log2f(pos_box->dm485enc.cpr / 256);
    pos_box->dm485enc.factor = M_2PI / pos_box->dm485enc.cpr;

    pos_box->mt6816.dir = 1;
    pos_box->mt6816.bit = 14;
    pos_box->mt6816.cpr = 16384;

    pos_box->mt6816.shift_bit = log2f(pos_box->mt6816.cpr / 256);
    pos_box->mt6816.factor = M_2PI / pos_box->mt6816.cpr;
}

_RAM_FUNC uint8_t encoder_parity(uint16_t v)
{
    uint8_t h_count = 0;
    for (uint8_t j = 0; j < 16; j++)
    {
        if (v & (0x0001 << j))
            h_count++;
    }
    return h_count;
}

_RAM_FUNC uint32_t read_mt6825_raw(void)
{
    uint16_t rx_data[2] = {0U, 0U};

    //cs_down;
    //spi_transmit_receive_sync(&hspi1, tx_data[0], &rx_data[0], 200);
    //spi_transmit_receive_sync(&hspi1, tx_data[1], &rx_data[1], 200);
    //cs_up;

    //while( hspi1.State == HAL_SPI_STATE_BUSY ) {
    //  if (timeOut-- ==0) return 0;
    //}   // wait for transmission complete

    uint16_t parity = (rx_data[0] << 8 | (rx_data[1] >> 8));
    uint8_t nomag = ((rx_data[1] >> 8) & 0x02) >> 1;
    if ((encoder_parity(parity) & 0x01) || nomag)
    {
        //		goto done;
    }

    uint32_t raw = ((rx_data[0] & 0x00FF) << 10) | ((rx_data[1] & 0xFC00) >> 6) | ((rx_data[1] & 0x00F0) >> 4);
    return raw;
}

/**
 * @brief 读取一次 MA732 原始位置并保存到指定编码器对象。
 *
 * @param[in,out] enc 接收本次 14 位原始计数和新数据标志。
 * @return SPI 读取成功返回 true，否则返回 false。
 */
_RAM_FUNC bool read_ma732_raw(encoder_data_t *enc)
{
    uint16_t raw_count;

    if (!target_encoder_read_ma732_raw(&raw_count))
    {
        enc->rev_flag = 0U;
        return false;
    }

    enc->rev_flag = 1U;
    enc->raw = raw_count;
    return true;
}

/**
 * @brief 读取一次 MT6816 原始位置并保存到指定编码器对象。
 *
 * @param[in,out] enc 接收本次原始计数和新数据标志的编码器对象。
 *
 * 硬件通信由 target_encoder 完成；本函数只把结果交给上层已有的数据结构。
 * 显式传入 enc，避免函数暗中修改全局电机对象。
 */
_RAM_FUNC bool read_mt6816_raw(encoder_data_t *enc)
{
    uint16_t raw_count;

    if (!target_encoder_read_mt6816_raw(&raw_count))
    {
        enc->rev_flag = 0;
        return false;
    }

    enc->rev_flag = 1;
    enc->raw = raw_count;
    return true;
}

_RAM_FUNC uint32_t read_dm485enc_raw(void)
{
    //sys_freq = clock_get_frequency(clock_cpu1);
    //tickinlus= sys_freq*0.000001f;
    //write_csr(CSR_MCYCLE, 0);
    // 1. 发送读取指令
    // tx_dma_buff[0] = 0x10;
    // bsp_uart8_transmit(tx_dma_buff, 1);
    //
    // return 0;
    return 0U;
}

_RAM_FUNC uint32_t send_mod_dm485enc(void)
{
    // // 1. 发送调制指令
    // tx_dma_buff[0] = 0x15;
    // tx_dma_buff[1] = 0x24;
    // tx_dma_buff[2] = 0x33;
    // tx_dma_buff[3] = 0x33;
    // tx_dma_buff[4] = 0x42;
    // tx_dma_buff[5] = 0x51;
    // bsp_uart8_transmit(tx_dma_buff, 6);
    //
    // return 0;
    return 0U;
}

/**
 * @brief 把编码器原始计数换算成一圈内的机械角度。
 *
 * @param[in,out] enc 输入原始计数和换算系数，输出范围为 0～2π 的位置角。
 *
 * factor 在初始化时设置为 2π/cpr，因此本函数只完成“计数 × 每计数弧度”。
 * 编码器校准 LUT 尚未接入这条运行链路，后续应通过独立节点实现。
 */
_RAM_FUNC void encoder_update_angle(encoder_data_t *enc)
{
    enc->pos = (float)enc->raw * enc->factor;
    wrap_0_2pi(enc->pos);
}

_RAM_FUNC void bsp_uart8_rxidle_isr(void)
{
    // if(g_foc.mode.sys == calibrat_mode && (g_foc.mode.calibrat == rotor_enc_mod||g_foc.mode.calibrat == output_enc_mod)) {
    //     g_foc.modenc.result = (rx_dma_buff[1] << 8) | rx_dma_buff[0];
    // } else {
    //     // uint8_t crc = bsp_crc8_cal(rx_dma_buff, 3);
    //     //uint8_t crc = bsp_soft_crc8_calc(rx_dma_buff, 3);
    //     //if (crc == rx_dma_buff[3]) {
    //         g_foc.enc.dm485enc.rev_flag = 1; // 设置标志位，表示接收到新数据
    //         g_foc.enc.dm485enc.raw = (rx_dma_buff[2] << 16) | (rx_dma_buff[1] << 8) | (rx_dma_buff[0]);
    //     //}
    // }
    //  //run_tick = read_csr(CSR_MCYCLE);
    //  //run_us = run_tick/tickinlus;
    // memset(rx_dma_buff, 0, Uart_BUFF_SIZE);
    // bsp_uart8_receive(rx_dma_buff, Uart_BUFF_SIZE);
}
