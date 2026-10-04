/**
 * @file service_usb.h
 * @brief USB CDC 收发薄缝——协议纯逻辑与 USB 硬件的唯一边界。
 * @note  设计动机（19 号架构 §6"以数据为缝"）：service 层协议逻辑不 include
 *        任何 USB/CDC 头，主机测试通过注入 rx 字节序列驱动全链。
 */

#ifndef SERVICE_USB_H
#define SERVICE_USB_H

#include <stdint.h>

/** @brief 注入一段收到的字节（来自 CDC_Receive / 主机测试）。可分任意大小到达。 */
void service_usb_rx(const uint8_t *data, uint16_t length);

/** @brief 发送缝：协议层需要发响应/遥测时回调（固件=CDC_Transmit；测试=捕获数组）。 */
typedef void (*service_usb_tx_fn)(const uint8_t *data, uint16_t length);
void service_usb_bind_tx(service_usb_tx_fn tx);

/** @brief 主循环轮询：搬运 rx 队列 → 帧同步 → command_core 分发 → 响应经 tx 缝发出。 */
void service_usb_poll(void);

/** @brief 诊断计数（GetLinkDiagnostics 用）。 */
typedef struct
{
    uint32_t rx_bytes;
    uint32_t rx_frames;
    uint32_t resync_count;
    uint32_t bad_crc_count;
    uint32_t rejected_count;
    uint32_t tx_frames;
} service_usb_stats_t;
const service_usb_stats_t *service_usb_stats(void);

#endif /* SERVICE_USB_H */
