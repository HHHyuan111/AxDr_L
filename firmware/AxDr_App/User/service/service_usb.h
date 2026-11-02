/**
 * @file service_usb.h
 * @brief USB CDC 收发薄缝——协议纯逻辑与 USB 硬件的唯一边界。
 * @note  设计动机（19 号架构 §6"以数据为缝"）：service 层协议逻辑不 include
 *        任何 USB/CDC 头，主机测试通过注入 rx 字节序列驱动全链。
 */

#ifndef SERVICE_USB_H
#define SERVICE_USB_H

#include <stdint.h>

#define SERVICE_USB_RX_POLL_BUDGET 256u

/** @brief 注入一段收到的字节（来自 CDC_Receive / 主机测试）。可分任意大小到达。 */
void service_usb_rx(const uint8_t *data, uint16_t length);

/** @brief 发送缝：0=已复制入队，1=队满，其他=失败；栈缓冲在返回后失效。 */
typedef int (*service_usb_tx_fn)(const uint8_t *data, uint16_t length);
void service_usb_bind_tx(service_usb_tx_fn tx);

/** @brief USB 重建连接时丢弃旧接收半帧和重试缓存；中断只登记，主循环清理。 */
void service_usb_reset_link(void);
/** @brief 装配层在 USB 锁内检查，防止命令执行期间重连后补发旧连接的响应。 */
uint8_t service_usb_response_is_current(void);

/** @brief 每轮最多消费 256 字节，半帧保留；Safety 与发送轮询每轮都能执行。 */
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
    uint32_t tx_queue_full;
    uint32_t tx_errors;
    uint32_t rx_dropped_bytes;
    uint32_t link_resets;
} service_usb_stats_t;
const service_usb_stats_t *service_usb_stats(void);

#endif /* SERVICE_USB_H */
