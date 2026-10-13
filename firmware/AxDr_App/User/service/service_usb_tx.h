#ifndef SERVICE_USB_TX_H
#define SERVICE_USB_TX_H

#include <stdint.h>

#define SERVICE_USB_TX_COMMAND_CAPACITY 4u
#define SERVICE_USB_TX_DATA_CAPACITY 2u
#define SERVICE_USB_TX_COMMAND_MAX_SIZE 160u
#define SERVICE_USB_TX_DATA_MAX_SIZE 512u

/* 提交缝只登记指针：0=接受，1=忙，其他=未接受。接受后必须最终完成或先关闭端点。 */
typedef int (*service_usb_tx_submit_fn)(const uint8_t *data, uint16_t length);
/* 只屏蔽 USB 中断；不得屏蔽 ADC 控制中断。返回值用于恢复原使能状态。 */
typedef uint32_t (*service_usb_tx_lock_fn)(void);
typedef void (*service_usb_tx_unlock_fn)(uint32_t state);

typedef struct
{
    uint32_t command_accepted;
    uint32_t data_accepted;
    uint32_t command_completed;
    uint32_t data_completed;
    uint32_t command_full;
    uint32_t data_full;
    uint32_t invalid_count;
    uint32_t offline_count;
    uint32_t busy_count;
    uint32_t submit_errors;
    uint32_t command_dropped;
    uint32_t data_dropped;
    uint32_t aborted_frames;
    uint32_t link_resets;
    uint32_t unexpected_completions;
    uint16_t command_peak;
    uint16_t data_peak;
    uint16_t queue_peak;
    uint16_t command_pending;
    uint16_t data_pending;
    uint8_t online;
    uint8_t active;
} service_usb_tx_stats_t;

/* USB 初始化前调用；host 测试可不提供锁，但板端必须提供一对锁函数。 */
void service_usb_tx_init(service_usb_tx_submit_fn submit,
                         service_usb_tx_lock_fn lock,
                         service_usb_tx_unlock_fn unlock);
/* 0=已复制到自有槽，源缓冲可立即复用；1=队满可重试；2=离线/非法。仅主循环调用。 */
int service_usb_tx_enqueue_command(const uint8_t *data, uint16_t length);
int service_usb_tx_enqueue_data(const uint8_t *data, uint16_t length);
/* 主循环每次至多提交一帧；命令优先，BUSY 不自旋，发送中的槽不被复用。 */
void service_usb_tx_poll(void);
/* 从 CDC 最终完成回调调用，多包和 ZLP 均完成后才允许释放。 */
void service_usb_tx_complete(const uint8_t *data, uint16_t length);
/* offline 必须在底层端点已关闭/取消后调用；重新 online 从空队列开始。 */
void service_usb_tx_set_link(uint8_t online);
void service_usb_tx_get_stats(service_usb_tx_stats_t *out);
uint32_t service_usb_tx_link_generation(void);

#endif
