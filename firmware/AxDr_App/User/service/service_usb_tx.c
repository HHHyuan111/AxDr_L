#include "service_usb_tx.h"

#include <string.h>

typedef struct
{
    uint8_t data[SERVICE_USB_TX_COMMAND_MAX_SIZE];
    uint16_t length;
} command_slot_t;

typedef struct
{
    uint8_t data[SERVICE_USB_TX_DATA_MAX_SIZE];
    uint16_t length;
} data_slot_t;

static command_slot_t command_queue[SERVICE_USB_TX_COMMAND_CAPACITY];
static data_slot_t data_queue[SERVICE_USB_TX_DATA_CAPACITY];
static uint16_t command_head;
static uint16_t data_head;
static service_usb_tx_submit_fn submit_fn;
static service_usb_tx_lock_fn lock_fn;
static service_usb_tx_unlock_fn unlock_fn;
static service_usb_tx_stats_t stats;

enum { TX_IDLE = 0, TX_COMMAND = 1, TX_DATA = 2 };

static uint32_t enter(void)
{
    return lock_fn != NULL ? lock_fn() : 0u;
}

static void leave(uint32_t state)
{
    if (unlock_fn != NULL)
    {
        unlock_fn(state);
    }
}

static void pop(uint8_t kind)
{
    if (kind == TX_COMMAND)
    {
        command_head = (uint16_t)((command_head + 1u) % SERVICE_USB_TX_COMMAND_CAPACITY);
        --stats.command_pending;
    }
    else
    {
        data_head = (uint16_t)((data_head + 1u) % SERVICE_USB_TX_DATA_CAPACITY);
        --stats.data_pending;
    }
}

void service_usb_tx_init(service_usb_tx_submit_fn submit,
                         service_usb_tx_lock_fn lock,
                         service_usb_tx_unlock_fn unlock)
{
    /* 此时 USB 尚未启动，初始化不依赖中断锁。 */
    submit_fn = submit;
    lock_fn = lock;
    unlock_fn = unlock;
    command_head = 0u;
    data_head = 0u;
    memset(&stats, 0, sizeof(stats));
}

static int enqueue(uint8_t kind, const uint8_t *data, uint16_t length)
{
    const uint16_t maximum = kind == TX_COMMAND ? SERVICE_USB_TX_COMMAND_MAX_SIZE : SERVICE_USB_TX_DATA_MAX_SIZE;
    const uint32_t state = enter();
    int result = 0;

    if ((data == NULL) || (length == 0u) || (length > maximum))
    {
        ++stats.invalid_count;
        result = 2;
    }
    else if (stats.online == 0u)
    {
        ++stats.offline_count;
        result = 2;
    }
    else if (kind == TX_COMMAND)
    {
        if (stats.command_pending == SERVICE_USB_TX_COMMAND_CAPACITY)
        {
            ++stats.command_full;
            result = 1;
        }
        else
        {
            const uint16_t tail = (uint16_t)((command_head + stats.command_pending) % SERVICE_USB_TX_COMMAND_CAPACITY);
            command_slot_t *slot = &command_queue[tail];
            memcpy(slot->data, data, length);
            slot->length = length;
            ++stats.command_accepted;
            ++stats.command_pending;
            if (stats.command_pending > stats.command_peak)
            {
                stats.command_peak = stats.command_pending;
            }
        }
    }
    else if (stats.data_pending == SERVICE_USB_TX_DATA_CAPACITY)
    {
        ++stats.data_full;
        result = 1;
    }
    else
    {
        const uint16_t tail = (uint16_t)((data_head + stats.data_pending) % SERVICE_USB_TX_DATA_CAPACITY);
        data_slot_t *slot = &data_queue[tail];
        memcpy(slot->data, data, length);
        slot->length = length;
        ++stats.data_accepted;
        ++stats.data_pending;
        if (stats.data_pending > stats.data_peak)
        {
            stats.data_peak = stats.data_pending;
        }
    }
    if (stats.command_pending + stats.data_pending > stats.queue_peak)
    {
        stats.queue_peak = (uint16_t)(stats.command_pending + stats.data_pending);
    }
    leave(state);
    return result;
}

int service_usb_tx_enqueue_command(const uint8_t *data, uint16_t length)
{
    return enqueue(TX_COMMAND, data, length);
}

int service_usb_tx_enqueue_data(const uint8_t *data, uint16_t length)
{
    return enqueue(TX_DATA, data, length);
}

void service_usb_tx_poll(void)
{
    const uint32_t state = enter();
    uint8_t kind;
    const uint8_t *data;
    uint16_t length;
    int result;

    if ((stats.online == 0u) || (stats.active != TX_IDLE) || (submit_fn == NULL))
    {
        leave(state);
        return;
    }
    if (stats.command_pending != 0u)
    {
        kind = TX_COMMAND;
        data = command_queue[command_head].data;
        length = command_queue[command_head].length;
    }
    else if (stats.data_pending != 0u)
    {
        kind = TX_DATA;
        data = data_queue[data_head].data;
        length = data_queue[data_head].length;
    }
    else
    {
        leave(state);
        return;
    }

    /* 锁只保护 USB 提交与所有权切换；即使已传首包，也不提前释放整帧。 */
    stats.active = kind;
    result = submit_fn(data, length);
    if (result != 0)
    {
        stats.active = TX_IDLE;
        if (result == 1)
        {
            ++stats.busy_count;
        }
        else
        {
            /* 错误表示底层未接收该指针，释放失败帧，让后续命令继续前进。 */
            ++stats.submit_errors;
            if (kind == TX_COMMAND) { ++stats.command_dropped; }
            else { ++stats.data_dropped; }
            pop(kind);
        }
    }
    leave(state);
}

void service_usb_tx_complete(const uint8_t *data, uint16_t length)
{
    const uint32_t state = enter();
    const uint8_t kind = stats.active;
    if ((kind == TX_COMMAND) && (data == command_queue[command_head].data) &&
        (length == command_queue[command_head].length))
    {
        ++stats.command_completed;
        pop(kind);
        stats.active = TX_IDLE;
    }
    else if ((kind == TX_DATA) && (data == data_queue[data_head].data) &&
             (length == data_queue[data_head].length))
    {
        ++stats.data_completed;
        pop(kind);
        stats.active = TX_IDLE;
    }
    else
    {
        ++stats.unexpected_completions;
    }
    leave(state);
}

void service_usb_tx_set_link(uint8_t online)
{
    const uint32_t state = enter();
    /* CDC DeInit 先关闭端点，因此可丢弃尚未完成的帧；绝不跨连接重放响应。 */
    stats.command_dropped += stats.command_pending;
    stats.data_dropped += stats.data_pending;
    stats.aborted_frames += (uint32_t)stats.command_pending + stats.data_pending;
    stats.command_pending = 0u;
    stats.data_pending = 0u;
    stats.active = TX_IDLE;
    command_head = 0u;
    data_head = 0u;
    stats.online = online != 0u ? 1u : 0u;
    ++stats.link_resets;
    leave(state);
}

void service_usb_tx_get_stats(service_usb_tx_stats_t *out)
{
    const uint32_t state = enter();
    if (out != NULL) { *out = stats; }
    leave(state);
}

uint32_t service_usb_tx_link_generation(void)
{
    const uint32_t state = enter();
    const uint32_t generation = stats.link_resets;
    leave(state);
    return generation;
}
