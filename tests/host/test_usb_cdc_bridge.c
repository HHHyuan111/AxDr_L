#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 只替换 USB 库类型与底层；直接编译生产 CDC 桥，验证实际回调装配。 */
#define __USBD_CDC_IF_H__
#define APP_RX_DATA_SIZE 2048u
#define APP_TX_DATA_SIZE 2048u
#define UNUSED(x) ((void)(x))
#define USBD_OK 0u
#define USBD_BUSY 1u
#define USBD_FAIL 2u
#define USBD_STATE_CONFIGURED 3u
#define CDC_SEND_ENCAPSULATED_COMMAND 0u
#define CDC_GET_ENCAPSULATED_RESPONSE 1u
#define CDC_SET_COMM_FEATURE 2u
#define CDC_GET_COMM_FEATURE 3u
#define CDC_CLEAR_COMM_FEATURE 4u
#define CDC_SET_LINE_CODING 0x20u
#define CDC_GET_LINE_CODING 0x21u
#define CDC_SET_CONTROL_LINE_STATE 0x22u
#define CDC_SEND_BREAK 0x23u

typedef struct
{
    uint32_t TxState;
    uint8_t *TxBuffer;
    uint32_t TxLength;
} USBD_CDC_HandleTypeDef;

typedef struct
{
    void *pClassData;
    uint8_t dev_state;
} USBD_HandleTypeDef;

typedef struct
{
    int8_t (*Init)(void);
    int8_t (*DeInit)(void);
    int8_t (*Control)(uint8_t, uint8_t *, uint16_t);
    int8_t (*Receive)(uint8_t *, uint32_t *);
    int8_t (*TransmitCplt)(uint8_t *, uint32_t *, uint8_t);
} USBD_CDC_ItfTypeDef;

USBD_HandleTypeDef hUsbDeviceFS;
static USBD_CDC_HandleTypeDef cdc;
static uint32_t rx_resets;
static uint32_t rx_bytes;

static uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *device, uint8_t *data, uint32_t length)
{
    USBD_CDC_HandleTypeDef *handle = device->pClassData;
    assert(handle != NULL);
    handle->TxBuffer = data;
    handle->TxLength = length;
    return USBD_OK;
}

static uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *device, uint8_t *data)
{
    (void)device;
    (void)data;
    return USBD_OK;
}

static uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *device)
{
    assert(device->pClassData != NULL);
    return USBD_OK;
}

static uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *device)
{
    USBD_CDC_HandleTypeDef *handle = device->pClassData;
    if (handle->TxState != 0u) { return USBD_BUSY; }
    handle->TxState = 1u;
    return USBD_OK;
}

void service_usb_reset_link(void) { ++rx_resets; }
void service_usb_rx(const uint8_t *data, uint16_t length)
{
    assert(data != NULL);
    rx_bytes += length;
}

#include "../../firmware/AxDr_App/USB_Device/App/usbd_cdc_if.c"

static int submit(const uint8_t *data, uint16_t length)
{
    return (int)CDC_Transmit_FS((uint8_t *)data, length);
}

static void configured(void)
{
    memset(&cdc, 0, sizeof(cdc));
    hUsbDeviceFS.pClassData = &cdc;
    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    assert(USBD_Interface_fops_FS.Init() == USBD_OK);
}

static void complete(void)
{
    cdc.TxState = 0u;
    assert(USBD_Interface_fops_FS.TransmitCplt(cdc.TxBuffer, &cdc.TxLength, 1u) == USBD_OK);
}

int main(void)
{
    uint8_t frame[160];
    uint8_t expected[160];
    service_usb_tx_stats_t stats;
    service_usb_tx_init(submit, NULL, NULL);
    assert(CDC_Transmit_FS(frame, sizeof(frame)) == USBD_FAIL);
    configured();
    assert(rx_resets == 1u);
    memset(frame, 0x37, sizeof(frame));
    memcpy(expected, frame, sizeof(frame));
    assert(service_usb_tx_enqueue_data(frame, sizeof(frame)) == 0);
    service_usb_tx_poll();
    memset(frame, 0xD9, sizeof(frame));
    assert(service_usb_tx_enqueue_command(frame, sizeof(frame)) == 0);
    assert(CDC_Transmit_FS(frame, sizeof(frame)) == USBD_BUSY);
    service_usb_tx_poll();
    assert(cdc.TxState == 1u);
    assert(memcmp(cdc.TxBuffer, expected, sizeof(expected)) == 0);
    complete();
    service_usb_tx_poll();
    assert(memcmp(cdc.TxBuffer, frame, sizeof(frame)) == 0);
    complete();
    service_usb_tx_get_stats(&stats);
    assert(stats.data_completed == 1u && stats.command_completed == 1u);

    /* 关闭串口的 DTR 请求不等于 USB 端点关闭，也不释放在途指针。 */
    assert(service_usb_tx_enqueue_data(frame, sizeof(frame)) == 0);
    service_usb_tx_poll();
    assert(USBD_Interface_fops_FS.Control(CDC_SET_CONTROL_LINE_STATE, NULL, 0u) == USBD_OK);
    service_usb_tx_get_stats(&stats);
    assert(stats.active != 0u && stats.online == 1u);

    /* 类驱动先关闭端点，再回调 DeInit。重新配置后只传新连接的数据。 */
    cdc.TxState = 0u;
    assert(USBD_Interface_fops_FS.DeInit() == USBD_OK);
    hUsbDeviceFS.pClassData = NULL;
    hUsbDeviceFS.dev_state = 0u;
    service_usb_tx_get_stats(&stats);
    assert(stats.active == 0u && stats.online == 0u && stats.aborted_frames == 1u);
    assert(CDC_Transmit_FS(frame, sizeof(frame)) == USBD_FAIL);
    configured();
    service_usb_tx_poll();
    assert(cdc.TxState == 0u);
    assert(service_usb_tx_enqueue_command(frame, sizeof(frame)) == 0);
    service_usb_tx_poll();
    complete();
    {
        uint32_t length = sizeof(frame);
        assert(USBD_Interface_fops_FS.Receive(frame, &length) == USBD_OK);
        assert(rx_bytes == sizeof(frame));
    }
    assert(rx_resets == 3u);
    puts("test_usb_cdc_bridge: all pass");
    return 0;
}
