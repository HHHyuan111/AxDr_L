#ifndef SERVICE_OBSERVATION_H
#define SERVICE_OBSERVATION_H

#include <stdint.h>

/* 返回 0 表示数据已经复制到发送方拥有的缓冲；1 表示忙，其他值为失败。 */
typedef int (*service_observation_tx_fn)(const uint8_t *data, uint16_t length);

/* init 仅启动时调用；capture 仅在完成当前拍控制更新后的 ADC ISR 调用。 */
void service_observation_init(void);
void service_observation_capture(void);
/* ADC ISR 入口报告迟到/溢出；不能把 fast_seq 连号当作硬件采样连续。 */
void service_observation_note_timing_gap(void);
void service_observation_poll(void);
void service_observation_bind_tx(service_observation_tx_fn tx);
/* 主循环调用。立即停止发送，采集取消在下一次 ISR 边界生效。 */
void service_observation_reset_link(void);
void service_observation_set_timing_stats(uint32_t last_cycles,
                                         uint32_t max_cycles,
                                         uint32_t overruns,
                                         uint32_t core_clock_hz);
void service_observation_set_tx_stats(uint32_t completed, uint32_t errors,
                                     uint32_t aborted, uint32_t queue_peak);
void service_observation_set_irq_stats(uint32_t count, uint32_t gaps,
                                      uint32_t max_interval_cycles,
                                      uint32_t adc_overruns);
uint8_t service_observation_route(void *context, uint16_t opcode,
                                 const uint8_t *request_payload,
                                 uint16_t request_payload_length,
                                 uint8_t *response_payload,
                                 uint16_t response_payload_capacity,
                                 uint16_t *response_payload_length,
                                 uint8_t *ack_state);

#endif
