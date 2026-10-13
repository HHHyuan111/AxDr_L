#ifndef TARGET_IRQ_PROFILE_H
#define TARGET_IRQ_PROFILE_H

#include <stdint.h>

typedef struct
{
    uint32_t count;
    uint32_t last_cycles;
    uint32_t max_cycles;
    uint32_t overruns;
    uint32_t gap_count;
    uint32_t max_interval_cycles;
    uint32_t adc_overruns;
    uint32_t core_clock_hz;
} target_irq_profile_t;

/* ADC1/2 IRQ 的 HAL 调用及全部回调耗时；不含异常入栈/退栈与计数器自身开销。
 * gap 是相邻入口超过名义周期 1.5 倍的诊断，不直接等于漏采次数。 */
uint8_t target_irq_profile_begin(uint32_t start_cycles, uint32_t adc_overrun);
void target_irq_profile_end(uint32_t start_cycles, uint32_t end_cycles);
void target_irq_profile_read(target_irq_profile_t *profile);

#endif
