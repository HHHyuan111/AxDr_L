#ifndef AXDR_HOST_TEST_FAKE_MAIN_H
#define AXDR_HOST_TEST_FAKE_MAIN_H

/* common.h 在电脑端只需要 CMSIS 提供的两个类型限定符。 */
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I volatile const
#endif

/* target_irq.c 的 HAL 回调在 Host 测试中只需要该句柄的不完整类型。 */
typedef struct ADC_HandleTypeDef ADC_HandleTypeDef;

#endif
