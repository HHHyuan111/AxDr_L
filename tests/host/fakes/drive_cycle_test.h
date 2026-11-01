#ifndef DRIVE_CYCLE_TEST_H
#define DRIVE_CYCLE_TEST_H
#include "foc_fwd.h"
/* 以测试已设置的反馈进入生产主流程，不读取硬件或替换状态调度。 */
void run_drive_cycle(foc_t *foc);
#endif
