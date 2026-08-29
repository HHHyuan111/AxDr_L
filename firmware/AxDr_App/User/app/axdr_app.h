/**
 * @file axdr_app.h
 * @brief AxDr 固件应用层总入口。
 *
 * 本模块负责把板级中断与现有电机控制流程连接起来。硬件中断只通知应用层“新的
 * 控制周期已经到来”，具体的采样、反馈更新和控制执行顺序由应用层统一安排。
 */

#ifndef AXDR_APP_H
#define AXDR_APP_H

/**
 * @brief 执行一次快速电机控制周期。
 *
 * 本函数由 ADC 注入转换完成中断调用，依次完成编码器采样、ADC 采样、反馈更新和
 * 电机状态机执行。当前只集中原有调用顺序，不修改任何控制算法和硬件输出结果。
 */
void axdr_app_fast_step(void);

#endif /* AXDR_APP_H */
