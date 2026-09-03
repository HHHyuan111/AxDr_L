# Target 采样与中断边界收口

## 本节点做了什么

原来 `main.c` 自己创建 ADC DMA 缓冲区，并直接启动 ADC、DMA 和 TIM1 CH4；
应用层 `fast_loop.c` 还直接实现了 STM32 HAL 的 ADC 回调。这样换 MCU 或换采样实现时，
需要同时修改主程序、应用层和 BSP。

现在边界调整为：

```text
main.c
  -> target_adc_start()            启动当前板卡的 ADC 采样链

ADC1 注入完成
  -> target_irq.c                  接住 STM32 HAL 回调
  -> fast_loop_step(&pm)           通知应用执行一拍控制
```

`target_adc.c` 现在独占 ADC DMA 缓冲区，负责：

- ADC1、ADC2 校准；
- 两组注入转换启动；
- 两组规则组 DMA 启动；
- TIM1 CH4 ADC 触发启动；
- ADC 原始值读取。

应用层不再包含 HAL 的回调类型。以后更换 MCU 时，硬件采样启动和中断入口集中在 Target，
`fast_loop_step()` 的控制编排接口保持不变。

## 本节点没有改什么

- 没有改变 ADC 通道、Rank 和 DMA 序号；
- 没有改变 TIM1 CH4 的比较值 3900；
- 没有改变“编码器更新 -> ADC 读取 -> 控制计算 -> PWM 提交”的软件顺序；
- 没有改变三相 PWM 功率输出的启停和占空比提交；
- 没有修改控制参数和算法公式。

## 验收条件

- 应用层不再实现 STM32 HAL 中断回调；
- `main.c` 不再拥有 ADC DMA 缓冲区或展开 ADC 启动步骤；
- CMake 与 Keil 正式源文件清单一致；
- 主机测试、代码规范检查、Debug 和 Release 构建全部通过。
