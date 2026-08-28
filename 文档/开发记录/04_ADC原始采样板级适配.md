# ADC 原始采样板级适配

日期：2026-08-28

## 1. 本节点目标

把电机控制代码中的 ADC 寄存器和 DMA 缓冲区访问集中到板级适配文件中，同时保持原有采样来源、相序映射、零偏计算和电流换算公式不变。

改动后的数据流：

`ADC 寄存器和 DMA 缓冲区 -> target_adc 原始采样适配 -> foc_adc_sample 相序映射和电流换算`

本节点不修改 ADC 触发方式、DMA 配置、采样周期、FOC 算法和保护逻辑。

## 2. 原始采样对应关系

| 物理采样信号 | 原工程数据来源 | 适配层字段 |
| --- | --- | --- |
| IA | `ADC1->JDR3` | `adc_raw.i.a` |
| IB | `ADC1->JDR2` | `adc_raw.i.b` |
| IC | `ADC1->JDR1` | `adc_raw.i.c` |
| VA | `adc2_buff[1]` | `adc_raw.v.a` |
| VB | `adc2_buff[2]` | `adc_raw.v.b` |
| VC | `adc2_buff[3]` | `adc_raw.v.c` |
| VBUS | `ADC2->JDR1` | `adc_raw.vbus` |

`i`、`v`、`a/b/c` 和 `vbus` 均采用电机控制中常见的简写，避免重复写出 ADC、原始值和三相等已经由上下文说明的信息。

## 3. 具体改动

1. 新增 `target_adc.h` 和 `target_adc.c`，集中读取 ADC 原始结果。
2. `target_adc_read_iabc_raw` 只读取三相电流，用于启动前的电流零偏采集。
3. `target_adc_read_raw` 读取三相电流、三相电压和母线电压，供快速控制周期使用。
4. `foc_get_curr_off` 改为通过适配接口采集 1000 次电流原始值，平均值算法不变。
5. `foc_adc_sample` 改为使用 `adc_raw`，继续负责 ABC/ACB 相序映射、扣除零偏和换算实际电流。
6. 两个适配读取函数继续放在 `.RamFunc`，保持快速链路的 RAM 执行属性。

## 4. 保持不变的行为

- IA、IB、IC 的 ADC1 注入组寄存器来源不变。
- VA、VB、VC 的 ADC2 DMA 缓冲区序号不变。
- VBUS 的 ADC2 注入组寄存器来源不变。
- ABC 和 ACB 两种相序的 B、C 相交换关系不变。
- 电流零偏仍采集 1000 次，每次间隔 1 ms。
- 电流仍按“原始值减零偏，再乘 `i_ratio`”进行换算。

## 5. 验证结果

- Debug 构建通过：`text 184,412 B`、`data 136 B`、`bss 10,200 B`。
- Release 构建通过：`text 145,692 B`、`data 136 B`、`bss 10,192 B`。
- 两个 ADC 适配函数在 Debug 和 Release 中均位于 RAM 地址区。
- FOC 目录不再直接访问 `ADC1->JDRx`、`ADC2->JDRx` 和 `adc2_buff`。
- 用户代码中的上述硬件读取已集中到 `target_adc.c`。
- `git diff --check` 通过。

当前尚未连接硬件，因此本节点只完成源码边界和编译验证。ADC 触发时序、三相采样波形和换算结果仍需后续上板验证。
