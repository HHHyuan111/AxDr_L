# PID 与 PDFF 纯算法兼容迁移

## 一句话说明这一步

把“给定目标和反馈，算出本周期控制输出”的 PID/PDFF 公式迁入 `User/control/control_pid.c`，生产路径改用新接口；旧 `motor/pid.c` 暂时保留，专门用于逐拍、逐位对照。

这一步没有改参数、公式、执行频率和限幅规则，也没有把整个电流环一次拆完。

## 现在代码怎么走

```text
电流环、速度环或位置环准备目标值和反馈值
    ↓
调用 control_pid_*_step()
    ↓
函数更新该控制器自己的历史状态并算出 out_value
    ↓
原来的 FOC 流程继续使用 out_value
```

新文件只负责数字计算。它不知道 ADC、编码器、PWM、HAL 或完整的 `pmsm_t`，因此以后可以把同一份算法直接用于电脑端回放和仿真。

## 本次迁移了哪些函数

| 新接口 | 作用 | 当前生产用途 |
|---|---|---|
| `control_pid_set_limits()` | 设置积分项和最终输出上下限 | 初始化电流、速度、位置控制器 |
| `control_pid_clear()` | 清空上一拍历史，保留参数和限值 | Drive 停止或重新启动时复位控制器 |
| `control_pid_parallel_step()` | 算一拍并联 PID | d/q 电流环和位置环 |
| `control_pid_serial_step()` | 算一拍串联 PID | 原 PT、CST 模式的速度控制 |
| `control_pid_pdff_step()` | 算一拍带目标权重的 PDFF | 速度环 |

没有生产调用者的 `pid_para_init()` 和 `pid_reset()` 没有顺手迁移。

## 为什么还保留原来的 pid_para_t 形状

当前 `pid_para_t` 同时装着三类东西：

- `kp/ki/kd`、`kfp/kf_damp` 等参数；
- `p_term/i_term/d_term/pre_err` 等运行历史；
- `ref_value/fback_value/out_value` 等便于观察的数值。

理想情况下可以继续拆成“参数”和“运行状态”，但现在立刻拆会牵动整个 `pmsm_t`、初始化、调试观察和多条模式路径。本节点先把原来 18 个 `volatile float` 的字段、顺序和内存布局原样搬到 Control 头文件，使算法先脱离硬件，避免为了结构漂亮一次改动太多。

是否继续拆分这个结构，留到有明确消费者和收益的后续节点再决定。

## 哪些旧行为被原样保留

- 并联 PID 仍先计算 P、D，再根据 P 动态收紧积分上下限；
- 串联 PID 的积分仍是 `ki * p_term * ts`，没有擅自改成对误差积分；
- PDFF 仍用 `kfp` 和 `kf_damp` 形成比例项，用普通误差形成积分项和微分项；
- `control_pid_clear()` 只清运行历史，不清增益、周期、PDFF 参数和上下限；
- 三个逐拍计算函数继续位于 RAM 执行区；设置限值和清零函数继续位于 Flash；
- 接口采用“调用者提供有效上下文”的实时约定，没有在每个快速函数里重复加入空指针防御。

## 怎样证明没有把公式改坏

新增 `tests/host/test_control_pid.c`，直接同时编译两份生产源码：

- Legacy：`User/motor/pid.c`；
- 新实现：`User/control/control_pid.c`。

测试把相同初值和相同输入分别送给新旧函数。每一拍都检查：

1. 返回的 `float` 位模式完全相同，容差为 0；
2. `pid_para_t` 的 18 个浮点字段全部逐字节相同；
3. 连续多拍覆盖正常累计、正负误差、方向反转和输出饱和；
4. 清零后参数、周期和限值仍保留；
5. 结构大小仍为 18 个 `float`。

Legacy 文件已有一处缩进警告，因此该对照目标只关闭 GCC 的 `misleading-indentation` 提示；其余警告仍按 `-Werror` 处理。新 Control 文件不依赖这项例外。

## 离线验收结果

| 检查项 | 结果 |
|---|---|
| CMake / Keil 用户源清单 | 34 / 34，一致 |
| Debug clean-first | 0 错误，26 个警告 |
| Release clean-first | 0 错误，26 个警告 |
| PID / PDFF 新旧连续逐拍对比 | 通过，返回值和完整上下文逐位相等 |
| Host C11 / FOC 数学回归 | 通过 |
| Host C11 / Drive 状态回归 | 通过 |
| Control 对 main/bsp/common/HAL/pm 的依赖 | 0 处 |
| 三个逐拍计算函数目标段 | RAM 地址区的 `.data`（输入段为 `.RamFunc`） |
| 设置限值与清零函数目标段 | `.text` |
| Debug ELF | text 187268 B，data 136 B，bss 10200 B |
| Release ELF | text 145308 B，data 136 B，bss 10200 B |

与上一个检查点相比，Debug 和 Release 的 text、data、bss 均未变化。链接器删除了不再被固件生产路径引用的 Legacy PID 函数，所以暂时保留源码没有增加最终固件体积。

## 这一步明确没有做什么

- 没有修改 PID/PDFF 增益、采样周期或上下限；
- 没有改变电流环、速度环、位置环的计数分频；
- 没有拆分完整 `pmsm_t` 或一次性重写控制模式；
- 没有新增 NaN、空指针或饱和策略；
- 没有删除 Legacy PID 源码；
- 没有执行烧录、上电或电机测试。
