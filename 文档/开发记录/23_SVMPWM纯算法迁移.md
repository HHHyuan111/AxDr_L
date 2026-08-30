# SVPWM 纯算法迁移

## 一句话说明这一步

把“归一化 alpha-beta 电压算成三相占空比候选值”的六扇区 SVM 公式迁入 `User/control/foc_svm.c`，四条生产 FOC 路径直接调用新函数；旧 `svm()` 保留作逐位数值对照。

这一步只迁移候选值计算，不操作定时器，不改变相序，也不把无效结果提交给 PWM。

## 新函数负责什么

```c
result = foc_svm(v_alpha_norm,
                 v_beta_norm,
                 &duty_a,
                 &duty_b,
                 &duty_c);
```

输入是已经除以母线电压的 alpha-beta 电压，输出是 A、B、C 三相占空比候选值。

| 返回值 | 含义 | 调用者动作 |
|---:|---|---|
| `0` | 三相候选值都在 `[0, 1]` | 可以继续提交 PWM |
| `-1` | 至少一相超出范围或结果无效 | 本周期不提交新 PWM |

函数不做限幅。这样保留了旧行为：过调制或无效结果会被明确拒绝，而不是悄悄夹到 0～1 后继续输出。

## 它与 Target PWM 的关系

```text
FOC 计算 v_alpha / v_beta
    ↓
乘以 inv_vbus，得到归一化电压
    ↓
foc_svm() 只计算 duty_a / duty_b / duty_c 候选值
    ↓
返回 0 时，原模式调用 foc_pwm_commit()
    ↓
Target 按 ABC / ACB 相序写 TIM1 CCR1～CCR3
```

`foc_svm()` 不知道 TIM1、CCR、PWM 通道、相序和 Gate 状态，因此可以在电脑端、离线回放和未来仿真中复用。

## 生产路径改了哪里

`foc_volt()`、`foc_curr()`、`foc_vel()` 和 `foc_pos()` 原来在末尾调用 `svm()`，现在同一位置改为 `foc_svm()`。

以下内容没有移动：

- Clarke、Park、PI 和逆 Park 的执行位置；
- `inv_vbus` 的乘法位置；
- 三相占空比写入 `pm->foc.dtc_a/b/c` 的位置；
- `== 0` 才向上返回成功的规则；
- 各模式在 FOC 成功后立即提交 PWM 的时序。

因此快速周期没有新增调用层，也没有把 PWM 提交延后。

## 六个扇区有没有接错

新函数保留了 Legacy 的全部扇区判断和每个扇区的作用时间公式：

| 扇区 | 主机对照输入 `(alpha, beta)` |
|---:|---|
| 1 | `(0.2, 0.1)` |
| 2 | `(0.05, 0.2)` |
| 3 | `(-0.2, 0.1)` |
| 4 | `(-0.2, -0.1)` |
| 5 | `(-0.05, -0.2)` |
| 6 | `(0.2, -0.1)` |

另外测试了零矢量 `(0, 0)` 和越界候选 `(2, 2)`。每组都同时调用未修改的 Legacy `svm()` 和新生产 `foc_svm()`，比较：

- 返回值精确相等；
- A、B、C 三相浮点结果逐位相等；
- 迁移比较容差为 0。

已有的零矢量和非零矢量固定期望测试改为直接验证新生产接口，原 FOC 管线回归继续保留。

## 为什么旧 `svm()` 暂时还在

旧函数及旧 `foc_calc()` 暂时保留为 Legacy 数值证据，生产电压、电流、速度和位置路径已经不再调用它。

保留旧实现会暂时占用一部分代码空间，但能在删除 Legacy 前继续做同编译器逐位比较。真正删除旧 SVM 属于后续 Legacy 清理确认点，不在本节点顺手处理。

## 可移植和实时边界

`foc_svm.h/.c`：

- 不包含 `main.h`、`bsp.h` 或 `common.h`；
- 不引用 HAL、`pmsm_t`、寄存器或全局对象；
- 没有动态内存、循环和阻塞调用；
- 目标机与主机编译同一份生产源；
- `foc_svm` 在 Debug ELF 中位于 `.RamFunc`。

输出指针由实时调用者提供有效对象，函数内部不加入重复的空指针检查。

## 离线验收结果

| 检查项 | 结果 |
|---|---|
| CMake / Keil 用户源清单 | 33 / 33，一致 |
| Debug clean-first | 0 错误，26 个警告 |
| Release clean-first | 0 错误，26 个警告 |
| 六扇区 + 零矢量 + 越界新旧对比 | 通过，返回值精确相等、浮点逐位相等 |
| Host C11 / FOC 固定数值回归 | 通过 |
| Host C11 / Drive 状态回归 | 通过 |
| Control 对 main/bsp/common/HAL/pm 的依赖 | 0 处 |
| `foc_svm` 目标段 | `.RamFunc` |
| Debug ELF | text 187268 B，data 136 B，bss 10200 B |
| Release ELF | text 145308 B，data 136 B，bss 10200 B |

与上一个检查点相比，Debug text 增加 1104 B，来自未优化构建中 Legacy 和新 SVM 同时保留；Release text 减少 8 B。data 和 bss 均不变。

## 这一步明确没有做什么

- 没有修改 SVM 公式和归一化方式；
- 没有增加新的过调制、饱和或 NaN 防御策略；
- 没有修改 PWM 提交和 Target；
- 没有迁移 PI、电流环完整管线或旧 `foc_calc()`；
- 没有删除 Legacy `svm()`；
- 没有执行硬件烧录或带电测试。
