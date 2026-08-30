# FOC 坐标变换纯算法迁移

## 一句话说明这一步

把 Clarke、Park 和逆 Park 三个无状态公式复制到独立的 `User/control/foc_transform.c`，四条生产 FOC 路径改为直接调用新接口；旧公式暂时保留，专门用于数值对照和后续回退。

这一步是第一个真正脱离 `pm`、`common.h`、BSP 和 HAL 的 Control 数值模块。

## 为什么先迁这三个公式

这三个变换具有共同特点：

- 只有当前周期的浮点输入和输出；
- 不保存积分、上次误差或计数器；
- 不读取硬件；
- 不决定状态和运行模式；
- 电压环、电流环、速度环和位置环都有真实调用者。

PID 暂时没有迁移，因为现有 `pid_para_t` 同时装了增益、限幅、积分历史、上次误差和诊断值，控制函数还会动态改写积分上下限。先迁 PID 更容易把状态语义一起改掉，不符合当前的最小迁移原则。

## 新接口长什么样

```c
foc_clarke(i_a, i_b, i_c, &i_alpha, &i_beta);

foc_park(i_alpha,
         i_beta,
         sin_theta,
         cos_theta,
         &i_d,
         &i_q);

foc_inv_park(v_d,
             v_q,
             sin_theta,
             cos_theta,
             &v_alpha,
             &v_beta);
```

接口只传公式真正需要的数值，不把整个 `pmsm_t` 或巨大的 `pmsm_foc_t` 交给纯算法。

| 接口 | 输入 | 输出 | 是否保存状态 |
|---|---|---|---|
| `foc_clarke()` | 三相电流 | alpha-beta 电流 | 否 |
| `foc_park()` | alpha-beta 电流、角度正余弦 | d-q 电流 | 否 |
| `foc_inv_park()` | d-q 电压、角度正余弦 | alpha-beta 电压 | 否 |

输出指针由实时调用者提供有效对象，函数内部不增加重复的空指针防御。三个函数都只有固定次数的乘加运算，没有循环、阻塞和动态内存。

## 生产调用链怎样变化

迁移前：

```text
foc_volt / foc_curr / foc_vel / foc_pos
    → clarke_transform(pmsm_foc_t *)
    → park_transform(pmsm_foc_t *)
    → inverse_park(pmsm_foc_t *)
```

迁移后：

```text
foc_volt / foc_curr / foc_vel / foc_pos
    → foc_clarke(明确的浮点输入和输出)
    → foc_park(明确的浮点输入和输出)
    → foc_inv_park(明确的浮点输入和输出)
```

角度归一化、正余弦计算、PI 调节、SVM 和 PWM 提交仍处于原来的先后位置。每个旧函数调用被一个新函数调用替代，没有在 20 kHz 路径上增加包装层。

## 为什么旧函数还保留

`foc_calc.c` 中原来的 `clarke_transform()`、`park_transform()` 和 `inverse_park()` 公式没有删除，也没有转调新函数。

当前保留它们有两个用途：

1. 主机测试可以在同一编译器下把 Legacy 结果和新结果逐位比较；
2. 在后续 Control 管线全部验证完成前，仍保留清楚的回退依据。

因此这一步不是“双实现同时提交 PWM”。生产路径只调用新实现；Legacy 只参加电脑端对照，不参与生产 FOC 调用。删除旧接口属于后续 Legacy 清理，必须到既定确认点再做。

## 数值是否真的没变

新模块逐字保留了原公式中的：

- 常量 `0.57735026919f`；
- 加减乘的左右操作数；
- `sin_theta`、`cos_theta` 的参数顺序；
- 每个输出的赋值顺序。

主机测试使用四组包含零值、正负值、非零角度正余弦和正负零的数据，同时调用 Legacy 和新生产函数，并用浮点对象的实际字节逐项比较。Clarke 的两个输出、Park 的两个输出和逆 Park 的两个输出全部逐位一致，迁移容差为 0。

另外保留已有的数值期望测试，确认新接口的代表性结果，并继续运行原 FOC/SVM 回归。

## 可移植边界

`foc_transform.h/.c`：

- 不包含 `main.h`；
- 不包含 `bsp.h`；
- 不包含 `common.h`；
- 不引用 HAL、寄存器、`pmsm_t` 或 `pmsm_foc_t`；
- 主机和 ARM 固件编译同一份生产源文件。

唯一目标机相关项是 `.RamFunc` 代码段属性，它只写在实现文件内部，不污染公共数值接口。Debug ELF 已确认三个新符号地址均位于 RAM。

## 离线验收结果

| 检查项 | 结果 |
|---|---|
| CMake / Keil 用户源清单 | 32 / 32，一致 |
| Debug clean-first | 0 错误，26 个警告 |
| Release clean-first | 0 错误，26 个警告 |
| Host C11 / 新旧坐标变换逐位对比 | 通过，容差 0 |
| Host C11 / FOC 与 SVM 回归 | 通过 |
| Host C11 / Drive 状态回归 | 通过 |
| Control 对 main/bsp/common/HAL/pm 的依赖 | 0 处 |
| 新坐标变换 RAM 符号 | 3 个，均位于 `.RamFunc` |
| Debug ELF | text 186164 B，data 136 B，bss 10200 B |
| Release ELF | text 145316 B，data 136 B，bss 10200 B |

与上一个检查点相比，Debug text 增加 816 B，Release text 增加 296 B；data 和 bss 不变。增加部分主要来自迁移期同时保留 Legacy 公式和新生产实现，属于有意的临时验证成本。

## 这一步明确没有做什么

- 没有迁移无生产消费者的逆 Clarke；
- 没有迁移 SVM、查表正余弦、PID 或完整电流环；
- 没有修改公式、常量、参数和调用频率；
- 没有改变角度、ADC 和反馈更新顺序；
- 没有删除旧 FOC 接口；
- 没有烧录、带电或驱动电机。
