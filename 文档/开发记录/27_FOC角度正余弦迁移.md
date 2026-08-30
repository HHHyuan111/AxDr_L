# FOC 角度正余弦迁移

## 一句话说明这一步

把生产 FOC 路径仍在使用的旧 `sin_cos_val()` 迁入现有纯算法模块，改为 `foc_sin_cos()`。四条 FOC 路径只替换函数接口，角度、调用位置和正余弦计算顺序不变。

完成后，`motor/foc_calc.c` 已经没有固件生产路径消费者，只作为 Legacy 数值对照源码保留。

## 代码现在怎么走

```text
模式函数确定本周期电角度 theta
    ↓
原有 wrap_0_2pi() 保持不变
    ↓
foc_sin_cos(theta, &sin_theta, &cos_theta)
    ↓
Clarke / Park / PID / 逆 Park / SVPWM
```

`foc_sin_cos()` 仍先调用 `sinf()`，再调用 `cosf()`，并继续放在 `.RamFunc`。它不接收完整 `pmsm_t`，只接收一个角度和两个输出指针。

## 替换了哪些生产调用

`foc_volt()`、`foc_curr()`、`foc_vel()` 和 `foc_pos()` 中的四处 `sin_cos_val(&pm->foc)`，改为：

```c
foc_sin_cos(pm->foc.theta,
            &pm->foc.sin_val,
            &pm->foc.cos_val);
```

没有改变角度归一化、FOC 执行顺序、控制参数、PID 分频或 PWM 提交位置。

## 怎样验证数值不变

Host 测试同时调用 Legacy `sin_cos_val()` 和新 `foc_sin_cos()`，覆盖：

- `+0.0f` 和 `-0.0f`；
- 正、负普通角度；
- 接近一整圈的角度。

每组 `sin`、`cos` 返回值都逐位比较，容差为 0。已有坐标变换、SVPWM、FOC 管线、PID/PDFF、限幅、Drive 和调试快照测试继续全部执行。

## 为什么固件体积反而变小

旧 `foc_calc.c` 中多个旧 FOC 函数共同放在 `.RamFunc` 输入段。此前生产代码还引用 `sin_cos_val()`，链接器会把同段的一批 Legacy 函数一起带入 ELF。

生产路径切换到 `foc_sin_cos()` 后，旧文件不再被固件引用，链接器可以把整批 Legacy 代码丢弃。因此虽然源码仍保留用于 Host 对照，目标固件 text 反而减少。

## 离线验收结果

| 检查项 | 结果 |
|---|---|
| CMake / Keil 用户源清单 | 36 / 36，一致 |
| Debug clean-first | 0 错误，26 个警告 |
| Release clean-first | 0 错误，26 个警告 |
| 新旧 sin/cos 多角度逐位对比 | 通过，容差 0 |
| 其余 Host 回归 | FOC、PID/PDFF、限幅、快照、Drive 全部通过 |
| `foc_sin_cos` 目标段 | RAM 地址区的 `.data`（输入段为 `.RamFunc`） |
| Control 对 main/bsp/common/HAL/pm 的依赖 | 0 处 |
| `motor/foc_calc.c` 的生产调用者 | 0 个 |
| Debug ELF | text 184340 B，data 136 B，bss 10304 B |
| Release ELF | text 144084 B，data 136 B，bss 10304 B |

与上一个检查点相比，Debug text 减少 3280 B，Release text 减少 1448 B，data 和 bss 不变。

## 这一步明确没有做什么

- 没有删除 `motor/foc_calc.c` 或旧函数声明；
- 没有修改 `sinf/cosf` 算法或改用查表；
- 没有迁移角度归一化宏；
- 没有重写完整电流环；
- 没有执行烧录、上电或电机测试。

下一步已经到达“删除 Legacy FOC/PID/限幅源码和旧声明”确认点，必须先完成阶段 2 总体验收并由用户确认，不能直接删除。
