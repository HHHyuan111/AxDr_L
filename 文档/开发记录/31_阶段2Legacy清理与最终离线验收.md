# 阶段 2 Legacy 清理与最终离线验收

## 一句话结论

阶段 2 的生产链已经收口：固件只保留正在使用的新架构实现，迁移前公式只留在电脑端测试区做数值对照，无调用的旧显示副本已经删除。

本次只清理重复代码和失效数据，不改控制公式、控制参数、采样顺序、状态机行为、PWM 提交顺序或硬件初始化。

## 用大白话说明这次做了什么

清理前，同一套功能在工程里有“新、旧两份代码”：实际运行的是新 Control 代码，但旧 FOC、PID、限幅、低通和测速代码仍放在固件目录中。这样以后读代码时容易误以为两套都会运行。

清理后分成两块：

```text
firmware/AxDr_App/User
    └─ 只放 MCU 真正编译、运行的生产代码

tests/legacy
    └─ 只放迁移前公式，电脑端测试用它证明新旧结果一致
```

因此，旧公式没有丢失，但已经不可能被误编进固件或被后续功能继续调用。

## 实际清理内容

### 1. 旧 FOC 和 PID 退出固件

- `motor/foc_calc.c` 移为 `tests/legacy/legacy_foc.c`；
- `motor/pid.c` 移为 `tests/legacy/legacy_pid.c`；
- CMake 和 Keil 同时删除这两个生产源项；
- `common.h` 删除旧 FOC/PID 公共声明；
- 仅声明、从无定义和调用的 `serial_pid_ctrl1()`、`svpwm()` 一并删除；
- Host 测试改从 `tests/legacy` 编译旧公式进行对照。

### 2. 旧基础控制入口退出固件

- 删除 `sat1_datf()`，生产路径统一使用 `control_limit()`；
- 删除 `low_pf_init()`、`low_pf()`，生产路径统一使用 `control_lpf_init()`、`control_lpf_step()`；
- 删除带函数内静态历史量的 `angle_speed_calc()`，生产路径统一使用带显式实例状态的 `control_angle_speed_step()`；
- 三组迁移前公式保留在 `tests/legacy/legacy_util.c`；
- 新增跨圈和正负 pi 用例，逐位比较新旧测速结果。

### 3. 删除无调用的旧 display 副本

- 删除 `pmsm_display_t`；
- 删除 `pmsm_t.display`；
- 删除 `pmsm_ctrl_display()` 的声明和实现。

这里删除的不是 LCD 的 `display_foc()`。`main.c` 仍正常调用 `display_foc()`；新的 `g_debug_snapshot` 也完整保留。

## 生产路径和测试证据的边界

| 内容 | 现在放在哪里 | 是否进入 MCU 固件 | 用途 |
|---|---|---:|---|
| 新 FOC 数学、SVPWM | `User/control` | 是 | 正式控制计算 |
| 新 PID/PDFF、限幅、低通、测速 | `User/control` | 是 | 正式控制计算 |
| 迁移前 FOC/PID/基础公式 | `tests/legacy` | 否 | Host 新旧数值对照 |
| Drive 状态与模式分派 | `User/drive` | 是 | 决定是否运行及运行哪个模式 |
| PWM 硬件提交 | `User/bsp/target_pwm.c` | 是 | 唯一写三相 PWM 比较值 |
| 只读调试快照 | `User/app/debug_snapshot.*` | 是 | 调试器观察，不反向控制 |

`tests/legacy` 是冻结的迁移证据，不是后续算法开发目录。旧代码原有写法被有意保留，避免“整理格式时顺手改变公式”。新增的生产接口和测试辅助声明继续遵守《代码规范1》的命名、头文件保护、接口注释和边界要求。

## 最终离线验收

| 检查项 | 最终结果 |
|---|---|
| CMake / Keil 用户源清单 | 36 / 36，一致 |
| Debug clean-first | 0 错误，25 个警告；未超过 104 条门限 |
| Release clean-first | 0 错误，25 个警告；未超过 104 条门限 |
| FOC 数学 Host 测试 | 通过 |
| PID/PDFF 连续状态逐位对照 | 通过 |
| 限幅、低通、测速新旧对照 | 通过 |
| Drive 状态测试 | 通过 |
| 只读调试快照测试 | 通过 |
| 生产源码 Legacy API 扫描 | 0 处 |
| 固件构建图中的旧 `motor/foc_calc.c`、`motor/pid.c` | 0 处 |
| Control 对 STM32/HAL/BSP/Target 的反向依赖 | 0 处 |
| Debug/Release ELF 中旧限幅、低通、测速、display 符号 | 0 处 |

## 资源变化

相对 Legacy 清理前的阶段 2 检查点：

| 配置 | 阶段 2 检查点 | 清理后 | 变化 |
|---|---:|---:|---:|
| Debug text | 184548 B | 183756 B | -792 B |
| Debug data | 136 B | 136 B | 0 B |
| Debug bss | 10304 B | 10296 B | -8 B |
| Release text | 144124 B | 143604 B | -520 B |
| Release data | 136 B | 136 B | 0 B |
| Release bss | 10304 B | 10304 B | 0 B |
| `pm` 对象 | 50240 B | 50136 B | -104 B |

`pm` 减少的 104 B 正是已删除的 `pmsm_display_t`。因此 `period` 以及它之后的成员地址会前移 104 B；调试时必须加载本次生成的新 ELF，不能继续使用旧 ELF 的 Watch 地址。仓库内没有发现按 `pm` 原始地址或整体二进制布局通信的代码。

## 本次 Git 检查点

| 提交 | 内容 |
|---|---|
| `d879088` | 旧 FOC/PID 移出生产目录并保留 Host 对照 |
| `4c3180c` | 清理旧限幅、低通和隐式测速入口 |
| `4163801` | 删除失效的旧 display 副本 |

## 明确未做的事情

- 未改变 ADC 与编码器更新顺序；最终同步方案仍按此前决定待定；
- 未改变任一 FOC、PID、SVPWM 公式或参数；
- 未改变 STOP/START/RUN/FAULT 状态行为和模式映射；
- 未改变三相 PWM 提交和 ABC/ACB 相序映射；
- 未增加通信协议、锁、双缓冲、环形缓冲或复杂状态；
- 未执行 Keil ArmClang Rebuild、烧录、带电、电机运行、示波器时序、HIL 或 Elmo 对标。

至此，本轮 Goal 约定的离线架构重构和 Legacy 清理已经完成。下一步应先按当前快速周期、Drive 和 Control 文件带用户逐段读懂；涉及采样同步或真实 PWM 行为的结论，必须另开硬件验证节点确认。
