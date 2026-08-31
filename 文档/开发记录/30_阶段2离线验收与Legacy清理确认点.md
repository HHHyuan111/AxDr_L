# 阶段 2 离线验收与 Legacy 清理确认点

## 一句话结论

阶段 2 已经把“谁读硬件、谁决定运行、谁算控制、谁写 PWM、调试数据从哪里看”分开，并建立了可重复的电脑端数值与状态测试。

当前新运行链已成立，Legacy FOC/PID/限幅/滤波实现已经退出生产调用链，但旧源码和部分无调用函数还没有删除。按照既定确认规则，本阶段在删除前暂停。

## 当前一次快速周期

```text
ADC1 注入转换完成中断
    ↓
App：fast_loop_step()
    ├─ 编码器采样与角度更新
    ├─ Target 读取 ADC，换算三相电流
    ├─ 更新母线、转矩和速度反馈
    ├─ Drive 判断 STOP / START / RUN / FAULT
    │      ↓
    │   按 sys_mode + op_mode 选择当前模式
    │      ↓
    │   Control 计算 sin/cos、坐标变换、PID/PDFF、LPF、测速、SVPWM
    │      ↓
    │   结果有效时调用 foc_pwm_commit()
    │      ↓
    │   Target 唯一写 TIM1 CCR1 / CCR2 / CCR3
    └─ 发布只读调试快照
```

这张图中的 Control 是纯数值模块；现有模式函数仍负责把 `pm` 字段映射为 Control 输入和输出。当前没有为了追求“全纯化”一次重写所有模式和大结构体。

## 每层现在负责什么

| 层 | 当前文件 | 负责 | 不负责 |
|---|---|---|---|
| App | `User/app/fast_loop.c` | 固定快速周期调用顺序 | 不展开控制公式，不直接写寄存器 |
| Target | `User/bsp/target_*.c` | ADC/编码器原始读取、PWM 启停与比较值提交 | 不决定状态和模式 |
| Drive | `User/drive/drive.c` | STOP/START/RUN/FAULT、PWM 软件所有权、模式分派 | 不读取寄存器，不计算 FOC 数学 |
| Control | `User/control/*.c` | 正余弦、坐标变换、PID/PDFF、限幅、LPF、角度差分测速、SVPWM | 不包含 HAL、Target、`pmsm_t` 或寄存器 |
| 兼容编排 | `motor/foc_ctrl.c`、`foc_drv.c` | 把现有 `pm` 数据送入 Control，组织各环和反馈 | 不直接写 CCR1～CCR3 |
| Debug | `app/debug_snapshot.*` | 单向复制本周期关键量供调试器观察 | 不控制 Drive、参数和 PWM，不发送通信 |

## 阶段 2 完成项

### 1. Drive 最小状态闭环

- 上电默认 `STOP`，三相功率 PWM 不再由 `main()` 直接启动；
- `START` 只执行一次启动、50% 占空比和控制器复位，下一周期进入 `RUN`；
- `RUN` 只在 Drive 持有 PWM 软件所有权时执行当前模式；
- `STOP` 和已有故障关闭三相 PWM；
- TIM1 通道 4 仍单独用于 ADC 触发，不与三相功率输出混淆；
- 模式分派从旧状态机归入 Drive，映射保持不变。

### 2. Control 纯数值基础

生产路径已经切换到以下接口：

- `foc_sin_cos()`；
- `foc_clarke()`、`foc_park()`、`foc_inv_park()`；
- `foc_svm()`；
- 并联 PID、串联 PID、PDFF；
- 通用上下限限制；
- 一阶低通滤波；
- 带显式实例状态的角度差分测速。

Control 目录静态扫描确认没有 `main.h`、`bsp.h`、`common.h`、HAL、Target、`pmsm_t` 和寄存器依赖。

### 3. PWM 硬件边界

- CCR1、CCR2、CCR3 的直接写入只存在于 `target_pwm.c`；
- 所有模式先得到三相候选占空比，再调用统一 `foc_pwm_commit()`；
- `foc_pwm_commit()` 仍负责 ABC/ACB 相序映射；
- 新旧算法没有同时写 PWM。

### 4. 调试与控制解耦

- 每个快速周期在 Drive 执行后发布 104 B 关键量快照；
- Drive 和 Control 对快照零引用；
- 修改快照不会改变完整 `pm`；
- 没有接 VOFA、USB、CAN、环形缓冲或通信协议。

## 离线验收证据

| 检查项 | 结果 |
|---|---|
| CMake / Keil 用户源清单 | 38 / 38，一致 |
| Debug clean-first | 0 错误，26 个警告，未超过 104 条基线 |
| Release clean-first | 0 错误，26 个警告，未超过 104 条基线 |
| FOC 数学 Host 测试 | 通过 |
| 新旧坐标变换、sin/cos、SVPWM、限幅、LPF 对照 | 通过；适用项逐位比较容差为 0 |
| PID/PDFF 连续状态对照 | 通过；返回值和完整上下文逐位相等 |
| Drive 状态与动作顺序测试 | 通过 |
| 调试快照字段、模式和单向关系测试 | 通过 |
| Control 硬件和大对象依赖扫描 | 0 处 |
| CCR1～CCR3 直接写入 | 仅 Target 三处 |
| Debug ELF | text 184548 B，data 136 B，bss 10304 B |
| Release ELF | text 144124 B，data 136 B，bss 10304 B |

相对阶段 1 检查点：Debug text 减少 672 B，Release text 减少 1424 B，data 不变，Berkeley 汇总中的 bss 增加 104 B。新增固定 bss 是调试快照；显式测速状态进入 `pm`，旧测速静态状态要到 Legacy 清理后才移除。

以上是 Arm GNU Debug/Release 离线构建，不等于 Keil ArmClang Rebuild，也不等于硬件运行合格。

## 本阶段的 Git 检查点

| 提交 | 内容 |
|---|---|
| `3a4b78d` | 建立 Drive 最小状态闭环 |
| `16c6169` | 运行模式分派归入 Drive |
| `1cc545e` | FOC 坐标变换纯算法迁移 |
| `f0854ca` | SVPWM 纯算法迁移 |
| `3c2f664` | PID 与 PDFF 纯算法迁移 |
| `33c8c6d` | 通用控制量限幅迁移 |
| `0a037d5` | 建立只读调试快照 |
| `1f187e6` | FOC 角度正余弦迁移 |
| `28177a8` | 一阶低通滤波器迁移 |
| `24e4ca7` | 角度差分测速显式状态迁移 |

## 当前仍保留的 Legacy

以下对象已经没有生产调用，但仍在源码或工程清单中：

- `motor/foc_calc.c` 中的旧 FOC、坐标变换和 SVM；
- `motor/pid.c` 中的旧 PID/PDFF；
- `motor/util.c` 中的旧限幅和低通滤波函数；
- `foc_drv.c` 中的旧静态状态测速函数；
- `common.h` 中对应的旧声明和无定义的 `serial_pid_ctrl1()` 声明；
- 无调用的 `pmsm_ctrl_display()`、`pmsm_display_t` 和 `pm.display`。

旧 FOC/PID 等源码目前用于 Host 新旧对照。直接删除会同时破坏数值证据，所以清理时不能简单一删了之。

## 用户确认后的最小清理计划

1. 把仍需保留的旧公式移到 `tests/legacy`，只参加电脑端对照，不再属于固件生产目录；
2. 从 CMake 和 Keil 生产清单移除旧 `foc_calc.c`、`pid.c`；
3. 删除 `util.c` 和 `foc_drv.c` 中已无调用的旧函数，清理 `common.h` 旧声明；
4. 删除无调用的旧 display 复制函数、类型和 `pm.display` 成员；
5. 保持所有新旧对照测试可运行，再做 2 配置全量构建、源清单、符号和资源复核；
6. 形成最终离线架构验收提交。

这一清理只删除重复实现和失效结构，不改参数、控制公式、状态转换、采样顺序和 PWM 时序。

## 仍需硬件或后续专项确认的事项

- 编码器角度更新与 ADC 电流采样的最终同步方案仍按用户要求待定；
- `pmsm_fault_check()` 尚未接入完整快速保护链；
- START 仍保留旧顺序“先使能输出，再写 50%”，需示波器确认瞬态；
- `pwm_active` 是软件所有权，不是硬件寄存器回读；
- 只读快照增加的快速周期耗时尚未上板测量；
- 当前没有正式 CAN/UART/USB START 命令入口；
- 尚未执行 Keil Rebuild、烧录、带电、电机运行、HIL 或 Elmo 对标。

这些事项不会被离线测试冒充为已经通过。

## 确认点

阶段 2 到此暂停。只有用户明确回复“确认进入 Legacy 清理”后，才执行上述旧源码移动和删除。
