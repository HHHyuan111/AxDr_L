# Drive 运行模式分派归位

## 一句话说明这一步

把“当前选中了哪种模式，就调用哪个控制函数”的 `switch` 从 `motor/foc_ctrl.c` 搬到 `drive/drive.c`，并改成 Drive 文件内的私有函数 `drive_run_selected_mode()`。

这一步只改变代码放在哪个模块，没有改变任何模式映射、FOC 公式、控制参数或 PWM 提交位置。

## 为什么应该由 Drive 管

快速周期进入 RUN 后，需要先回答一个调度问题：

```text
现在是正常对外模式、停车模式、标定模式，还是实验模式？
    ↓
这个大类下面具体选了哪一个子模式？
    ↓
调用对应的控制函数
```

这是“选择谁运行”，属于 Drive 的编排职责；电流环、速度环、位置环和 MIT 等函数内部“具体怎么算”，仍属于 Control / Motor。

搬迁后依赖方向很直接：

```text
Fast App
    ↓
Drive：状态、启停、模式选择
    ↓
Motor / Control：执行具体算法
    ↓
Target：提交 PWM 到硬件
```

Control 不需要反过来引用 Drive，因此没有形成循环依赖。

## 当前模式映射

### 对外运行模式 `release_mode`

| 子模式 | 实际调用 |
|---|---|
| `mit_mode` | `pm_mit_mode()` |
| `tor_mode` | `pt_tor_mode()` |
| `vel_mode` | `pv_vel_mode()` |
| `pos_mode` | `pp_pos_mode()` |
| `cst_mode` | `cst_tor_mode()` |
| `csv_mode` | `csv_vel_mode()` |
| `csp_mode` | `csp_pos_mode()` |

### 停车模式 `halt_mode`

| 子模式 | 实际调用 |
|---|---|
| `quick_mode` | `pmsm_quick_stop_mode()` |
| `fault_mode` | `pmsm_fault_stop_mode()` |

### 标定与辨识模式 `calibrat_mode`

| 子模式 | 实际调用 |
|---|---|
| `rotor_enc_cali` | `cali_mag_encoder()` |
| `output_enc_mod` | 当前未实现，保持空操作 |
| `output_enc_cali` | 当前未实现，保持空操作 |
| `iden_pm` | `iden_pmsm_first()` |
| `anticogging_pm` | `anticogging_calibration()` |

### 旧实验模式 `debug_mode`

这里的 `debug_mode` 是旧工程用于实验控制的模式大类，不是日志开关或调试结构体。

| 子模式 | 实际调用 |
|---|---|
| `drag_vf` | `force_volt_mode()` |
| `drag_if` | `force_curr_mode()` |
| `volt_op` | `foc_volt()` 成功后提交 PWM |
| `curr_cl` | `foc_curr()` 成功后提交 PWM |
| `spd_curr_cl` | `foc_vel()` 成功后提交 PWM |
| `pos_spd_curr_cl` | `foc_pos()` 成功后提交 PWM |
| `spd_volt_cl` | 当前未实现，保持空操作 |
| `pos_spd_volt_cl` | 当前未实现，保持空操作 |

以上映射与搬迁前逐项相同。独立复核也对照提交 `3a4b78d` 检查了每一个 `case`。

## 对外接口怎样变得更干净

搬迁前，`common.h` 对整个工程公开了 `pmsm_run_selected_mode()`；实际上它只有状态机一个调用者。

搬迁后：

- 外部只看见 `drive_fast_step()`；
- `drive_run_selected_mode()` 使用 `static`，只能在 `drive.c` 内调用；
- `common.h` 删除旧的模式分派公开声明；
- Debug ELF 中该函数是局部 RAM 符号，Release 构建中被编译器内联。

这没有新增接口，反而减少了一个不需要公开的入口。

## 主机测试怎样处理

Drive 状态测试仍直接编译生产 `drive.c`。因为模式分派也进入该文件，测试补齐了各模式函数的最小实现，用来记录 RUN 动作。

测试直接包含生产 `common.h`，只用既有的 `main.h` 和 `bsp.h` 空壳隔离 STM32 头文件；没有复制 Drive 枚举或 `pmsm_t`，因此生产类型变化会直接参与测试编译。

## 离线验收结果

| 检查项 | 结果 |
|---|---|
| 旧 `pmsm_run_selected_mode` 生产符号 | 0 处 |
| 新模式分派可见性 | `static`，Debug ELF 为局部符号 |
| 唯一调用点 | Drive 的 `RUN + pwm_active` 分支 |
| 模式映射对照 | 与 `3a4b78d` 完全一致 |
| CMake / Keil 用户源清单 | 31 / 31，一致 |
| Debug clean-first | 0 错误，26 个警告 |
| Release clean-first | 0 错误，26 个警告 |
| Host C11 / FOC 数学回归 | 通过 |
| Host C11 / Drive 状态回归 | 通过 |
| Debug ELF | text 185348 B，data 136 B，bss 10200 B |
| Release ELF | text 145020 B，data 136 B，bss 10200 B |

与上一个检查点相比，Debug 资源不变；Release text 减少 544 B，data 和 bss 不变。原因是模式分派改成文件私有后，Release 优化器可以在唯一调用点进行更充分的内联和裁剪，不代表删除了某个运行模式。

## 这一步没有做什么

- 没有新增或删除运行模式；
- 没有修改模式的枚举数值；
- 没有把具体 FOC 算法搬进 Drive；
- 没有集中延后 PWM 提交；每个模式仍在原位置计算成功后立即提交；
- 没有接入通信命令、参数整定、故障检测或硬件测试。
