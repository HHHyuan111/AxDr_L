# MIT、轨迹与观测器正式接入

## 本节点完成了什么

本节点把已有算法从“仓库里有文件”推进到“正式固件有清楚入口”：

| 功能 | Core | 适配/编排 | 正式模式 |
|---|---|---|---|
| MIT | `control_mit.c` | `motor_modes.c::mit_step()` | `release/mit_mode` |
| 速度轨迹 | `control_traj.c` | `motor_modes.c::pv_step()` | `release/vel_mode` |
| 位置轨迹 | `control_traj.c` | `motor_modes.c::pp_step()` | `release/pos_mode` |
| 磁链观测 | `mc_flux_observer.c` | `observer_adapter.c` | 每拍旁路运行，不参与闭环 |

Core 只接收带单位的数值和状态，不读取 `g_foc`，也不调用 HAL、ADC、SPI 或 PWM。
Motor/Adapter 才负责把当前电机对象映射成算法输入，并把结果交给已有 FOC 电流环。

## 三条关键数据流

### MIT

```text
位置/速度/前馈转矩命令
  -> drive_cmd_apply() 限幅和极性处理
  -> control_mit_step() 合成输出轴转矩
  -> 减速比、Kt 换算为 iq_ref
  -> foc_cur_step()
  -> drive_pwm_commit()
```

### Profile Velocity / Position

```text
目标速度或位置
  -> control_traj.c 生成平滑参考
  -> foc_spd_step() 或 foc_pos_step()
  -> 电流环、SVPWM
  -> drive_pwm_commit()
```

位置轨迹按剩余距离计算制动速度：短距离自然形成三角形轨迹，长距离自然形成
梯形轨迹。加减速度属于算法整定参数，已经移到 `algorithm_config.h`；电阻、
电感、磁链、极对数等物理参数仍留在 `motor_config.h`。

### 在线磁链观测

```text
本拍 id/iq、vd/vq、转速
  -> observer_adapter.c
  -> mc_flux_observer_step()
  -> g_obs / g_debug_snapshot
```

它目前只用于核对磁链和电机模型，不回写角度、不改变控制给定、不碰 PWM。
旧的无感角度算法仍未放行，避免把“磁链观测”和“无感换相”混为一件事。

## 为什么 Profile Torque 暂不放行

旧 `pt_tor_mode()` 中的速度限制结果随后会被转矩换算覆盖，控制含义不闭合。
在实物侧尚未确认“转矩斜率”和“超速后的处理方式”前，本节点不猜测新规则，
因此 `tor_mode` 继续由模式白名单拒绝。CST 直接转矩模式已经可以用于验证。

## 验收结果

- MIT 公式、轨迹正反向和速度限制均有 Host 测试；
- Drive 模式表验证 MIT、PV、PP 分派；
- 观测器适配验证数据映射和 PWM 关闭时不推进；
- 调试快照增加观测状态、有效样本数和估计磁链；
- CMake 与 Keil 均为 48 个用户源文件；
- Debug/Release 构建均为 0 错误、0 警告；
- 完整发布候选离线验收通过。
