# 开发记录 61：P7 service 层上机验收

## 1. 当前结论

P7 service 层代码全部完成（S1 协议核心 / S2 USB 薄缝 / S3 opcode 分发器 /
S3.5 审查修复 / S4a 遥测纯逻辑 / S4b 板级 CDC 接线 / S5 参数持久化），
三闸全绿：

| 闸 | 结果 |
|---|---|
| 主机测试（gcc -Werror） | 全部 pass（含 S3 旅程 / S4 遥测 / S5 持久化三块新测试） |
| gcc 交叉编译 | 106 文件 0 失败 0 警告，链接成功 |
| Keil MDK | 0 Error 0 Warning |

资源占用：flash_text=68000 B、rodata=14636 B、ram_data=13440 B、bss=22800 B。
G474RET6（512KB flash / 128KB RAM）余量充足。

本手册验收通过后打 `layer-service` 标签，P7 关闭。

## 2. 烧录与上电前静态确认

- [ ] Git 位于 `d65561c`（P7-S5）或其后验收修复提交，工作区干净；
- [ ] Keil 重新全量编译后烧录（确认 `service_param_store.c` 在编译清单内）；
- [ ] 电机为沉沙 200W + ABZ 2500 线，机械安装与档案一致（`MOTOR_PROFILE_REVISION=2`）；
- [ ] 使用限流电源（24V），急停/断电手段在手；第一次不带负载。

## 3. 第一次上电：遥测探针（不发电流指令）

上电后固件立即以 20Hz 持续发送 160B 遥测帧（无需任何使能操作）。
设备管理器确认 ST CDC 枚举出的 COM 口号，执行：

```powershell
& 'C:\Users\ready\Desktop\Axdr\AX_Driver_Lab\.tools\QtMotorTool-src\deploy_v2\QtMotorTool.exe' --probe-usb COMx
```

预期 `USB_PROBE PASS`：`frames≈60`（3 秒×20Hz）、`version=1`、`size=160`、
`sequence_gaps=0`、`tick_delta_ms≈3000`、`bad_crc=0`、`bad_header=0`、
`state=0`（STOP）、`fault=0x0`。

排查：
- `frames=0` → CDC 收路径问题（查 `usbd_cdc_if.c` 的 `service_usb_rx` 接线）；
- `bad_crc>0` → 遥测组帧 CRC 范围不一致（S4a 契约 156B）；
- `sequence_gaps>0` → 发送缝 BUSY 保留逻辑异常或主机读取抖动。

## 3a. 一键命令旅程（usb_acceptance，替代第 4/5 节手动步骤）

`tools/usb_acceptance.c` 是协议级验收工具，与固件共用同一份
`axdr_command_core.c` 编译（零格式漂移），自动执行完整命令旅程并断言：

```
P0 遥测探针（帧率/CRC/跳号/故障位） → P1 协议查询（版本/档案/确认）
→ P2 ARM（错 challenge 拒 → 正确 ARM → 未授权断言）
→ P3 运动旅程（首上电对齐进 RUN → 3s 速度曲线 CSV → ControlStop）
→ P4 租约超时（零速续租停发 → 自动撤权 stop_reason=2）
→ P5 心跳超时（停发心跳 → 自动 DISARMED disarm_reason=3 → 失效会话拒绝）
→ P6 收尾（DISARM → STOP + fault=0）
```

工具已由 `run_host_tests.ps1` 编译到
`firmware/AxDr_App/build/host-tests/usb_acceptance.exe`（每次跑闸自动重编）。
上电后执行：

```powershell
& 'C:\Users\ready\Desktop\DriverLab\AxDr_L\firmware\AxDr_App\build\host-tests\usb_acceptance.exe' COMx
```

- 全部 PASS + exit=0 → 第 4/5 节的命令链与负路径验收**自动完成**；
- 输出 CSV（默认 `usb_motion.csv`）保留 3s 速度曲线供记录；
- 首上电：P3 会报告 `STARTING 对齐后进 RUN`（约 1.6s，属预期）；
- 电机实际动作：P3 升速 8 rad/s 约 3s → 停；P4/P5 仅零速闭环/静置，无运动。

断电重启后的免对齐复查（第 6 节核心场景）：

```powershell
& 'C:\...\usb_acceptance.exe' COMx --reboot-check
```

只跑探针 + ARM + 启动：`had_starting=0`（无 STARTING 段、约 0.8s 内进 RUN）
即免对齐生效。另可选 `--probe-only`（只跑 P0）、`--speed 5` / `--iq 1.5`
（改运动档位）。

第 4 节 GUI 旅程降级为**可选**（想人工看曲线时再走）；
第 5 节负路径全部由 P4/P5 自动覆盖，无需手动。
工具自身可信度：`usb_acceptance --selftest` 用内置模拟固件回环自检
（已入 run_host_tests.ps1 闸）。

## 4. 使能旅程（GUI 手动，可选——已由 3a 节工具覆盖）

启动 QtMotorTool GUI（不带参数），连接同一 COM 口后按序执行，
每步记录应答 `status`（APPLIED=0 / 拒绝码）：

- [ ] `GetProtocolInfo` → 版本与 opcode 掩码（15 位）符合契约；
- [ ] `GetMotorProfile` → id=1 / revision=2，CRC 非零；
- [ ] `ConfirmMotorProfile` → APPLIED（回发同一 CRC）；
- [ ] `SafetyArm`（challenge 错误）→ 拒 `CHALLENGE_MISMATCH`；
- [ ] `SafetyArm`（正确 challenge）→ APPLIED，进入会话；
- [ ] `SetSpeed`（未 ARM 状态补测，若在 ARM 前做）→ 拒 `INVALID_STATE`；
- [ ] `SetSpeed`（±314 rad/s 内、iq_limit 0<q≤8）→ APPLIED，电机升速；
- [ ] GUI 观察遥测速度曲线与指令一致性、`cmd_iq` 跟随负载；
- [ ] `ControlStop` → APPLIED，电机停，`stop_reason=HOST_REQUEST`；
- [ ] `SafetyDisarm` → APPLIED，会话退出。

首次使能会触发一次对齐：START 后约 1.2s 静止保持 + 0.3s 采样，
随后自动晋升 RUN（见第 6 节——这是第一次上电，之后不再有）。

## 5. 安全链负路径（逐项制造，确认 fail-closed）

- [ ] ARM 后不耐心跳：>100ms 后状态转 `LEASE/HEARTBEAT` 预警区，>2000ms
      自动 DISARM，遥测 `safety_state` 可见；
- [ ] SetSpeed 后停止发运动命令：租约 100–1000ms（按 lease 参数）超时
      → 自动撤权停机，`GetControlState` 的 `stop_reason=2`（LEASE_TIMEOUT）；
- [ ] 心跳超时后再 SetSpeed → 拒（会话已失效）；
- [ ] 重新 ARM 旅程可完整重入（session id 变化）。

## 6. 参数持久化验收（S5 核心场景）

- [ ] 第一次上电（或换档案 revision 后）：START 走 1.5s 对齐 → RUN；
      对齐完成瞬间零位已写入 Flash（Bank2 页 0x08040000）；
- [ ] 断电 → 重新上电 → START：**跳过对齐直接进 RUN**（电机无 1.2s
      定位抖动）——`service_param_store_loaded()==1` 生效；
- [ ] 遥测 `encoder_raw_angle_rad` 在静止时稳定（零位复用正确）；
- [ ] （可选）调试器读 0x08040000：magic `41 50 58 53`（"APXS"），
      offset 20 处 float 为上次对齐零位。

注意：换电机/机械重装后必须把 `User/config/motor_config.h` 的
`MOTOR_PROFILE_REVISION` 加一重编译烧录——旧零位按档案绑定自动作废，
重新走一次对齐。不做这一步旧零位会被继续使用（转矩角错误，电机
可能堵转/过流）。

## 7. 会话记录验收（遥测链路终检）

```powershell
& 'C:\Users\ready\Desktop\Axdr\AX_Driver_Lab\.tools\QtMotorTool-src\deploy_v2\QtMotorTool.exe' --usb-session-acceptance COMx C:\Temp\p7_session
```

预期 `USB_SESSION PASS`：两段各 1300ms 采集 + 中间端口重开，
`live_frames=replay_frames`、`sequence_gaps=0`、`bad_crc=0`、
`recorder_drops=0`，输出目录生成 `P0-01_Qt_live_session.csv`。

## 8. 验收记录表

（标 ※ 的行由 `usb_acceptance` 自动断言——工具 PASS 即整组通过，
实测栏填工具报告值即可）

| 项目 | 预期 | 实测 | 结论 |
|---|---|---|---|
| probe 遥测帧率 ※ | ≈20 Hz |  |  |
| probe CRC/头错误 ※ | 0 |  |  |
| probe seq gaps ※ | 0 |  |  |
| ConfirmProfile ※ | APPLIED |  |  |
| 错 challenge ARM ※ | 拒 11 |  |  |
| 正确 ARM ※ | APPLIED+ARMED |  |  |
| SetSpeed 运行 ※ | APPLIED+升速 |  |  |
| ControlStop ※ | 停+stop_reason=1 |  |  |
| 心跳超时自锁 ※ | 自动 DISARM(reason=3) |  |  |
| 租约超时停机 ※ | stop_reason=2 |  |  |
| 首上电对齐 ※ | ≈1.6s 后 RUN |  |  |
| 重上电免对齐 ※ | had_starting=0 直进 RUN |  |  |
| session acceptance | PASS |  |  |

## 9. 通过标准

`usb_acceptance COMx` 全 PASS + `usb_acceptance COMx --reboot-check`
报 `had_starting=0`，辅以第 6 节断电重启场景与第 7 节 session acceptance，
记录表无空白项 → 打标签：

```bash
git tag -a layer-service -m "P7 service 层实机验收通过" && git push origin layer-service
```

任何一步不符：记录现象 + 遥测 CSV，回 `DriverLab\AxDr_L` 按节定位，
修复后重跑该节与第 3 节（探针回归）。
