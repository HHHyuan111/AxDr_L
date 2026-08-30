# 开发记录 35：Drive 状态机与模式分派分离

## 1. 本节点解决什么

原 `drive.c` 同时包含两类变化节奏不同的代码：

- START、RUN、STOP、FAULT 的状态与功率动作；
- MIT、轮廓模式、调试闭环、标定和辨识的具体分派。

继续把新算法加在同一个文件里，会让核心状态机越来越难读。因此本节点新增 `drive_mode.c/.h`，只做现有模式分派；`drive.c` 只保留最小状态闭环。

## 2. 当前职责

```text
drive.c
    -> 处理 STOP / START / RUN 请求
    -> 启停 PWM
    -> 汇总故障并更新 Drive 状态
    -> RUN 时调用 drive_mode_step()

drive_mode.c
    -> release：MIT、PT/PV/PP、CST/CSV/CSP
    -> halt：快速停车、故障停车
    -> calibrat：编码器标定、参数辨识、齿槽转矩标定
    -> debug：V/f、I/f、电压/电流/速度/位置闭环
```

未实现的输出编码器模式、速度电压环和位置速度电压环继续保持空操作，没有用占位代码伪造功能。

## 3. Host 验证

新增 `tests/host/test_drive_mode.c`，直接编译生产 `drive_mode.c`，验证：

- 7 个 release 子模式分别进入原有实现；
- 2 个 halt 子模式分别进入原有实现；
- 已实现的标定与辨识分支映射正确；
- 已实现的 6 个 debug 子模式映射正确；
- FOC 返回有效结果时才提交 PWM；
- FOC 返回无效结果时不提交 PWM；
- 当前未实现的模式保持空操作。

原 Drive 状态测试改为只提供一个 Fake `drive_mode_step()`，因此状态机测试不再依赖所有具体算法函数。

## 4. 保持不变的行为

- 所有模式枚举值不变；
- 所有模式到原函数的映射不变；
- START/RUN/STOP/FAULT 顺序不变；
- FOC 有效性判断和 PWM 提交位置不变；
- 控制参数、公式和硬件调用不变。

## 5. 验证结果

- 全部八组 Host 测试：通过；
- CMake/Keil 用户源文件：40/40，一致；
- Debug：0 错误，25 警告；
- Release：0 错误，25 警告。

资源占用：

```text
Debug   text 183188, data 136, bss 10336
Release text 143812, data 136, bss 10344
```

现在新增算法模式时，先在独立算法模块实现，再只在 `drive_mode.c` 增加一条明确映射，不需要修改核心状态机。
