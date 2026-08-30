# 开发记录 33：Drive PWM 边界与 Fake Target 验证

## 1. 本节点解决什么

旧 `foc_drv.c` 同时负责 FOC 计算辅助、PWM 启停、相序映射和 Target 调用，算法与硬件执行边界不够清楚。

本节点新增 `drive_pwm` 模块，把职责整理为：

```text
FOC/模式代码：生成 dtc_a、dtc_b、dtc_c
Drive PWM：启停、50% 启动值、相序映射、命令与提交记录
Target PWM：把三个物理通道占空比写入 TIM1
```

## 2. 代码调整

新增：

- `User/drive/drive_pwm.h`
- `User/drive/drive_pwm.c`
- `tests/host/test_drive_pwm.c`

生产代码统一使用四个入口：

- `drive_pwm_start()`：启动三相主输出和互补输出；
- `drive_pwm_stop()`：停止三相主输出和互补输出；
- `drive_pwm_set_neutral()`：START 流程写入三相 50%；
- `drive_pwm_commit()`：按 ABC/ACB 相序提交 FOC 三相占空比。

旧 `foc_pwm_start/stop/duty_set/commit` 已删除，`motor` 目录不再直接包含或调用 `target_pwm`。

## 3. 为什么这样拆

- FOC 只关心“算出多少”，不用知道 TIM1 通道；
- Drive 关心“当前能不能输出、逻辑三相怎样映射”；
- Target 关心“怎样写当前 MCU 的寄存器”；
- 以后把 Target 换成 Host Fake、仿真或 HIL 接口时，不需要复制 FOC 公式。

这不是引入通用框架，只是把现有三段职责放回对应目录。

## 4. 保持不变的行为

- START 仍然先启动三相输出，再写 50% 占空比，再清控制器；
- STOP 仍然关闭三相主输出和互补输出；
- ABC 相序仍按 A、B、C 写入通道 1、2、3；
- ACB 相序仍按 A、C、B 写入通道 1、2、3；
- 相序无效时仍不写 PWM；
- FOC 公式、控制参数、快速周期顺序和 Target 寄存器写函数不变。

## 5. Fake Target 验证

Host 测试不连接 STM32 HAL，而是提供同名 Fake Target，记录 Drive 实际发出的调用和三个物理通道参数。已验证：

- PWM 启动和停止各调用一次正确接口；
- ABC 相序映射正确；
- ACB 相序交换 B、C 正确；
- 无效相序不写 Target，也不伪造新提交；
- START 的三通道 50% 命令和提交记录正确。

## 6. 全部门禁结果

- Host FOC、PID/PDFF、级联控制、调试快照、Drive 状态：通过；
- Host Drive PWM + Fake Target：通过；
- CMake/Keil 用户源文件：39/39，一致；
- Motor 到 Target PWM 的直接依赖：0 项；
- Debug：0 错误，25 警告；
- Release：0 错误，25 警告。

资源占用：

```text
Debug   text 183172, data 136, bss 10336
Release text 143948, data 136, bss 10344
```

`drive_pwm_*` 和 `target_pwm_set_duty_ratios` 在 Release ELF 中仍链接到 RAM 地址区域。硬件引脚实际更新拍数尚未测试，不在离线结果中提前下结论。
