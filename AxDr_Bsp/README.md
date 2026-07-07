# AxDr_Bsp —— 电机控制底层 BSP（教学）

按 STM32CubeMX 教程顺序组织电机控制的底层（板级支持包）代码，
从建工程一路配到电流采样、PWM、编码器、CAN。

## 目录与学习顺序

| 目录 | 内容 | 对应教学 |
|------|------|---------|
| 01_project/        | CubeMX 建 Keil 工程、时钟树、系统配置 | #2 |
| 02_led/            | GPIO：点亮 LED | #3 |
| 03_timer/          | 定时器与中断 | #4 |
| 04_dac/            | DAC 模拟量调试输出 | #5 |
| 05_uart/           | 串口 + 示波器联调 | #6 #7 |
| 06_adc/            | ADC 基础采样 | #8 |
| 07_current_sense/  | 电流采样：运放/同步采样/offset 标定（FOC 核心） | 进阶 |
| 08_pwm/            | PWM：互补/死区/中心对齐（逆变桥驱动） | 进阶 |
| 09_encoder/        | 编码器：TIM 编码器模式 / SPI 磁编 | 进阶 |
| 10_can/            | CAN/FDCAN：关节间通信 | 进阶 |
| bsp_common/        | 公共：中断回调、错误处理、板级头 | — |

## 说明
- 当前只建了目录骨架（每个目录一个占位 README），代码后续逐步填。
- 建议每个模块放 `bsp_<name>.c / bsp_<name>.h`。
- 还没加 .gitignore：开始放 Keil 工程前，记得忽略编译产物（*.o *.axf *.hex *.map *.uvgu.* 等）。
