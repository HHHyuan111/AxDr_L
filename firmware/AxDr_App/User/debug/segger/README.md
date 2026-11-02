# SEGGER RTT 源文件

来源：SEGGER 官方镜像 https://github.com/SEGGERMicro/RTT （RTT/ 目录，2026-11-01 抓取）。
用途：J-Link 实时日志通道（本项目持有 J-Link Pro，符合 SEGGER 许可条件）。
纪律：vendor 文件不改动；配置宏用 SEGGER_RTT_ConfDefaults.h 默认值（上行 1024B / 下行 16B）。
     printf 变体（SEGGER_RTT_printf.c）未引入——日志格式由 rtt_log 薄封装自实现。
