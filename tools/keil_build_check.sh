#!/bin/bash
# Keil 命令行构建闸门——双清单问题的第二道验证（CMake 绿 ≠ Keil 绿）
# 用法: bash tools/keil_build_check.sh [UV4路径]
# 退出码 0 = 0 Error 0 Warning
set -u
UV4=${1:-"/d/Users/ready/AppData/Local/Keil_v5/UV4/UV4.exe"}
PROJ="$(cd "$(dirname "$0")/.." && pwd)/firmware/AxDr_App/MDK-ARM/AxDr.uvprojx"
LOG="$(dirname "$PROJ")/build_log.txt"
[ -f "$UV4" ] || { echo "UV4 未找到: $UV4"; exit 2; }
# 构建前生成唯一 build_id（贯穿固件、构建回执与资源包）
python "$(cd "$(dirname "$0")/.." && pwd)/tools/gen_build_id.py" AxDr || { echo "build_id 生成失败"; exit 2; }
# 显式指定 Release target：不依赖上次在 GUI 里激活的是哪个 target。
"$UV4" -r -t "AxDr" "$PROJ" -j0 -o "$LOG" >/dev/null 2>&1
sleep 1
# UV4 输出为 GBK，统计 Error/Warning 行
ERR=$(iconv -f GBK -t UTF-8 "$LOG" 2>/dev/null | grep -cE "Error\(s\)|error:" )
SUM=$(iconv -f GBK -t UTF-8 "$LOG" 2>/dev/null | grep -E "Error\(s\)" | tail -1)
echo "Keil 构建: $SUM"
iconv -f GBK -t UTF-8 "$LOG" 2>/dev/null | grep -E "error:|warning:" | head -5
echo "$SUM" | grep -q "0 Error" || exit 1
exit 0
