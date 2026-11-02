#!/bin/bash
# 04 号规范审计（Stop 钩子）：跑在每次交付停顿时，命中注入下一轮上下文。
# 检查：分支命名 / 改动行黑名单与 Tab / 行宽 / 新增 .c 双登记 / 生成文件手改。
cd "$(dirname "$0")/.." || exit 0
OUT=""

# 1) 分支命名（存量 codex/ 分支经用户确认，不自动改）
branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
case "$branch" in
  codex/*) OUT="$OUT [分支] $branch 使用 codex/ 前缀（04 规范要求 feat//fix/；存量分支待用户确认）";;
esac

# 2) 未提交改动行的黑名单与 Tab（仅固件，排除 vendor）
DIFF_FILES=$(git diff HEAD --name-only 2>/dev/null | grep -E '^firmware/AxDr_App/(User|Core/Src)/.*\.(c|h)$' | grep -v segger)
for f in $DIFF_FILES; do
  [ -f "$f" ] || continue
  hits=$(git diff HEAD -U0 -- "$f" 2>/dev/null | grep -E '^\+' | grep -vE '^\+\+\+' \
    | grep -nE '不是|不能|不会|并不能|不负责|不访问|不写入|不读取|不依赖|不执行|不接管|不授权|不重新|不自动|需要说明|值得注意的是|换句话说' | head -3)
  tabs=$(git diff HEAD -U0 -- "$f" 2>/dev/null | grep -E '^\+' | grep -vE '^\+\+\+' | grep -n "$(printf '\t')" | head -3)
  [ -n "$hits" ] && OUT="$OUT [黑名单] $f: $(echo "$hits" | tr '\n' ' ')"
  [ -n "$tabs" ] && OUT="$OUT [TAB] $f"
  terns=$(git diff HEAD -U0 -- "$f" 2>/dev/null | grep -E '^\+' | grep -vE '^\+\+\+' \
    | grep -nE '\? .* :' | grep -vE '^\S+:.*\*|//' | head -3)
  [ -n "$terns" ] && OUT="$OUT [三元] $f: $(echo "$terns" | tr '\n' ' ')"
done

# 3) 新增 .c 双登记（Keil 两 target + CMake）
NEW_C=$(git ls-files --others --exclude-standard 2>/dev/null | grep -E '^firmware/AxDr_App/User/.*\.c$')
for f in $NEW_C; do
  base=$(basename "$f")
  keil_cnt=$(grep -c "$base" firmware/AxDr_App/MDK-ARM/AxDr.uvprojx 2>/dev/null || echo 0)
  cmake_cnt=$(grep -c "$base" firmware/AxDr_App/CMakeLists.txt 2>/dev/null || echo 0)
  if [ "$keil_cnt" -lt 2 ] || [ "$cmake_cnt" -lt 1 ]; then
    OUT="$OUT [登记] $f 未双登记（Keil=$keil_cnt CMake=$cmake_cnt）"
  fi
done

# 4) 生成文件手改检测
if git diff HEAD --name-only 2>/dev/null | grep -q "axdr_build_id.h"; then
  OUT="$OUT [生成文件] axdr_build_id.h 被手改（只应由 gen_build_id.py 生成）"
fi

if [ -n "$OUT" ]; then
  printf '{"hookSpecificOutput":{"hookEventName":"Stop","additionalContext":"04 号规范审计命中：%s"}}' "$OUT"
fi
exit 0
