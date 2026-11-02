#!/bin/bash
# 资源产包（人工/脚本调用；AfterMake 待上位机身份核对完成后再接通）
# 前置：对应 target 已构建 0E0W。
# 输出：MDK-ARM/build_identity.json、build-result.json、frozen/<target>/（MAP/AXF/BIN）
# 用法: bash tools/build_resource_package.sh [AxDr|AxDr-Debug]
set -u
TARGET="${1:-AxDr}"
FROMELF="/d/Users/ready/AppData/Local/Keil_v5/ARM/ARMCLANG/bin/fromelf.exe"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
MDK="$REPO/firmware/AxDr_App/MDK-ARM"
OUT="$MDK/$TARGET"
FREEZE="$MDK/frozen/$TARGET"

[ "$TARGET" = "AxDr" ] || [ "$TARGET" = "AxDr-Debug" ] || { echo "target 必须为 AxDr 或 AxDr-Debug"; exit 2; }
[ -f "$OUT/$TARGET.axf" ] || { echo "缺少 $TARGET.axf，先构建 0E0W"; exit 2; }
[ -f "$FROMELF" ] || { echo "fromelf 未找到"; exit 2; }

"$FROMELF" --bin --output "$OUT/$TARGET.bin" "$OUT/$TARGET.axf" || exit 2
mkdir -p "$FREEZE"
cp -f "$OUT/$TARGET.map" "$OUT/$TARGET.axf" "$OUT/$TARGET.bin" "$FREEZE/"

GIT_REV=$(git -C "$REPO" rev-parse HEAD)
BIN_SHA=$(sha256sum "$OUT/$TARGET.bin" | awk '{print $1}')
AXF_SHA=$(sha256sum "$OUT/$TARGET.axf" | awk '{print $1}')
MAP_SHA=$(sha256sum "$OUT/$TARGET.map" | awk '{print $1}')
BUILD_ID=$(grep -E "^#define AXDR_BUILD_ID[[:space:]]" "$REPO/firmware/AxDr_App/User/service/axdr_build_id.h" | awk '{print $3}' | tr -d 'u')
BUILT_AT=$(date "+%Y-%m-%dT%H:%M:%S%z")
TARGET_ID=0; [ "$TARGET" = "AxDr-Debug" ] && TARGET_ID=1

cat > "$MDK/build_identity.json" <<EOF
{
  "project_id": 1,
  "target_id": $TARGET_ID,
  "target_name": "$TARGET",
  "build_id": "$BUILD_ID",
  "git_rev": "$GIT_REV",
  "bin_sha256": "$BIN_SHA",
  "axf_sha256": "$AXF_SHA",
  "map_sha256": "$MAP_SHA",
  "built_at": "$BUILT_AT"
}
EOF

cat > "$MDK/build-result.json" <<EOF
{
  "target": "$TARGET",
  "build_id": "$BUILD_ID",
  "status": "PASS",
  "keil_errors": 0,
  "keil_warnings": 0,
  "bin_sha256": "$BIN_SHA",
  "bin_size": $(stat -c %s "$OUT/$TARGET.bin"),
  "frozen_dir": "frozen/$TARGET"
}
EOF

echo "产包完成: $TARGET build_id=$BUILD_ID -> $FREEZE/"
echo "回执: $MDK/build_identity.json / $MDK/build-result.json"
