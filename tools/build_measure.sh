#!/bin/bash
# AxDr 固件资源测量脚本 —— 编译 dev 分支固件并输出资源占用数据
# 用法: bash build_measure.sh <AxDr_App工程根目录> [输出json路径]
# 依赖: arm-none-eabi-gcc 10.3+ / make(可选) / python3
# 说明: 不依赖 cmake，直接从 CMakeLists 抽取源码清单手工编译
set -u
ROOT=${1:?用法: build_measure.sh <AxDr_App根目录>}
OUTJSON=${2:-$ROOT/build/rpt/resources.json}
B=$ROOT/build/rpt
CC=arm-none-eabi-gcc
FLAGS="-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Os -g -gdwarf-4 -Wall -Wextra -fdata-sections -ffunction-sections -fstack-usage"
DEFS="-DUSE_HAL_DRIVER -DSTM32G474xx"
INC="-I$ROOT/User/adapter -I$ROOT/User/app -I$ROOT/User/bsp/inc -I$ROOT/User/common -I$ROOT/User/config -I$ROOT/User/control -I$ROOT/User/diagnostic/include -I$ROOT/User/drive -I$ROOT/User/motor -I$ROOT/User/moldue/inc -I$ROOT/Core/Inc -I$ROOT/USB_Device/App -I$ROOT/USB_Device/Target -I$ROOT/Drivers/STM32G4xx_HAL_Driver/Inc -I$ROOT/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I$ROOT/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I$ROOT/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I$ROOT/Drivers/CMSIS/Device/ST/STM32G4xx/Include -I$ROOT/Drivers/CMSIS/Include"

rm -rf "$B"; mkdir -p "$B/obj"
cd "$ROOT" || exit 1
SRCS=$( (grep -oE '\.\./\.\./[A-Za-z0-9_/]+\.(c|s)' cmake/stm32cubemx/CMakeLists.txt | sed 's|^\.\./\.\./|./|'; grep -oE 'User/[A-Za-z0-9_/]+\.c' CMakeLists.txt) | sort -u )
N=0; FAIL=0
for s in $SRCS; do
  o="$B/obj/$(echo "$s" | sed 's|^\./||; s|/|_|g; s|\.c$|.o|; s|\.s$|.o|')"
  if [[ "$s" == *.s ]]; then
    $CC $FLAGS $DEFS $INC -x assembler-with-cpp -c "$s" -o "$o" 2>"$o.err" || { echo "FAIL: $s"; FAIL=$((FAIL+1)); }
  else
    $CC $FLAGS $DEFS $INC -c "$s" -o "$o" 2>"$o.err" || { echo "FAIL: $s"; FAIL=$((FAIL+1)); }
  fi
  N=$((N+1))
done
echo "compiled=$N failed=$FAIL"
[ $FAIL -gt 0 ] && exit 1

arm-none-eabi-gcc -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
  $(find "$B/obj" -name "*.o" | tr '\n' ' ') \
  -T stm32g474retx_flash.ld --specs=nano.specs -Wl,--gc-sections -Wl,-Map="$B/AxDr.map" \
  -o "$B/AxDr.elf" && echo "LINK OK" || exit 1

PY=""
for c in python3 python; do
  if command -v $c >/dev/null 2>&1 && $c -c "import sys" >/dev/null 2>&1; then PY=$c; break; fi
done
[ -n "$PY" ] || { echo "ERROR: 找不到可用的 python 解释器"; exit 1; }
$PY - "$B" "$OUTJSON" <<'PYEOF'
import sys, os, glob, subprocess, json, re
B, out = sys.argv[1], sys.argv[2]
elf = os.path.join(B, "AxDr.elf")
def nm():
    r = subprocess.run(["arm-none-eabi-nm","-S","--size-sort",elf],capture_output=True,text=True).stdout
    syms=[]
    for line in r.splitlines():
        p=line.split()
        if len(p)==4:
            syms.append((p[0], int(p[1],16), p[2], p[3]))
    return syms
syms=nm()
sec = {}
r = subprocess.run(["arm-none-eabi-size","-A",elf],capture_output=True,text=True).stdout
for line in r.splitlines():
    p=line.split()
    if len(p)==3 and p[0].startswith("."):
        sec[p[0][1:]] = int(p[1])
ram_top  = sorted([s for s in syms if s[0].startswith("2000") and s[2] in "bBdD"], key=lambda x:-x[1])[:20]
code_top = sorted([s for s in syms if s[0].startswith("0800") and s[2] in "tT"], key=lambda x:-x[1])[:15]
ro_top   = sorted([s for s in syms if s[0].startswith("0800") and s[2] in "rR"], key=lambda x:-x[1])[:10]
ramfunc  = sorted([s for s in syms if s[0].startswith("2000") and s[2] in "tT"], key=lambda x:-x[1])[:15]
su=[]
for f in glob.glob(os.path.join(B,"obj","*.su")):
    for line in open(f, encoding="utf-8", errors="ignore"):
        p=line.split("\t")
        if len(p)>=2:
            try: su.append((int(p[0]), p[1].strip()))
            except ValueError: pass
su.sort(reverse=True)
data={
 "meta":{"build":"gcc10.3 -Os","elf":elf},
 "sections":sec,
 "ram_top":[{"addr":a,"size":s,"type":t,"name":n} for a,s,t,n in ram_top],
 "code_top":[{"addr":a,"size":s,"type":t,"name":n} for a,s,t,n in code_top],
 "rodata_top":[{"addr":a,"size":s,"type":t,"name":n} for a,s,t,n in ro_top],
 "ramfunc_top":[{"addr":a,"size":s,"type":t,"name":n} for a,s,t,n in ramfunc],
 "stack_top":[{"bytes":b,"fn":f} for b,f in su[:20]],
}
json.dump(data, open(out,"w",encoding="utf-8"), ensure_ascii=False, indent=1)
print("JSON ->", out)
print("flash_text=%d rodata=%d | ram_data=%d bss=%d" % (sec.get("text",0), sec.get("rodata",0), sec.get("data",0), sec.get("bss",0)))
PYEOF
