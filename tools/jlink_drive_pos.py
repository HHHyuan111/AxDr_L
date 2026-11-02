#!/usr/bin/env python3
"""J-Link 自动驱动位置环验证（两阶段）。

阶段 1：烧录 → 初始化 → START 对齐 → 读对齐后转子位置。
阶段 2：目标 = 对齐后位置 + DELTA → 位置模式发布 → 验证到位 → STOP。

所有内存地址每次运行从 AxDr.map 重新解析 g_foc，并用 -fshort-enums 探针
校验结构布局与 map 一致（Keil AC6 短枚举布局，见 04 规范附录）。
电机会转动：运行前确保台架安全。
用法: python tools/jlink_drive_pos.py [DELTA_RAD，默认 0.5]
"""

import io
import re
import struct
import subprocess
import sys
import tempfile
import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP = os.path.join(REPO, "firmware", "AxDr_App", "MDK-ARM", "AxDr", "AxDr.map")
BIN = os.path.join(REPO, "firmware", "AxDr_App", "MDK-ARM", "AxDr", "AxDr.bin")
JLINK = "D:/Users/ready/AppData/Local/Keil_v5/ARM/Segger/JLink.exe"
FROMELF = "D:/Users/ready/AppData/Local/Keil_v5/ARM/ARMCLANG/bin/fromelf.exe"
GCC = ("C:/Users/ready/Desktop/Axdr/AX_Driver_Lab/.tools/Qt/Tools/"
       "mingw1310_64/bin/gcc.exe")
USER = os.path.join(REPO, "firmware", "AxDr_App", "User")
INC = ["-I%s" % d for d in (
    os.path.join(REPO, "tests/host/fakes"), USER + "/common", USER + "/control",
    USER + "/algorithm", USER + "/motor", USER + "/drive", USER + "/adapter",
    USER + "/bsp/inc", USER + "/config")]

FIELDS = [
    "mode.sys", "mode.debug", "req", "state", "pwm_active", "fault", "fast_seq",
    "ref.vd", "ref.vq", "ref.id", "ref.iq", "ref.spd_r", "ref.pos_r",
    "fb.pos_r", "fb.spd_r", "fb.theta_e", "enc_aligned",
    "pos_pi", "pos_pi.ref_value", "pos_pi.fback_value", "pos_pi.out_value",
]


def float_bits(value):
    return struct.unpack("<I", struct.pack("<f", value))[0]


def bits_float(bits):
    return struct.unpack("<f", struct.pack("<I", bits))[0]


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def get_symbol():
    text = io.open(MAP, encoding="utf-8", errors="replace").read()
    m = re.search(r"g_foc\s+(0x[0-9A-Fa-f]+)\s+Data\s+(\d+)", text)
    if not m:
        sys.exit("map 里找不到 g_foc 符号")
    return int(m.group(1), 16), int(m.group(2))


PROBE_TEMPLATE = r"""
#include <stdio.h>
#include <stddef.h>
#include "common.h"
#define P(name, expr) printf(#name " %u\n", (unsigned)(expr))
int main(void)
{
    P(SIZE, sizeof(foc_t));
    P(mode.sys, offsetof(foc_t, mode) + offsetof(mode_ctrl_e, sys));
    P(mode.debug, offsetof(foc_t, mode) + offsetof(mode_ctrl_e, debug));
    P(req, offsetof(foc_t, req));
    P(state, offsetof(foc_t, state));
    P(pwm_active, offsetof(foc_t, pwm_active));
    P(fault, offsetof(foc_t, fault));
    P(fast_seq, offsetof(foc_t, fast_seq));
    P(ref.vd, offsetof(foc_t, ref) + offsetof(foc_ref_state_t, vd));
    P(ref.vq, offsetof(foc_t, ref) + offsetof(foc_ref_state_t, vq));
    P(ref.id, offsetof(foc_t, ref) + offsetof(foc_ref_state_t, id));
    P(ref.iq, offsetof(foc_t, ref) + offsetof(foc_ref_state_t, iq));
    P(ref.spd_r, offsetof(foc_t, ref) + offsetof(foc_ref_state_t, spd_r));
    P(ref.pos_r, offsetof(foc_t, ref) + offsetof(foc_ref_state_t, pos_r));
    P(fb.pos_r, offsetof(foc_t, fb) + offsetof(foc_fb_state_t, pos_r));
    P(fb.spd_r, offsetof(foc_t, fb) + offsetof(foc_fb_state_t, spd_r));
    P(fb.theta_e, offsetof(foc_t, fb) + offsetof(foc_fb_state_t, theta_e));
    P(enc_aligned, offsetof(foc_t, enc_aligned));
    P(pos_pi, offsetof(foc_t, pos_pi));
    P(pos_pi.ref_value, offsetof(foc_t, pos_pi) + offsetof(pid_para_t, ref_value));
    P(pos_pi.fback_value, offsetof(foc_t, pos_pi) + offsetof(pid_para_t, fback_value));
    P(pos_pi.out_value, offsetof(foc_t, pos_pi) + offsetof(pid_para_t, out_value));
    return 0;
}
"""


def get_offsets(size):
    with tempfile.NamedTemporaryFile("w", suffix=".c", delete=False,
                                     encoding="utf-8") as fh:
        fh.write(PROBE_TEMPLATE)
        probe = fh.name
    exe = probe + ".exe"
    r = run([GCC, "-std=c11", "-fshort-enums"] + INC + [probe, "-o", exe])
    if r.returncode != 0:
        sys.exit("探针编译失败:\n" + r.stderr)
    r = run([exe])
    off = {}
    for line in r.stdout.strip().splitlines():
        name, value = line.split()
        off[name] = int(value)
    if off["SIZE"] != size:
        sys.exit("布局不一致：探针 %d != map %d" % (off["SIZE"], size))
    return off


def jlink(script_path):
    r = run([JLINK, "-CommanderScript", script_path, "-ExitOnError", "0"])
    reads = []
    for line in r.stdout.splitlines():
        if line.startswith("2000") and " = " in line:
            reads.append(line.strip())
    return reads


def stage(base, off):
    addr = {f: base + off[f] for f in FIELDS}

    # 从当前 axf 重新生成 bin：避免"烧了旧镜像、按新 map 读地址"的错位。
    axf = BIN.replace(".bin", ".axf")
    r = run([FROMELF, "--bin", "--output", BIN, axf])
    if r.returncode != 0:
        sys.exit("fromelf 生成 bin 失败:\n" + r.stderr)

    s1 = os.path.join(REPO, "artifacts", "jlink_pos_stage1.jlink")
    io.open(s1, "w", newline="\n").write("\n".join([
        "si SWD", "speed 4000", "device STM32G474RETx", "connect", "h",
        "loadfile %s 0x08000000" % BIN.replace("/", "\\"), "r", "go",
        "sleep 4500", "h",
        "mem32 0x%08X 1" % addr["fb.pos_r"],
        "w1 0x%08X 1" % addr["req"], "go", "sleep 4000", "h",
        "mem8 0x%08X 1" % addr["state"],
        "mem8 0x%08X 1" % addr["enc_aligned"],
        "mem32 0x%08X 1" % addr["fault"],
        "mem32 0x%08X 1" % addr["fb.pos_r"],
        "q", ""]))
    out1 = jlink(s1)
    pos_after = bits_float(int(out1[-1].split("= ")[1], 16))
    aligned = int(out1[-3].split("= ")[1], 16)
    state1 = int(out1[-4].split("= ")[1], 16)
    if aligned != 1:
        sys.exit("对齐未成功：state=%d enc_aligned=%d 读数=%s"
                 % (state1, aligned, out1))
    target = pos_after + float(sys.argv[1] if len(sys.argv) > 1 else "0.5")

    s2 = os.path.join(REPO, "artifacts", "jlink_pos_stage2.jlink")
    io.open(s2, "w", newline="\n").write("\n".join([
        "si SWD", "speed 4000", "device STM32G474RETx", "connect", "h",
        "w1 0x%08X 0" % addr["mode.sys"],
        "w1 0x%08X 5" % addr["mode.debug"],
        "w4 0x%08X 0x%08X" % (addr["ref.pos_r"], float_bits(target)),
        "w4 0x%08X 0x40A00000" % addr["ref.spd_r"],
        "w4 0x%08X 0x40000000" % addr["ref.iq"],
        "w4 0x%08X 0" % addr["ref.id"],
        "w1 0x%08X 1" % addr["req"],
        "go", "sleep 2500", "h",
        "mem8 0x%08X 1" % addr["state"],
        "mem32 0x%08X 1" % addr["fault"],
        "mem32 0x%08X 1" % addr["fb.pos_r"],
        "mem32 0x%08X 1" % addr["pos_pi.ref_value"],
        "w1 0x%08X 0" % addr["req"],
        "go", "sleep 1000", "h",
        "mem8 0x%08X 1" % addr["pwm_active"],
        "q", ""]))
    out2 = jlink(s2)
    final = bits_float(int(out2[2].split("= ")[1], 16))
    ref = bits_float(int(out2[3].split("= ")[1], 16))
    state2 = int(out2[0].split("= ")[1], 16)
    pwm = int(out2[4].split("= ")[1], 16)

    print("对齐后位置 %.4f rad → 目标 %.4f rad" % (pos_after, target))
    print("阶段2：state=%d 到位=%.4f（误差 %.4f rad） pos_pi.ref=%.4f"
          % (state2, final, final - target, ref))
    print("停机：pwm_active=%d" % pwm)
    ok = (state2 == 2) and (abs(final - target) < 0.05) and (pwm == 0)
    print("位置环验证：%s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    base, size = get_symbol()
    off = get_offsets(size)
    print("g_foc = 0x%08X（map 动态解析）" % base)
    sys.exit(stage(base, off))
