#!/usr/bin/env python3
"""生成 axdr_build_id.h：每次构建唯一的 build_id，贯穿固件、构建回执与资源包。

用法: python tools/gen_build_id.py <target_name>   （AxDr | AxDr-Debug）
build_id = SHA256(git HEAD + Unix 秒 + 本地构建计数器) 的前 4 字节。
版本号 0.7.9 也在此单一来源维护（service_command 与 service_scope 共用）。
"""

import hashlib
import io
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COUNTER = os.path.join(REPO, "tools", ".build_counter")
OUT = os.path.join(REPO, "firmware", "AxDr_App", "User", "service",
                   "axdr_build_id.h")

FW_MAJOR = 0
FW_MINOR = 7
FW_PATCH = 9


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else "AxDr"
    if target not in ("AxDr", "AxDr-Debug"):
        sys.exit("target 必须是 AxDr 或 AxDr-Debug")

    git = subprocess.run(["git", "-C", REPO, "rev-parse", "HEAD"],
                         capture_output=True, text=True).stdout.strip()
    if not git:
        git = "nogit"

    counter = 0
    try:
        with open(COUNTER, encoding="utf-8") as f:
            counter = int(f.read().strip())
    except (OSError, ValueError):
        pass
    counter += 1
    with open(COUNTER, "w", encoding="utf-8") as f:
        f.write(str(counter))

    seed = "%s|%d|%d" % (git, int(time.time()), counter)
    build_id = int.from_bytes(hashlib.sha256(seed.encode()).digest()[:4], "little")

    header = (
        "/* 构建脚本生成，勿手改：tools/gen_build_id.py */\n"
        "#ifndef AXDR_BUILD_ID_H\n"
        "#define AXDR_BUILD_ID_H\n"
        "\n"
        "#define AXDR_FW_MAJOR     %uu\n" % FW_MAJOR +
        "#define AXDR_FW_MINOR     %uu\n" % FW_MINOR +
        "#define AXDR_FW_PATCH     %uu\n" % FW_PATCH +
        "#define AXDR_PROJECT_ID   0x00000001u\n"
        "#define AXDR_TARGET_ID    %du\n" % (0 if target == "AxDr" else 1) +
        "#define AXDR_TARGET_NAME  \"%s\"\n" % target +
        "#define AXDR_BUILD_ID     0x%08Xu\n" % build_id +
        "\n"
        "#endif /* AXDR_BUILD_ID_H */\n"
    )
    with io.open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write(header)
    print("build_id 0x%08X target=%s -> %s" % (build_id, target, OUT))


if __name__ == "__main__":
    main()
