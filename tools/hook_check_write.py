#!/usr/bin/env python3
"""PostToolUse(Write|Edit) 扫描：对固件 .c/.h 做 04 号规范黑名单 + Tab 检查。
只报告不拦截；命中注入 additionalContext 提醒下一轮修正。
排除 vendor 目录（User/debug/segger）。"""

import io
import json
import re
import sys


def main():
    data = json.load(sys.stdin)
    path = (data.get("tool_input", {}).get("file_path")
            or data.get("tool_response", {}).get("filePath") or "")
    norm = path.replace("\\", "/")
    if (not norm.endswith(".c") and not norm.endswith(".h")) or "/segger/" in norm:
        sys.exit(0)
    if ("firmware/AxDr_App/User" not in norm
            and "firmware/AxDr_App/Core/Src" not in norm):
        sys.exit(0)

    try:
        lines = io.open(path, encoding="utf-8", errors="replace").read().splitlines()
    except OSError:
        sys.exit(0)

    bad = re.compile(
        r"不是|不能|不会|并不能|不负责|不访问|不写入|不读取|不依赖|不执行"
        r"|不接管|不授权|不重新|不自动|需要说明|值得注意的是|换句话说")
    ternary = re.compile(r"\?\s*[^:]+:")  # 三元表达式（04 规范 §3.2：少用三元）
    hits = []
    for i, line in enumerate(lines, 1):
        if bad.search(line):
            hits.append("%d: %s" % (i, line.strip()[:60]))
        elif ternary.search(line) and not line.lstrip().startswith(("//", "*")):
            hits.append("%d: [三元]" % i)
        elif "\t" in line:
            hits.append("%d: [TAB]" % i)
        if len(hits) >= 10:
            break

    if hits:
        print(json.dumps({
            "systemMessage": "规范扫描: %s 命中 %d 处" % (norm, len(hits)),
            "hookSpecificOutput": {
                "hookEventName": "PostToolUse",
                "additionalContext": "04 号规范扫描命中（%s）：\n%s\n"
                                     "请按 01/04 号规范修正后再交付。"
                                     % (norm, "\n".join(hits)),
            },
        }, ensure_ascii=False))
    sys.exit(0)


if __name__ == "__main__":
    main()
