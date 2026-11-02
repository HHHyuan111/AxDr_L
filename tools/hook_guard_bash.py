#!/usr/bin/env python3
"""PreToolUse(Bash) 拦截：危险动作硬拒绝（04 号规范第 8/9 条）。
拦截项：J-Link loadfile 烧录、git 强制推送、taskkill。
解除：用户明确授权（对话中要求，或经确认后在 settings.local.json 放行）。"""

import json
import re
import sys

RULES = [
    (r"\bloadfile\b.*0x08", "J-Link loadfile 烧录"),
    (r"git\s+push\b.*--force", "git 强制推送"),
    (r"\btaskkill\b", "结束进程"),
]


def main():
    data = json.load(sys.stdin)
    cmd = data.get("tool_input", {}).get("command", "") or ""
    for pattern, what in RULES:
        if re.search(pattern, cmd, re.I | re.S):
            print(json.dumps({
                "systemMessage": "已拦截危险动作：%s" % what,
                "hookSpecificOutput": {
                    "hookEventName": "PreToolUse",
                    "permissionDecision": "deny",
                    "permissionDecisionReason":
                        "04 号规范第 8/9 条：%s 必须由用户明确授权，"
                        "不得自行触发。若用户已授权，请让其确认后"
                        "在 settings.local.json 放行该命令。" % what,
                },
            }, ensure_ascii=False))
            sys.exit(2)
    sys.exit(0)


if __name__ == "__main__":
    main()
