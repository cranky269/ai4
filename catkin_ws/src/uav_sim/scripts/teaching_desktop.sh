#!/usr/bin/env bash
# 保留终端中的绝对数据路径与错误信息，便于桌面用户阅读。
AI4_DEMO_SCRIPTS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
bash "$AI4_DEMO_SCRIPTS/start_teaching_demo.sh" start
AI4_DEMO_EXIT=$?
printf '\n演示入口退出码：%s。数据路径和收尾结果见上方输出。\n' "$AI4_DEMO_EXIT"
if [[ -t 0 ]]; then read -r -p '按回车关闭此终端……' AI4_DEMO_REPLY; fi
exit "$AI4_DEMO_EXIT"
