#!/bin/bash
# =============================================================================
# 启动 M4 实时直播服务（在虚拟机里运行）
# -----------------------------------------------------------------------------
#   bash start_stream.sh          # 默认端口 8090
#   bash start_stream.sh 8091     # 换端口
#
# 起好之后，在**宿主机**浏览器打开：
#   http://192.168.183.128:8090/
# =============================================================================
set -u

PORT="${1:-8090}"
HERE="$(cd "$(dirname "$0")" && pwd)"

set +u
source /opt/ros/noetic/setup.bash
source /home/demo/桌面/ai4/catkin_ws/devel/setup.bash
set -u
export PYTHONUNBUFFERED=1

# 行首锚定，避免误杀调用它的 shell
pkill -f "^python3 ${HERE}/live_stream.py" 2>/dev/null
sleep 2

setsid nohup python3 "${HERE}/live_stream.py" "$PORT" \
    > /tmp/m4_live/stream.log 2>&1 < /dev/null &
sleep 6

echo "  监听端口 $PORT: $(ss -lnt 2>/dev/null | grep -c ":$PORT ")"
echo "  进程: $(pgrep -cf 'live_stream[.]py' || echo 0)"
echo "  日志:"
head -4 /tmp/m4_live/stream.log 2>/dev/null | sed 's/^/    /'
echo
echo "  本机自测:"
curl -s -m 5 "http://127.0.0.1:$PORT/status" 2>&1 | head -c 400 | sed 's/^/    /'
echo
