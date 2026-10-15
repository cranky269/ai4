#!/bin/bash
# =============================================================================
# 单独更换 camera_node 的数据源（其他三个节点不动）—— 手工测试用
# -----------------------------------------------------------------------------
#   bash set_camera_source.sh image  /路径/你的图.jpg      # 换成指定图片
#   bash set_camera_source.sh video  /路径/视频.mp4        # 换成指定视频
#   bash set_camera_source.sh default                       # 还原成演示默认（动态视频）
#
# 典型用途：
#   * 验证「换一张自己的图，检测结果会不会跟着变」
#   * 想看某个特定画面的检测效果
#
# ★ 为什么要写成脚本文件，而不是让你在命令行里拼 ★
#   如果用 `pkill -f camera_node.py` 之后再 `rosrun ... camera_node.py`，
#   **同一行命令里出现了同一个字符串**，pkill 会把执行它的那个 shell 一起杀掉，
#   后面的 rosrun 根本不会执行（实测踩到过，现象是"命令没反应"）。
#   放进脚本文件后，进程名不出现在调用者的命令行里，就不会误杀。
# =============================================================================
set -u

MODE="${1:-default}"
ARG="${2:-}"

HERE="$(cd "$(dirname "$0")" && pwd -P)"
ASSETS="$HERE/assets"
LOG="/tmp/m4_live"

set +u
source /opt/ros/noetic/setup.bash
source /home/demo/桌面/ai4/catkin_ws/devel/setup.bash
set -u
export PYTHONUNBUFFERED=1

if ! rostopic list > /dev/null 2>&1; then
  echo "!! roscore 不在，先跑: bash $HERE/demo.sh"
  exit 1
fi

case "$MODE" in
  image)
    [ -f "$ARG" ] || { echo "!! 文件不存在: $ARG"; exit 1; }
    SRC_ARGS=(_source:=image _image_path:="$ARG" _fps:=5)
    DESC="图片 $ARG"
    ;;
  video)
    [ -f "$ARG" ] || { echo "!! 文件不存在: $ARG"; exit 1; }
    SRC_ARGS=(_source:=video _video_path:="$ARG" _loop:=true _fps:=10)
    DESC="视频 $ARG"
    ;;
  default)
    SRC_ARGS=(_source:=video _video_path:="$ASSETS/bus_moving.mp4" _loop:=true _fps:=10)
    DESC="默认动态视频 bus_moving.mp4"
    ;;
  *)
    echo "用法: bash $0 image <图片> | video <视频> | default"
    exit 1
    ;;
esac

echo "  停掉旧 camera_node ..."
pkill -f 'camera_node[.]py' 2>/dev/null
sleep 3

echo "  启动新 camera_node: $DESC"
setsid nohup rosrun uav_tracking camera_node.py "${SRC_ARGS[@]}" \
    > "$LOG/camera.log" 2>&1 < /dev/null &
sleep 7

if grep -q "ready" "$LOG/camera.log" 2>/dev/null; then
  grep -m1 "ready" "$LOG/camera.log" | sed 's/^/  /'
else
  echo "  !! 没起来，日志尾部："
  tail -6 "$LOG/camera.log" | sed 's/^/    /'
  exit 1
fi

echo
echo "  当前 /camera/image_raw 的规格（应跟着新源变）："
timeout 12 rostopic echo -n1 --noarr /camera/image_raw 2>/dev/null \
  | grep -E "width|height|encoding" | head -3 | sed 's/^/    /'
