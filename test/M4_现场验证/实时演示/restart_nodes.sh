#!/bin/bash
# =============================================================================
# 只重启 M4 的四个节点（**不动 roscore**）
# -----------------------------------------------------------------------------
# 用途：直播演示时，让你在浏览器里看到整条链一步步起来。
#   保留 roscore  → 直播服务（live_stream.py）不会掉线
#   只换 M4 节点  → 页面上能看到「画面停住 → 逐级 ready → VISUAL_LOCK」
#
# 运行：bash restart_nodes.sh
# =============================================================================
set -u

HERE="$(cd "$(dirname "$0")" && pwd -P)"   # -P 解析软链
ASSETS="$HERE/assets"                      # 素材在工程内，不受 /tmp 清空影响
WS="/home/demo/桌面/ai4/catkin_ws"
LOG="/tmp/m4_live"
MODEL="$ASSETS/yolov8n.pt"
PIC="$ASSETS/bus.jpg"

set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u
export PYTHONUNBUFFERED=1

if ! rostopic list >/dev/null 2>&1; then
  echo "!! roscore 不在，先跑 m4_live.sh"
  exit 1
fi

echo "[0] 停掉旧的 M4 节点（roscore 与直播服务保持不动）..."
pkill -f "uav_tracking/scripts" 2>/dev/null
sleep 3
echo "    已停，画面会先停住"
echo

echo "[1] camera_node（真图 5 FPS）..."
rosrun uav_tracking camera_node.py \
    _source:=image _image_path:="$PIC" _fps:=5 > "$LOG/camera.log" 2>&1 &
sleep 5
grep -h "ready" "$LOG/camera.log" | sed 's/^/    /'

echo "[2] detector_node（真模型，CPU 预热约 25 秒）..."
rosrun uav_tracking detector_node.py \
    _model_path:="$MODEL" _conf_thres:=0.4 _imgsz:=320 _device:=cpu \
    > "$LOG/detector.log" 2>&1 &
sleep 30
grep -hE "ready|numpy" "$LOG/detector.log" | sed 's/^/    /'

echo "[3] visual_tracker_node（必须先于状态机）..."
rosrun uav_tracking visual_tracker_node.py > "$LOG/tracker.log" 2>&1 &
sleep 8
grep -h "ready" "$LOG/tracker.log" | sed 's/^/    /'

echo "[4] state_machine_node ..."
rosrun uav_tracking state_machine_node.py > "$LOG/state.log" 2>&1 &
sleep 6
grep -hE "ready|->" "$LOG/state.log" | sed 's/^/    /'
echo
echo "完成。"
