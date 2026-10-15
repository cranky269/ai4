#!/bin/bash
# =============================================================================
# M4 实时演示 —— 一键启动（节点 + 实时画面窗口 + 状态面板终端）
# -----------------------------------------------------------------------------
# 用途：把 M4 整条链跑起来并**一直开着**，供人肉眼观察。
#
# 运行：bash m4_live.sh          （跑完不退出，画面和面板都留着）
# 停止：bash m4_live.sh stop
#
# 起来之后你会看到：
#   * 右侧一个窗口：真实图片 + 绿色检测框，随帧刷新；
#   * 左侧一个终端：实时状态面板（当前状态 / 目标数 / 航迹 ID / area_ratio）。
# =============================================================================
set -u

ROOT="/home/demo/桌面/ai4"
WS="$ROOT/catkin_ws"
HERE="$(cd "$(dirname "$0")" && pwd -P)"  # 本脚本所在目录（-P 解析软链；live_view.py 在旁边）
ASSETS="$HERE/assets"                     # 素材在工程内，不受 /tmp 清空影响
LOG="/tmp/m4_live"
MODEL="${MODEL:-$ASSETS/yolov8n.pt}"
PIC="${PIC:-$ASSETS/bus.jpg}"

# --------------------------------------------------------------------------- #
# 停止：只按**精确命令行前缀**杀，避免误杀调用它的 shell
# --------------------------------------------------------------------------- #
stop_all() {
  echo "[停止] 关闭 M4 相关进程 ..."
  for pat in "python3 .*live_view.py" "python3 .*dashboard.py"; do
    pkill -f "$pat" 2>/dev/null
  done
  pkill -f "uav_tracking/scripts"      2>/dev/null
  pkill -f "rosmaster|roscore|rosout"  2>/dev/null
  sleep 2
  echo "[停止] 完成"
}

if [ "${1:-}" = "stop" ]; then
  stop_all
  exit 0
fi

# --------------------------------------------------------------------------- #
# 环境
# --------------------------------------------------------------------------- #
set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u
export PYTHONUNBUFFERED=1

# 图形相关：既能从 SSH 调用，也能从虚拟机自己的终端调用
export DISPLAY="${DISPLAY:-:0}"
if [ -z "${XAUTHORITY:-}" ]; then
  for xa in "/run/user/1000/gdm/Xauthority" "$HOME/.Xauthority"; do
    if [ -f "$xa" ]; then export XAUTHORITY="$xa"; break; fi
  done
fi

mkdir -p "$LOG"

echo "=============================================================="
echo " M4 实时演示    $(date '+%Y-%m-%d %H:%M:%S')"
echo "--------------------------------------------------------------"
echo " 模型  : $MODEL"
echo " 测试图: $PIC"
echo " 日志  : $LOG"
echo " DISPLAY=$DISPLAY"
echo "=============================================================="

echo "[1/6] 清理旧进程 ..."
stop_all

echo "[2/6] roscore ..."
roscore > "$LOG/roscore.log" 2>&1 &
sleep 7

echo "[3/6] camera_node（真图，5 FPS）..."
rosrun uav_tracking camera_node.py \
    _source:=image _image_path:="$PIC" _fps:=5 > "$LOG/camera.log" 2>&1 &
sleep 4

echo "[4/6] detector_node（真模型，CPU 预热约 25 秒）..."
rosrun uav_tracking detector_node.py \
    _model_path:="$MODEL" _conf_thres:=0.4 _imgsz:=320 _device:=cpu \
    > "$LOG/detector.log" 2>&1 &
sleep 28

echo "[5/6] visual_tracker_node（必须先于状态机，见 README 第四节）..."
rosrun uav_tracking visual_tracker_node.py > "$LOG/tracker.log" 2>&1 &
sleep 6

echo "[6/6] state_machine_node + 可视化窗口 ..."
rosrun uav_tracking state_machine_node.py > "$LOG/state.log" 2>&1 &
sleep 4

# --- 实时画面窗口（OpenCV，自动放到右侧）---
if [ -f "$HERE/live_view.py" ]; then
  setsid python3 "$HERE/live_view.py" > "$LOG/live_view.log" 2>&1 < /dev/null &
  sleep 5
  echo "  画面窗口: $(pgrep -cf live_view.py) 个进程"
else
  echo "  (跳过画面窗口：找不到 $HERE/live_view.py)"
fi

# --- 状态面板终端（放左侧）---
if [ -f "$HERE/dashboard.py" ]; then
  gnome-terminal --window --title="M4 实时状态面板" --geometry=74x44+0+0 \
    -- bash -lc "python3 '$HERE/dashboard.py'" > "$LOG/gt_dash.log" 2>&1 &
  sleep 8
  echo "  状态面板: $(pgrep -cf dashboard.py) 个进程"
else
  echo "  (跳过状态面板：找不到 $HERE/dashboard.py)"
fi

echo
echo "=============================================================="
echo " 全部就绪"
echo "--------------------------------------------------------------"
rostopic list 2>/dev/null | sed 's/^/   /'
echo
echo " 当前状态:"
grep -hE "ready|->" "$LOG"/state.log "$LOG"/tracker.log 2>/dev/null | tail -4 | sed 's/^/   /'
echo "=============================================================="
