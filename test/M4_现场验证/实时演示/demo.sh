#!/bin/bash
# =============================================================================
# M4 演示一键脚本
# -----------------------------------------------------------------------------
#   bash demo.sh           起全部（roscore + 四个节点 + 直播服务），最后给出网址
#   bash demo.sh restart   只重启四个节点（保留 roscore 与直播）→ 看整条链起来
#   bash demo.sh stop      全部停掉
#   bash demo.sh status    看现在什么状态
#
# 起好之后，在**宿主机浏览器**打开脚本最后打印的那个网址即可。
# =============================================================================
set -u

# pwd -P：解析软链（本脚本常通过 ~/m4 调用）
HERE="$(cd "$(dirname "$0")" && pwd -P)"
ASSETS="$HERE/assets"
WS="/home/demo/桌面/ai4/catkin_ws"
LOG="/tmp/m4_live"
# ★ 素材放**工程内**（不受 /tmp 被清空影响）。实测踩过：一次重启后
#   /tmp/m4_infer 被清空，camera_node 直接抛错、整条链卡在 RADAR_GUIDE。
MODEL="${MODEL:-$ASSETS/yolov8n.pt}"
PIC="${PIC:-$ASSETS/bus.jpg}"
CLIP="${CLIP:-$ASSETS/bus_moving.mp4}"
CLIP_MODE="${CLIP_MODE:-both}"     # both | approach | sweep（都是真照片的取景框运动）
# SRC=image（默认，静态图，证明链路通）
# SRC=video（动态视频，证明"跟踪能动"：框动、ID 保持、area_ratio 随尺度变化）
SRC="${SRC:-image}"
# 这两个是为了**验证**用：调 CONF 看目标数是否跟着变（证明像素真进了模型）
CONF="${CONF:-0.40}"
IMGSZ="${IMGSZ:-320}"
PORT="${PORT:-8090}"

mkdir -p "$LOG"

# --------------------------------------------------------------------------- #
# 工具
# --------------------------------------------------------------------------- #
stop_all() {
  echo "  停止直播服务 ..."
  # 不锚定路径：无论从哪个目录起的 live_stream.py 都收掉
  # （实测踩过：早期从 /tmp/m4demo 起的那份会漏杀，一直占着 8090 端口）
  pkill -f "live_stream\.py" 2>/dev/null
  # GUI 窗口（画面窗口 / 状态面板）也要一起收掉，否则它会一直留在桌面上
  echo "  停止图形窗口 ..."
  pkill -f "live_view\.py"  2>/dev/null
  pkill -f "dashboard\.py"  2>/dev/null
  pkill -f "eog "            2>/dev/null
  echo "  停止 M4 节点 ..."
  pkill -f "uav_tracking/scripts" 2>/dev/null
  echo "  停止 roscore ..."
  pkill -f "rosmaster|roscore|rosout" 2>/dev/null
  sleep 3
  echo "  已全部停止（窗口也应该消失了）"
}

ensure_assets() {
  # 素材必须在。缺了就自动补，而不是让 camera_node 抛错、脚本卡住。
  local need_clip=0
  [ "$SRC" = "video" ] && need_clip=1

  if [ ! -s "$MODEL" ] || [ ! -s "$PIC" ] \
     || { [ "$need_clip" = 1 ] && [ ! -s "$CLIP" ]; }; then
    echo "  ⚠ 素材缺失，自动补齐 ..."
    echo "    模型: $MODEL"
    echo "    图片: $PIC"
    bash "$HERE/get_assets.sh" || true
    if [ "$need_clip" = 1 ] && [ ! -s "$CLIP" ]; then
      echo "  生成动态视频: $CLIP（模式 $CLIP_MODE）"
      python3 "$HERE/make_moving_clip.py" "$CLIP" 150 10 "$CLIP_MODE" || true
    fi
  fi

  if [ ! -s "$MODEL" ] || [ ! -s "$PIC" ]; then
    echo
    echo "  ✗ 素材仍不齐，无法继续。请手动执行："
    echo "      bash $HERE/get_assets.sh"
    exit 1
  fi
  if [ "$need_clip" = 1 ] && [ ! -s "$CLIP" ]; then
    echo "  ✗ 动态视频缺失，无法用视频源。请执行："
    echo "      python3 $HERE/make_moving_clip.py"
    exit 1
  fi
  echo "  素材已就绪${need_clip:+（含动态视频）}"
}

start_roscore() {
  roscore > "$LOG/roscore.log" 2>&1 &
  sleep 7
  rostopic list > /dev/null 2>&1 || { echo "  !! roscore 起不来"; exit 1; }
}

start_stream() {
  setsid nohup python3 "${HERE}/live_stream.py" "$PORT" \
      > "$LOG/stream.log" 2>&1 < /dev/null &
  sleep 5
}

start_nodes() {
  if [ "$SRC" = "video" ]; then
    echo "  camera_node（★动态视频源★ 10 FPS 循环 —— 目标是会动的）..."
    rosrun uav_tracking camera_node.py \
        _source:=video _video_path:="$CLIP" _loop:=true _fps:=10 \
        > "$LOG/camera.log" 2>&1 &
  else
    echo "  camera_node（静态图 5 FPS —— 只能证明链路通）..."
    rosrun uav_tracking camera_node.py \
        _source:=image _image_path:="$PIC" _fps:=5 > "$LOG/camera.log" 2>&1 &
  fi
  sleep 5

  echo "  detector_node（真模型，conf=$CONF imgsz=$IMGSZ，CPU 预热约 30 秒，请等）..."
  rosrun uav_tracking detector_node.py \
      _model_path:="$MODEL" _conf_thres:="$CONF" _imgsz:="$IMGSZ" _device:=cpu \
      > "$LOG/detector.log" 2>&1 &
  sleep 30

  echo "  visual_tracker_node（必须先于状态机）..."
  rosrun uav_tracking visual_tracker_node.py > "$LOG/tracker.log" 2>&1 &
  sleep 8

  echo "  state_machine_node ..."
  rosrun uav_tracking state_machine_node.py > "$LOG/state.log" 2>&1 &
  sleep 6
}

wait_lock() {
  echo "  等状态机跳到 VISUAL_LOCK（最多 60 秒）..."
  for _ in $(seq 1 30); do
    if grep -q "VISUAL_LOCK" "$LOG/state.log" 2>/dev/null; then return 0; fi
    sleep 2
  done
  return 1
}

report() {
  echo
  echo "=================== 现在去浏览器看这个网址 ==================="
  for ip in $(hostname -I 2>/dev/null); do
    echo "      http://${ip}:${PORT}/"
  done
  echo "=============================================================="
  echo
  echo "  节点:"
  grep -h "ready" "$LOG"/camera.log "$LOG"/detector.log "$LOG"/tracker.log \
       "$LOG"/state.log 2>/dev/null | sed 's/^/    /'
  echo "  状态跃迁:"
  grep -h -- "->" "$LOG"/state.log 2>/dev/null | sed 's/^/    /'
  echo
}

# --------------------------------------------------------------------------- #
# 子命令
# --------------------------------------------------------------------------- #
case "${1:-start}" in
  stop)
    echo "=== 停止全部 ==="
    stop_all
    echo "已停止。"
    exit 0
    ;;
  status)
    echo "=== 当前状态 ==="
    echo "  roscore      : $(pgrep -cf 'rosmaster|roscore' || echo 0)"
    echo "  M4 节点      : $(pgrep -cf 'uav_tracking/scripts' || echo 0)"
    echo "  直播服务     : $(pgrep -cf 'live_strea[m].py' || echo 0)"
    echo "  监听 $PORT    : $(ss -lnt 2>/dev/null | grep -c ":$PORT ") "
    if command -v curl >/dev/null 2>&1; then
      echo -n "  页面自测     : "
      curl -s -m 5 "http://127.0.0.1:$PORT/status" 2>/dev/null \
        | head -c 160 || echo "(不通)"
      echo
    fi
    exit 0
    ;;
  restart)
    set +u; source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"; set -u
    export PYTHONUNBUFFERED=1
    ensure_assets
    echo "=== 只重启 M4 四个节点（roscore 和直播服务不动）==="
    echo "    页面上的画面会先停住，然后逐级恢复 —— 盯着浏览器看。"
    echo
    pkill -f "uav_tracking/scripts" 2>/dev/null
    sleep 3
    start_nodes
    wait_lock && echo "  已捕获 VISUAL_LOCK" || echo "  未捕获到（看日志）"
    report
    exit 0
    ;;
  moving|video)
    # 动态视频源：设好 SRC 后落到下面的默认流程（起全部）
    SRC=video
    ;;
  approach|sweep)
    # 单维演示：只看缩放 / 只看横移（都是真照片的取景框运动）
    SRC=video
    CLIP="$ASSETS/bus_moving.mp4"
    CLIP_MODE="$1"
    ;;
esac

# --- 默认：起全部 ---
set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u
export PYTHONUNBUFFERED=1

ensure_assets

echo "=============================================================="
echo " M4 演示启动    $(date '+%Y-%m-%d %H:%M:%S')"
echo " 脚本目录: $HERE"
if [ "$SRC" = "video" ]; then
  echo " 相机源  : video（动态！$CLIP）"
else
  echo " 相机源  : image（静态图 $PIC —— 只证明链路通）"
  echo "           想看跟踪在动，用:  bash demo.sh moving"
fi
echo "=============================================================="
echo

echo "[1/6] 清理旧进程"
stop_all

echo "[2/6] roscore"
start_roscore

echo "[3/6] 直播服务（浏览器看画面）"
start_stream

echo "[4/6] 起四个节点"
start_nodes

echo "[5/6] 确认状态"
wait_lock && echo "  VISUAL_LOCK 已达成" || echo "  未达成，见日志 $LOG/state.log"

echo "[6/6] 完成"
report
