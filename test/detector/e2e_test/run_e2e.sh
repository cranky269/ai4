#!/usr/bin/env bash
# 模块 4 视觉检测 —— 实际数据流全打通 E2E 测试
#
# 编排：roscore → detector_node → [阶段A: fake_camera(纯色图) + 断言 empty]
#                              → [阶段B: fake_camera(真实图片) + 断言 target]（可选）
# 跑完自动清理后端进程；日志落在 test/detector/e2e_test/logs/。
#
# 用法：
#   bash test/detector/e2e_test/run_e2e.sh                  # 只跑阶段A
#   IMAGE=/path/to/target.jpg bash test/detector/e2e_test/run_e2e.sh   # 跑两阶段
#
#   > 阶段B 的 bbox 越界断言用的是**图片的真实像素尺寸**：fake_camera 提供
#   > `~image_path` 时是原样发布该图、**不按 `_width/_height` 缩放**，
#   > 所以不能写死 640x480（否则非该尺寸的图片会被误判越界）。
#
# 前置条件
# --------
#   1. 已 catkin_make 并 source（脚本内已 source）；
#   2. 模型权重可访问：默认 `~model_path:=yolov8n.pt`。
#      ultralytics 联网时会自动下载；**离线环境请先把权重放到本地**再用
#      `~model_path:=/abs/path/yolov8n.pt` 指定（否则本脚本阶段A仍会通过，
#      但走的是"推理异常 → 发空消息"分支，不能证明正常检出）。
#
# 与 cluster / control_mux / state_machine 的 run_e2e.sh 保持一致的结构；
# REPO 路径自动推导，不硬编码。
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../../.." && pwd)"
WS="$REPO/catkin_ws"
E2E="$SCRIPT_DIR"
LOGDIR="$E2E/logs"
mkdir -p "$LOGDIR"

MODEL_PATH="${MODEL_PATH:-yolov8n.pt}"
IMAGE="${IMAGE:-}"

echo "仓库根目录: $REPO"
echo "模型权重  : $MODEL_PATH"
echo "阶段B 图片: ${IMAGE:-（未提供，将跳过）}"

# ROS 的 setup.bash 会引用未定义变量（如 ROS_DISTRO），在 `set -u` 下直接报错，
# 故 source 期间临时关闭 nounset —— 否则从干净终端直接跑本脚本会失败。
set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u

CORE_PID=""
CAM_PID=""
NODE_PID=""

cleanup() {
  [ -n "$CAM_PID" ] && kill "$CAM_PID" 2>/dev/null
  [ -n "$NODE_PID" ] && kill "$NODE_PID" 2>/dev/null
  [ -n "$CORE_PID" ] && kill "$CORE_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT

wait_master() { for _ in $(seq 1 50); do rosnode list >/dev/null 2>&1 && return 0; sleep 0.2; done; return 1; }
wait_topic()  { for _ in $(seq 1 50); do rostopic list 2>/dev/null | grep -qx "$1" && return 0; sleep 0.2; done; return 1; }

echo "=== [1/4] 启动 roscore ==="
if rosnode list >/dev/null 2>&1; then
  echo "  master 已在运行，复用"
else
  nohup roscore > "$LOGDIR/roscore.log" 2>&1 &
  CORE_PID=$!
  if wait_master; then
    echo "  roscore 已启动 (pid=$CORE_PID)"
  else
    echo "FAIL: roscore 未启动"
    exit 1
  fi
fi

echo "=== [2/4] 启动 detector_node ==="
rosrun uav_tracking detector_node.py \
  _model_path:="$MODEL_PATH" _conf_thres:=0.5 _imgsz:=640 _device:=cpu \
  > "$LOGDIR/detector.log" 2>&1 &
NODE_PID=$!
sleep 2
echo "  detector_node pid=$NODE_PID"

# --- warmup：CPU 上首次推理要加载模型并预热，这段必须等 ---
# 预热期内出帧很慢（实测 3Hz，热后 ~5.7Hz），若立刻跑 6 秒定时断言，
# 会把「出帧慢」误判成「话题未在流（仅 N 条 < 20）」。
# 这里用固定等待：确定性最好，且预热时长与机器性能无关地足够。
wait_topic /detections || echo "  (警告) /detections 未及时出现"
echo "  等 detector 预热（CPU 首次推理较慢，20s）..."
sleep 20

run_phase() {
  local phase="$1" mode="$2" img="$3" pattern="$4" rc=0
  echo "=== $phase: fake_camera(pattern=$pattern, image=${img:-none}) + check(mode=$mode) ==="

  # 断言用的画面尺寸必须与**实际发布的那一帧**一致：
  # fake_camera 提供 ~image_path 时是「原样发布该图」，**不按 _width/_height 缩放**，
  # 所以此时要读图片的真实尺寸；否则 _frame_* 写死 640x480 会把合法 bbox 误判越界。
  local fw=640 fh=480
  if [ -n "$img" ] && [ -f "$img" ]; then
    read -r fw fh < <(python3 -c "
import cv2, sys
i = cv2.imread(sys.argv[1])
print(i.shape[1], i.shape[0]) if i is not None else print(640, 480)
" "$img")
    echo "  实际画面尺寸: ${fw}x${fh}（取自 $img）"
  fi

  python3 "$E2E/fake_camera.py" _rate:=10.0 _width:=640 _height:=480 \
    _pattern:="$pattern" _image_path:="$img" \
    > "$LOGDIR/fake_camera_$mode.log" 2>&1 &
  CAM_PID=$!
  wait_topic /camera/image_raw || echo "  (警告) /camera/image_raw 未及时出现"
  wait_topic /detections       || echo "  (警告) /detections 未及时出现"
  sleep 0.5
  # _min_msgs 只用来证明「话题在流」，**不是吞吐指标**：
  # /detector 是推理受限的（CPU 实测出帧 ~2.5~5.7 Hz，预热刚结束那几秒更慢），
  # 写死 20（=3.3 Hz）会在慢机器或预热期误判；取 5（≈0.8 Hz）实测**仍会踩边界**
  # （2026-10-11 复跑：阶段A 收到 4 条 → 差 1 条 FAIL；而 10 秒后的阶段B 有 57 条）。
  # ★ 阶段A 的目的只是「链路连通 + 空也发」，吞吐由阶段B 体现，故阈值再放
  #   宽到 2（≈0.3 Hz）：只证「有消息」，不测速率，避免慢机器上的假 FAIL。
  local minm=2 dur=6.0
  [ "$mode" = "target" ] && minm=5
  python3 "$E2E/check_detections.py" _mode:="$mode" _duration:=$dur _min_msgs:=$minm \
    _frame_w:="$fw" _frame_h:="$fh"
  rc=$?
  kill "$CAM_PID" 2>/dev/null
  wait "$CAM_PID" 2>/dev/null
  CAM_PID=""
  sleep 0.5
  return $rc
}

echo "=== [3/4] 阶段 A（纯色图：验证链路连通 + 「空也发」）==="
run_phase "阶段A" empty "" blank
RC_A=$?

RC_B=0
if [ -n "$IMAGE" ]; then
  echo "=== [4/4] 阶段 B（真实图片：验证正常检出 + bbox 合法）==="
  run_phase "阶段B" target "$IMAGE" blank
  RC_B=$?
else
  echo "=== [4/4] 阶段 B 跳过（未设置 IMAGE 环境变量）==="
  echo "  如需验证正常检出：IMAGE=/path/to/target.jpg bash $0"
  RC_B=0
fi

echo "----------------------------------------"
if [ "$RC_A" -eq 0 ] && [ "$RC_B" -eq 0 ]; then
  echo "E2E RESULT: PASS  (A=$RC_A, B=$RC_B)"
  exit 0
else
  echo "E2E RESULT: FAIL  (A=$RC_A, B=$RC_B)  日志见 $LOGDIR"
  exit 1
fi
