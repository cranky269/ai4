#!/usr/bin/env bash
# 模块 5 视觉跟踪 —— 实际数据流全打通 E2E 测试
#
# 编排：roscore → visual_tracker_node → [阶段A: fake_detections(有目标) + 断言]
#                                     → [阶段B: fake_detections(无目标) + 断言]
#                                     → [阶段C: fake_camera(320x240) + 有目标 + 断言]
# 跑完自动清理后端进程；日志落在 test/visual_tracker/e2e_test/logs/。
#
# 用法：bash test/visual_tracker/e2e_test/run_e2e.sh
#
# 与 test/cluster/e2e_test/run_e2e.sh 的差异（改进点，建议全项目推广）：
#   REPO 路径**自动推导**（从脚本位置向上三级），不再硬编码 /home/demo/桌面/ai4，
#   队友 clone 到任意目录都能直接跑。
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../../.." && pwd)"
WS="$REPO/catkin_ws"
E2E="$SCRIPT_DIR"
LOGDIR="$E2E/logs"
mkdir -p "$LOGDIR"

echo "仓库根目录: $REPO"

# ROS 的 setup.bash 会引用未定义变量（如 ROS_DISTRO），在 `set -u` 下直接报错，
# 故 source 期间临时关闭 nounset —— 否则从干净终端直接跑本脚本会失败。
set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u

CORE_PID=""
FAKE_PID=""
NODE_PID=""
CAM_PID=""

cleanup() {
  [ -n "$FAKE_PID" ] && kill "$FAKE_PID" 2>/dev/null
  [ -n "$CAM_PID" ]  && kill "$CAM_PID" 2>/dev/null
  [ -n "$NODE_PID" ] && kill "$NODE_PID" 2>/dev/null
  [ -n "$CORE_PID" ] && kill "$CORE_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT

wait_master() { for _ in $(seq 1 50); do rosnode list >/dev/null 2>&1 && return 0; sleep 0.2; done; return 1; }
wait_topic()  { for _ in $(seq 1 50); do rostopic list 2>/dev/null | grep -qx "$1" && return 0; sleep 0.2; done; return 1; }

echo "=== [1/5] 启动 roscore ==="
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

echo "=== [2/5] 启动 visual_tracker_node ==="
rosrun uav_tracking visual_tracker_node.py \
  _track_buffer:=30 _width:=640 _height:=480 \
  > "$LOGDIR/visual_tracker.log" 2>&1 &
NODE_PID=$!
sleep 1
echo "  visual_tracker_node pid=$NODE_PID"

# run_phase <阶段名> <with_target> <mode> [frame_w] [frame_h] [相机附加参数]
# 传了相机参数则同时启动 fake_camera，用于验证「area_ratio 走图像实际尺寸」
run_phase() {
  local phase="$1" with_target="$2" mode="$3"
  local frame_w="${4:-640}" frame_h="${5:-480}" cam="${6:-}" rc=0
  echo "=== $phase: fake_detections(with_target=$with_target) + check(mode=$mode) ==="
  if [ -n "$cam" ]; then
    python3 "$REPO/test/detector/e2e_test/fake_camera.py" $cam \
      > "$LOGDIR/fake_camera_$mode.log" 2>&1 &
    CAM_PID=$!
    wait_topic /camera/image_raw || echo "  (警告) /camera/image_raw 未及时出现"
  fi
  python3 "$E2E/fake_detections.py" _with_target:="$with_target" \
    _x:=280.0 _y:=200.0 _w:=64.0 _h:=48.0 _conf:=0.9 _rate:=10.0 \
    > "$LOGDIR/fake_detections_$mode.log" 2>&1 &
  FAKE_PID=$!
  wait_topic /detections      || echo "  (警告) /detections 未及时出现"
  wait_topic /visual/tracks   || echo "  (警告) /visual/tracks 未及时出现"
  sleep 0.5
  # mode=target 时额外要求「航迹中心确实在移动」：
  #   fake_detections 的默认 ~vx/~vy = 4/3 像素每帧，6 秒约 60 帧 → x 方向跨度约 240 像素。
  #   只查 ID 稳定是有漏洞的：跟踪器若输出一个**固定不变的框**，照样能过。
  local extra_move=""
  if [ "$mode" = "target" ]; then
    extra_move="_check_move:=true _min_move:=100"
  fi
  python3 "$E2E/check_tracks.py" _mode:="$mode" _duration:=6.0 _min_msgs:=20 \
    _frame_w:="$frame_w" _frame_h:="$frame_h" $extra_move
  rc=$?
  kill "$FAKE_PID" 2>/dev/null; wait "$FAKE_PID" 2>/dev/null; FAKE_PID=""
  if [ -n "$CAM_PID" ]; then
    kill "$CAM_PID" 2>/dev/null; wait "$CAM_PID" 2>/dev/null; CAM_PID=""
  fi
  sleep 0.5
  return $rc
}

echo "=== [3/5] 阶段 A（有目标：验证航迹输出 / area_ratio / ID 稳定）==="
run_phase "阶段A" true target
RC_A=$?

echo "=== [4/5] 阶段 B（无目标：验证「空也发」契约）==="
run_phase "阶段B" false empty
RC_B=$?

# 阶段 C：节点启动时兜底尺寸写的是 640x480，这里发布 320x240 的图像；
# 若 area_ratio 按 320x240 算，说明「图像入参优先」生效（组长确认的补正方案）。
echo "=== [5/5] 阶段 C（图像入参：area_ratio 应改用图像实际尺寸 320x240）==="
run_phase "阶段C" true target 320 240 "_width:=320 _height:=240 _rate:=15"
RC_C=$?

echo "----------------------------------------"
if [ "$RC_A" -eq 0 ] && [ "$RC_B" -eq 0 ] && [ "$RC_C" -eq 0 ]; then
  echo "E2E RESULT: PASS  (A=0, B=0, C=0)"
  exit 0
else
  echo "E2E RESULT: FAIL  (A=$RC_A, B=$RC_B, C=$RC_C)  日志见 $LOGDIR"
  exit 1
fi
