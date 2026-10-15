#!/usr/bin/env bash
# 模块 4 相机驱动 —— 实际数据流全打通 E2E 测试
#
# 编排：生成素材 → roscore → [阶段A: camera_node(video) + 断言]
#                          → [阶段B: camera_node(image) + 断言]
# 跑完自动清理后端进程；日志落在 test/camera/e2e_test/logs/。
#
# 用法：bash test/camera/e2e_test/run_e2e.sh
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
TMPDIR="$LOGDIR/tmp"
mkdir -p "$LOGDIR" "$TMPDIR"

echo "仓库根目录: $REPO"

# ROS 的 setup.bash 会引用未定义变量（如 ROS_DISTRO），在 `set -u` 下直接报错，
# 故 source 期间临时关闭 nounset —— 否则从干净终端直接跑本脚本会失败。
set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u

CORE_PID=""
NODE_PID=""

cleanup() {
  [ -n "$NODE_PID" ] && kill "$NODE_PID" 2>/dev/null
  [ -n "$CORE_PID" ] && kill "$CORE_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT

wait_master() { for _ in $(seq 1 50); do rosnode list >/dev/null 2>&1 && return 0; sleep 0.2; done; return 1; }

AW="320"; AH="240"; AFPS="15"

echo "=== [1/4] 生成测试素材 ==="
python3 "$E2E/make_media.py" "$TMPDIR" "$AFPS" || { echo "FAIL: 素材生成失败"; exit 1; }

echo "=== [2/4] 启动 roscore ==="
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

# 启动 camera_node 并等它就绪
start_camera() {
  local src="$1" extra="$2" log="$3"
  rosrun uav_tracking camera_node.py _source:="$src" $extra \
    > "$LOGDIR/$log" 2>&1 &
  NODE_PID=$!
  for _ in $(seq 1 50); do
    rostopic list 2>/dev/null | grep -qx "/camera/image_raw" && return 0
    sleep 0.2
  done
  return 1
}

echo "=== [3/4] 阶段 A（video 源：频率 / 编码 / 分辨率 / 帧在变化）==="
if start_camera video \
     "_video_path:=$TMPDIR/e2e_test.avi _fps:=$AFPS _loop:=true _width:=$AW _height:=$AH" \
     "camera_video.log"; then
  echo "  camera_node(video) pid=$NODE_PID"
  sleep 1
  python3 "$E2E/check_camera.py" _mode:=video _fps:=$AFPS \
    _width:=$AW _height:=$AH _duration:=6.0 _min_msgs:=20
  RC_A=$?
else
  echo "FAIL: camera_node(video) 未发布 /camera/image_raw"
  cat "$LOGDIR/camera_video.log" 2>/dev/null | tail -5
  RC_A=1
fi
kill "$NODE_PID" 2>/dev/null; wait "$NODE_PID" 2>/dev/null; NODE_PID=""
sleep 1

echo "=== [4/4] 阶段 B（image 源：定频重发单张图）==="
if start_camera image \
     "_image_path:=$TMPDIR/e2e_test.png _fps:=10 _width:=$AW _height:=$AH" \
     "camera_image.log"; then
  echo "  camera_node(image) pid=$NODE_PID"
  sleep 1
  python3 "$E2E/check_camera.py" _mode:=image _fps:=10 \
    _width:=$AW _height:=$AH _duration:=6.0 _min_msgs:=15
  RC_B=$?
else
  echo "FAIL: camera_node(image) 未发布 /camera/image_raw"
  cat "$LOGDIR/camera_image.log" 2>/dev/null | tail -5
  RC_B=1
fi
kill "$NODE_PID" 2>/dev/null; wait "$NODE_PID" 2>/dev/null; NODE_PID=""

echo "----------------------------------------"
if [ "${RC_A:-1}" -eq 0 ] && [ "${RC_B:-1}" -eq 0 ]; then
  echo "E2E RESULT: PASS  (A=0, B=0)"
  exit 0
else
  echo "E2E RESULT: FAIL  (A=${RC_A:-1}, B=${RC_B:-1})  日志见 $LOGDIR"
  exit 1
fi
