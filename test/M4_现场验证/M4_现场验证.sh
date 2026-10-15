#!/bin/bash
# =============================================================================
# M4 现场验证脚本 —— 真图 + 真模型 + 四节点全链路
# -----------------------------------------------------------------------------
# 用途：一条命令复现 M4 完整视觉链，把全部证据落到 /tmp/m4_evidence/
#
# 运行：bash M4_现场验证.sh          （跑完证据在 /tmp/m4_evidence/）
#
# 证据清单：
#   detections.txt     /detections    的原始消息（真实 bbox）
#   tracks.txt         /visual/tracks 的原始消息（航迹 ID + area_ratio）
#   state.txt          /state         的原始消息（状态跃迁）
#   debug_image_*.png  带检测框的真实图片（可直接打开看）
#   node_*.log         四个节点各自的完整日志
#   topics.txt         话题清单 + 发布者
#   summary.txt        汇总结论
#
# 设计说明（踩过的坑，别改回去）：
#   * /state **只在状态变化时发布**，不是周期话题。所以必须「先订阅 /state，
#     再启动 visual_tracker_node」，否则抓不到 RADAR_GUIDE -> VISUAL_LOCK 那次跃迁。
#   * 所有 rostopic 抓取都套 timeout，避免任何一条等不到就永久挂住。
#   * 环境变量 PYTHONUNBUFFERED=1：否则节点/rostopic 输出到文件会被块缓冲，
#     脚本还在跑但文件是空的。
# =============================================================================
set -u

HERE="$(cd "$(dirname "$0")" && pwd -P)"   # -P 解析软链
ROOT="/home/demo/桌面/ai4"
WS="$ROOT/catkin_ws"
OUT="/tmp/m4_evidence"
# ★ 素材放工程内（不受 /tmp 被清空影响）；缺了就先跑 实时演示/get_assets.sh
ASSETS="$HERE/实时演示/assets"
MODEL="${MODEL:-$ASSETS/yolov8n.pt}"
PIC="${PIC:-$ASSETS/bus.jpg}"

rm -rf "$OUT"
mkdir -p "$OUT"

# --- 环境（source 期间必须关 nounset，ROS 的 setup.bash 会引用未定义变量）---
set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u
export PYTHONUNBUFFERED=1   # 关键：否则输出到文件会被块缓冲

echo "=============================================================="
echo " M4 现场验证    $(date '+%Y-%m-%d %H:%M:%S %Z')"
echo "--------------------------------------------------------------"
echo " 工程根  : $ROOT"
echo " 模型    : $MODEL"
echo " 测试图  : $PIC"
echo " 证据目录: $OUT"
echo "=============================================================="
echo

wait_topic() {
  local t="$1" n="${2:-40}" i=0
  while [ $i -lt "$n" ]; do
    if rostopic list 2>/dev/null | grep -qx "$t"; then return 0; fi
    sleep 1; i=$((i+1))
  done
  return 1
}

PIDS=""
cleanup() {
  echo
  echo "[清理] 停止所有节点 ..."
  for p in $PIDS; do kill -INT "$p" 2>/dev/null; done
  sleep 3
  pkill -f "uav_tracking/scripts"     2>/dev/null
  pkill -f "rosmaster|roscore|rosout" 2>/dev/null
  sleep 1
  echo "[清理] 完成"
}
trap cleanup EXIT

# --------------------------------------------------------------------------- #
echo "[1/8] 启动 roscore ..."
roscore > "$OUT/node_roscore.log" 2>&1 &
PIDS="$PIDS $!"
sleep 6
if ! rostopic list > /dev/null 2>&1; then
  echo "  roscore 起不来，退出"; exit 1
fi
echo "  OK"

# --------------------------------------------------------------------------- #
echo "[2/8] 先订阅 /state（它只在变化时发布，必须提前蹲守）..."
( timeout 180 rostopic echo /state > "$OUT/state.txt" 2>&1 ) &
echo "  已蹲守"

# --------------------------------------------------------------------------- #
echo "[3/8] 启动 camera_node（真图：bus.jpg）..."
rosrun uav_tracking camera_node.py \
    _source:=image _image_path:="$PIC" _fps:=5 \
    > "$OUT/node_camera.log" 2>&1 &
PIDS="$PIDS $!"
wait_topic /camera/image_raw 40 && echo "  /camera/image_raw OK" || echo "  警告: 图像话题未出现"

# --------------------------------------------------------------------------- #
echo "[4/8] 启动 detector_node（真模型 yolov8n.pt / imgsz=320 / CPU）..."
rosrun uav_tracking detector_node.py \
    _model_path:="$MODEL" _conf_thres:=0.4 _imgsz:=320 _device:=cpu \
    > "$OUT/node_detector.log" 2>&1 &
PIDS="$PIDS $!"
wait_topic /detections 40 && echo "  /detections OK" || echo "  警告: 检测话题未出现"
echo "  等 CPU 推理预热（25 秒）..."
sleep 25

# --------------------------------------------------------------------------- #
# ★ 顺序很重要：**先起跟踪器，后起状态机** ★
#
# 状态机在 RADAR_GUIDE 下，对「视觉锁定」和「雷达丢失」**各自独立计数**：
#     counters[visual_lock] >= N(=3)  → VISUAL_LOCK   （视觉优先）
#     counters[radar_lost]  >= K(=5)  → SEARCH/RTH
#
# 本环境**没有 /radar/tracks 的发布者**（雷达属别的模块），radar_lost 恒为真，
# 约 5 拍（≈1 秒）就跳 SEARCH/RTH；而 SEARCH/RTH **只有「雷达恢复航迹」才能
# 回到 RADAR_GUIDE** —— 一旦进去就再也出不来。
#
# 所以必须让 /visual/tracks 先就绪：视觉锁定计数先到 3，才会走 VISUAL_LOCK 分支。
# （实测：若状态机比跟踪器早 7 秒起，就会先跳 SEARCH/RTH。）
echo "[5/8] 启动 visual_tracker_node（必须先于状态机）..."
rosrun uav_tracking visual_tracker_node.py > "$OUT/node_tracker.log" 2>&1 &
PIDS="$PIDS $!"
wait_topic /visual/tracks 40 && echo "  /visual/tracks OK" || echo "  警告: 航迹话题未出现"
echo "  等跟踪器积累几帧航迹（5 秒）..."
sleep 5

# --------------------------------------------------------------------------- #
echo "[6/8] 启动 state_machine_node（应直接 RADAR_GUIDE -> VISUAL_LOCK）..."
rosrun uav_tracking state_machine_node.py > "$OUT/node_state.log" 2>&1 &
PIDS="$PIDS $!"
wait_topic /state 40 && echo "  /state OK" || echo "  警告: 状态话题未出现"

echo "  等状态机跳转（最多 60 秒）..."
for _ in $(seq 1 30); do
  if grep -q "VISUAL_LOCK" "$OUT/state.txt" 2>/dev/null; then break; fi
  sleep 2
done
grep -q "VISUAL_LOCK" "$OUT/state.txt" 2>/dev/null \
  && echo "  已捕获 VISUAL_LOCK 跃迁" || echo "  未捕获到跃迁（见 summary）"

# --------------------------------------------------------------------------- #
echo "[7/8] 抓取证据 ..."

rostopic list > "$OUT/topics.txt" 2>&1
echo "--- 各话题的发布者 ---" >> "$OUT/topics.txt"
for t in /camera/image_raw /detections /visual/tracks /state /detector/debug_image; do
  echo "[$t]" >> "$OUT/topics.txt"
  timeout 15 rostopic info "$t" 2>&1 | sed 's/^/    /' >> "$OUT/topics.txt"
done

timeout 30 rostopic echo -n 3 /detections    > "$OUT/detections.txt" 2>&1
timeout 30 rostopic echo -n 3 /visual/tracks > "$OUT/tracks.txt" 2>&1

cat > "$OUT/grab_debug.py" <<'PYEOF'
# 订阅 /detector/debug_image，把带 bbox 的真实图片存成 PNG
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node("grab_debug", anonymous=True, disable_signals=True)
br = CvBridge()
saved = []


def cb(msg):
    if len(saved) >= 3:
        return
    img = br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    path = "/tmp/m4_evidence/debug_image_%d.png" % len(saved)
    cv2.imwrite(path, img)
    saved.append(path)
    print("  已保存 %s  shape=%s" % (path, img.shape))


rospy.Subscriber("/detector/debug_image", Image, cb, queue_size=1)
t0 = rospy.get_time()
while not rospy.is_shutdown() and len(saved) < 3 and (rospy.get_time() - t0) < 60:
    rospy.sleep(0.2)
print("  共抓到 %d 张调试图" % len(saved))
PYEOF

timeout 90 python3 "$OUT/grab_debug.py" 2>&1 | tee "$OUT/grab_debug.log"

# --------------------------------------------------------------------------- #
echo "[8/8] 汇总 ..."
{
  echo "=============================================================="
  echo " M4 现场验证结果    $(date '+%Y-%m-%d %H:%M:%S %Z')"
  echo "=============================================================="
  echo
  echo "--- 一、节点与话题 ---"
  sed -n '1,40p' "$OUT/topics.txt"
  echo
  echo "--- 二、/detections 真实目标（共 $(grep -c 'class_id' "$OUT/detections.txt" 2>/dev/null || echo 0) 条）---"
  grep -E "class_id|confidence|bbox" "$OUT/detections.txt" 2>/dev/null | head -16 | sed 's/^/  /'
  echo
  echo "--- 三、/visual/tracks 航迹 ---"
  grep -E "track_id|area_ratio|class_id" "$OUT/tracks.txt" 2>/dev/null | head -16 | sed 's/^/  /'
  echo
  echo "--- 四、/state 状态跃迁 ---"
  cat "$OUT/state.txt" 2>/dev/null | sed 's/^/  /'
  echo
  echo "--- 五、带检测框的图片 ---"
  ls -l "$OUT"/debug_image_*.png 2>/dev/null | sed 's/^/  /'
  echo
  echo "--- 六、节点启动日志 ---"
  echo "  [camera]";  grep -iE "ready|source|fps|图像" "$OUT/node_camera.log" 2>/dev/null | head -4 | sed 's/^/    /'
  echo "  [detector]"; grep -iE "ready|numpy|隔离|模型" "$OUT/node_detector.log" 2>/dev/null | head -4 | sed 's/^/    /'
  echo "  [tracker]"; grep -iE "ready|track_buffer|frame" "$OUT/node_tracker.log" 2>/dev/null | head -4 | sed 's/^/    /'
  echo "  [state]";   grep -iE "ready|->" "$OUT/node_state.log" 2>/dev/null | head -6 | sed 's/^/    /'
  echo
  echo "--- 七、有没有 ERROR / Traceback ---"
  if grep -hiE "error|traceback|exception" "$OUT"/node_camera.log "$OUT"/node_detector.log "$OUT"/node_tracker.log "$OUT"/node_state.log 2>/dev/null | head -8 | sed 's/^/  /'; then :; else echo "  无"; fi
  echo
  echo "=============================================================="
} > "$OUT/summary.txt" 2>&1

cat "$OUT/summary.txt"
echo
echo "=============================================================="
echo " 证据已全部写入 $OUT"
echo "=============================================================="
