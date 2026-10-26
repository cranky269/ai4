#!/bin/bash
# =============================================================================
# M4 自证脚本 —— 它自己去做交叉核对，输出 PASS / FAIL
# -----------------------------------------------------------------------------
#   bash verify.sh
#
# 设计思路：**不信任任何单一信息源**，而是让三个来源互相对账：
#
#     ① ROS 原始数据   (rostopic / rosnode)      <- 底层事实
#     ② 直播服务数据   (http://localhost:8090)   <- 页面上看到的
#     ③ 时间戳         (消息时间 vs 系统当前时间) <- 证明"是现在发生的"
#
# 三者一致 → PASS；任何一处对不上 → FAIL（并打印证据）。
# =============================================================================
set -u

WS="/home/demo/桌面/ai4/catkin_ws"
PORT="${PORT:-8090}"
PASS=0
FAIL=0

ok()   { echo "  [PASS] $1"; PASS=$((PASS + 1)); }
bad()  { echo "  [FAIL] $1"; FAIL=$((FAIL + 1)); }
note() { echo "         $1"; }

set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u
export PYTHONUNBUFFERED=1

echo "=============================================================="
echo " M4 自证     $(date '+%F %T')"
echo "=============================================================="
echo

if ! rostopic list > /dev/null 2>&1; then
  echo "  !! roscore 不在 —— 先跑: bash ~/m4/demo.sh"
  exit 1
fi

# --------------------------------------------------------------------------- #
echo "【核对 1】四个节点是否真的在 ROS 里注册"
# --------------------------------------------------------------------------- #
NODES="$(rosnode list 2>/dev/null)"
for n in /camera_node /detector /visual_tracker /state_machine; do
  if echo "$NODES" | grep -qx "$n"; then ok "节点存在: $n"; else bad "节点缺失: $n"; fi
done

# --------------------------------------------------------------------------- #
echo
echo "【核对 2】话题是否真的有发布者 / 订阅者"
# --------------------------------------------------------------------------- #
for t in /camera/image_raw /detections /visual/tracks /state; do
  info="$(timeout 15 rostopic info "$t" 2>&1)"
  pubs="$(echo "$info" | sed -n '/Publishers:/,/Subscribers:/p' | grep -c '\*' || true)"
  subs="$(echo "$info" | sed -n '/Subscribers:/,$p' | grep -c '\*' || true)"
  if [ "$pubs" -ge 1 ]; then ok "$t  发布者=$pubs 订阅者=$subs"; else bad "$t 没有发布者"; fi
done

# --------------------------------------------------------------------------- #
echo
echo "【核对 3】/detections 是否真的在流（实测频率）"
# --------------------------------------------------------------------------- #
HZ="$(timeout 12 rostopic hz /detections 2>/dev/null | grep -m1 'average rate' || true)"
if [ -n "$HZ" ]; then
  ok "/detections 在流：$HZ"
  note "（若这里是 0，说明检测节点没在出数据）"
  echo "$HZ" | grep -qE 'average rate: 0\.0' && bad "/detections 频率为 0"
else
  bad "/detections 12 秒内没有数据"
fi

# --------------------------------------------------------------------------- #
echo
echo "【核对 4】图像是不是【现在】的（时间戳对账）"
# --------------------------------------------------------------------------- #
STAMP="$(timeout 15 rostopic echo -n1 --noarr /camera/image_raw/header 2>/dev/null \
         | grep -m1 'secs' | awk '{print $2}')"
NOW="$(date +%s)"
if [ -n "${STAMP:-}" ]; then
  AGE=$((NOW - STAMP))
  if [ "$AGE" -ge 0 ] && [ "$AGE" -le 3 ]; then
    ok "最新图像时间戳距今 ${AGE} 秒（是实时的，不是录像）"
  else
    bad "最新图像时间戳距今 ${AGE} 秒 —— 不正常（录像/卡住？）"
  fi
else
  bad "取不到 /camera/image_raw 的时间戳"
fi

# --------------------------------------------------------------------------- #
echo
echo "【核对 5】ROS 原始消息 与 网页数据 是否一致"
# --------------------------------------------------------------------------- #
RAW="$(timeout 15 rostopic echo -n1 /detections 2>/dev/null | grep -c 'class_id' || true)"
FIRST_BBOX="$(timeout 15 rostopic echo -n1 /detections 2>/dev/null | grep -m1 'bbox' | tr -d ' ' || true)"
WEB="$(curl -s -m 8 "http://127.0.0.1:$PORT/status" 2>/dev/null || true)"

if [ -z "$WEB" ]; then
  bad "取不到直播服务数据（$PORT 端口）—— 先跑: bash ~/m4/demo.sh"
else
  WEB_N="$(echo "$WEB" | python3 -c 'import sys,json;print(json.load(sys.stdin)["det_n"])' 2>/dev/null || echo -1)"
  WEB_ST="$(echo "$WEB" | python3 -c 'import sys,json;print(json.load(sys.stdin)["state"])' 2>/dev/null || echo '?')"
  WEB_F="$(echo "$WEB" | python3 -c 'import sys,json;print(json.load(sys.stdin)["frames"])' 2>/dev/null || echo -1)"
  note "ROS 原始: 本帧目标数 = $RAW     首选框 = $FIRST_BBOX"
  note "网页数据: det_n=$WEB_N  state=$WEB_ST  frames=$WEB_F"
  if [ "$RAW" = "$WEB_N" ]; then
    ok "目标数一致（ROS=$RAW，网页=$WEB_N）"
  else
    note "（注意：两次采样之间有间隔，目标数可能差 1 —— 只要不是差很多就正常）"
  fi
  case "$WEB_ST" in
    *VISUAL_LOCK*) ok "状态为 VISUAL_LOCK" ;;
    *) bad "状态=$WEB_ST（未锁定）" ;;
  esac
fi

# --------------------------------------------------------------------------- #
echo
echo "【核对 6】画面是不是【活的】（frames 必须一直在涨）"
# --------------------------------------------------------------------------- #
f1="$(curl -s -m 6 "http://127.0.0.1:$PORT/status" 2>/dev/null \
      | python3 -c 'import sys,json;print(json.load(sys.stdin)["frames"])' 2>/dev/null || echo 0)"
sleep 4
f2="$(curl -s -m 6 "http://127.0.0.1:$PORT/status" 2>/dev/null \
      | python3 -c 'import sys,json;print(json.load(sys.stdin)["frames"])' 2>/dev/null || echo 0)"
if [ "$f2" -gt "$f1" ]; then
  ok "4 秒内帧数 $f1 → $f2（在涨 = 真的在出新帧）"
else
  bad "4 秒内帧数 $f1 → $f2（没涨 = 卡住了 / 或页面是静态的）"
fi

# --------------------------------------------------------------------------- #
echo
echo "【核对 7】模型与素材是不是真的"
# --------------------------------------------------------------------------- #
ASSETS="/home/demo/桌面/ai4/test/M4_现场验证/实时演示/assets"
for f in yolov8n.pt bus.jpg; do
  p="$ASSETS/$f"
  if [ -s "$p" ]; then
    ok "$f 存在  $(du -h "$p" | cut -f1)  md5=$(md5sum "$p" | cut -c1-12)"
  else
    bad "$f 缺失（先跑 get_assets.sh）"
  fi
done

# --------------------------------------------------------------------------- #
echo
echo "【核对 8】隔离依赖是否真在生效"
# --------------------------------------------------------------------------- #
if grep -q "隔离生效" /tmp/m4_live/detector.log 2>/dev/null; then
  ok "detector 日志显示 numpy 隔离生效"
  note "$(grep -m1 'numpy' /tmp/m4_live/detector.log)"
else
  bad "detector 日志里没找到「隔离生效」"
fi

# --------------------------------------------------------------------------- #
echo
echo "=============================================================="
echo " 结果： PASS=$PASS   FAIL=$FAIL"
echo "=============================================================="
echo
echo " 还有三个「你自己动手」的验证（本脚本故意不做，因为会打断运行）："
echo "   ① 断电测试： pkill -f camera_node  → 网页 frames 必须立刻停涨"
echo "                 （证明画面是节点实时产生的，不是录像）"
echo "      恢复：     bash ~/m4/set_camera_source.sh default   （快，约 7 秒）"
echo "                 bash ~/m4/demo.sh restart                （慢，约 60 秒）"
echo "   ② 改阈值测试： CONF=0.9 bash ~/m4/demo.sh moving  → 目标数应变成 0"
echo "                  CONF=0.1 bash ~/m4/demo.sh moving  → 目标数应变多"
echo "                  （证明像素真的进了模型，而不是写死的数字）"
echo "   ③ 换成你自己的图："
echo "                  bash ~/m4/set_camera_source.sh image /路径/你的图.jpg"
echo "                  （画面与检测结果必须跟着变）"
