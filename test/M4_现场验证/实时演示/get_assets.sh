#!/bin/bash
# =============================================================================
# 下载 M4 演示所需的模型与测试图，**放到工程内的持久目录**
# -----------------------------------------------------------------------------
#   bash get_assets.sh
#
# 为什么要有这个脚本：
#   素材原先放在 /tmp/m4_infer —— 而 /tmp 在**重启后会被清空**（实测踩到：
#   一次重启后 bus.jpg 消失，camera_node 直接抛错，整条链卡在 RADAR_GUIDE）。
#   现在统一改放 `test/M4_现场验证/assets/`（在工程里，重启不丢）。
#
# 已存在则跳过，不会重复下载。
# =============================================================================
set -u

# 注意 pwd -P：解析软链。
# 本脚本可能通过 ~/m4（软链）调用，若用 pwd 会得到 /home/demo/m4，
# 再 dirname 就变成 /home/demo —— 素材会被下到错误的地方（实测踩过）。
HERE="$(cd "$(dirname "$0")" && pwd -P)"
ASSETS="$HERE/assets"                       # 素材就放在实时演示/assets 下，最省事
MODEL="$ASSETS/yolov8n.pt"
PIC="$ASSETS/bus.jpg"

mkdir -p "$ASSETS"

MODEL_URLS=(
  "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt"
  "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt"
  "https://github.com/ultralytics/assets/releases/latest/download/yolov8n.pt"
)
PIC_URLS=(
  "https://raw.githubusercontent.com/ultralytics/ultralytics/main/ultralytics/assets/bus.jpg"
  "https://ultralytics.com/images/bus.jpg"
)

try_download() {          # $1=输出文件 $2..=候选 URL
  local out="$1"; shift
  if [ -s "$out" ]; then
    echo "  已存在，跳过: $out（$(du -h "$out" | cut -f1)）"
    return 0
  fi
  for u in "$@"; do
    echo "  尝试: $u"
    if curl -fL --retry 2 --connect-timeout 15 -m 300 -o "$out" "$u" 2>&1 | tail -2; then
      if [ -s "$out" ]; then
        echo "  成功: $out（$(du -h "$out" | cut -f1)）"
        return 0
      fi
    fi
    rm -f "$out"
  done
  echo "  !! 全部下载源失败: $out"
  return 1
}

echo "=============================================================="
echo " M4 素材下载    目标目录: $ASSETS"
echo "=============================================================="
echo

echo "[1/2] 模型 yolov8n.pt（约 6.4 MB）"
try_download "$MODEL" "${MODEL_URLS[@]}"
echo
echo "[2/2] 测试图 bus.jpg（约 140 KB，810x1080）"
try_download "$PIC" "${PIC_URLS[@]}"
echo

echo "=============================================================="
echo " 结果"
echo "=============================================================="
ls -l "$ASSETS" 2>&1 | sed 's/^/  /'
echo
if [ -s "$MODEL" ] && [ -s "$PIC" ]; then
  echo "  ✅ 素材齐全"
  python3 -c "
import cv2, sys
i = cv2.imread('$PIC')
print('    测试图尺寸:', None if i is None else i.shape)
" 2>/dev/null
else
  echo "  ❌ 素材不全 —— demo.sh 会提前报错并告诉你缺什么"
  exit 1
fi
