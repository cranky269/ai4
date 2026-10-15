#!/bin/bash
# =============================================================================
# M4 视觉依赖安装脚本（组长批准方案：装到工程内 lib/m4_deps，严格隔离）
#
# 用法：
#   bash install_m4_deps.sh            # 安装（约下载 300MB，装完 1.1GB）
#   bash install_m4_deps.sh --check    # 只检查现状，不安装
#   bash install_m4_deps.sh --uninstall# 删掉 lib/m4_deps（系统不受影响）
#
# 设计约束（**不可破坏**）：
#   1. numpy 锁 1.21.6，且**只装在 m4_deps 里**；系统 numpy 1.17.4 一动不动；
#   2. torch 用 **CPU 版**（仿真机无 CUDA，避免拉几百 MB 无用数据）；
#   3. **不装 opencv-python**（否则遮蔽系统 cv2，与 cv_bridge 版本错配）；
#   4. 装完必须打印隔离自检结果，并**逐包验证**（缺一个就报错）。
#
# 踩过的坑（详见 M4交付说明/组长确认冲突库下载说明文档.md §3.1）：
#   ① 清华 PyPI 镜像会返回 403 → 本脚本用阿里云
#   ② pip 默认给 torch 拉 CUDA 版 → 本脚本用 pytorch 官方 CPU 索引
#   ③ pip --target 不把目标目录当"已安装"，且遇同名目录会**静默不覆盖**
#      → 全程 --no-deps 分步装，numpy 放最后并先清目录
#   ④ opencv-python 遮蔽系统 cv2 → 装完主动删除
#   ⑤ 网络会 ECONNRESET（实测）→ 加 pip 重试 + shell 层重试
#   ⑥ **本脚本曾有的真 bug**：`pip ... | tail` 会被 tail 的退出码掩盖，
#      导致"装失败了仍报 OK" → 改为 run_pip 取真实退出码 + 逐包验证
# =============================================================================

set -u

# --- 路径：脚本自己定位工程根，不写死 ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIR="$SCRIPT_DIR/catkin_ws/src/uav_tracking/lib/m4_deps"

# --- 镜像（可用环境变量覆盖）---
PYPI="${M4_PYPI_INDEX:-https://mirrors.aliyun.com/pypi/simple/}"
TORCH_INDEX="${M4_TORCH_INDEX:-https://download.pytorch.org/whl/cpu}"

NUMPY_VER="1.21.6"
TORCH_VER="2.4.1+cpu"
TV_VER="0.19.1+cpu"

# 装完必须能导入的包（缺一个即失败）
REQUIRED_MODULES="numpy torch torchvision ultralytics matplotlib polars psutil yaml PIL requests"

# 注意：**保留 pip 缓存**（不要 --no-cache-dir）——
# 本工程所在网络实测会 ECONNRESET，保留缓存后重跑本脚本不必重新下载。
PIP_COMMON=(--target="$DIR" --disable-pip-version-check
            --no-warn-script-location --retries 5 --timeout 60)

MODE="install"
[ $# -ge 1 ] && MODE="$1"

hr() { echo "------------------------------------------------------------"; }
ok() { echo "  [OK]   $*"; }
warn() { echo "  [注意] $*"; }
bad() { echo "  [失败] $*"; }

# --------------------------------------------------------------------------- #
# 跑 pip 并**返回真实退出码**（不再被 tail 掩盖）
run_pip() {
  local log rc
  log="$(mktemp)"
  python3 -m pip install "${PIP_COMMON[@]}" "$@" > "$log" 2>&1
  rc=$?
  tail -4 "$log"
  rm -f "$log"
  return $rc
}

# 网络抖动重试（实测会遇到 ECONNRESET）
pip_retry() {
  local n=1
  while [ "$n" -le 3 ]; do
    run_pip "$@" && return 0
    warn "第 $n 次失败（可能是网络抖动），5 秒后重试…"
    n=$((n + 1))
    sleep 5
  done
  return 1
}

# 逐包验证（**光看 pip 成功不算数**）
verify_modules() {
  PYTHONPATH="$DIR" python3 - "$REQUIRED_MODULES" <<'PY'
import importlib, sys
missing = []
for m in sys.argv[1].split():
    try:
        importlib.import_module(m)
    except Exception:
        missing.append(m)
if missing:
    print("  缺失: " + " ".join(missing))
    sys.exit(1)
print("  全部可导入: " + sys.argv[1])
PY
}

# --------------------------------------------------------------------------- #
check_now() {
  echo "=== 当前状态 ==="
  echo "隔离目录: $DIR"
  if [ -d "$DIR" ]; then
    echo "  大小: $(du -sh "$DIR" 2>/dev/null | cut -f1)"
  else
    warn "隔离目录不存在"
  fi
  echo
  echo "--- 系统 python3（应始终是 1.17.4）---"
  python3 -c "import numpy;print('  numpy',numpy.__version__,'@',numpy.__file__)" 2>&1 | sed 's/^/  /'
  python3 -c "import cv2;print('  cv2  ',cv2.__version__,'@',cv2.__file__)" 2>&1 | sed 's/^/  /'
  echo
  echo "--- 隔离目录内（装了才显示）---"
  if [ -d "$DIR" ]; then
    PYTHONPATH="$DIR" python3 -c "
for m in ['numpy','torch','torchvision','ultralytics','matplotlib','cv2']:
    try:
        mod=__import__(m); print('  %-12s %-11s %s'%(m,getattr(mod,'__version__','?'),getattr(mod,'__file__','?')))
    except Exception as e:
        print('  %-12s 不可导入 (%s)'%(m,type(e).__name__))
" 2>&1
  fi
}

# --------------------------------------------------------------------------- #
do_uninstall() {
  echo "=== 卸载 ==="
  if [ -d "$DIR" ]; then
    du -sh "$DIR" 2>/dev/null | sed 's/^/  删除前: /'
    rm -rf "$DIR"
    ok "已删除 $DIR"
    echo "  （系统 numpy / cv2 从未被改动，无需恢复）"
  else
    warn "目录不存在，无需删除"
  fi
}

# --------------------------------------------------------------------------- #
do_install() {
  echo "=== 0/6 环境检查 ==="
  which python3 >/dev/null 2>&1 || { bad "找不到 python3"; exit 1; }
  python3 -m pip --version >/dev/null 2>&1 || { bad "python3 没有 pip"; exit 1; }
  echo "  python3: $(python3 --version 2>&1)"
  echo "  pip    : $(python3 -m pip --version 2>&1 | cut -d' ' -f1-2)"
  echo "  磁盘可用: $(df -h "$SCRIPT_DIR" | tail -1 | awk '{print $4}')（需要约 2 GB）"
  echo "  PyPI 镜像: $PYPI"

  local SYS_NP_BEFORE
  SYS_NP_BEFORE="$(python3 -c "import numpy;print(numpy.__version__)" 2>/dev/null || echo '未知')"
  echo "  系统 numpy（安装前）: $SYS_NP_BEFORE"
  ok "环境可用"

  mkdir -p "$DIR"

  hr
  echo "=== 1/6 torch + torchvision（CPU 版，走 pytorch 官方索引）==="
  echo "  （约 195 MB，最耗时的一步）"
  if ! pip_retry --index-url "$TORCH_INDEX" "torch==$TORCH_VER" "torchvision==$TV_VER"; then
    bad "torch 安装失败"; exit 1
  fi
  [ -d "$DIR/torch" ] && ok "torch 已就位" || { bad "torch 目录不存在"; exit 1; }

  hr
  echo "=== 2/6 ultralytics 本体（--no-deps，防止它覆盖 CPU 版 torch）==="
  if ! pip_retry --index-url "$PYPI" --no-deps ultralytics; then
    bad "ultralytics 安装失败"; exit 1
  fi
  [ -d "$DIR/ultralytics" ] && ok "ultralytics 已就位" || { bad "ultralytics 目录不存在"; exit 1; }

  hr
  echo "=== 3/6 ultralytics 的运行时依赖（显式列出，不含 torch/numpy/opencv）==="
  if ! pip_retry --index-url "$PYPI" \
      "matplotlib>=3.3.0" "pillow>=7.1.2" "pyyaml>=5.3.1" "requests>=2.23.0" \
      "psutil>=5.8.0" "polars>=0.20.0" "nvidia-ml-py>=12.0.0"; then
    bad "运行时依赖安装失败（网络？可重跑本脚本，已装的不会重下）"; exit 1
  fi
  ok "运行时依赖已装"

  hr
  echo "=== 4/6 ultralytics-thop（同样 --no-deps，它依赖 torch）==="
  if ! pip_retry --index-url "$PYPI" --no-deps "ultralytics-thop>=2.2.2"; then
    bad "ultralytics-thop 安装失败"; exit 1
  fi
  ok "ultralytics-thop 已装"

  hr
  echo "=== 5/6 numpy 锁到 $NUMPY_VER（放最后 + 先清目录，否则会静默不覆盖）==="
  rm -rf "$DIR"/numpy "$DIR"/numpy.libs "$DIR"/numpy-*.dist-info
  if ! pip_retry --index-url "$PYPI" --no-deps --upgrade "numpy==$NUMPY_VER"; then
    bad "numpy 安装失败"; exit 1
  fi
  local nv
  nv="$(PYTHONPATH="$DIR" python3 -c "import numpy;print(numpy.__version__)" 2>/dev/null)"
  if [ "$nv" = "$NUMPY_VER" ]; then
    ok "numpy = $nv"
  else
    bad "numpy = $nv（期望 $NUMPY_VER）—— 常见原因：--target 遇同名目录未覆盖，请重跑"
    exit 1
  fi

  hr
  echo "=== 6/6 剔除 opencv（否则遮蔽系统 cv2，与 cv_bridge 错配）==="
  if ls -d "$DIR"/cv2* "$DIR"/opencv_python* >/dev/null 2>&1; then
    rm -rf "$DIR"/cv2 "$DIR"/cv2-* "$DIR"/opencv_python* "$DIR"/opencv_python.libs
    ok "已删除 m4_deps 内的 opencv"
  else
    ok "本来就没有 opencv（无需处理）"
  fi

  hr
  echo "=== 逐包验证（缺任何一个都算失败）==="
  if ! verify_modules; then
    bad "有包缺失 —— 多为网络中断导致，可直接重跑本脚本（已装的不重下）"
    exit 1
  fi
  ok "依赖完整"

  hr
  echo "=== 隔离自检 ==="
  check_now

  hr
  echo "=== 结论 ==="
  local sys_np mdep_np sys_cv
  sys_np="$(python3 -c "import numpy;print(numpy.__version__)" 2>/dev/null)"
  mdep_np="$(PYTHONPATH="$DIR" python3 -c "import numpy;print(numpy.__version__)" 2>/dev/null)"
  sys_cv="$(python3 -c "import cv2;print(cv2.__version__)" 2>/dev/null)"

  echo "  系统 numpy : $sys_np   （应保持 $SYS_NP_BEFORE，未被改动）"
  echo "  隔离 numpy : $mdep_np  （必须是 $NUMPY_VER）"
  echo "  系统 cv2   : $sys_cv   （必须仍是系统版本）"
  echo
  if [ "$sys_np" = "$SYS_NP_BEFORE" ] && [ "$mdep_np" = "$NUMPY_VER" ]; then
    echo "  ✅ 隔离成功：系统包未受影响，依赖完整"
    echo
    echo "  下一步（真跑一次检测）："
    echo "    bash $SCRIPT_DIR/test/detector/e2e_test/run_e2e.sh --help 2>/dev/null || true"
    echo "    MODEL_PATH=<你的.pt> IMAGE=<你的图> \\"
    echo "        bash $SCRIPT_DIR/test/detector/e2e_test/run_e2e.sh"
  else
    echo "  ❌ 隔离异常，请把上面的输出发给 M4 负责人"
    exit 1
  fi
}

# --------------------------------------------------------------------------- #
echo "============================================================"
echo " M4 视觉依赖安装脚本"
echo " 隔离目录: $DIR"
echo " 模式: $MODE"
echo "============================================================"
echo

case "$MODE" in
  --check|-c)     check_now ;;
  --uninstall|-u) do_uninstall ;;
  install|--install|"") do_install ;;
  *) echo "未知参数: $MODE"; echo "用法: bash $0 [--check|--uninstall]"; exit 2 ;;
esac
