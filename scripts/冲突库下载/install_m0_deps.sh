#!/usr/bin/env bash
# Rebuild only the M0 isolated packages; never install into the system Python.
set -eo pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ "${1:-}" == --check ]]; then
  exec bash "$REPO/catkin_ws/src/uav_sim/scripts/start_mounted_radar.sh" --check-env
fi
if [[ $# -gt 0 ]]; then
  echo '用法：bash scripts/冲突库下载/install_m0_deps.sh [--check]' >&2
  exit 2
fi
if [[ -d "$REPO/lib/venv/sensor_python" ]]; then
  echo '隔离依赖已存在；补齐缺失的回归测试依赖，不升级现有包。' >&2
  if ! PYTHONPATH="$REPO/lib/venv/sensor_python${PYTHONPATH:+:$PYTHONPATH}" /usr/bin/python3 -c 'import pytest' 2>/dev/null; then
    /usr/bin/python3 -m pip install --target "$REPO/lib/venv/sensor_python" -r "$REPO/lib/venv/requirements.txt"
  fi
else
  /usr/bin/python3 -m pip install --target "$REPO/lib/venv/sensor_python" -r "$REPO/lib/venv/requirements.txt"
fi
exec bash "$REPO/catkin_ws/src/uav_sim/scripts/start_mounted_radar.sh" --check-env
