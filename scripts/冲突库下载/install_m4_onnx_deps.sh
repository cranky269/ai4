#!/usr/bin/env bash
# Add only ONNX Runtime to the existing isolated M4 dependencies.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
target="$root/lib/m4_onnx"
check() {
  PYTHONPATH="$root/lib/m4:$target" /usr/bin/python3 -B - <<'PY'
import onnx, onnxruntime, numpy, sympy, packaging, coloredlogs, flatbuffers, google.protobuf
assert onnxruntime.__version__ == '1.19.2'
print('ORT', onnxruntime.__version__, onnxruntime.__file__)
print('numpy', numpy.__version__, numpy.__file__)
PY
}
if [[ ${1:-} == --check ]]; then check; exit; fi
mkdir -p "$root/lib"
exec 9>"$root/lib/.m4_onnx.install.lock"
flock -n 9 || { echo "另一个 M4 ONNX 安装正在运行" >&2; exit 1; }
if [[ -e "$target" ]]; then
  echo "已有 $target；请使用 --check 检查，或在新副本中准备依赖。" >&2
  exit 1
fi
mkdir -p "$root/lib"
stage=$(mktemp -d "$root/lib/m4_onnx.pending.XXXXXX")
printf 'Dependency staging directory: %s\n' "$stage"
options=()
if [[ ${1:-} == --wheelhouse ]]; then
  [[ -d ${2:-} ]] || { echo '缺少 wheelhouse 目录' >&2; exit 1; }
  options=(--no-index --find-links "$2")
fi
bootstrap=$(mktemp -d "$root/lib/m4_pip.pending.XXXXXX")
/usr/bin/python3 -m pip download "${options[@]}" --no-deps --dest "$bootstrap" pip==24.3.1
wheel="$bootstrap/pip-24.3.1-py3-none-any.whl"
[[ -f "$wheel" ]] || { echo '未取得隔离 pip wheel' >&2; exit 1; }
PYTHONPATH="$wheel" /usr/bin/python3 -m pip install --target "$stage" --no-deps "${options[@]}"   onnx==1.16.2 onnxruntime==1.19.2 coloredlogs==15.0.1 humanfriendly==10.0 flatbuffers==24.3.25 protobuf==4.25.8
PYTHONPATH="$root/lib/m4:$stage" /usr/bin/python3 -B -c 'import onnx,onnxruntime,numpy,sympy,packaging,coloredlogs,flatbuffers,google.protobuf; assert onnxruntime.__version__=="1.19.2"'
# Publish the validated directory while holding the installer lock.
/usr/bin/python3 - "$stage" "$target" <<'PY'
import os, sys
if os.path.exists(sys.argv[2]):
    raise SystemExit('Dependency target appeared during installation; staging directory preserved')
os.rename(sys.argv[1], sys.argv[2])
PY
check
