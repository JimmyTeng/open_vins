#!/usr/bin/env bash
#
# deveping1 测试脚本：一键 configure + build + ctest（可选运行 sandbox）
#
# 用法：
#   bash script/test_deveping1.sh [preset] [--clean] [-j|--jobs N] [--run-sandbox] [--perf] [--run-perf] [--perf-iters N]
#
# 示例：
#   bash script/test_deveping1.sh x64-debug-vcpkg --clean -j 8 --run-sandbox
#   bash script/test_deveping1.sh x64-debug
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

die() {
  echo "错误：$*" >&2
  exit 1
}

has_cmd() {
  command -v "$1" >/dev/null 2>&1
}

resolve_build_dir() {
  # 从 CMakePresets.json 解析 binaryDir，避免脚本硬编码目录约定
  local preset="$1"
  local src_dir="$2"

  if ! has_cmd python3; then
    return 1
  fi

  local presets_file="${src_dir}/CMakePresets.json"
  [[ -f "${presets_file}" ]] || return 1

  python3 -c $'import json,sys,pathlib\npreset=sys.argv[1]\nsrc_dir=sys.argv[2]\np=pathlib.Path(src_dir)/"CMakePresets.json"\ndata=json.loads(p.read_text(encoding="utf-8"))\npresets=data.get("configurePresets", [])\nbinary_dir_template=None\nfor item in presets:\n    if isinstance(item, dict) and item.get("name")==preset:\n        binary_dir_template=item.get("binaryDir")\n        break\nif not binary_dir_template:\n    sys.exit(2)\nout=binary_dir_template.replace("${sourceDir}", src_dir).replace("${presetName}", preset)\nprint(out)\n' "${preset}" "${src_dir}"
}

pick_default_preset() {
  # 优先 vcpkg 预设，其次 system 预设；找不到则报错
  if ! has_cmd python3 || [[ ! -f "${PROJECT_ROOT}/CMakePresets.json" ]]; then
    die "未找到 CMakePresets.json 或缺少 python3，请显式传入 preset：bash script/test_deveping1.sh <preset>"
  fi

  python3 - <<'PY'
import json, pathlib
p = pathlib.Path("CMakePresets.json")
data = json.loads(p.read_text(encoding="utf-8"))
presets = [x.get("name") for x in data.get("configurePresets", []) if isinstance(x, dict) and x.get("name")]
for candidate in ("x64-debug-vcpkg", "x64-debug", "x64-release-vcpkg", "x64-release"):
    if candidate in presets:
        print(candidate)
        raise SystemExit(0)
raise SystemExit(2)
PY
}

has_preset() {
  local preset="$1"
  if ! has_cmd python3 || [[ ! -f "${PROJECT_ROOT}/CMakePresets.json" ]]; then
    return 1
  fi
  python3 - "${preset}" <<'PY'
import json, pathlib, sys
preset = sys.argv[1] if len(sys.argv) > 1 else ""
p = pathlib.Path("CMakePresets.json")
data = json.loads(p.read_text(encoding="utf-8"))
names = [x.get("name") for x in data.get("configurePresets", []) if isinstance(x, dict)]
sys.exit(0 if preset in names else 2)
PY
  return $?
}

prefer_release_preset_for_perf() {
  local preset="$1"
  # 约定：preset 名里包含 debug/release
  if [[ "${preset}" == *debug* || "${preset}" == *Debug* ]]; then
    local candidate
    candidate="${preset//debug/release}"
    candidate="${candidate//Debug/Release}"
    if has_preset "${candidate}"; then
      echo "${candidate}"
      return 0
    fi
  fi
  echo "${preset}"
  return 0
}

usage() {
  cat <<'EOF'
用法：
  bash script/test_deveping1.sh [preset] [选项]

选项：
  --clean         清理该 preset 的 build 目录
  -j, --jobs N    并行编译任务数（默认：nproc）
  --run-sandbox   额外运行一次 myvio_deveping1_sandbox
  --perf          额外构建 deveping1 性能基准（需要重新 configure）
  --run-perf      构建后运行性能基准（等价于 --perf + 执行可执行程序）
  --perf-iters N  性能基准迭代次数（默认：2000000）
  -h, --help      显示帮助

说明：
  - 脚本会强制开启：-DMYVIO_BUILD_DEVEPING=ON -DMYVIO_DEVEPING_PROJECTS=deveping1
  - 默认 ctest 运行：-R Deveping1Smoke（只跑 deveping1 的 smoke tests）
EOF
}

PRESET=""
JOBS="$(nproc 2>/dev/null || echo 4)"
CLEAN=false
RUN_SANDBOX=false
BUILD_PERF=false
RUN_PERF=false
PERF_ITERS="2000000"

ARGS=("$@")
if [[ ${#ARGS[@]} -gt 0 && "${ARGS[0]}" != "-"* ]]; then
  PRESET="${ARGS[0]}"
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean)
      CLEAN=true
      shift
      ;;
    -j|--jobs)
      [[ $# -ge 2 ]] || die "$1 需要一个参数（并行任务数）"
      JOBS="$2"
      [[ "${JOBS}" =~ ^[0-9]+$ ]] || die "--jobs 参数必须是整数：${JOBS}"
      shift 2
      ;;
    --run-sandbox)
      RUN_SANDBOX=true
      shift
      ;;
    --perf)
      BUILD_PERF=true
      shift
      ;;
    --run-perf)
      BUILD_PERF=true
      RUN_PERF=true
      shift
      ;;
    --perf-iters)
      [[ $# -ge 2 ]] || die "--perf-iters 需要一个参数（迭代次数）"
      PERF_ITERS="$2"
      [[ "${PERF_ITERS}" =~ ^[0-9]+$ ]] || die "--perf-iters 参数必须是整数：${PERF_ITERS}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "未知选项：$1"
      ;;
  esac
done

cd "${PROJECT_ROOT}"

if [[ -z "${PRESET}" ]]; then
  PRESET="$(pick_default_preset || true)"
  [[ -n "${PRESET}" ]] || die "无法自动选择默认 preset，请显式传入：bash script/test_deveping1.sh <preset>"
fi

if [[ "${BUILD_PERF}" == "true" ]]; then
  # 性能测试必须在 Release 版本执行：自动从 debug preset 切到 release preset（若存在）
  OLD_PRESET="${PRESET}"
  PRESET="$(prefer_release_preset_for_perf "${PRESET}")"
  if [[ "${PRESET}" != "${OLD_PRESET}" ]]; then
    echo "perf: auto switch preset ${OLD_PRESET} -> ${PRESET}"
  fi
fi

BUILD_DIR="$(resolve_build_dir "${PRESET}" "${PROJECT_ROOT}" || true)"
if [[ -z "${BUILD_DIR}" ]]; then
  die "无法解析 preset=${PRESET} 的 build 目录（请确认 preset 是否存在于 CMakePresets.json）"
fi

if [[ "${BUILD_DIR}" != "${PROJECT_ROOT}/build/"* || -z "${BUILD_DIR}" || "${BUILD_DIR}" == "/" ]]; then
  die "构建目录不安全：${BUILD_DIR}"
fi

if [[ "${CLEAN}" == "true" ]]; then
  echo "清理构建目录：${BUILD_DIR}"
  rm -rf "${BUILD_DIR}"
fi

echo "配置：cmake --preset ${PRESET} (deveping1 enabled)"
CONFIG_ARGS=(
  -DMYVIO_BUILD_DEVEPING=ON
  -DMYVIO_DEVEPING_PROJECTS=deveping1
)
if [[ "${BUILD_PERF}" == "true" ]]; then
  CONFIG_ARGS+=(-DMYVIO_BUILD_DEVEPING1_PERF_TESTS=ON)
fi

cmake --preset "${PRESET}" "${CONFIG_ARGS[@]}"

echo "编译：myvio_deveping1_sandbox / myvio_deveping1_smoke_test"
cmake --build "${BUILD_DIR}" -j "${JOBS}" --target \
  myvio_deveping1_sandbox \
  myvio_deveping1_smoke_test

echo "运行测试：ctest -R Deveping1Smoke"
ctest --test-dir "${BUILD_DIR}" -R Deveping1Smoke --output-on-failure

if [[ "${RUN_SANDBOX}" == "true" ]]; then
  SANDBOX_BIN="${BUILD_DIR}/src/deveping1/myvio_deveping1_sandbox"
  [[ -x "${SANDBOX_BIN}" ]] || die "未找到可执行文件：${SANDBOX_BIN}"
  echo "运行 sandbox：${SANDBOX_BIN}"
  "${SANDBOX_BIN}"
fi

if [[ "${BUILD_PERF}" == "true" ]]; then
  echo "编译：myvio_deveping1_perf_logger_benchmark"
  cmake --build "${BUILD_DIR}" -j "${JOBS}" --target myvio_deveping1_perf_logger_benchmark

  if [[ "${RUN_PERF}" == "true" ]]; then
    PERF_BIN="${BUILD_DIR}/src/deveping1/myvio_deveping1_perf_logger_benchmark"
    [[ -x "${PERF_BIN}" ]] || die "未找到可执行文件：${PERF_BIN}"
    echo "运行 perf：${PERF_BIN} ${PERF_ITERS}"
    "${PERF_BIN}" "${PERF_ITERS}"
  fi
fi

echo "OK: deveping1 tests passed."

