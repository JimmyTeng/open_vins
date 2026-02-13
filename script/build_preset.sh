#!/usr/bin/env bash

# 统一构建脚本：基于 CMake Presets（支持本机/交叉编译）
#
# 用法：
#   ./script/build_preset.sh <preset> [--clean] [-j|--jobs N] [--install] [--configure-only] [--build-only] [--target <t>] [--config <cfg>]
#   ./script/build_preset.sh --list-presets
#   ./script/build_preset.sh --completion-bash
#
# 示例：
#   ./script/build_preset.sh x64-debug-vcpkg --clean -j 8
#   ./script/build_preset.sh arm64-release-vcpkg --install
#   ./script/build_preset.sh x64-release --clean -j 8

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

  python3 -c $'import json,sys,pathlib\npreset=sys.argv[1]\nsrc_dir=sys.argv[2]\np=pathlib.Path(src_dir)/"CMakePresets.json"\ndata=json.loads(p.read_text(encoding="utf-8"))\npresets=list(data.get("configurePresets", []))\nfor inc in data.get("include", []):\n  inc_path=pathlib.Path(src_dir)/inc\n  inc_data=json.loads(inc_path.read_text(encoding="utf-8"))\n  presets.extend(inc_data.get("configurePresets", []))\nbinary_dir_template=None\nfor item in presets:\n  if isinstance(item, dict) and item.get("name")==preset:\n    binary_dir_template=item.get("binaryDir")\n    break\nif not binary_dir_template:\n  sys.exit(2)\nout=binary_dir_template.replace("${sourceDir}", src_dir).replace("${presetName}", preset)\nprint(out)\n' "${preset}" "${src_dir}"
}

list_presets() {
  cd "${PROJECT_ROOT}"

  if has_cmd cmake; then
    # CMake 3.19+ 支持 --list-presets；3.20+ 严格匹配本仓库 cmakeMinimumRequired
    if cmake --list-presets >/dev/null 2>&1; then
      cmake --list-presets
      return 0
    fi
  fi

  # 兜底：仅用于“列出 preset 名称”，不保证等价于 CMake 的显示格式（支持 include）
  if has_cmd python3 && [[ -f "${PROJECT_ROOT}/CMakePresets.json" ]]; then
    python3 - <<'PY'
import json, pathlib
root = pathlib.Path("CMakePresets.json")
data = json.loads(root.read_text(encoding="utf-8"))
presets = list(data.get("configurePresets", []))
for inc in data.get("include", []):
  inc_path = root.parent / inc
  inc_data = json.loads(inc_path.read_text(encoding="utf-8"))
  presets.extend(inc_data.get("configurePresets", []))
names = [x.get("name") for x in presets if isinstance(x, dict) and x.get("name")]
print("Available configure presets:")
for n in names:
  print(f"  {n}")
PY
    return 0
  fi

  die "无法列出 presets：需要可用的 cmake（建议 >= 3.20）或 python3。"
}

usage() {
  cat <<'EOF'
用法:
  ./script/build_preset.sh <preset> [选项]
  ./script/build_preset.sh --list-presets
  ./script/build_preset.sh --completion-bash

选项:
  --clean               清理 build/<preset> 目录
  -j, --jobs N          并行编译任务数（默认：nproc）
  --install             在 build 后执行 cmake --install
  --configure-only      只做 configure（不 build）
  --build-only          只做 build（要求 build/<preset> 已存在且已配置）
  --target <name>       仅构建指定 target（等价于 cmake --build ... --target <name>）
  --config <cfg>        多配置生成器时指定配置（Debug/Release/...），会透传给 --build/--install 的 --config
  --list-presets        列出 CMakePresets.json 中可用的 presets
  --completion-bash     输出 bash tab 补全脚本；当前 shell 启用：source <(./script/build_preset.sh --completion-bash)；永久启用：将上述 source 行加入 ~/.bashrc
  -h, --help            显示帮助

说明:
  - 需要 CMake 支持 Presets（本仓库要求 CMake >= 3.20，见 CMakePresets.json 的 cmakeMinimumRequired）。
  - 仅当 preset 为 vcpkg / 或以 -vcpkg 结尾时，才要求设置 VCPKG_ROOT（与 CMakePresets.json 一致）。
EOF
}

print_completion_bash() {
  # 用法（任选其一）：
  #   当前 shell 启用： source <(./script/build_preset.sh --completion-bash)
  #   永久启用：在 ~/.bashrc 末尾加一行（路径按实际修改）：
  #     source <("/path/to/myvio/script/build_preset.sh" --completion-bash 2>/dev/null) || true
  #
  # 只做 bash 补全；zsh 用户可用 bashcompinit 兼容。
  local script_abs="${SCRIPT_DIR}/build_preset.sh"
  if command -v realpath >/dev/null 2>&1; then
    script_abs="$(realpath "${script_abs}" 2>/dev/null || echo "${script_abs}")"
  elif command -v readlink >/dev/null 2>&1; then
    script_abs="$(readlink -f "${script_abs}" 2>/dev/null || echo "${script_abs}")"
  fi

  cat <<EOF
_myvio_build_preset__list_presets_names() {
  local cmd script_path script_dir project_root
  cmd="\${1:-}"
  [[ -n "\${cmd}" ]] || return 1

  if [[ "\${cmd}" == */* ]]; then
    script_path="\${cmd}"
  else
    script_path="\$(command -v "\${cmd}" 2>/dev/null || true)"
  fi
  [[ -n "\${script_path}" ]] || return 1

  if command -v realpath >/dev/null 2>&1; then
    script_path="\$(realpath "\${script_path}" 2>/dev/null || echo "\${script_path}")"
  elif command -v readlink >/dev/null 2>&1; then
    script_path="\$(readlink -f "\${script_path}" 2>/dev/null || echo "\${script_path}")"
  fi

  script_dir="\$(cd "\$(dirname "\${script_path}")" && pwd)"
  project_root="\$(cd "\${script_dir}/.." && pwd)"

  if command -v python3 >/dev/null 2>&1 && [[ -f "\${project_root}/CMakePresets.json" ]]; then
    python3 -c '
import json, pathlib, sys
src_dir = sys.argv[1]
p = pathlib.Path(src_dir) / "CMakePresets.json"
d = json.loads(p.read_text(encoding="utf-8"))
ps = list(d.get("configurePresets", []))
for inc in d.get("include", []):
  inc_path = pathlib.Path(src_dir) / inc
  inc_d = json.loads(inc_path.read_text(encoding="utf-8"))
  ps.extend(inc_d.get("configurePresets", []))
print(" ".join([x.get("name") for x in ps if isinstance(x, dict) and x.get("name")]))
' "\${project_root}" 2>/dev/null | tr '\\n' ' '
    return 0
  fi
  return 1
}

_myvio_build_preset_completions() {
  local cur prev
  cur="\${COMP_WORDS[COMP_CWORD]}"
  prev="\${COMP_WORDS[COMP_CWORD-1]}"

  local opts="--clean -j --jobs --install --configure-only --build-only --target --config --list-presets --list --completion-bash -h --help"

  # 第一个参数：补全 preset 或全局开关
  if [[ \${COMP_CWORD} -eq 1 ]]; then
    local presets=""
    presets="\$(_myvio_build_preset__list_presets_names "\${COMP_WORDS[0]}" 2>/dev/null || true)"
    COMPREPLY=( \$(compgen -W "\${opts} \${presets}" -- "\${cur}") )
    return 0
  fi

  # 选项参数位补全（简单处理）
  if [[ "\${prev}" == "--target" ]]; then
    # target 名太多且依赖 build dir，这里不做枚举，让用户手输
    COMPREPLY=()
    return 0
  fi
  if [[ "\${prev}" == "--config" ]]; then
    COMPREPLY=( \$(compgen -W "Debug Release RelWithDebInfo MinSizeRel" -- "\${cur}") )
    return 0
  fi
  if [[ "\${prev}" == "-j" || "\${prev}" == "--jobs" ]]; then
    # 补全常见并行数
    COMPREPLY=( \$(compgen -W "1 2 4 8 16 32" -- "\${cur}") )
    return 0
  fi

  COMPREPLY=( \$(compgen -W "\${opts}" -- "\${cur}") )
  return 0
}

complete -F _myvio_build_preset_completions build_preset.sh ./script/build_preset.sh script/build_preset.sh "${script_abs}"
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

if [[ "${1:-}" == "--completion-bash" ]]; then
  print_completion_bash
  exit 0
fi

if [[ "${1:-}" == "--list-presets" || "${1:-}" == "--list" ]]; then
  list_presets
  exit 0
fi

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

PRESET="${1:-}"
shift || true

if [[ -z "${PRESET}" || "${PRESET}" == -* ]]; then
  echo "" >&2
  usage >&2
  exit 1
fi

CLEAN=false
INSTALL=false
CONFIGURE_ONLY=false
BUILD_ONLY=false
TARGET=""
CONFIG=""
JOBS="$(nproc 2>/dev/null || echo 4)"

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
    --install)
      INSTALL=true
      shift
      ;;
    --configure-only)
      CONFIGURE_ONLY=true
      shift
      ;;
    --build-only)
      BUILD_ONLY=true
      shift
      ;;
    --target)
      [[ $# -ge 2 ]] || die "--target 需要一个参数（target 名称）"
      TARGET="$2"
      shift 2
      ;;
    --config)
      [[ $# -ge 2 ]] || die "--config 需要一个参数（Debug/Release/...）"
      CONFIG="$2"
      shift 2
      ;;
    --list-presets|--list)
      list_presets
      exit 0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "未知选项: $1"
      ;;
  esac
done

cd "${PROJECT_ROOT}"

# 解析 preset 对应的 build 目录（binaryDir）；解析失败则退回到旧约定
BUILD_DIR="$(resolve_build_dir "${PRESET}" "${PROJECT_ROOT}" || true)"
if [[ -z "${BUILD_DIR}" ]]; then
  BUILD_DIR="${PROJECT_ROOT}/build/${PRESET}"
fi

# 基础安全防护：避免误删/误写到奇怪路径
if [[ "${BUILD_DIR}" != "${PROJECT_ROOT}/build/"* || -z "${BUILD_DIR}" || "${BUILD_DIR}" == "/" ]]; then
  die "构建目录不安全：${BUILD_DIR}"
fi

is_vcpkg_preset=false
if [[ "${PRESET}" == "vcpkg" || "${PRESET}" == *-vcpkg ]]; then
  is_vcpkg_preset=true
fi

if [[ "${PRESET}" == arm64-*-vcpkg ]]; then
  # 交叉编译 ARM64 时，LAPACK/OpenBLAS/Ceres 等通常需要目标架构的 gfortran
  for tool in aarch64-linux-gnu-gcc aarch64-linux-gnu-g++ aarch64-linux-gnu-gfortran; do
    if ! has_cmd "${tool}"; then
      die "缺少交叉编译工具：${tool}。Ubuntu/WSL 可执行：sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu gfortran-aarch64-linux-gnu"
    fi
  done
fi

if [[ "${is_vcpkg_preset}" == "true" ]]; then
  if [[ -z "${VCPKG_ROOT:-}" ]]; then
    # 与 CMakePresets.json 的默认值保持一致（仓库自带 vcpkg）
    VCPKG_ROOT="${PROJECT_ROOT}/build-tool/vcpkg"
    export VCPKG_ROOT
  fi

  if [[ ! -f "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" ]]; then
    die "未找到 vcpkg toolchain：${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake（请检查 VCPKG_ROOT）"
  fi

  # 确保 vcpkg 缓存目录存在（preset 中 VCPKG_DOWNLOADS 指向此处，否则 bootstrap 会报错）
  rbs_cache_dir="${RBS_VCPKG_CACHE_DIR:-$HOME/.cache/vcpkg-rbs}"
  if [[ ! -d "${rbs_cache_dir}/downloads" ]]; then
    mkdir -p "${rbs_cache_dir}/downloads"
    echo "已创建 vcpkg 缓存目录：${rbs_cache_dir}/downloads"
  fi
fi

if [[ "${CLEAN}" == "true" ]]; then
  echo "清理构建目录：${BUILD_DIR}"
  rm -rf "${BUILD_DIR}"
fi

if [[ "${BUILD_ONLY}" != "true" ]]; then
  echo "配置（configure preset）：${PRESET}"
  # 若 CMake 版本过旧不支持 presets，这里会直接报错；提示用户升级
  if ! cmake --preset "${PRESET}"; then
    echo "" >&2
    echo "提示：可能原因包括：CMake 版本过旧（本仓库要求 >= 3.20）、preset 名不存在、或环境依赖未满足。" >&2
    echo "" >&2
    echo "可用 presets（若可列出）：" >&2
    (list_presets || true) >&2
    exit 3
  fi
fi

if [[ "${CONFIGURE_ONLY}" == "true" ]]; then
  exit 0
fi

if [[ ! -d "${BUILD_DIR}" ]]; then
  die "构建目录不存在：${BUILD_DIR}（请先执行：./script/build_preset.sh ${PRESET}，或去掉 --build-only）"
fi

echo "编译（build dir）：${BUILD_DIR}"
BUILD_ARGS=()
if [[ -n "${CONFIG}" ]]; then
  BUILD_ARGS+=(--config "${CONFIG}")
fi
if [[ -n "${TARGET}" ]]; then
  cmake --build "${BUILD_DIR}" -j "${JOBS}" --target "${TARGET}" "${BUILD_ARGS[@]}"
else
  cmake --build "${BUILD_DIR}" -j "${JOBS}" "${BUILD_ARGS[@]}"
fi

if [[ "${INSTALL}" == "true" ]]; then
  echo "安装（install）：${BUILD_DIR}"
  if [[ -n "${CONFIG}" ]]; then
    cmake --install "${BUILD_DIR}" --config "${CONFIG}"
  else
    cmake --install "${BUILD_DIR}"
  fi
fi

