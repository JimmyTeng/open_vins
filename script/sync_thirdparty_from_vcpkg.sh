#!/usr/bin/env bash
# 从 vcpkg_installed 同步依赖库到 install/.../thirdparty/lib，供 run.sh 运行时加载
# 静态链接构建时：vcpkg 仅含 .a，无 .so，故只同步系统库（如 libgfortran）；动态链接时则含 OpenCV/Ceres 等。
# 使用 -L 解引用符号链接，拷贝实体文件，便于通过 FAT32 的 SD 卡转存（FAT32 不支持符号链接）。
#
# 用法：
#   ./script/sync_thirdparty_from_vcpkg.sh [preset]
#   ./script/sync_thirdparty_from_vcpkg.sh arm64-release-vcpkg-docker
#   ./script/sync_thirdparty_from_vcpkg.sh arm64-release-vcpkg
#   （x64 本机若用系统依赖 preset，无 vcpkg_installed，请勿用本脚本）
# 若不传 preset，则根据 install 目录推断（仅支持单一 vcpkg install 目录存在时）

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# preset -> (build_dir, install_prefix, triplet)
case "${1:-}" in
  arm64-release-vcpkg-docker)
    BUILD_DIR="${ROOT}/build/aarch64/Release-docker/arm64-release-vcpkg-docker"
    INSTALL_PREFIX="${ROOT}/install/aarch64/Release-docker"
    TRIPLET="arm64-linux-custom"
    ;;
  arm64-release-vcpkg)
    BUILD_DIR="${ROOT}/build/aarch64/Release/arm64-release-vcpkg"
    INSTALL_PREFIX="${ROOT}/install/aarch64/Release"
    TRIPLET="arm64-linux-custom"
    ;;
  x64-release-vcpkg-docker)
    BUILD_DIR="${ROOT}/build/x86_64/Release-docker/x64-release-vcpkg-docker"
    INSTALL_PREFIX="${ROOT}/install/x86_64/Release-docker"
    TRIPLET="x64-linux-custom-release"
    ;;
  "")
    # 自动推断：查找最近有 vcpkg_installed 的 build 目录
    for cand in \
      "${ROOT}/build/aarch64/Release-docker/arm64-release-vcpkg-docker" \
      "${ROOT}/build/aarch64/Release/arm64-release-vcpkg" \
      "${ROOT}/build/x86_64/Release-docker/x64-release-vcpkg-docker"; do
      if [[ -d "${cand}/vcpkg_installed" ]]; then
        BUILD_DIR="$cand"
        if [[ "$cand" == *arm64-release-vcpkg-docker* ]]; then
          INSTALL_PREFIX="${ROOT}/install/aarch64/Release-docker"
          TRIPLET="arm64-linux-custom"
        elif [[ "$cand" == *arm64-release-vcpkg* ]]; then
          INSTALL_PREFIX="${ROOT}/install/aarch64/Release"
          TRIPLET="arm64-linux-custom"
        elif [[ "$cand" == *x64-release-vcpkg-docker* ]]; then
          INSTALL_PREFIX="${ROOT}/install/x86_64/Release-docker"
          TRIPLET="x64-linux-custom-release"
        fi
        break
      fi
    done
    if [[ -z "${BUILD_DIR:-}" ]]; then
      echo "错误：未指定 preset 且未找到 vcpkg_installed。请先构建或指定 preset，例如：" >&2
      echo "  ./script/sync_thirdparty_from_vcpkg.sh arm64-release-vcpkg-docker" >&2
      exit 1
    fi
    echo "自动推断：BUILD_DIR=$BUILD_DIR, INSTALL_PREFIX=$INSTALL_PREFIX"
    ;;
  -h|--help)
    echo "用法: $0 [preset] [--full]"
    echo "preset: arm64-release-vcpkg-docker | arm64-release-vcpkg | x64-release-vcpkg-docker"
    echo "--full: 拷贝 vcpkg 全部 .so（默认仅拷贝 bin/lib 的传递依赖闭包以精简 thirdparty）"
    exit 0
    ;;
  *)
    echo "错误：未知 preset: $1" >&2
    exit 1
    ;;
esac

VCPKG_LIB="${BUILD_DIR}/vcpkg_installed/${TRIPLET}/lib"
THIRDPARTY="${INSTALL_PREFIX}/thirdparty/lib"

if [[ ! -d "${VCPKG_LIB}" ]]; then
  echo "错误：vcpkg lib 目录不存在: ${VCPKG_LIB}" >&2
  echo "请先完成 vcpkg 构建（例如：./script/build_arm64_docker.sh 或 cmake --build --preset <preset>）" >&2
  exit 1
fi

# 先清空目标目录，避免残留旧的符号链接
rm -rf "${THIRDPARTY}"
mkdir -p "${THIRDPARTY}"

# 根据 triplet 选择 readelf 与系统库搜索路径（LAPACK 依赖 libgfortran）
# 使用 LC_ALL=C 保证 readelf 输出英文 "Shared library: [...]"，避免中文 locale 下解析失败
READELF="env LC_ALL=C readelf"
SYSROOT_LIBS=("/lib/x86_64-linux-gnu" "/usr/lib/x86_64-linux-gnu" "/lib" "/usr/lib")
if [[ "${TRIPLET}" == "arm64-linux-custom" ]]; then
  READELF="env LC_ALL=C aarch64-linux-gnu-readelf"
  SYSROOT_LIBS=("/usr/aarch64-linux-gnu/lib" "/lib/aarch64-linux-gnu" "/usr/lib/aarch64-linux-gnu")
  for gccdir in /usr/lib/gcc/aarch64-linux-gnu/*/ /usr/lib/gcc-cross/aarch64-linux-gnu/*/; do
    [[ -d "$gccdir" ]] && SYSROOT_LIBS+=("$gccdir")
  done
fi

# 1. 拷贝 vcpkg 依赖：仅拷贝 bin/* 与 lib/*.so 的传递依赖闭包，避免多余 OpenCV contrib 等
# 使用 --full 可恢复旧行为（拷贝 vcpkg 全部 .so）
COPY_MODE="minimal"
[[ "${2:-}" == "--full" ]] && COPY_MODE="full"

SKIP_SYSTEM_LIBS="ld-linux libc.so libm.so libpthread libdl.so librt.so"
resolve_needed() {
  local f="$1"
  ${READELF} -d "$f" 2>/dev/null | sed -n 's/.*Shared library: \[\(.*\)\]/\1/p' || true
}
find_lib_file() {
  local name="$1"
  local search_dirs=("${THIRDPARTY}" "${VCPKG_LIB}" "${INSTALL_PREFIX}/lib" "${SYSROOT_LIBS[@]}")
  for d in "${search_dirs[@]}"; do
    [[ -d "$d" ]] || continue
    for c in "$d/$name" "$d/${name}".*; do
      [[ -f "$c" ]] && echo "$c" && return 0
    done
  done
  return 1
}

if [[ "${COPY_MODE}" == "minimal" ]]; then
  # 收集 bin/* 和 lib/*.so 的传递依赖闭包（仅 vcpkg 内的）
  declare -A NEEDED_SET
  QUEUE=()
  for f in "${INSTALL_PREFIX}"/bin/* "${INSTALL_PREFIX}"/lib/*.so*; do
    [[ -f "$f" ]] || continue
    file -b "$f" 2>/dev/null | grep -q ELF || continue
    while read -r n; do [[ -n "$n" ]] && NEEDED_SET["$n"]=1 && QUEUE+=("$n"); done < <(resolve_needed "$f")
  done
  while [[ ${#QUEUE[@]} -gt 0 ]]; do
    name="${QUEUE[0]}"
    QUEUE=("${QUEUE[@]:1}")
    for p in $SKIP_SYSTEM_LIBS; do [[ "$name" == $p* ]] && continue 2; done
    found=$(find_lib_file "$name")
    if [[ -n "$found" && "$found" == *"${VCPKG_LIB}"* ]]; then
      while read -r n; do
        if [[ -n "$n" && -z "${NEEDED_SET[$n]:-}" ]]; then
          NEEDED_SET["$n"]=1
          QUEUE+=("$n")
        fi
      done < <(resolve_needed "$found")
    fi
  done
  # 按 base 名拷贝：libopencv_core4.so.412 -> 拷贝 libopencv_core4.so* 整组
  for name in "${!NEEDED_SET[@]}"; do
    for p in $SKIP_SYSTEM_LIBS; do [[ "$name" == $p* ]] && continue 2; done
    base="${name%%.so*}"
    [[ -z "$base" ]] && continue
    for f in "${VCPKG_LIB}"/${base}.so*; do
      [[ -f "$f" ]] || continue
      cp -fL "$f" "${THIRDPARTY}/$(basename "$f")" 2>/dev/null || true
    done
  done
else
  # 旧行为：拷贝 vcpkg 全部
  rsync -a -L --include='*.so*' --exclude='*' "${VCPKG_LIB}/" "${THIRDPARTY}/" 2>/dev/null || {
    while IFS= read -r -d '' f; do cp -fL "$f" "${THIRDPARTY}/"; done < <(find "${VCPKG_LIB}" -maxdepth 1 -name "*.so*" -print0 2>/dev/null)
  }
fi

# 2. 补充系统库（libgfortran、libgcc_s 等），按 DT_NEEDED 递归收集
# 排除 libc/ld-linux/libm 等核心库：必须使用设备系统自带的，打包构建机的会导致 Segfault
SKIP_SYSTEM_LIBS="ld-linux libc.so libm.so libpthread libdl.so librt.so"
collect_system_libs() {
  local needed=()
  for f in "${INSTALL_PREFIX}"/bin/* "${INSTALL_PREFIX}"/lib/*.so* "${THIRDPARTY}"/*.so*; do
    [[ -f "$f" ]] || continue
    file -b "$f" 2>/dev/null | grep -q ELF || continue
    while read -r line; do
      [[ "$line" =~ Shared\ library:\ \[([^\]]+)\] ]] && needed+=("${BASH_REMATCH[1]}")
    done < <(${READELF} -d "$f" 2>/dev/null || true)
  done
  needed=($(printf '%s\n' "${needed[@]}" | sort -u))
  for name in "${needed[@]}"; do
    [[ -f "${THIRDPARTY}/${name}" ]] && continue
    # 跳过必须由系统提供的核心库
    skip=false
    for p in $SKIP_SYSTEM_LIBS; do [[ "$name" == $p* ]] && skip=true && break; done
    [[ "$skip" == true ]] && continue
    for d in "${THIRDPARTY}" "${VCPKG_LIB}" "${SYSROOT_LIBS[@]}"; do
      [[ -d "$d" ]] || continue
      for c in "$d/$name" "$d/${name}".*; do
        [[ -f "$c" ]] && cp -fL "$c" "${THIRDPARTY}/${name}" && break 2
      done
    done
  done
}
for _ in 1 2 3 4 5; do collect_system_libs; done

# arm64：若仍缺少 libgfortran / libgcc_s，显式从常见路径拷贝（Ubuntu 多在 /usr/aarch64-linux-gnu/lib 或 gcc-cross）
if [[ "${TRIPLET}" == "arm64-linux-custom" ]]; then
  for libbase in libgfortran libgcc_s; do
    if compgen -G "${THIRDPARTY}/${libbase}.so*" >/dev/null 2>&1; then continue; fi
    found=""
    for searchdir in /usr/aarch64-linux-gnu/lib /usr/lib/gcc/aarch64-linux-gnu/*/ /usr/lib/gcc-cross/aarch64-linux-gnu/*/; do
      [[ -d "$searchdir" ]] || continue
      for f in "${searchdir}"${libbase}.so*; do
        [[ -f "$f" ]] || continue
        cp -fL "$f" "${THIRDPARTY}/$(basename "$f")"
        found=1
      done
      [[ -n "$found" ]] && break
    done
    if [[ -z "$found" ]]; then
      echo "错误：未找到 ${libbase}.so*，无法拷贝到 ${THIRDPARTY}。请安装 aarch64 交叉编译 Fortran 运行时后重试：" >&2
      echo "  sudo apt install gfortran-aarch64-linux-gnu" >&2
      exit 1
    fi
  done
fi

# 3. 将 lib/ 与 thirdparty/lib 中的符号链接替换为实体文件
flatten_symlinks() {
  local dir="$1"
  [[ ! -d "$dir" ]] && return 0
  while IFS= read -r -d '' f; do
    if [[ -L "$f" ]]; then
      tmp="$(dirname "$f")/tmp_deref_$$_$RANDOM"
      cp -fL "$f" "$tmp" && rm -f "$f" && mv "$tmp" "$f"
    fi
  done < <(find "$dir" -maxdepth 1 -name "*.so*" -print0 2>/dev/null)
}
flatten_symlinks "${INSTALL_PREFIX}/lib"
flatten_symlinks "${THIRDPARTY}"
N=$(find "${THIRDPARTY}" -maxdepth 1 -name "*.so*" ! -type l 2>/dev/null | wc -l)
echo "已同步 ${N} 个库到 ${THIRDPARTY}（${COPY_MODE} 模式，vcpkg + 系统如 libgfortran），并去除所有符号链接（FAT32 兼容）"
