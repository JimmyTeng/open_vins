#!/usr/bin/env bash
# 从 vcpkg_installed 同步依赖库到 install/.../thirdparty/lib，供 run.sh 运行时加载
# 含 BLAS/LAPACK、OpenCV、Ceres、Boost 等；部署到设备时需随 install 一并拷贝。
# 使用 -L 解引用符号链接，拷贝实体文件，便于通过 FAT32 的 SD 卡转存（FAT32 不支持符号链接）。
#
# 用法：
#   ./script/sync_thirdparty_from_vcpkg.sh [preset]
#   ./script/sync_thirdparty_from_vcpkg.sh arm64-release-vcpkg-docker
#   ./script/sync_thirdparty_from_vcpkg.sh arm64-release-vcpkg
#   ./script/sync_thirdparty_from_vcpkg.sh x64-release-vcpkg
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
  x64-release-vcpkg)
    BUILD_DIR="${ROOT}/build/x86_64/Release/x64-release-vcpkg"
    INSTALL_PREFIX="${ROOT}/install/x86_64/Release"
    TRIPLET="x64-linux-custom"
    ;;
  x64-debug-vcpkg)
    BUILD_DIR="${ROOT}/build/x86_64/Debug/x64-debug-vcpkg"
    INSTALL_PREFIX="${ROOT}/install/x86_64/Debug"
    TRIPLET="x64-linux-custom"
    ;;
  "")
    # 自动推断：查找最近有 vcpkg_installed 的 build 目录
    for cand in \
      "${ROOT}/build/aarch64/Release-docker/arm64-release-vcpkg-docker" \
      "${ROOT}/build/aarch64/Release/arm64-release-vcpkg" \
      "${ROOT}/build/x86_64/Release/x64-release-vcpkg"; do
      if [[ -d "${cand}/vcpkg_installed" ]]; then
        BUILD_DIR="$cand"
        if [[ "$cand" == *arm64-release-vcpkg-docker* ]]; then
          INSTALL_PREFIX="${ROOT}/install/aarch64/Release-docker"
          TRIPLET="arm64-linux-custom"
        elif [[ "$cand" == *arm64-release-vcpkg* ]]; then
          INSTALL_PREFIX="${ROOT}/install/aarch64/Release"
          TRIPLET="arm64-linux-custom"
        else
          INSTALL_PREFIX="${ROOT}/install/x86_64/Release"
          TRIPLET="x64-linux-custom"
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
    echo "用法: $0 [preset]"
    echo "preset: arm64-release-vcpkg-docker | arm64-release-vcpkg | x64-release-vcpkg | x64-debug-vcpkg"
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

# 根据 triplet 选择 readelf 与系统库搜索路径（LAPACK 依赖 libgfortran，位于 gcc 目录）
READELF="readelf"
SYSROOT_LIBS=("/lib/x86_64-linux-gnu" "/usr/lib/x86_64-linux-gnu" "/lib" "/usr/lib")
if [[ "${TRIPLET}" == "arm64-linux-custom" ]]; then
  READELF="aarch64-linux-gnu-readelf"
  SYSROOT_LIBS=("/usr/aarch64-linux-gnu/lib" "/lib/aarch64-linux-gnu" "/usr/lib/aarch64-linux-gnu")
  # libgfortran.so.4 在 gfortran-aarch64-linux-gnu 包中，路径为 /usr/lib/gcc/aarch64-linux-gnu/<ver>/
  for gccdir in /usr/lib/gcc/aarch64-linux-gnu/*/; do
    [[ -d "$gccdir" ]] && SYSROOT_LIBS+=("$gccdir")
  done
fi

# 1. 拷贝 vcpkg 依赖，-L 解引用
rsync -a -L --include='*.so*' --exclude='*' "${VCPKG_LIB}/" "${THIRDPARTY}/" 2>/dev/null || {
  while IFS= read -r -d '' f; do
    cp -fL "$f" "${THIRDPARTY}/"
  done < <(find "${VCPKG_LIB}" -maxdepth 1 -name "*.so*" -print0 2>/dev/null)
}

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
echo "已同步 ${N} 个库到 ${THIRDPARTY}（vcpkg + 系统如 libgfortran），并去除所有符号链接（FAT32 兼容）"
