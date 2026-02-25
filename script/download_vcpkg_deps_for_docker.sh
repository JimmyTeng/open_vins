#!/usr/bin/env bash
# 预下载 vcpkg 依赖（boost + opencv/ceres/eigen 等）到 script/vcpkg-downloads/，供 Docker 构建使用
# 用法：./script/download_vcpkg_deps_for_docker.sh [boost-模块名...]  或  无参数下载全部（boost + 其他）
# 支持代理：HTTPS_PROXY=http://127.0.0.1:10808 ./script/...
# 换镜像：VCPKG_GH_MIRROR=https://ghproxy.com/https://github.com ./script/...

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BOOST_VERSION="1.90.0"
DEST_DIR="${SCRIPT_DIR}/vcpkg-downloads"

# GitHub 镜像；可设置 VCPKG_GH_MIRROR 指定唯一镜像
MIRRORS=(
  "${VCPKG_GH_MIRROR}"
  "https://github.com"
)

mkdir -p "$DEST_DIR"

download_one() {
  local url="$1" file="$2"
  curl -fL --connect-timeout 15 --max-time 300 -o "$file" "$url" 2>/dev/null
}

# 校验 SHA512（可选）；若传入且不匹配则返回 1
check_sha512() {
  local file="$1" expected="$2"
  [ -z "$expected" ] && return 0
  [ ! -f "$file" ] && return 1
  local actual
  actual="$(sha512sum -b "$file" 2>/dev/null | awk '{print $1}')"
  [ "$actual" = "$expected" ]
}

# 下载 GitHub 归档，文件名与 vcpkg 一致：owner-repo-ref.tar.gz
# 若传入第 4 个参数（SHA512），下载后会校验，不通过则删文件并尝试下一镜像（vcpkg 会校验，必须一致才会用缓存）
download_github() {
  local owner="$1" repo="$2" ref="$3" expected_sha512="${4:-}"
  local file="${owner}-${repo}-${ref}.tar.gz"
  local path="${owner}/${repo}/archive/${ref}.tar.gz"
  if [ -f "$DEST_DIR/$file" ] && check_sha512 "$DEST_DIR/$file" "$expected_sha512"; then
    echo "已存在且校验通过: $file"
    return 0
  fi
  for base in "${MIRRORS[@]}"; do
    [ -z "$base" ] && continue
    local url="${base}/${path}"
    echo "下载: $file"
    if download_one "$url" "$DEST_DIR/$file"; then
      if check_sha512 "$DEST_DIR/$file" "$expected_sha512"; then
        echo "已保存: $file"
        return 0
      fi
      echo "  SHA512 与 vcpkg 不一致，尝试下一源（vcpkg 必须校验通过才会用缓存）"
      rm -f "$DEST_DIR/$file"
    fi
  done
  echo "失败: $path" >&2
  return 1
}

# 下载 GitLab 归档（如 eigen）
download_gitlab() {
  local repo="$1" ref="$2"
  # vcpkg 文件名：REPO 中 / 替换为 -，再加 ref
  local base="${repo//\//-}"
  local file="${base}-${ref}.tar.gz"
  local repo_name="${repo##*/}"
  local url="https://gitlab.com/${repo}/-/archive/${ref}/${repo_name}-${ref}.tar.gz"
  if [ -f "$DEST_DIR/$file" ]; then
    echo "已存在: $file"
    return 0
  fi
  echo "下载: $file"
  if download_one "$url" "$DEST_DIR/$file"; then
    echo "已保存: $file"
    return 0
  fi
  rm -f "$DEST_DIR/$file"
  echo "失败: $url" >&2
  return 1
}

# 下载单个 boost 模块
download_boost() {
  local name="$1"
  local repo="${name#boost-}"
  repo="${repo//-/_}"
  local file="boostorg-${repo}-boost-${BOOST_VERSION}.tar.gz"
  local path="boostorg/${repo}/archive/boost-${BOOST_VERSION}.tar.gz"
  if [ -f "$DEST_DIR/$file" ]; then
    echo "已存在: $file"
    return 0
  fi
  for base in "${MIRRORS[@]}"; do
    [ -z "$base" ] && continue
    local url="${base}/${path}"
    echo "下载: $file"
    if download_one "$url" "$DEST_DIR/$file"; then
      echo "已保存: $file"
      return 0
    fi
    rm -f "$DEST_DIR/$file"
  done
  echo "失败: $path" >&2
  return 1
}

# 默认下载 vcpkg 构建顺序中靠前的 boost 包（避免容器内连 GitHub 卡住）
DEFAULT_BOOST_MODULES=(
  boost-cmake
  boost-headers
  boost-config
  boost-assert
  boost-throw-exception
  boost-predef
  boost-mp11
  boost-core
  boost-describe
  boost-container-hash
  boost-unordered
  boost-static-assert
  boost-type-traits
  boost-tuple
  boost-preprocessor
  boost-optional
  boost-io
  boost-detail
  boost-exception
  boost-algorithm
  boost-align
  boost-array
  boost-atomic
  boost-bind
  boost-chrono
  boost-concept-check
  boost-container
  boost-conversion
  boost-date-time
  boost-dynamic-bitset
  boost-endian
  boost-format
  boost-function
  boost-function-types
  boost-functional
  boost-fusion
  boost-integer
  boost-intrusive
  boost-iterator
  boost-lexical-cast
  boost-math
  boost-move
  boost-mpl
  boost-numeric-conversion
  boost-random
  boost-range
  boost-ratio
  boost-regex
  boost-scope
  boost-serialization
  boost-smart-ptr
  boost-pool
  boost-test
  boost-timer
  boost-stacktrace
  boost-leaf
  boost-outcome
  boost-parameter
  boost-pfr
  boost-vmd
  boost-tti
  boost-stl-interfaces
  boost-safe-numerics
  boost-system
  boost-filesystem
  boost-tokenizer
  boost-typeof
  boost-utility
  boost-variant2
  boost-winapi
)

# vcpkg.json 及其传递依赖：GitHub(owner repo ref [SHA512])；带 SHA512 的会校验，确保 vcpkg 能用缓存
NON_BOOST_DEPS=(
  "opencv opencv 4.12.0"
  "opencv opencv_contrib 4.12.0"
  "ceres-solver ceres-solver 85331393dc0dff09f6fb9903ab0c4bfa3e134b01"
  "DrTimothyAldenDavis SuiteSparse v7.8.3 fc0fd0aaf55a6712a3b8ca23bf7536a31d52033e090370ebbf291f05d0e073c7dcfd991a80b037f54663f524804582b87af86522c2e4435091527f0d3c189244"
  "gflags gflags v2.3.0"
  "google glog v0.7.1"
  "OpenMathLib OpenBLAS v0.3.29"
  "Reference-LAPACK lapack v3.12.1"
  "libjpeg-turbo libjpeg-turbo 3.1.3"
  "pnggroup libpng v1.6.54"
  "madler zlib v1.3.1"
)
EIGEN_REPO="libeigen/eigen"
EIGEN_REF="5.0.1"

FAIL=0
if [ $# -eq 0 ]; then
  echo "==== 下载 vcpkg.json 中非 boost 依赖（opencv/ceres/eigen/suitesparse/...） ===="
  for spec in "${NON_BOOST_DEPS[@]}"; do
    read -r owner repo ref rest <<< "$spec"
    download_github "$owner" "$repo" "$ref" "$rest" || FAIL=1
  done
  download_gitlab "$EIGEN_REPO" "$EIGEN_REF" || FAIL=1
  echo ""
  echo "==== 下载 boost 模块 ===="
  for m in "${DEFAULT_BOOST_MODULES[@]}"; do
    download_boost "$m" || FAIL=1
  done
else
  for m in "$@"; do
    if [[ "$m" == boost-* ]]; then
      download_boost "$m" || FAIL=1
    elif [[ "$m" == "eigen3" ]] || [[ "$m" == "eigen" ]]; then
      download_gitlab "$EIGEN_REPO" "$EIGEN_REF" || FAIL=1
    elif [[ "$m" == "opencv" ]]; then
      download_github opencv opencv 4.12.0 || FAIL=1
      download_github opencv opencv_contrib 4.12.0 || FAIL=1
    elif [[ "$m" == "ceres" ]]; then
      download_github ceres-solver ceres-solver 85331393dc0dff09f6fb9903ab0c4bfa3e134b01 || FAIL=1
    elif [[ "$m" == "suitesparse" ]]; then
      download_github DrTimothyAldenDavis SuiteSparse v7.8.3 || FAIL=1
    elif [[ "$m" == "gflags" ]]; then
      download_github gflags gflags v2.3.0 || FAIL=1
    elif [[ "$m" == "glog" ]]; then
      download_github google glog v0.7.1 || FAIL=1
    elif [[ "$m" == "openblas" ]]; then
      download_github OpenMathLib OpenBLAS v0.3.29 || FAIL=1
    elif [[ "$m" == "lapack" ]]; then
      download_github Reference-LAPACK lapack v3.12.1 || FAIL=1
    elif [[ "$m" == "libjpeg-turbo" ]]; then
      download_github libjpeg-turbo libjpeg-turbo 3.1.3 || FAIL=1
    elif [[ "$m" == "libpng" ]]; then
      download_github pnggroup libpng v1.6.54 || FAIL=1
    elif [[ "$m" == "zlib" ]]; then
      download_github madler zlib v1.3.1 || FAIL=1
    else
      echo "未知模块: $m（支持: boost-*、eigen3、opencv、ceres、suitesparse、gflags、glog、openblas、lapack、libjpeg-turbo、libpng、zlib）" >&2
      FAIL=1
    fi
  done
fi

echo ""
echo "已写入目录: $DEST_DIR"
echo "构建时脚本会同步到 .vcpkg-docker-cache，vcpkg 将直接使用。"
[ $FAIL -eq 1 ] && exit 1
exit 0
