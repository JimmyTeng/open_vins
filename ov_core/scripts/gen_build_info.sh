#!/bin/sh
# 编译时生成 build_info.h：构建时间、Git 版本/分支/标签/状态、用户信息
# 用法: gen_build_info.sh <输出头文件路径> [仓库根目录，默认自动检测]
set -e
out="$1"
repo="${2:-}"
dir=$(dirname "$out")
mkdir -p "$dir"

# 确定仓库根目录（含 .git）
if [ -n "$repo" ] && [ -d "$repo/.git" ]; then
  GIT_ROOT="$repo"
else
  # 从脚本所在目录向上查找
  script_dir=$(cd "$(dirname "$0")" && pwd)
  GIT_ROOT="$script_dir/../.."
  while [ "$GIT_ROOT" != "/" ]; do
    [ -d "$GIT_ROOT/.git" ] && break
    GIT_ROOT=$(dirname "$GIT_ROOT")
  done
  [ ! -d "$GIT_ROOT/.git" ] && GIT_ROOT=""
fi

# 若值中含 " 则转义为 \"
escape() { echo "$1" | sed 's/"/\\"/g'; }

# 北京时间，英文月份
BUILD_TIME=$(TZ=Asia/Shanghai LC_TIME=C date +"%b %d %Y %H:%M:%S")
BUILD_TIMEZONE="CST (UTC+8)"

if [ -n "$GIT_ROOT" ]; then
  cd "$GIT_ROOT"
  GIT_COMMIT=$(git rev-parse --short HEAD 2>/dev/null) || GIT_COMMIT="unknown"
  GIT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null) || GIT_BRANCH="unknown"
  GIT_TAG=$(git describe --tags --exact-match 2>/dev/null) || GIT_TAG="none"
  if git status --porcelain 2>/dev/null | grep -q .; then
    GIT_DIRTY="dirty"
  else
    GIT_DIRTY="clean"
  fi
  GIT_USER_NAME=$(git config user.name 2>/dev/null) || GIT_USER_NAME="unknown"
  GIT_USER_EMAIL=$(git config user.email 2>/dev/null) || GIT_USER_EMAIL="unknown"
  GIT_FULL_HASH=$(git rev-parse HEAD 2>/dev/null) || GIT_FULL_HASH="unknown"
else
  GIT_COMMIT="unknown"
  GIT_BRANCH="unknown"
  GIT_TAG="none"
  GIT_DIRTY="unknown"
  GIT_USER_NAME="unknown"
  GIT_USER_EMAIL="unknown"
  GIT_FULL_HASH="unknown"
fi

# 写入头文件，值中双引号已转义
{
  echo "/* Auto-generated at build time. Do not edit. */"
  echo "#ifndef BUILD_INFO_H"
  echo "#define BUILD_INFO_H"
  echo ""
  printf '#define BUILD_TIMESTAMP   "%s"\n' "$(escape "$BUILD_TIME")"
  printf '#define BUILD_TIMEZONE    "%s"\n' "$(escape "$BUILD_TIMEZONE")"
  printf '#define BUILD_GIT_COMMIT  "%s"\n' "$(escape "$GIT_COMMIT")"
  printf '#define BUILD_GIT_BRANCH  "%s"\n' "$(escape "$GIT_BRANCH")"
  printf '#define BUILD_GIT_TAG     "%s"\n' "$(escape "$GIT_TAG")"
  printf '#define BUILD_GIT_DIRTY   "%s"\n' "$(escape "$GIT_DIRTY")"
  printf '#define BUILD_GIT_USER    "%s"\n' "$(escape "$GIT_USER_NAME")"
  printf '#define BUILD_GIT_EMAIL   "%s"\n' "$(escape "$GIT_USER_EMAIL")"
  printf '#define BUILD_GIT_HASH    "%s"\n' "$(escape "$GIT_FULL_HASH")"
  echo ""
  echo "#endif /* BUILD_INFO_H */"
} > "$out"
