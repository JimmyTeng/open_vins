#!/usr/bin/env bash
# 部署 OpenVINS 到 SD 卡：删除旧文件并拷贝新构建
# 用法: ./script/deploy_to_sdcard.sh [选项]
#   --src DIR    源目录，默认 install/aarch64/Release-docker
#   --dest DIR   目标目录，默认 /media/jimmy/6391-1657/VIO
#   -y, --yes    不确认直接执行
#   -h, --help   显示帮助
#
# 示例:
#   ./script/deploy_to_sdcard.sh
#   ./script/deploy_to_sdcard.sh -y
#   ./script/deploy_to_sdcard.sh --dest /media/jimmy/6391-1657/VIO -y

set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

SRC="${ROOT}/install/aarch64/Release-docker"
DEST="/media/jimmy/6391-1657/VIO"
YES=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --src)   SRC="$2";  shift 2 ;;
    --dest)  DEST="$2"; shift 2 ;;
    -y|--yes) YES=1; shift ;;
    -h|--help)
      head -16 "$0" | tail -14
      exit 0
      ;;
    *) echo "未知选项: $1" >&2; exit 1 ;;
  esac
done

if [[ ! -d "$SRC" ]]; then
  echo "错误: 源目录不存在: $SRC" >&2
  echo "请先构建: ./script/build_arm64_docker.sh" >&2
  exit 1
fi

if [[ ! -d "$DEST" ]]; then
  echo "错误: 目标目录不存在: $DEST" >&2
  echo "请先插入 SD 卡并挂载，或指定 --dest 路径" >&2
  exit 1
fi

echo "=== 部署 OpenVINS 到 SD 卡 ==="
echo "  源: $SRC"
echo "  目标: $DEST"
echo ""

# 仅删除 bin/lib/config/thirdparty，保留 data、run.sh、ld.config
echo "将删除: bin/ lib/ config/ thirdparty/"
echo "将保留: data/ run.sh ld.config"
echo ""

if [[ -z "$YES" ]]; then
  read -p "确认删除上述目录并拷贝新文件? [y/N] " -r
  [[ "$REPLY" =~ ^[yY]$ ]] || { echo "已取消"; exit 0; }
fi

echo "删除 bin/ lib/ config/ thirdparty/..."
rm -rf "$DEST"/bin "$DEST"/lib "$DEST"/config "$DEST"/thirdparty 2>/dev/null || true

echo "拷贝新文件..."
for d in bin lib config thirdparty; do
  [[ -d "$SRC/$d" ]] && cp -a "$SRC/$d" "$DEST/"
done

echo ""
echo "同步写入到存储..."
sync

echo "完成。已部署到 $DEST"
echo "  已更新: bin/ lib/ config/ thirdparty/"
echo "  已保留: data/ run.sh ld.config"
ls -la "$DEST"
