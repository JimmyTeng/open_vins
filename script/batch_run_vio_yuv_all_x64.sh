#!/usr/bin/env bash
# 对 init 下发现的每个数据集，调用 run_vio_yuv_runner_x64.sh 重复运行多次；
# 使用单进程串行执行（避免并发引发资源/稳定性问题）。
#
# 用法:
#   ./script/batch_run_vio_yuv_all_x64.sh
#   REPEATS=100 ./script/batch_run_vio_yuv_all_x64.sh
#   LOG_ROOT=/path/to/logs ./script/batch_run_vio_yuv_all_x64.sh
#   ./script/batch_run_vio_yuv_all_x64.sh -- 额外参数会传给每次 vio_yuv_runner（如 --no-display）
#
# 环境变量:
#   REPEATS         每个数据集运行次数，默认 100
#   LOG_ROOT        日志根目录；默认 <仓库>/logs/vio_yuv_batch_<时间戳>
#   SKIP_BATCH_METRICS  设为 1 则跳过批次结束后的指标 CSV + Markdown 报告
#   THRESH_TIGHT_M / THRESH_LOOSE_M  飘移门限（米），传给 export_vio_log_metrics.py，默认 0.3 / 10

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_ROOT="$(cd "$ROOT/.." && pwd)"
RUNNER="$SCRIPT_DIR/run_vio_yuv_runner_x64.sh"
DEFAULT_CONFIG="$ROOT/ov_yuv_parser/config/openvins/estimator_config.yaml"

REPEATS="${REPEATS:-100}"

# run 序号文件名宽度：1–99 两位，≥100 三位（如 run100）
run_suffix() {
  local r="$1"
  if [ "$REPEATS" -lt 100 ]; then
    printf 'run%02d' "$r"
  else
    printf 'run%03d' "$r"
  fi
}

resolve_init_base() {
  for candidate in "$WS_ROOT/data/yuv_data/init" "$ROOT/data/yuv_data/init"; do
    if [ -d "$candidate" ]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

discover_dataset_roots() {
  local base="$1"
  [ -d "$base" ] || return 1
  find "$base" -mindepth 1 -type d 2>/dev/null | while IFS= read -r d; do
    [ -d "$d/dump_yuv" ] && printf '%s\n' "$d"
  done | LC_ALL=C sort -u
}

# ---------- 解析：-- 之后为传给 runner 的额外参数 ----------
EXTRA=()
while [ $# -gt 0 ]; do
  if [ "$1" = "--" ]; then
    shift
    EXTRA=("$@")
    break
  fi
  echo "未知参数: $1（可用 -- 传递 runner 选项）" >&2
  exit 1
done

INIT_BASE=""
if ! INIT_BASE="$(resolve_init_base)"; then
  echo "错误: 未找到 data/yuv_data/init（已尝试 $WS_ROOT 与 $ROOT）" >&2
  exit 1
fi

mapfile -t DATASETS < <(discover_dataset_roots "$INIT_BASE" || true)
n=${#DATASETS[@]}
if [ "$n" -eq 0 ]; then
  echo "错误: $INIT_BASE 下未发现含 dump_yuv/ 的子目录" >&2
  exit 1
fi

if ! [[ "$REPEATS" =~ ^[0-9]+$ ]] || [ "$REPEATS" -lt 1 ]; then
  echo "错误: REPEATS 须为正整数" >&2
  exit 1
fi
TS="$(date +%Y%m%d_%H%M%S)"
LOG_ROOT="${LOG_ROOT:-$ROOT/logs/vio_yuv_batch_$TS}"
mkdir -p "$LOG_ROOT"

[ -x "$RUNNER" ] || { echo "错误: 找不到可执行脚本: $RUNNER" >&2; exit 1; }

SUMMARY_LOCK="$LOG_ROOT/summary.lock"
touch "$SUMMARY_LOCK"

append_summary() {
  local line="$1"
  flock "$SUMMARY_LOCK" sh -c 'echo "$1" >> "$2"' _ "$line" "$LOG_ROOT/summary.txt"
}

echo "数据根: $INIT_BASE"
echo "数据集数量: $n，每个跑 $REPEATS 次，执行模式: 单进程串行"
echo "日志目录: $LOG_ROOT"
echo "汇总: $LOG_ROOT/summary.txt"
echo ""

{
  echo "batch_start_ts=$TS"
  echo "init_base=$INIT_BASE"
  echo "repeats=$REPEATS"
  echo "run_mode=single_process_serial"
  echo "extra_args=${EXTRA[*]-}"
  echo ""
} >"$LOG_ROOT/summary.txt"

# 单次任务：由 (ds_idx, D, rel, r) 标识
run_one_task() {
  local ds_idx="$1" D="$2" rel="$3" r="$4"
  local safe_name="${rel//\//__}"
  local rsuf
  rsuf="$(run_suffix "$r")"
  local logfile="$LOG_ROOT/${safe_name}__${rsuf}.log"
  local stamp finished ec

  stamp="$(date -Iseconds)"
  {
    echo "==== vio_yuv_runner batch ===="
    echo "dataset_index=$ds_idx/$n"
    echo "dataset_rel=$rel"
    echo "dataset_abs=$D"
    echo "run=$r/$REPEATS"
    echo "started_at=$stamp"
    echo "command: $RUNNER" "$D" "$DEFAULT_CONFIG" "${EXTRA[@]}"
    echo "=============================="
    echo ""
  } >"$logfile"

  set +e
  "$RUNNER" "$D" "$DEFAULT_CONFIG" "${EXTRA[@]}" >>"$logfile" 2>&1
  ec=$?
  set -e

  finished="$(date -Iseconds)"
  {
    echo ""
    echo "exit_code=$ec finished_at=$finished"
  } >>"$logfile"

  append_summary "[$stamp -> $finished] ds=$ds_idx/$n rel=$rel run=$r/$REPEATS exit=$ec log=$logfile"
  if [ "$ec" -ne 0 ]; then
    echo "警告: exit=$ec rel=$rel run=$r/$REPEATS" >&2
  fi
}

ds_idx=0
for D in "${DATASETS[@]}"; do
  ds_idx=$((ds_idx + 1))
  rel="${D#$INIT_BASE/}"
  rel="${rel#/}"

  echo "排队: [$ds_idx/$n] $rel × $REPEATS 次"

  for r in $(seq 1 "$REPEATS"); do
    run_one_task "$ds_idx" "$D" "$rel" "$r"
  done
done

echo ""
echo "全部完成。日志: $LOG_ROOT"

# ---------- 批次结束：导出每条日志指标 CSV，并生成汇总报告 ----------
if [ "${SKIP_BATCH_METRICS:-0}" != "1" ]; then
  METRICS_CSV="$LOG_ROOT/vio_metrics.csv"
  METRICS_REPORT="$LOG_ROOT/vio_metrics_REPORT.md"
  TIGHT="${THRESH_TIGHT_M:-0.3}"
  LOOSE="${THRESH_LOOSE_M:-10.0}"
  echo ""
  echo "========== 分析：导出指标并生成报告 =========="
  set +e
  python3 "$SCRIPT_DIR/export_vio_log_metrics.py" \
    --tight-m "$TIGHT" \
    --loose-m "$LOOSE" \
    -o "$METRICS_CSV" \
    "$LOG_ROOT"
  _ex=$?
  if [ "$_ex" -eq 0 ]; then
    python3 "$SCRIPT_DIR/generate_vio_metrics_report.py" "$METRICS_CSV" -o "$METRICS_REPORT"
    _ex=$?
  fi
  set -e
  if [ "$_ex" -eq 0 ]; then
    echo "指标 CSV:   $METRICS_CSV"
    echo "汇总报告:   $METRICS_REPORT"
  else
    echo "警告: 指标导出或报告生成失败（exit=$_ex）；可设 SKIP_BATCH_METRICS=1 跳过）" >&2
  fi
fi
