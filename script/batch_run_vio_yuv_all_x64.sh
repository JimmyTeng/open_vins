#!/usr/bin/env bash
# 对 init 下发现的数据集调用 run_vio_yuv_runner_x64.sh；
# 日志按数据集写入 LOG_ROOT/<相对 init 的路径>/，批次结束 export 递归扫描该树生成指标。
# 调度顺序：共 REPEATS 轮，每轮按数据集列表顺序跑完所有数据集，再进入下一轮
#（不是「单个数据集跑满 REPEATS 次再换下一个」）。
# 默认单进程串行；可通过第二个参数指定并发进程数（需 Bash ≥ 4.3，支持 wait -n）。
#
# 用法:
#   ./script/batch_run_vio_yuv_all_x64.sh
#   ./script/batch_run_vio_yuv_all_x64.sh <重复次数>
#   ./script/batch_run_vio_yuv_all_x64.sh <重复次数> <并发进程数>
#   REPEATS=100 ./script/batch_run_vio_yuv_all_x64.sh
#   LOG_ROOT=/path/to/logs ./script/batch_run_vio_yuv_all_x64.sh
#   ./script/batch_run_vio_yuv_all_x64.sh 20 4 -- 额外参数会传给每次 vio_yuv_runner（如 --no-display）
#
# 环境变量:
#   REPEATS         每个数据集运行次数（可被第一个数字参数覆盖），默认 10
#   JOBS            并发进程数（可被第二个数字参数覆盖），默认 1
#   LOG_ROOT        日志根目录；默认 <仓库>/logs/vio_yuv_batch_<时间戳>
#                   每个数据集写入 LOG_ROOT/<相对 init 的路径>/，与数据目录层级一致
#   SKIP_BATCH_METRICS  设为 1 则跳过批次结束后的指标 CSV + Markdown 报告
#   THRESH_TIGHT_M / THRESH_LOOSE_M  飘移门限（米），传给 export_vio_log_metrics.py；
#                   紧场景 takeoff/hover/… 用 TIGHT；move、integrate 用 LOOSE；默认 0.3 / 5
#   PROGRESS_BAR_WIDTH  单行进度条宽度（字符），默认 36
#   NO_PROGRESS           设为 1 则关闭单行进度（仍正常跑批次）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_ROOT="$(cd "$ROOT/.." && pwd)"
RUNNER="$SCRIPT_DIR/run_vio_yuv_runner_x64.sh"
DEFAULT_CONFIG="$ROOT/ov_yuv_parser/config/openvins/estimator_config.yaml"

REPEATS="${REPEATS:-10}"
JOBS="${JOBS:-1}"

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

# ---------- 解析：可选 1～2 个正整数（重复次数、并发数），-- 之后为传给 runner 的额外参数 ----------
EXTRA=()
HAVE_REPEAT_ARG=0
HAVE_JOBS_ARG=0
while [ $# -gt 0 ]; do
  if [ "$1" = "--" ]; then
    shift
    EXTRA=("$@")
    break
  fi
  if [[ "$1" =~ ^[0-9]+$ ]]; then
    if [ "$HAVE_REPEAT_ARG" -eq 0 ]; then
      REPEATS="$1"
      HAVE_REPEAT_ARG=1
    elif [ "$HAVE_JOBS_ARG" -eq 0 ]; then
      JOBS="$1"
      HAVE_JOBS_ARG=1
    else
      echo "多余参数: $1（最多两个数字：重复次数、并发进程数）" >&2
      exit 1
    fi
    shift
    continue
  fi
  echo "未知参数: $1（先写重复次数/并发数，或用 -- 传递 runner 选项）" >&2
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
if ! [[ "$JOBS" =~ ^[0-9]+$ ]] || [ "$JOBS" -lt 1 ]; then
  echo "错误: JOBS（并发进程数）须为正整数" >&2
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

if [ "$JOBS" -eq 1 ]; then
  _mode_desc="单进程串行"
  _run_mode_tag="single_process_serial"
else
  _mode_desc="并发 ${JOBS} 进程（Bash wait -n）"
  _run_mode_tag="parallel_jobs_${JOBS}"
fi
echo "数据根: $INIT_BASE"
echo "数据集数量: $n，共 $REPEATS 轮；每轮跑遍全部 $n 个数据集（顺序：轮次外层、数据集内层），执行模式: $_mode_desc"
echo "日志目录: $LOG_ROOT（各数据集子目录与 init 下相对路径一致）"
echo "汇总: $LOG_ROOT/summary.txt"
echo ""

{
  echo "batch_start_ts=$TS"
  echo "init_base=$INIT_BASE"
  echo "repeats=$REPEATS"
  echo "parallel_jobs=$JOBS"
  echo "run_mode=$_run_mode_tag"
  echo "extra_args=${EXTRA[*]-}"
  echo ""
} >"$LOG_ROOT/summary.txt"

# 单次任务：由 (ds_idx, D, rel, r) 标识
run_one_task() {
  local ds_idx="$1" D="$2" rel="$3" r="$4"
  local safe_name="${rel//\//__}"
  local rsuf
  rsuf="$(run_suffix "$r")"
  local ds_dir="$LOG_ROOT/$rel"
  mkdir -p "$ds_dir"
  local logfile="$ds_dir/${safe_name}__${rsuf}.log"
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

if [ "$JOBS" -gt 1 ]; then
  if ! bash -c 'help wait' 2>/dev/null | grep -q -- '-n'; then
    echo "错误: 并发 JOBS=$JOBS 需要 Bash 4.3+（内置 wait -n）" >&2
    exit 1
  fi
fi

# 总任务数 = 轮次 × 数据集个数；单行 \r 进度输出到 stderr，避免与 stdout 管道冲突
TOTAL_BATCH_TASKS=$((REPEATS * n))
DONE_BATCH_TASKS=0
PROGRESS_BAR_WIDTH="${PROGRESS_BAR_WIDTH:-36}"
# 批次开始时刻（秒），用于已用/预计剩余
BATCH_START_SEC=$(date +%s)

# 将秒数格式化为简短可读串（如 45s / 3m05s / 1h02m30s）
fmt_duration() {
  local s="${1:-0}"
  [ "$s" -lt 0 ] && s=0
  if [ "$s" -lt 60 ]; then
    printf '%ds' "$s"
  elif [ "$s" -lt 3600 ]; then
    printf '%dm%02ds' $((s / 60)) $((s % 60))
  else
    printf '%dh%02dm%02ds' $((s / 3600)) $(((s % 3600) / 60)) $((s % 60))
  fi
}

draw_batch_progress() {
  [ "${NO_PROGRESS:-0}" = "1" ] && return 0
  local cur="${1:-0}" tot="${2:-1}" w="${PROGRESS_BAR_WIDTH:-36}"
  [ "$tot" -lt 1 ] && tot=1
  local pct=$((cur * 100 / tot))
  local filled=$((cur * w / tot))
  [ "$filled" -gt "$w" ] && filled=$w
  local empty=$((w - filled)) i bar=""
  for ((i = 0; i < filled; i++)); do bar+="#"; done
  for ((i = 0; i < empty; i++)); do bar+="-"; done
  local now elapsed elapsed_s eta_s
  now=$(date +%s)
  elapsed=$((now - BATCH_START_SEC))
  elapsed_s="$(fmt_duration "$elapsed")"
  local eta_str="--"
  if [ "$cur" -gt 0 ] && [ "$cur" -lt "$tot" ]; then
    # 按已完成任务平均耗时线性外推剩余：elapsed * (tot-cur) / cur
    eta_s=$((elapsed * (tot - cur) / cur))
    eta_str="$(fmt_duration "$eta_s")"
  elif [ "$cur" -eq "$tot" ] && [ "$tot" -gt 0 ]; then
    eta_str="—"
  fi
  if [ -t 2 ]; then
    # \033[K 清到行尾，避免上一条残留字符
    printf '\r\033[K[批次进度] [%s] %d/%d (%d%%) 已用:%s 预计剩余:%s' \
      "$bar" "$cur" "$tot" "$pct" "$elapsed_s" "$eta_str" >&2
  else
    # 非终端：约每 5% 打一行，避免日志刷屏
    local step=$(( (tot + 19) / 20 ))
    [ "$step" -lt 1 ] && step=1
    if [ "$cur" -eq 0 ] || [ "$cur" -eq "$tot" ] || [ $((cur % step)) -eq 0 ]; then
      printf '[批次进度] %d/%d (%d%%) 已用:%s 预计剩余:%s\n' \
        "$cur" "$tot" "$pct" "$elapsed_s" "$eta_str" >&2
    fi
  fi
}

batch_task_done() {
  DONE_BATCH_TASKS=$((DONE_BATCH_TASKS + 1))
  draw_batch_progress "$DONE_BATCH_TASKS" "$TOTAL_BATCH_TASKS"
}

running=0
draw_batch_progress 0 "$TOTAL_BATCH_TASKS"
for r in $(seq 1 "$REPEATS"); do
  ds_idx=0
  for D in "${DATASETS[@]}"; do
    ds_idx=$((ds_idx + 1))
    rel="${D#$INIT_BASE/}"
    rel="${rel#/}"

    while [ "$running" -ge "$JOBS" ]; do
      wait -n || true
      running=$((running - 1))
      batch_task_done
    done
    run_one_task "$ds_idx" "$D" "$rel" "$r" &
    running=$((running + 1))
  done
done
while [ "$running" -gt 0 ]; do
  wait -n || true
  running=$((running - 1))
  batch_task_done
done
if [ "${NO_PROGRESS:-0}" != "1" ]; then
  printf '\n' >&2
fi

echo "全部完成。日志: $LOG_ROOT"

# ---------- 批次结束：导出每条日志指标 CSV，并生成汇总报告 ----------
if [ "${SKIP_BATCH_METRICS:-0}" != "1" ]; then
  METRICS_CSV="$LOG_ROOT/vio_metrics.csv"
  METRICS_REPORT="$LOG_ROOT/vio_metrics_REPORT.md"
  TIGHT="${THRESH_TIGHT_M:-0.3}"
  LOOSE="${THRESH_LOOSE_M:-5.0}"
  echo ""
  echo "========== 分析：递归扫描 LOG_ROOT 下 *.log，导出指标并生成报告 =========="
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
