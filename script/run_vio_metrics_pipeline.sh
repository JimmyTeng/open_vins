#!/usr/bin/env bash
# 1) 导出批次日志指标 CSV
# 2) 从 CSV 生成 Markdown 详细报告
#
# 用法:
#   ./script/run_vio_metrics_pipeline.sh /path/to/logs/vio_yuv_batch_xxx
#   LOG_DIR=/path/to/batch OUT_PREFIX=/path/to/batch/metrics ./script/run_vio_metrics_pipeline.sh
#
# 默认在批次目录下生成:
#   vio_metrics.csv
#   VIO_METRICS_REPORT.md

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

LOG_DIR="${1:-${LOG_DIR:-}}"
if [ -z "$LOG_DIR" ] || [ ! -d "$LOG_DIR" ]; then
  echo "用法: $0 <批次日志目录>" >&2
  echo "  或: LOG_DIR=<目录> $0" >&2
  exit 1
fi

OUT_PREFIX="${OUT_PREFIX:-$LOG_DIR/vio_metrics}"
CSV="${OUT_PREFIX}.csv"
REPORT="${OUT_PREFIX}_REPORT.md"

python3 "$SCRIPT_DIR/export_vio_log_metrics.py" -o "$CSV" "$LOG_DIR"
python3 "$SCRIPT_DIR/generate_vio_metrics_report.py" "$CSV" -o "$REPORT"

echo "完成: $CSV"
echo "      $REPORT"
