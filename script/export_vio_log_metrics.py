#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
扫描 vio_yuv_runner 批次日志，导出每条日志的初始化 / 飘移指标为 CSV，
供 generate_vio_metrics_report.py 生成详细报告。

用法:
  python3 script/export_vio_log_metrics.py -o metrics.csv logs/vio_yuv_batch_xxx/
  python3 script/export_vio_log_metrics.py -o out.csv --tight-m 0.3 --loose-m 5 path/to/logs/

输出列（UTF-8，首列为 BOM 便于 Excel）:
  log_file, dataset_key, run_id, scenario, drift_threshold_m,
  init_success, init_duration_s, max_drift_m, drift_ok, drift_status, exit_code
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import sys
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from vio_log_metrics_core import analyze_text_for_export, gather_files


def dataset_key_and_run(log_name: str) -> tuple[str, str]:
    """takeoff__outdoor__grass__run042.log -> (takeoff__outdoor__grass, run042)"""
    m = re.match(r"^(.+)__(run\d+)\.log$", log_name, re.I)
    if m:
        return m.group(1), m.group(2)
    return Path(log_name).stem, ""


def drift_status_zh(idm: dict) -> str:
    ok = idm.get("drift_ok")
    if ok is True:
        return "达标"
    if ok is False:
        return "超门限"
    return "无法判定"


def main() -> int:
    ap = argparse.ArgumentParser(description="导出 VIO 批次日志指标 CSV")
    ap.add_argument(
        "paths",
        nargs="+",
        help="日志文件或目录（目录下递归收集 *.log，不含 summary.txt）",
    )
    ap.add_argument(
        "-o",
        "--output",
        required=True,
        help="输出 CSV 路径",
    )
    ap.add_argument(
        "--tight-m",
        type=float,
        default=float(os.environ.get("THRESH_TIGHT_M", "0.3")),
    )
    ap.add_argument(
        "--loose-m",
        type=float,
        default=float(os.environ.get("THRESH_LOOSE_M", "5.0")),
        help="move / integrate 场景飘移门限（米）；takeoff/hover 等仍用 --tight-m",
    )
    args = ap.parse_args()

    files = gather_files(args.paths)
    if not files:
        print("未找到 .log 文件", file=sys.stderr)
        return 1

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fieldnames = [
        "log_file",
        "dataset_key",
        "run_id",
        "scenario",
        "drift_threshold_m",
        "init_success",
        "init_duration_s",
        "max_drift_m",
        "drift_ok",
        "drift_status",
        "exit_code",
    ]

    n = 0
    with out_path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for fp in files:
            try:
                text = fp.read_text(encoding="utf-8", errors="replace")
            except OSError as e:
                print(f"跳过 {fp}: {e}", file=sys.stderr)
                continue
            r = analyze_text_for_export(
                text,
                str(fp),
                tight_m=args.tight_m,
                loose_m=args.loose_m,
            )
            name = fp.name
            ds_key, run_id = dataset_key_and_run(name)
            idm = r.get("init_drift") or {}
            oc = r.get("other_counts") or {}
            init_ok = bool(idm.get("init_line_found"))
            max_d = idm.get("max_drift_m")
            dk = idm.get("drift_ok")
            row = {
                "log_file": name,
                "dataset_key": ds_key,
                "run_id": run_id,
                "scenario": idm.get("scenario_from_filename", ""),
                "drift_threshold_m": idm.get("drift_threshold_m", ""),
                "init_success": 1 if init_ok else 0,
                "init_duration_s": (
                    f"{idm['init_duration_s']:.6f}"
                    if idm.get("init_duration_s") is not None
                    else ""
                ),
                "max_drift_m": (
                    f"{max_d:.6f}" if max_d is not None and math.isfinite(max_d) else ""
                ),
                "drift_ok": (
                    "" if dk is None else (1 if dk else 0)
                ),
                "drift_status": drift_status_zh(idm),
                "exit_code": oc.get("exit_code", ""),
            }
            w.writerow(row)
            n += 1

    print(f"已写入 {n} 行 -> {out_path}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
