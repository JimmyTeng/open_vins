#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取 export_vio_log_metrics.py 生成的 CSV，生成 Markdown 详细报告：
  - 全批次：初始化成功率、飘移（达标/超门限/无法判定）统计
  - 漂移量 max_drift_m（米）：全批次与各分组下的 n、min、max、mean、std、median、p95、p99
  - 每个数据集（dataset_key）：运行次数、初始化成功率、是否飘了（超门限次数与占比）

用法:
  python3 script/generate_vio_metrics_report.py metrics.csv -o REPORT.md
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def pct(num: float, den: float) -> str:
    if den <= 0:
        return "—"
    return f"{100.0 * num / den:.2f}%"


def parse_max_drift_m(row: dict[str, Any]) -> float | None:
    """CSV 列 max_drift_m，空或非有限则 None。"""
    s = str(row.get("max_drift_m", "")).strip()
    if not s:
        return None
    try:
        v = float(s)
    except ValueError:
        return None
    if not math.isfinite(v):
        return None
    return v


def quantile_linear(xs: list[float], q: float) -> float | None:
    """q in [0,1]，线性插值分位数；xs 可为空。"""
    if not xs:
        return None
    xs = sorted(xs)
    n = len(xs)
    if n == 1:
        return xs[0]
    pos = (n - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return xs[lo]
    return xs[lo] + (pos - lo) * (xs[hi] - xs[lo])


def drift_statistics(xs: list[float]) -> dict[str, Any]:
    """返回 min/max/mean/std/median/p95/p99，xs 非空。"""
    if not xs:
        return {}
    return {
        "n": len(xs),
        "min": min(xs),
        "max": max(xs),
        "mean": statistics.mean(xs),
        "std": statistics.stdev(xs) if len(xs) > 1 else 0.0,
        "median": statistics.median(xs),
        "p95": quantile_linear(xs, 0.95),
        "p99": quantile_linear(xs, 0.99),
    }


def fmt_m(v: float | None, digits: int = 4) -> str:
    if v is None:
        return "—"
    return f"{v:.{digits}f}"


def fmt_stats_cells(st: dict[str, Any]) -> str:
    """一行表格中 n～p99 共 8 列，不含首尾竖线。"""
    return (
        f"{st['n']} | {fmt_m(st['min'])} | {fmt_m(st['max'])} | {fmt_m(st['mean'])} | {fmt_m(st['std'])} | "
        f"{fmt_m(st['median'])} | {fmt_m(st['p95'])} | {fmt_m(st['p99'])}"
    )


def empty_stats_cells() -> str:
    return "— | — | — | — | — | — | — | —"


def load_rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8-sig", newline="") as f:
        for row in csv.DictReader(f):
            rows.append(dict(row))
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description="从 metrics CSV 生成 VIO 指标 Markdown 报告")
    ap.add_argument("csv_path", help="export_vio_log_metrics.py 输出的 CSV")
    ap.add_argument(
        "-o",
        "--output",
        required=True,
        help="输出 Markdown 路径",
    )
    args = ap.parse_args()

    csv_path = Path(args.csv_path)
    if not csv_path.is_file():
        print(f"文件不存在: {csv_path}", file=sys.stderr)
        return 1

    rows = load_rows(csv_path)
    if not rows:
        print("CSV 无数据行", file=sys.stderr)
        return 1

    # 全批次
    n = len(rows)
    init_ok = sum(1 for r in rows if str(r.get("init_success", "")).strip() == "1")
    d_ok = sum(1 for r in rows if r.get("drift_status") == "达标")
    d_fail = sum(1 for r in rows if r.get("drift_status") == "超门限")
    d_unk = sum(1 for r in rows if r.get("drift_status") == "无法判定")

    # 漂移数值（用于统计）；仅 CSV 中 max_drift_m 可解析为有限浮点数者计入
    drift_all: list[float] = []
    # 与「达标/超门限」判定一致、且存在数值的子集（排除「无法判定」等）
    drift_judged: list[float] = []
    for r in rows:
        v = parse_max_drift_m(r)
        if v is not None:
            drift_all.append(v)
            if r.get("drift_status") in ("达标", "超门限"):
                drift_judged.append(v)

    # 按数据集聚合
    by_ds: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for r in rows:
        by_ds[r.get("dataset_key") or ""].append(r)

    lines: list[str] = []
    lines.append("# VIO 批次指标详细报告")
    lines.append("")
    lines.append(f"- **生成时间**: {datetime.now(timezone.utc).astimezone().strftime('%Y-%m-%d %H:%M:%S %z')}")
    lines.append(f"- **数据表**: `{csv_path.resolve()}`")
    lines.append("")

    lines.append("## 1. 全批次汇总")
    lines.append("")
    lines.append("| 指标 | 数值 |")
    lines.append("|------|------|")
    lines.append(f"| 日志总条数 | {n} |")
    lines.append(f"| 成功初始化次数 | {init_ok} |")
    lines.append(f"| **初始化成功率** | **{pct(init_ok, n)}**（{init_ok}/{n}） |")
    lines.append(f"| 飘移判定：达标 | {d_ok} |")
    lines.append(f"| 飘移判定：超门限（判定为飘移过大） | {d_fail} |")
    lines.append(f"| 飘移判定：无法判定 | {d_unk} |")
    lines.append("")

    lines.append("### 1.1 漂移量（max_drift_m，米）统计")
    lines.append("")
    lines.append(
        "说明：**全批次** = 凡 CSV 中 `max_drift_m` 可解析为有限数值的条目。"
        "**可判定子集** = `drift_status` 为「达标」或「超门限」且同上（与日志里是否给出飘移数值一致）。"
        "无有效数值的条目不参与 min/max/mean 等。"
    )
    lines.append("")
    hdr_drift_cols = "| n | min | max | mean | std | median | p95 | p99 |"
    sep_drift_cols = "|---|-----|-----|------|-----|--------|-----|-----|"
    if drift_all:
        lines.append("| 分组 " + hdr_drift_cols)
        lines.append("|------" + sep_drift_cols)
        st0 = drift_statistics(drift_all)
        lines.append(f"| 全批次（凡有数值） | {fmt_stats_cells(st0)} |")
        if drift_judged:
            stj = drift_statistics(drift_judged)
            lines.append(f"| 可判定子集（达标或超门限） | {fmt_stats_cells(stj)} |")
    else:
        lines.append("全批次无有效 `max_drift_m` 数值，无法计算漂移统计。")
    lines.append("")

    lines.append("## 2. 按数据集（每个数据）详细统计")
    lines.append("")
    lines.append(
        "说明：**数据集** = 日志文件名去掉 `__runXXX.log` 后的前缀（同一场景多次重复运行归为一组）。"
        "**初始化成功率** = 该组内 `init_success=1` 的条数 / 该组条数"
        "（解析规则见 `vio_log_metrics_core.py`：含 `Init:1` 或动态/静态初始化成功等标记）。"
        "**飘移超门限** = 该组内「飘移过大」条数；在「达标+超门限」子集内的 **飘移达标率** = 达标/(达标+超门限)。"
    )
    lines.append("")
    lines.append(
        "| 数据集 | 运行次数 | 成功初始化次数 | 初始化成功率 | 飘移达标 | 飘移超门限 | 飘移无法判定 | 可判定时飘移达标率 |"
    )
    lines.append(
        "|--------|----------|----------------|--------------|----------|------------|--------------|-------------------|"
    )

    for ds in sorted(by_ds.keys()):
        rs = by_ds[ds]
        t = len(rs)
        iok = sum(1 for r in rs if str(r.get("init_success", "")).strip() == "1")
        a = sum(1 for r in rs if r.get("drift_status") == "达标")
        b = sum(1 for r in rs if r.get("drift_status") == "超门限")
        u = sum(1 for r in rs if r.get("drift_status") == "无法判定")
        sub = a + b
        rate_d = pct(a, sub) if sub > 0 else "—"
        lines.append(
            f"| `{ds}` | {t} | {iok} | {pct(iok, t)} | {a} | {b} | {u} | {rate_d} |"
        )
    lines.append("")

    lines.append("### 2.1 各数据集漂移量（max_drift_m，米）")
    lines.append("")
    lines.append("| 数据集 " + hdr_drift_cols)
    lines.append("|--------" + sep_drift_cols)
    for ds in sorted(by_ds.keys()):
        rs = by_ds[ds]
        vals = [v for r in rs if (v := parse_max_drift_m(r)) is not None]
        if not vals:
            lines.append(f"| `{ds}` | {empty_stats_cells()} |")
            continue
        st = drift_statistics(vals)
        lines.append(f"| `{ds}` | {fmt_stats_cells(st)} |")
    lines.append("")

    # 按场景 scenario
    by_sc: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for r in rows:
        by_sc[r.get("scenario") or "unknown"].append(r)

    lines.append("## 3. 按工况（scenario）汇总")
    lines.append("")
    lines.append(
        "| 工况 | 日志条数 | 成功初始化次数 | 初始化成功率 | 飘移达标 | 飘移超门限 | 飘移无法判定 | 可判定时飘移达标率 |"
    )
    lines.append(
        "|------|----------|----------------|--------------|----------|------------|--------------|-------------------|"
    )
    for sc in sorted(by_sc.keys()):
        rs = by_sc[sc]
        t = len(rs)
        iok = sum(1 for r in rs if str(r.get("init_success", "")).strip() == "1")
        a = sum(1 for r in rs if r.get("drift_status") == "达标")
        b = sum(1 for r in rs if r.get("drift_status") == "超门限")
        u = sum(1 for r in rs if r.get("drift_status") == "无法判定")
        sub = a + b
        rate_d = pct(a, sub) if sub > 0 else "—"
        lines.append(
            f"| `{sc}` | {t} | {iok} | {pct(iok, t)} | {a} | {b} | {u} | {rate_d} |"
        )
    lines.append("")

    lines.append("### 3.1 各工况漂移量（max_drift_m，米）")
    lines.append("")
    lines.append("| 工况 " + hdr_drift_cols)
    lines.append("|------" + sep_drift_cols)
    for sc in sorted(by_sc.keys()):
        rs = by_sc[sc]
        vals = [v for r in rs if (v := parse_max_drift_m(r)) is not None]
        if not vals:
            lines.append(f"| `{sc}` | {empty_stats_cells()} |")
            continue
        st = drift_statistics(vals)
        lines.append(f"| `{sc}` | {fmt_stats_cells(st)} |")
    lines.append("")

    lines.append("## 4. 每条日志索引（详见 CSV）")
    lines.append("")
    lines.append(f"逐条明细（文件名、run、初始化、飘移数值）见 **`{csv_path.name}`**，本报告为聚合表。")
    lines.append("")

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines), encoding="utf-8")
    print(f"已写入: {out.resolve()}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
