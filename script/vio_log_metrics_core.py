#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VIO 批次日志解析核心（初始化时间、飘移、exit_code），供 export_vio_log_metrics.py 使用。
与 analyze_vio_log_zupt_state.py 中 init_drift 逻辑保持一致。
"""

from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any

ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")

VM_LINE_RE = re.compile(
    r"VioManager\.cpp:\d+\s+\[VM\]\s+帧\s+#(\d+)\s*:\s*([0-9.]+)\s*s,.*Init:\s*([01])\b"
)
VM_LINE_LOOSE_RE = re.compile(
    r"\[VM\]\s+帧\s+#(\d+)\s*:\s*([0-9.]+)\s*s,.*Init:\s*([01])\b"
)
PIN_RE = re.compile(
    r"p_IinG\s*=\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)"
)
EXIT_RE = re.compile(r"exit_code=(\d+)")

TIGHT_SCENARIO_NAMES = frozenset({"takeoff", "hover", "landing", "static"})


def strip_ansi(s: str) -> str:
    return ANSI_RE.sub("", s)


def scenario_and_threshold(
    source: str,
    tight_m: float,
    loose_m: float,
) -> tuple[str, float]:
    base = Path(source).name.lower()
    prefix = base.split("__", 1)[0] if "__" in base else ""
    if prefix in TIGHT_SCENARIO_NAMES:
        return (prefix, tight_m)
    if prefix == "move":
        return ("move", loose_m)
    return ("integrate", loose_m)


def compute_init_drift_metrics(
    clean: list[str],
    source: str,
    tight_m: float,
    loose_m: float,
) -> dict[str, Any]:
    scenario, drift_threshold_m = scenario_and_threshold(source, tight_m, loose_m)

    t_start: float | None = None
    t_init: float | None = None
    init_line_idx: int | None = None

    for i, ln in enumerate(clean):
        m = VM_LINE_RE.search(ln) or VM_LINE_LOOSE_RE.search(ln)
        if not m:
            continue
        ts = float(m.group(2))
        init_flag = int(m.group(3))
        if t_start is None:
            t_start = ts
        if init_flag == 1 and t_init is None:
            t_init = ts
            init_line_idx = i

    init_duration_s: float | None = None
    if t_start is not None and t_init is not None:
        init_duration_s = t_init - t_start

    max_drift_m: float | None = None
    drift_ok: bool | None = None
    num_pos_samples = 0
    _sanity = 1.0e5

    if init_line_idx is not None:
        positions: list[tuple[float, float, float]] = []
        for j in range(init_line_idx + 1, len(clean)):
            m = PIN_RE.search(clean[j])
            if m:
                x, y, z = float(m.group(1)), float(m.group(2)), float(m.group(3))
                if not all(math.isfinite(c) and abs(c) <= _sanity for c in (x, y, z)):
                    continue
                positions.append((x, y, z))
        num_pos_samples = len(positions)
        if positions:
            p0 = positions[0]

            def dist3(
                a: tuple[float, float, float], b: tuple[float, float, float]
            ) -> float:
                return math.hypot(
                    a[0] - b[0],
                    math.hypot(a[1] - b[1], a[2] - b[2]),
                )

            try:
                max_drift_m = max(dist3((x, y, z), p0) for x, y, z in positions)
            except (OverflowError, ValueError):
                max_drift_m = None
            if max_drift_m is not None and math.isfinite(max_drift_m):
                drift_ok = max_drift_m <= drift_threshold_m
            else:
                max_drift_m = None
                drift_ok = None

    return {
        "scenario_from_filename": scenario,
        "drift_threshold_m": drift_threshold_m,
        "t_first_frame_s": t_start,
        "t_first_init1_s": t_init,
        "init_duration_s": init_duration_s,
        "init_line_found": init_line_idx is not None,
        "max_drift_m": max_drift_m,
        "drift_ok": drift_ok,
        "pos_samples_after_init": num_pos_samples,
    }


def analyze_text_for_export(
    text: str,
    source: str,
    *,
    tight_m: float = 0.3,
    loose_m: float = 10.0,
) -> dict[str, Any]:
    """供 CSV 导出：仅 init_drift + exit_code。"""
    clean = [strip_ansi(ln) for ln in text.splitlines()]
    init_drift = compute_init_drift_metrics(clean, source, tight_m, loose_m)
    exit_code: int | None = None
    for ln in clean:
        m = EXIT_RE.search(ln)
        if m:
            exit_code = int(m.group(1))
    return {
        "init_drift": init_drift,
        "other_counts": {"exit_code": exit_code},
    }


def gather_files(paths: list[str]) -> list[Path]:
    out: list[Path] = []
    for p in paths:
        pp = Path(p)
        if pp.is_file():
            out.append(pp)
        elif pp.is_dir():
            out.extend(sorted(pp.glob("*.log")))
        else:
            if Path(p).exists():
                out.append(Path(p))
    seen = set()
    uniq: list[Path] = []
    for f in out:
        r = f.resolve()
        if r not in seen:
            seen.add(r)
            uniq.append(f)
    return uniq
