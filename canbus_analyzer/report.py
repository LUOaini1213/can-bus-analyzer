"""Write the single-run Markdown report + CSV files."""
from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict
from pathlib import Path

from .config import CYCLE_TIME_S, PRESSURE_X, TTC_TRIGGER_S
from .logio import RoundTrip
from .pipeline import RunResult


def fmt(x: float | None, nd: int = 1) -> str:
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "n/a"
    return f"{x:.{nd}f}"


def _r3(x: float | None) -> float | None:
    return None if x is None else round(x, 3)


def _write_csv(path: Path, header: list[str], rows: list[list]) -> None:
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh, lineterminator="\n")
        w.writerow(header)
        w.writerows(rows)


def write_run(res: RunResult, roundtrips: list[RoundTrip], out: Path) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    a = res.analysis
    lat = a.latency
    truth_req, truth_prs = res.truth_latency_ms()

    _write_csv(out / "cycle_stats.csv",
               ["message", "nominal_ms", "frames", "mean_ms", "p95_ms", "max_ms", "missing", "timing_violations"],
               [[s.name, fmt(s.nominal_ms, 0), s.n, fmt(s.mean_ms, 3), fmt(s.p95_ms, 3), fmt(s.max_ms, 3),
                 s.missing, s.timing_violations] for s in a.stats])
    _write_csv(out / "findings.csv", ["kind", "message", "t_start_s", "t_s", "detail"],
               [[f.kind, f.name, f"{f.t_start:.6f}", f"{f.t:.6f}", f.detail] for f in a.findings])
    _write_csv(out / "detection.csv", ["fault", "injected", "detected", "recall"],
               [[k, s.injected, s.detected, fmt(s.recall, 3)] for k, s in res.scores.items()])
    lat_rows = [
        ["trigger_to_request_ms", fmt(lat.to_request_ms, 3), fmt(truth_req, 3),
         fmt(None if lat.to_request_ms is None or truth_req is None else lat.to_request_ms - truth_req, 3)],
        ["trigger_to_pressure_ms", fmt(lat.to_pressure_ms, 3), fmt(truth_prs, 3),
         fmt(None if lat.to_pressure_ms is None or truth_prs is None else lat.to_pressure_ms - truth_prs, 3)],
    ]
    _write_csv(out / "latency.csv", ["metric", "bus_measured", "ground_truth", "error"], lat_rows)
    _write_csv(out / "roundtrip.csv",
               ["format", "file", "written", "read", "id_mismatch", "data_mismatch", "dlc_mismatch",
                "max_dt_us", "tolerance_us", "ok"],
               [[r.fmt, r.path, r.written, r.read, r.id_mismatch, r.data_mismatch, r.dlc_mismatch,
                 f"{r.max_dt_us:.3f}", f"{r.tolerance_us:.1f}", r.ok] for r in roundtrips])

    summary = {
        "seed": res.seed,
        "faults": asdict(res.faults),
        "scenario": {k: v for k, v in asdict(res.params).items()},
        "frames_sent": res.sent,
        "frames_received": len(res.received),
        "finding_counts": dict(sorted(a.counts.items())),
        "bus_load_pct": [round(a.bus_load_pct[0], 2), round(a.bus_load_pct[1], 2)],
        "latency_ms": {"bus_request": _r3(lat.to_request_ms), "bus_pressure": _r3(lat.to_pressure_ms),
                       "truth_request": _r3(truth_req), "truth_pressure": _r3(truth_prs)},
        "truth": {"ttc_cross_s": res.truth.ttc_cross_s, "request_s": res.truth.request_s,
                  "pressure_x_s": res.truth.pressure_x_s, "collision": res.truth.collision,
                  "min_gap_m": round(res.truth.min_gap_m, 3)},
        "detection": {k: {"injected": s.injected, "detected": s.detected} for k, s in res.scores.items()},
        "roundtrip": [{"format": r.fmt, "written": r.written, "read": r.read, "ok": r.ok,
                       "max_dt_us": round(r.max_dt_us, 3)} for r in roundtrips],
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    # ---- markdown ---------------------------------------------------------
    L: list[str] = []
    L.append(f"# Run report: seed {res.seed}\n")
    active = {k: v for k, v in asdict(res.faults).items() if v and k != "delay_frac"}
    L.append(f"Faults: `{json.dumps(active) if active else 'none'}`  ")
    L.append(f"Scenario: gap0 {res.params.gap0_m} m, lead brakes at {res.params.lead_brake_t_s} s "
             f"with {res.params.lead_decel_mps2} m/s^2, ego/lead {res.params.v0_mps*3.6:.0f} km/h.  ")
    L.append(f"Frames sent {res.sent}, received over python-can virtual bus {len(res.received)}.  ")
    L.append(f"Estimated load at 500 kbit/s: {a.bus_load_pct[0]:.2f}% (no stuffing) to "
             f"{a.bus_load_pct[1]:.2f}% (worst-case stuffing).\n")
    L.append("## Cycle time per message\n")
    L.append("| message | nominal ms | frames | mean ms | p95 ms | max ms | missing | timing violations |")
    L.append("|---|---:|---:|---:|---:|---:|---:|---:|")
    for s in a.stats:
        L.append(f"| {s.name} | {s.nominal_ms:.0f} | {s.n} | {fmt(s.mean_ms, 3)} | {fmt(s.p95_ms, 3)} | "
                 f"{fmt(s.max_ms, 3)} | {s.missing} | {s.timing_violations} |")
    L.append("\n## Findings by kind\n")
    L.append("| kind | count |\n|---|---:|")
    for k, v in sorted(a.counts.items()):
        L.append(f"| {k} | {v} |")
    if not a.counts:
        L.append("| (none) | 0 |")
    L.append("\n## Injected faults vs detected (event level)\n")
    L.append("| fault | injected | detected | recall |\n|---|---:|---:|---:|")
    for k, s in res.scores.items():
        L.append(f"| {k} | {s.injected} | {s.detected} | {fmt(s.recall, 3)} |")
    L.append("\n## AEB latency\n")
    L.append(f"Trigger = first LEAD_INFO frame with TTC < {TTC_TRIGGER_S} s (TTC = LEAD_LONG_DIST / "
             f"-LEAD_REL_SPEED). Request = first PRE_COLLISION frame with PRECOLLISION_ACTIVE=1 and "
             f"FORCE<0. Pressure = first BRAKE_MODULE frame with BRAKE_PRESSURE >= {PRESSURE_X}. "
             f"Ground truth uses the simulator's continuous TTC crossing (1 ms resolution).\n")
    L.append("| metric | bus measured ms | ground truth ms | error ms |\n|---|---:|---:|---:|")
    for row in lat_rows:
        L.append(f"| {row[0]} | {row[1]} | {row[2]} | {row[3]} |")
    L.append("\n## ASC / BLF round trip (python-can writer -> python-can reader)\n")
    L.append("| format | written | read | id mismatch | data mismatch | dlc mismatch | max dt us | ok |")
    L.append("|---|---:|---:|---:|---:|---:|---:|---|")
    for r in roundtrips:
        L.append(f"| {r.fmt} | {r.written} | {r.read} | {r.id_mismatch} | {r.data_mismatch} | "
                 f"{r.dlc_mismatch} | {r.max_dt_us:.3f} | {r.ok} |")
    L.append("\nCycle times are assumptions from `canbus_analyzer/config.py` (the DBC has no "
             "GenMsgCycleTime attributes): " + ", ".join(f"{m} {T*1000:.0f} ms" for m, T in CYCLE_TIME_S.items()) + ".")
    (out / "report.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    return summary
