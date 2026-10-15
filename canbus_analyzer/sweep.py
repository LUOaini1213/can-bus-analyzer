"""Sweep experiments over seeds and fault intensity.

1. Detection vs intensity: each fault kind alone, several intensities, N
   seeds.  Per row: event recall, run-level detection rate, *spurious*
   findings of other kinds (e.g. jitter producing 'missing' findings) and the
   error of the bus-measured trigger->request latency vs ground truth.
2. Latency measurement vs jitter: bus-measured trigger->request and
   trigger->pressure latency against the simulator's ground truth, and
   against the same seed's jitter-free bus measurement.
"""
from __future__ import annotations

import csv
import json
import math
import statistics
from pathlib import Path

from .analyzer import percentile
from .config import STUCK_WHEEL_DELTA_KPH
from .faults import FaultConfig
from .pipeline import run_once
from .scoring import EXPECTED, matches

LEVELS: list[tuple[str, str, list[float]]] = [
    # (fault kind, FaultConfig field, intensities)
    ("drop", "drop_prob", [0.001, 0.005, 0.01, 0.05, 0.1]),
    ("jitter", "jitter_ms", [0.25, 0.5, 1.0, 2.0, 4.0, 8.0]),
    ("delay", "delay_frac", [0.1, 0.15, 0.25, 0.4, 0.6]),
    ("range", "range_n", [1, 5, 20]),
    ("dlc", "dlc_n", [1, 5, 20]),
    ("counter", "counter_n", [1, 5, 20]),
    ("checksum", "checksum_n", [1, 5, 20]),
    ("stuck", "stuck_s", [0.1, 0.25, 0.5, 1.0, 2.0]),
]
JITTER_FOR_LATENCY = [0.0, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0]


def _cfg(field: str, level: float) -> FaultConfig:
    if field == "delay_frac":
        return FaultConfig(delay_n=5, delay_frac=level)
    if field.endswith("_n"):
        return FaultConfig(**{field: int(level)})
    return FaultConfig(**{field: level})


def _stats(xs: list[float]) -> dict[str, float | int | None]:
    if not xs:
        return {"n": 0, "mean": None, "sd": None, "p95_abs": None, "max_abs": None}
    ab = [abs(x) for x in xs]
    return {"n": len(xs), "mean": statistics.fmean(xs),
            "sd": statistics.stdev(xs) if len(xs) > 1 else 0.0,
            "p95_abs": percentile(ab, 95), "max_abs": max(ab)}


def _r(x, nd=3):
    return None if x is None or (isinstance(x, float) and math.isnan(x)) else round(x, nd)


def _row(kind: str, level: float, seeds: list[int], cfg: FaultConfig) -> dict:
    inj = det = runs_det = obs = det_obs = 0
    spurious: dict[str, int] = {}
    lat_err: list[float] = []
    want = EXPECTED.get(kind)
    for s in seeds:
        r = run_once(s, cfg)
        fnd = r.analysis.findings
        hit = False
        if kind == "jitter":
            hit = any(f.kind == want for f in fnd)
        elif kind != "none":
            evs = [e for e in r.injected.events if e.kind == kind]
            hits = [e for e in evs if any(matches(e, f) for f in fnd)]
            inj += len(evs)
            det += len(hits)
            hit = bool(hits)
            if kind == "stuck":
                for e in evs:
                    # observable = true ego speed moved by more than the detector's
                    # wheel-speed threshold inside the frozen window
                    if r.truth.ego_speed_range_kph(e.t, e.t_end) > STUCK_WHEEL_DELTA_KPH:
                        obs += 1
                        det_obs += int(e in hits)
        runs_det += int(hit)
        for f in fnd:
            if f.kind != want:
                spurious[f.kind] = spurious.get(f.kind, 0) + 1
        bus, (truth, _) = r.analysis.latency.to_request_ms, r.truth_latency_ms()
        if bus is not None and truth is not None:
            lat_err.append(bus - truth)
    st = _stats(lat_err)
    per_event = kind not in ("none", "jitter")
    return {
        "fault": kind, "level": level, "runs": len(seeds),
        "injected": inj if per_event else None,
        "detected": det if per_event else None,
        "recall": _r(det / inj) if inj else None,
        "runs_detected": runs_det if kind != "none" else None,
        "observable": obs if kind == "stuck" else None,
        "detected_observable": det_obs if kind == "stuck" else None,
        "spurious": dict(sorted(spurious.items())),
        "latency_runs": st["n"],
        "req_latency_err_mean_ms": _r(st["mean"]),
        "req_latency_err_p95abs_ms": _r(st["p95_abs"]),
        "req_latency_err_maxabs_ms": _r(st["max_abs"]),
    }


def detection_sweep(seeds: list[int]) -> list[dict]:
    rows = [_row("none", 0, seeds, FaultConfig())]
    for kind, field, levels in LEVELS:
        for level in levels:
            rows.append(_row(kind, level, seeds, _cfg(field, level)))
    return rows


def latency_sweep(seeds: list[int]) -> tuple[list[dict], list[dict]]:
    raw = []
    base: dict[int, tuple[float | None, float | None]] = {}
    for sigma in JITTER_FOR_LATENCY:
        for s in seeds:
            r = run_once(s, FaultConfig(jitter_ms=sigma))
            lat = r.analysis.latency
            treq, tprs = r.truth_latency_ms()
            if sigma == 0.0:
                base[s] = (lat.to_request_ms, lat.to_pressure_ms)
            raw.append({
                "jitter_ms": sigma, "seed": s,
                "bus_request_ms": _r(lat.to_request_ms), "truth_request_ms": _r(treq),
                "bus_pressure_ms": _r(lat.to_pressure_ms), "truth_pressure_ms": _r(tprs),
                "trigger_lag_ms": _r(None if lat.trigger_s is None or r.truth.ttc_cross_s is None
                                     else (lat.trigger_s - r.truth.ttc_cross_s) * 1000.0),
            })
    rows = []
    for sigma in JITTER_FOR_LATENCY:
        sel = [x for x in raw if x["jitter_ms"] == sigma]

        def vs_truth(a: str, b: str) -> list[float]:
            return [x[a] - x[b] for x in sel if x[a] is not None and x[b] is not None]

        def vs_base(a: str, i: int) -> list[float]:
            return [x[a] - base[x["seed"]][i] for x in sel
                    if x[a] is not None and base[x["seed"]][i] is not None]

        row = {"jitter_ms": sigma, "runs": len(sel),
               "missing_request": sum(1 for x in sel if x["bus_request_ms"] is None),
               "missing_pressure": sum(1 for x in sel if x["bus_pressure_ms"] is None)}
        for key, st in (("req_vs_truth", _stats(vs_truth("bus_request_ms", "truth_request_ms"))),
                        ("prs_vs_truth", _stats(vs_truth("bus_pressure_ms", "truth_pressure_ms"))),
                        ("req_vs_nojitter", _stats(vs_base("bus_request_ms", 0))),
                        ("prs_vs_nojitter", _stats(vs_base("bus_pressure_ms", 1))),
                        ("trigger_lag", _stats([x["trigger_lag_ms"] for x in sel
                                                if x["trigger_lag_ms"] is not None]))):
            for m, v in st.items():
                row[f"{key}_{m}"] = _r(v) if isinstance(v, float) else v
        rows.append(row)
    return rows, raw


COLUMNS = ["fault", "level", "runs", "injected", "detected", "recall", "runs_detected",
           "observable", "detected_observable", "spurious", "latency_runs",
           "req_latency_err_mean_ms", "req_latency_err_p95abs_ms", "req_latency_err_maxabs_ms"]


def write_sweep(out: Path, n_seeds: int = 20) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    seeds = list(range(n_seeds))
    rows = detection_sweep(seeds)
    lat, raw = latency_sweep(seeds)
    collisions = sum(run_once(s, FaultConfig()).truth.collision for s in seeds)
    with (out / "sweep.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh, lineterminator="\n")
        w.writerow(COLUMNS)
        for d in rows:
            w.writerow([";".join(f"{k}={v}" for k, v in d[c].items()) if c == "spurious" else d[c]
                        for c in COLUMNS])
    for name, data in (("latency_vs_jitter.csv", lat), ("latency_runs.csv", raw)):
        with (out / name).open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(data[0].keys()), lineterminator="\n")
            w.writeheader()
            w.writerows(data)
    summary = {"seeds": n_seeds, "rows": rows, "latency": lat, "collisions": collisions}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    (out / "sweep.md").write_text(
        "# Sweep\n\n## Detection vs fault intensity\n\n" + render_sweep_md(summary)
        + "\n\n## Latency measurement vs bus jitter\n\n" + render_latency_md(summary) + "\n",
        encoding="utf-8")
    return summary


def _f(x, nd=1):
    return "-" if x is None else f"{x:.{nd}f}"


def render_sweep_md(s: dict) -> str:
    """One table; also embedded verbatim in README (checked by tests)."""
    L = ["| fault | level | injected | detected | recall | runs with detection | spurious findings "
         "(other kinds) | request-latency error mean / p95 abs (ms) |",
         "|---|---:|---:|---:|---:|---:|---|---:|"]
    for d in s["rows"]:
        sp = ", ".join(f"{k} {v}" for k, v in d["spurious"].items()) or "0"
        rec = _f(d["recall"], 2)
        if d["fault"] == "stuck":
            rec += f" (observable {d['detected_observable']}/{d['observable']})"
        runs = "-" if d["runs_detected"] is None else f"{d['runs_detected']}/{d['runs']}"
        L.append(f"| {d['fault']} | {d['level']:g} | {_f(d['injected'], 0)} | {_f(d['detected'], 0)} | "
                 f"{rec} | {runs} | {sp} | {_f(d['req_latency_err_mean_ms'])} / "
                 f"{_f(d['req_latency_err_p95abs_ms'])} |")
    L.append(f"\n{s['seeds']} seeds per row. Collision in {s['collisions']} of {s['seeds']} "
             f"clean scenarios (toy plant, not tuned).")
    return "\n".join(L)


def render_latency_md(s: dict) -> str:
    """Latency-vs-jitter table; also embedded verbatim in README (checked by tests)."""
    L = ["| jitter sigma (ms) | req err mean | req err sd | req p95 abs | req max abs | prs err mean "
         "| prs p95 abs | req vs no-jitter p95 abs | prs vs no-jitter p95 abs | trigger lag mean | missing req |",
         "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for r in s["latency"]:
        L.append(f"| {r['jitter_ms']:g} | {_f(r['req_vs_truth_mean'])} | {_f(r['req_vs_truth_sd'])} | "
                 f"{_f(r['req_vs_truth_p95_abs'])} | {_f(r['req_vs_truth_max_abs'])} | "
                 f"{_f(r['prs_vs_truth_mean'])} | {_f(r['prs_vs_truth_p95_abs'])} | "
                 f"{_f(r['req_vs_nojitter_p95_abs'])} | {_f(r['prs_vs_nojitter_p95_abs'])} | "
                 f"{_f(r['trigger_lag_mean'])} | {r['missing_request']} |")
    L.append(f"\n{s['seeds']} seeds per row; all values in ms. err = bus-measured latency minus "
             f"simulator ground truth; 'vs no-jitter' = same seed, bus-measured with minus without jitter; "
             f"trigger lag = bus-visible trigger (LEAD_INFO frame) minus physical TTC crossing.")
    return "\n".join(L)
