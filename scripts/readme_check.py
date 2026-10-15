"""Keep README numbers identical to the generated files under results/.

Every result number in README.md lives between <!-- BEGIN:x --> / <!-- END:x -->
markers; the block contents are rendered here from results/*/summary.json and
results/mutation.csv.

    python scripts/readme_check.py           # exit 1 if README differs from results/
    python scripts/readme_check.py --write   # rewrite the blocks from results/
"""
from __future__ import annotations

import csv
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from canbus_analyzer.sweep import render_latency_md, render_sweep_md  # noqa: E402

RES = ROOT / "results"
README = ROOT / "README.md"


def _load(p: str) -> dict:
    return json.loads((RES / p).read_text(encoding="utf-8"))


def _n(x, nd=1):
    return "n/a" if x is None else f"{x:.{nd}f}"


def block_run() -> str:
    clean, faulted = _load("run_seed0_none/summary.json"), _load("run_seed0_all/summary.json")
    L = ["| 项目 | 无故障 (seed 0) | 全部故障 (seed 0) |", "|---|---:|---:|"]
    L.append(f"| 发送 / 接收帧数 | {clean['frames_sent']} / {clean['frames_received']} | "
             f"{faulted['frames_sent']} / {faulted['frames_received']} |")
    kinds = sorted(set(clean["finding_counts"]) | set(faulted["finding_counts"]))
    for k in kinds:
        L.append(f"| 发现项 `{k}` | {clean['finding_counts'].get(k, 0)} | {faulted['finding_counts'].get(k, 0)} |")
    for k, v in faulted["detection"].items():
        L.append(f"| 注入 `{k}` 检出 / 注入 | - | {v['detected']} / {v['injected']} |")
    for key, label in (("bus_request", "总线测得 触发→制动请求 (ms)"),
                       ("truth_request", "仿真真值 触发→制动请求 (ms)"),
                       ("bus_pressure", "总线测得 触发→制动压力≥200 (ms)"),
                       ("truth_pressure", "仿真真值 触发→制动压力≥200 (ms)")):
        L.append(f"| {label} | {_n(clean['latency_ms'][key])} | {_n(faulted['latency_ms'][key])} |")
    L.append(f"| 500 kbit/s 估算负载 (%) 无填充–最坏填充 | {clean['bus_load_pct'][0]:.2f}–{clean['bus_load_pct'][1]:.2f} | "
             f"{faulted['bus_load_pct'][0]:.2f}–{faulted['bus_load_pct'][1]:.2f} |")
    for rt_c, rt_f in zip(clean["roundtrip"], faulted["roundtrip"]):
        L.append(f"| {rt_c['format'].upper()} 回读逐帧一致 (帧数, 最大时间差 µs) | {rt_c['ok']} ({rt_c['read']}, "
                 f"{rt_c['max_dt_us']:.3f}) | {rt_f['ok']} ({rt_f['read']}, {rt_f['max_dt_us']:.3f}) |")
    return "\n".join(L)


def block_sweep() -> str:
    return render_sweep_md(_load("sweep/summary.json"))


def block_mutation() -> str:
    with (RES / "mutation.csv").open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    eq = [r for r in rows if r["equivalent_reason"]]
    ne = [r for r in rows if not r["equivalent_reason"]]
    killed = sum(r["status"] == "killed" for r in ne)
    L = [f"变异体 {len(rows)} 个；非等价变异体被测试杀死 {killed}/{len(ne)}；声明为等价 {len(eq)} 个"
         f"（{', '.join(r['id'] + ' ' + r['status'] for r in eq) or '无'}）。"]
    surv = [r for r in ne if r["status"] != "killed"]
    if surv:
        L.append("未杀死: " + ", ".join(f"{r['id']} {r['mutation']}" for r in surv))
    return "\n".join(L)


def block_latency() -> str:
    return render_latency_md(_load("sweep/summary.json"))


BLOCKS = {"run": block_run, "sweep": block_sweep, "latency": block_latency, "mutation": block_mutation}


def render(readme: str) -> str:
    for name, fn in BLOCKS.items():
        pat = re.compile(rf"(<!-- BEGIN:{name} -->\n)(.*?)(<!-- END:{name} -->)", re.S)
        if not pat.search(readme):
            raise SystemExit(f"README is missing block {name}")
        readme = pat.sub(lambda m: m.group(1) + fn() + "\n" + m.group(3), readme)
    return readme


def main() -> int:
    text = README.read_text(encoding="utf-8")
    new = render(text)
    if "--write" in sys.argv:
        README.write_text(new, encoding="utf-8")
        print("README blocks rewritten from results/")
        return 0
    if new != text:
        print("README numbers differ from results/ -- run: python scripts/readme_check.py --write")
        return 1
    print("README numbers match results/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
