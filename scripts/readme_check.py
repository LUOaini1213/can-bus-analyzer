"""Keep README numbers identical to the generated files under results/.

Result tables and the headline live between <!-- BEGIN:x --> / <!-- END:x -->
markers; the block contents are rendered here from results/*/summary.json and
results/mutation.csv.  Result numbers quoted in hand-written prose (Part 1
"读表" and the English summary) are listed in prose_facts(): each sentence
fragment is rendered from results/ and must appear verbatim in README.md.

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
from canbus_analyzer.function_matrix import (render_kpi_md, render_requirements_md,  # noqa: E402
                                             render_testcases_md, render_transitions_md)
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


def block_fsm() -> str:
    return render_transitions_md()


def block_reqs() -> str:
    return render_requirements_md()


def block_testcases() -> str:
    return render_testcases_md(_load("function/summary.json"))


def block_kpi() -> str:
    return render_kpi_md(_load("function/summary.json"))


MINUS = "−"


def _signed(x: float, nd: int) -> str:
    v = f"{abs(x):.{nd}f}"
    return (MINUS if x < 0 else "+") + v if float(v) else v


def _fcw_brake_ms(F: dict) -> tuple[float, float]:
    v = [k["fcw_to_brake_ms"] for k in F["kpi"] if k["sid"][:4] in ("CCRs", "CCRm")]
    return min(v), max(v)


def _span(xs: list[float]) -> str:
    return f"{min(xs):.0f}–{max(xs):.0f}"


def block_headline() -> str:
    F = _load("function/summary.json")
    K = {k["sid"]: k for k in F["kpi"]}
    c = F["config"]
    lo, hi = _fcw_brake_ms(F)
    fails = "; ".join(f"{f['scenario']} {f['failed']}" for f in F["failed_cases"])
    b2, b6, f2 = K["CCRb-12m-2"], K["CCRb-12m-6"], K["FLT-02"]
    return "\n".join([
        "- **Built:** a project-defined AEB function (6-state table-driven state machine, input supervision, "
        "fail-silent FAULT) in closed loop with a toy plant on a python-can virtual bus, frames encoded with the "
        "opendbc Toyota DBC; plus a CAN log analyzer (fault injection, cycle-time and latency checks, ASC/BLF round-trip).",
        f"- **Found:** {F['n_cases']} generated test cases, {F['n_pass']} PASS / {F['n_fail']} FAIL ({fails}). "
        f"Both CCRb 12 m cases collide (true relative impact {b2['v_rel_impact_true_kph']:.1f} / "
        f"{b6['v_rel_impact_true_kph']:.1f} km/h) because the brake is released once ego stops closing on a "
        f"still-decelerating target. FCW (TTC {c['ttc_fcw_s']} s) and partial brake (TTC {c['ttc_pb_s']} s) are "
        f"{lo:.0f}–{hi:.0f} ms apart in CCRs/CCRm, shorter than the 1.2 s driver reaction Euro NCAP assumes in "
        f"FCW tests. Fail-silent aborts braking in FLT-02 (true impact {f2['v_impact_true_kph']:.1f} km/h from "
        f"{f2['v0_kph']:.0f} km/h).",
        "- **Limits:** toy 1-D point-mass plant, ideal radar, 100 % overlap only; in-process virtual bus (no "
        "arbitration, bit timing or error frames); no real ECU, files never opened in Vector tools; no Euro NCAP "
        "score or conformance claimed.",
        "- **做了什么：** 自己设计的 AEB 功能（六状态表驱动状态机、输入监控、fail-silent 降级）与玩具车辆模型在 "
        "python-can 虚拟总线上闭环运行，报文用 opendbc Toyota DBC 编解码；另有总线日志分析器（故障注入、周期与时延检查、ASC/BLF 回读）。",
        f"- **结果：** 生成 {F['n_cases']} 条测试用例，{F['n_pass']} 条通过、{F['n_fail']} 条不通过。CCRb 12 m 两例相撞"
        f"（\"不再接近就释放\"对持续减速的目标偏早）；FCW 与部分制动只隔 {lo:.0f}–{hi:.0f} ms，短于 Euro NCAP "
        f"FCW 测试假定的 1.2 s 驾驶员反应时间；FLT-02 雷达丢失后 fail-silent 中止制动，真值 "
        f"{f2['v_impact_true_kph']:.1f} km/h 相撞。",
        "- **局限：** 一维点质量模型、理想雷达、只做 100 % 重叠；进程内虚拟总线（无仲裁、位定时、错误帧）；"
        "没接真实 ECU，也没在 Vector 工具里打开过；不声称 Euro NCAP 评分或符合性。",
    ])


def prose_facts() -> list[str]:
    """Hand-written README sentences that quote result numbers, rendered from results/."""
    F = _load("function/summary.json")
    K = {k["sid"]: k for k in F["kpi"]}
    fam = F["families"]
    b2, b6 = K["CCRb-12m-2"], K["CCRb-12m-6"]
    f1, f2, f3 = K["FLT-01"], K["FLT-02"], K["FLT-03"]
    lo, hi = _fcw_brake_ms(F)
    ccrs = [k for k in F["kpi"] if k["sid"].startswith("CCRs-")]
    ok = [k["v0_kph"] for k in ccrs if not k["contact_true"]]
    hit = [k["v0_kph"] for k in ccrs if k["contact_true"]]
    ccrm_hit = [k["v0_kph"] for k in F["kpi"] if k["sid"].startswith("CCRm-") and k["contact_true"]]
    ab = F["ab_contacts"]
    tf, tb, vi = F["ttc_fcw_bus_minus_true"], F["ttc_brake_bus_minus_true"], F["v_impact_bus_minus_true_kph"]
    worst = min((k for k in F["kpi"] if k["v_impact_bus_kph"] is not None and k["v_impact_true_kph"] is not None),
                key=lambda k: k["v_impact_bus_kph"] - k["v_impact_true_kph"])
    nt = fam["NoThreat"]
    words = {1: "one", 2: "two", 3: "three", 4: "four", 5: "five", 6: "six"}
    standby_first = [k["sid"] for k in F["kpi"] if k["states"].startswith("STANDBY")]
    return [
        f"FCW 到制动请求只有 {b6['fcw_to_brake_ms']:.0f} ms，仿真真值相对碰撞速度 {b6['v_rel_impact_true_kph']:.1f} km/h",
        f"最后以 {b2['v_rel_impact_true_kph']:.1f} km/h 相撞",
        f"CCRs / CCRm 各例 FCW→制动请求 {lo:.0f}–{hi:.0f} ms",
        f"CCRs {_span(ok)} km/h 全部避免碰撞；{_span(hit)} km/h 相撞" if max(ok) < min(hit) else "CCRs contact not monotonic",
        f"CCRm 只有 {ccrm_hit[0]:.0f} km/h 一例相撞" if len(ccrm_hit) == 1 else f"CCRm {len(ccrm_hit)} contacts",
        f"CCR {ab['n']} 例的碰撞从 {ab['default']} 例增加到 {ab['ttc_hyst_release']} 例",
        f"总线上 {f1['fault_reaction_bus_ms']:.0f} ms（FLT-01）/ {f2['fault_reaction_bus_ms']:.0f} ms（FLT-02）出现 "
        f"PCS_INDICATOR = 2；ECU 内部进入 FAULT 是 {f1['fault_reaction_true_ms']:.0f} / {f2['fault_reaction_true_ms']:.0f} ms",
        f"SPEED 连续 checksum 错误 {f3['fault_reaction_bus_ms']:.0f} ms 降级（2 帧防抖），输入恢复 {f3['heal_after_valid_s']:.2f} s 后解除",
        f"仿真真值里以 {f2['v_impact_true_kph']:.1f} km/h 相撞（初速 {f2['v0_kph']:.0f} km/h）",
        f"{nt['n']} 个无威胁场景 {nt['fcw']} 次 FCW / 制动" if nt["fcw"] == nt["brake"] else "NoThreat FCW != brake",
        f"FCW {tf['min']:.3f}–{tf['max']:.3f} s，制动请求 {tb['min']:.3f}–{tb['max']:.3f} s",
        f"碰撞速度的总线估计误差 {_signed(vi['min'], 2)} 到 {_signed(vi['max'], 2)} km/h，最差的是 {worst['sid']}"
        f"（真值 {worst['v_rel_impact_true_kph']:.1f} km/h，总线估计 {_signed(worst['v_rel_impact_bus_kph'], 1)} km/h）",
        f"{'、'.join(standby_first)} 在总线上从 STANDBY 开始",
        f"generated by running the matrix: {F['n_pass']} of {F['n_cases']} pass",
        f"would add {words.get(len(ab['ttc_hyst_release_only']), len(ab['ttc_hyst_release_only']))} low-speed CCRs collisions",
    ]


def missing_prose(readme: str) -> list[str]:
    return [s for s in prose_facts() if s not in readme]


BLOCKS = {"headline": block_headline, "run": block_run, "sweep": block_sweep, "latency": block_latency, "mutation": block_mutation,
          "fsm": block_fsm, "reqs": block_reqs, "testcases": block_testcases, "kpi": block_kpi}


def render(readme: str) -> str:
    for name, fn in BLOCKS.items():
        pat = re.compile(rf"(<!-- BEGIN:{name} -->\n)(.*?)(<!-- END:{name} -->)", re.S)
        if not pat.search(readme):
            raise SystemExit(f"README is missing block {name}")
        readme = pat.sub(lambda m: m.group(1) + fn() + "\n" + m.group(3), readme)
    return readme


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):      # non-UTF-8 Windows consoles
        sys.stdout.reconfigure(errors="backslashreplace")
    text = README.read_text(encoding="utf-8")
    new = render(text)
    if "--write" in sys.argv:
        README.write_text(new, encoding="utf-8")
        print("README blocks rewritten from results/")
        return 0
    if new != text:
        print("README numbers differ from results/ -- run: python scripts/readme_check.py --write")
        return 1
    miss = missing_prose(text)
    if miss:
        print("README prose numbers differ from results/ (fix the sentence by hand); expected fragments:")
        for m in miss:
            print("  " + m)
        return 1
    print("README numbers match results/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
