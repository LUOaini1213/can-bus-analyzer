"""Scenario matrix, project requirements and the generated functional test-case catalogue.

Scenario sources
----------------
Read from Euro NCAP "TEST PROTOCOL - AEB Car-to-Car systems", Version 3.0.2,
July 2019 (cdn.euroncap.com/cars/assets/euro_ncap_aeb_c2c_test_protocol_v302_30dae6950b.pdf):
  * CCRs VUT speeds 10-50 km/h (AEB) and 30-80 km/h (FCW), 5 km/h steps   (section 8.2.3)
  * CCRm VUT speeds 30-80 km/h, 5 km/h steps                               (section 8.2.4)
  * CCRb: VUT and GVT 50 km/h, GVT deceleration -2 and -6 m/s^2,
          headway 12 and 40 m, all combinations                            (section 8.2.5)
  * T0 = TTC 4 s for CCRs/CCRm; for CCRb T0 = GVT starts decelerating      (section 4.2.1)
  * FCW tests: braking robot reacts 1.2 s after the warning                (section 8.4.6)
NOT taken from the protocol (project settings): the CCRm target speed of
20 km/h (the protocol gives it in Figure 8-2, which did not survive text
extraction, so it is not claimed as read), 100 % overlap only (the protocol
also tests -50 %..+50 %), a step GVT deceleration (the protocol allows 1.0 s
to reach it), all no-threat / override / fault / OFF scenarios, and every
pass criterion.  No Euro NCAP score or conformance is claimed.
"""
from __future__ import annotations

import csv
import json
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable

from .aeb_function import TRANSITIONS, GUARDS, ARMED_TEXT, AebConfig, S
from .closed_loop import PREROLL_S, FaultSpec, Spec, run
from .function_kpi import Kpi, extract

NCAP = "Euro NCAP AEB C2C test protocol v3.0.2"
TTC_AT_T0 = 4.0
PROJECT = "project setting"


def _ccr_gap(v_ego: float, v_target: float) -> float:
    """Gap so that TTC = 4 s (T0) is reached after PREROLL_S of steady driving."""
    return round((v_ego - v_target) / 3.6 * (TTC_AT_T0 + PREROLL_S), 3)


def build_matrix() -> list[Spec]:
    specs: list[Spec] = []
    for v in range(10, 85, 5):
        specs.append(Spec(f"CCRs-{v:02d}", "CCRs", f"CCRs, VUT {v} km/h, stationary target",
                          v, 0.0, _ccr_gap(v, 0.0), True,
                          f"{NCAP} §8.2.3 speed range/steps; 100% overlap only ({PROJECT})",
                          duration_s=PREROLL_S + TTC_AT_T0 + 4.0))
    for v in range(30, 85, 5):
        specs.append(Spec(f"CCRm-{v:02d}", "CCRm", f"CCRm, VUT {v} km/h, target 20 km/h",
                          v, 20.0, _ccr_gap(v, 20.0), True,
                          f"{NCAP} §8.2.4 VUT speeds; target 20 km/h = {PROJECT}",
                          duration_s=PREROLL_S + TTC_AT_T0 + 4.0))
    for hw in (12, 40):
        for a in (-2.0, -6.0):
            specs.append(Spec(f"CCRb-{hw}m-{abs(a):.0f}", "CCRb",
                              f"CCRb, 50/50 km/h, headway {hw} m, target {a:g} m/s^2",
                              50.0, 50.0, float(hw), True,
                              f"{NCAP} §8.2.5 (speeds, decel, headway); step decel ({PROJECT})",
                              target_accel_mps2=a, target_accel_t_s=PREROLL_S,
                              duration_s=PREROLL_S + 12.0))
    specs += [
        Spec("NT-01", "NoThreat", "lead pulls away: ego 60, lead 20 km/h at 40 m, lead +3 m/s^2 from 0.5 s",
             60.0, 20.0, 40.0, False, PROJECT, target_accel_mps2=3.0, target_accel_t_s=0.5, duration_s=8.0),
        Spec("NT-02", "NoThreat", "cut-out: ego 70, lead 40 km/h at 40 m leaves the lane at 2.0 s (TTC ~2.8 s)",
             70.0, 40.0, 40.0, False, PROJECT, cutout_t_s=2.0, duration_s=8.0),
        Spec("NT-03", "NoThreat", "steady following: ego and lead 50 km/h, 15 m gap",
             50.0, 50.0, 15.0, False, PROJECT, duration_s=8.0),
        Spec("OVR-01", "Override", "CCRs 50 km/h, accelerator 60 % held from the start",
             50.0, 0.0, _ccr_gap(50, 0), True, PROJECT, gas_pct=60.0, gas_t_s=0.0,
             duration_s=PREROLL_S + TTC_AT_T0 + 3.0),
        Spec("OVR-02", "Override", "CCRs 50 km/h, accelerator 60 % pressed at 4.7 s (during partial braking)",
             50.0, 0.0, _ccr_gap(50, 0), True, PROJECT, gas_pct=60.0, gas_t_s=4.7,
             duration_s=PREROLL_S + TTC_AT_T0 + 3.0),
        Spec("DRV-01", "Override", "CCRs 50 km/h, driver brakes 4 m/s^2 1.2 s after the FCW frame",
             50.0, 0.0, _ccr_gap(50, 0), True, f"1.2 s reaction from {NCAP} §8.4.6; rest {PROJECT}",
             driver_brake_after_fcw_s=1.2, duration_s=PREROLL_S + TTC_AT_T0 + 4.0),
        Spec("OFF-01", "Off", "CCRs 30 km/h with the AEB main switch off",
             30.0, 0.0, _ccr_gap(30, 0), True, PROJECT, main_switch=False,
             duration_s=PREROLL_S + TTC_AT_T0 + 2.0),
        Spec("FLT-01", "Fault", "CCRs 50 km/h, LEAD_INFO stops at 3.0 s (before the threat)",
             50.0, 0.0, _ccr_gap(50, 0), True, PROJECT,
             faults=(FaultSpec("stop", "LEAD_INFO", t_start=3.0),), duration_s=PREROLL_S + TTC_AT_T0 + 2.0),
        Spec("FLT-02", "Fault", "CCRs 60 km/h, LEAD_INFO stops 0.1 s after the first brake request",
             60.0, 0.0, _ccr_gap(60, 0), True, PROJECT,
             faults=(FaultSpec("stop", "LEAD_INFO", after_brake_s=0.1),), duration_s=PREROLL_S + TTC_AT_T0 + 2.0),
        Spec("FLT-03", "Fault", "CCRs 40 km/h, SPEED checksum corrupted in 5 consecutive frames from 1.0 s",
             40.0, 0.0, _ccr_gap(40, 0), True, PROJECT,
             faults=(FaultSpec("checksum", "SPEED", t_start=1.0, n_frames=5),),
             duration_s=PREROLL_S + TTC_AT_T0 + 4.0),
        Spec("FLT-04", "Fault", "CCRs 40 km/h, one corrupted LEAD_INFO frame at 3.0 s (must be tolerated)",
             40.0, 0.0, _ccr_gap(40, 0), True, PROJECT,
             faults=(FaultSpec("checksum", "LEAD_INFO", t_start=3.0, n_frames=1),),
             duration_s=PREROLL_S + TTC_AT_T0 + 4.0),
    ]
    return specs


# ------------------------------------------------------------ requirements ---
REQUIREMENTS = {
    "REQ-01": "Threat scenario: FCW is on the bus while the bus TTC is still >= 2.0 s.",
    "REQ-02": "Threat scenario: a brake request is on the bus before contact, and not before the FCW.",
    "REQ-03": "Threat scenario: no contact if the initial closing speed is <= 50 km/h; "
              "otherwise the relative impact speed is >= 20 km/h below the initial closing speed.",
    "REQ-04": "No-threat scenario: no FCW and no brake request (false activation).",
    "REQ-05": "Accelerator >= override threshold: no brake request later than 150 ms after the pedal frame.",
    "REQ-06": "Driver brakes during FCW: the FCW is cancelled within 200 ms of BRAKE_PRESSED.",
    "REQ-07": "Input timeout / E2E error: FAULT (PCS_INDICATOR = 2) on the bus within 200 ms of the "
              "bus-observed fault onset, and no brake request while FAULT is shown.",
    "REQ-08": "FAULT heals only after the inputs have been valid for >= 1.0 s.",
    "REQ-09": "Main switch off: no FCW, no brake request, PCS_INDICATOR = 1 (off).",
    "REQ-10": "The state sequence on the bus contains the expected states in order and none of the "
              "forbidden ones.",
}

Crit = Callable[[Spec, Kpi], bool]
ANY_BRAKE = (S.PARTIAL_BRAKE.name, S.FULL_BRAKE.name)


@dataclass(frozen=True)
class Expectation:
    sequence: tuple[tuple[str, ...], ...]   # each entry: acceptable states at that position
    forbidden: tuple[str, ...]
    reqs: tuple[str, ...]


def expectation(spec: Spec) -> Expectation:
    st, fcw, flt, off = S.STANDBY.name, S.FCW.name, S.FAULT.name, S.OFF.name
    act = (fcw,) + ANY_BRAKE
    if spec.family in ("CCRs", "CCRm", "CCRb"):
        return Expectation(((st,), (fcw,), ANY_BRAKE), (flt,), ("REQ-01", "REQ-02", "REQ-03", "REQ-10"))
    if spec.family == "NoThreat":
        return Expectation(((st,),), act + (flt,), ("REQ-04", "REQ-10"))
    if spec.sid == "OVR-01":
        return Expectation(((st,),), act + (flt,), ("REQ-05", "REQ-10"))
    if spec.sid == "OVR-02":
        return Expectation(((st,), (fcw,), ANY_BRAKE, (st,)), (flt,), ("REQ-05", "REQ-10"))
    if spec.sid == "DRV-01":
        return Expectation(((st,), (fcw,), (st,)), (flt,), ("REQ-06", "REQ-10"))
    if spec.family == "Off":
        return Expectation(((off,),), (st, flt) + act, ("REQ-09", "REQ-10"))
    if spec.sid == "FLT-01":
        return Expectation(((st,), (flt,)), act, ("REQ-07", "REQ-10"))
    if spec.sid == "FLT-02":
        return Expectation(((st,), (fcw,), ANY_BRAKE, (flt,)), (), ("REQ-07", "REQ-10"))
    if spec.sid == "FLT-03":
        return Expectation(((st,), (flt,), (st,), (fcw,), ANY_BRAKE), (),
                           ("REQ-07", "REQ-08", "REQ-03", "REQ-10"))
    if spec.sid == "FLT-04":
        return Expectation(((st,), (fcw,), ANY_BRAKE), (flt,), ("REQ-01", "REQ-02", "REQ-03", "REQ-10"))
    raise KeyError(spec.sid)


def sequence_ok(actual: list[str], exp: Expectation) -> bool:
    if any(s in exp.forbidden for s in actual):
        return False
    i = 0
    for s in actual:
        if i < len(exp.sequence) and s in exp.sequence[i]:
            i += 1
    return i == len(exp.sequence)


def _closing0_kph(spec: Spec) -> float:
    return spec.v_ego_kph - spec.v_target_kph


def check(req: str, spec: Spec, k: Kpi) -> bool:
    if req == "REQ-01":
        return k.ttc_fcw_bus is not None and k.ttc_fcw_bus >= 2.0
    if req == "REQ-02" and k.contact_bus is not None:
        if k.t_brake is None or k.t_fcw is None or k.t_fcw > k.t_brake:
            return False
        return not k.contact_bus or k.t_brake < k.t_contact_bus
    if req in ("REQ-02", "REQ-03") and k.contact_bus is None:
        return False                      # cannot be verified from the bus
    if req == "REQ-03":
        if spec.family == "CCRb" or _closing0_kph(spec) <= 50.0:
            return not k.contact_bus
        return (not k.contact_bus) or (k.v_rel_impact_bus_kph is not None
                                       and k.v_rel_impact_bus_kph <= _closing0_kph(spec) - 20.0)
    if req == "REQ-04":
        return k.n_fcw == 0 and k.n_brake == 0
    if req == "REQ-05":
        return k.t_brake_after_gas is None
    if req == "REQ-06":
        return k.fcw_cancel_ms is not None and k.fcw_cancel_ms <= 200.0
    if req == "REQ-07":
        return (k.fault_reaction_bus_ms is not None and k.fault_reaction_bus_ms <= 200.0
                and k.brake_while_faulted == 0)
    if req == "REQ-08":
        return k.heal_after_valid_s is not None and k.heal_after_valid_s >= 1.0
    if req == "REQ-09":
        return k.n_fcw == 0 and k.n_brake == 0 and k.off_indicated
    if req == "REQ-10":
        return sequence_ok(k.states, expectation(spec))
    raise KeyError(req)


# ------------------------------------------------------------- execution ---
def _fmt_seq(exp: Expectation) -> str:
    return " > ".join("|".join(x) for x in exp.sequence) + (
        f"; never {', '.join(exp.forbidden)}" if exp.forbidden else "")


def _stimulus(spec: Spec) -> str:
    parts = []
    if spec.target_accel_mps2:
        parts.append(f"target {spec.target_accel_mps2:+g} m/s^2 from {spec.target_accel_t_s:g} s")
    if spec.cutout_t_s is not None:
        parts.append(f"target cut-out at {spec.cutout_t_s:g} s")
    if spec.gas_pct:
        parts.append(f"accelerator {spec.gas_pct:g} % from {spec.gas_t_s:g} s")
    if spec.driver_brake_after_fcw_s is not None:
        parts.append(f"driver brake {spec.driver_decel_mps2:g} m/s^2, {spec.driver_brake_after_fcw_s:g} s after FCW")
    for f in spec.faults:
        when = f"{f.t_start:g} s" if f.t_start is not None else f"{f.after_brake_s:g} s after first brake request"
        n = "until end" if f.n_frames is None else f"{f.n_frames} frame(s)"
        parts.append(f"{f.message} {f.kind} at {when}, {n}")
    if not spec.main_switch:
        parts.append("main switch off")
    return "; ".join(parts) or "closing in on the target, driver passive"


def _actual(k: Kpi) -> str:
    bits = [f"states {' > '.join(k.states)}"]
    if k.t_fcw is not None:
        bits.append(f"FCW at bus-TTC {k.ttc_fcw_bus}")
    if k.t_brake is not None:
        bits.append(f"brake at bus-TTC {k.ttc_brake_bus}")
    bits.append(f"contact, v_rel {k.v_rel_impact_bus_kph} km/h" if k.contact_bus else
                "contact not observable (no radar frames)" if k.contact_bus is None else "no contact")
    if k.fault_reaction_bus_ms is not None:
        bits.append(f"fault->FAULT {k.fault_reaction_bus_ms} ms")
    return "; ".join(bits)


def run_matrix(cfg: AebConfig = AebConfig()) -> list[tuple[Spec, Kpi, dict[str, bool]]]:
    out = []
    for spec in build_matrix():
        res = run(spec, cfg)
        k = extract(res, cfg)
        verdict = {r: check(r, spec, k) for r in expectation(spec).reqs}
        out.append((spec, k, verdict))
    return out


def build_testcases(results) -> list[dict]:
    rows = []
    for n, (spec, k, verdict) in enumerate(results, 1):
        exp = expectation(spec)
        rows.append({
            "id": f"TC-{n:03d}",
            "scenario": spec.sid,
            "family": spec.family,
            "requirements": ", ".join(exp.reqs),
            "preconditions": (f"ego {spec.v_ego_kph:g} km/h, target {spec.v_target_kph:g} km/h at "
                              f"{spec.gap0_m:g} m, main switch {'on' if spec.main_switch else 'off'}, "
                              f"{PREROLL_S:g} s steady driving before T0"),
            "stimulus": _stimulus(spec),
            "expected_states": _fmt_seq(exp),
            "pass_criteria": " AND ".join(exp.reqs),
            "actual": _actual(k),
            "failed": ", ".join(r for r, ok in verdict.items() if not ok),
            "result": "PASS" if all(verdict.values()) else "FAIL",
            "source": spec.source,
        })
    return rows


KPI_COLS = ["sid", "v0_kph", "ttc_fcw_bus", "ttc_fcw_true", "ttc_brake_bus", "ttc_brake_true",
            "ttc_full_bus", "fcw_to_brake_ms", "n_fcw", "n_brake", "contact_bus", "contact_true",
            "v_impact_bus_kph", "v_impact_true_kph", "v_rel_impact_bus_kph", "v_rel_impact_true_kph",
            "speed_reduction_bus_kph", "fault_reaction_bus_ms", "fault_reaction_true_ms",
            "brake_while_faulted", "heal_after_valid_s", "states"]


def _write_csv(path: Path, rows: list[dict], cols: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, lineterminator="\n", extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def _family_summary(results) -> dict:
    fam: dict[str, dict] = {}
    for spec, k, verdict in results:
        f = fam.setdefault(spec.family, {"n": 0, "pass": 0, "contact": 0, "fcw": 0, "brake": 0})
        f["n"] += 1
        f["pass"] += int(all(verdict.values()))
        f["contact"] += int(bool(k.contact_bus))
        f["fcw"] += int(k.n_fcw > 0)
        f["brake"] += int(k.n_brake > 0)
    return fam


def _span(xs: list[float]) -> dict:
    return {"n": len(xs), "min": min(xs) if xs else None, "max": max(xs) if xs else None}


def write_function_results(out: Path) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    cfg = AebConfig()
    results = run_matrix(cfg)
    alt = run_matrix(replace(cfg, brake_release_ttc_hyst=True))

    kpi_rows = [k.row() for _, k, _ in results]
    _write_csv(out / "kpi.csv", kpi_rows, KPI_COLS)
    tc = build_testcases(results)
    _write_csv(out / "testcases.csv", tc, list(tc[0].keys()))

    def errs(a: str, b: str) -> list[float]:
        pairs = ((getattr(k, a), getattr(k, b)) for _, k, _ in results)
        return [round(x - y, 3) for x, y in pairs if x is not None and y is not None]

    ab = []
    for (s1, k1, _), (_, k2, _) in zip(results, alt):
        if s1.family in ("CCRs", "CCRm", "CCRb"):
            ab.append({"sid": s1.sid,
                       "default_contact": k1.contact_bus, "default_v_rel_kph": k1.v_rel_impact_bus_kph,
                       "ttc_hyst_release_contact": k2.contact_bus,
                       "ttc_hyst_release_v_rel_kph": k2.v_rel_impact_bus_kph,
                       "ttc_hyst_release_states": " > ".join(k2.states)})
    _write_csv(out / "release_ab.csv", ab, list(ab[0].keys()))

    summary = {
        "config": dict(cfg.__dict__),
        "n_cases": len(tc),
        "n_pass": sum(r["result"] == "PASS" for r in tc),
        "n_fail": sum(r["result"] == "FAIL" for r in tc),
        "failed_cases": [{"id": r["id"], "scenario": r["scenario"], "failed": r["failed"]}
                         for r in tc if r["result"] == "FAIL"],
        "families": _family_summary(results),
        "families_ttc_hyst_release": _family_summary(alt),
        "ab_contacts": {"default": sum(bool(r["default_contact"]) for r in ab),
                        "ttc_hyst_release": sum(bool(r["ttc_hyst_release_contact"]) for r in ab),
                        "n": len(ab),
                        "ttc_hyst_release_only": [r["sid"] for r in ab
                                                  if r["ttc_hyst_release_contact"] and not r["default_contact"]]},
        "ttc_fcw_bus_minus_true": _span(errs("ttc_fcw_bus", "ttc_fcw_true")),
        "ttc_brake_bus_minus_true": _span(errs("ttc_brake_bus", "ttc_brake_true")),
        "v_impact_bus_minus_true_kph": _span(errs("v_impact_bus_kph", "v_impact_true_kph")),
        "fault_reaction_bus_minus_true_ms": errs("fault_reaction_bus_ms", "fault_reaction_true_ms"),
        "contact_bus_vs_true_mismatch": [k.sid for _, k, _ in results
                                         if k.contact_bus is not None and k.contact_bus != k.contact_true],
        "contact_unobservable_on_bus": [k.sid for _, k, _ in results if k.contact_bus is None],
        "kpi": kpi_rows,
        "testcases": tc,
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (out / "testcases.md").write_text("# Functional test cases\n\n" + render_testcases_md(summary, full=True)
                                      + "\n", encoding="utf-8")
    (out / "kpi.md").write_text("# Function KPIs per scenario\n\n" + render_kpi_md(summary) + "\n", encoding="utf-8")
    return summary


# ------------------------------------------------------------ rendering ---
def _v(x, nd=2):
    if x is None:
        return "-"
    if isinstance(x, bool):
        return "yes" if x else "no"
    if x is None:
        return "-"
    if isinstance(x, float):
        return f"{x:.{nd}f}"
    return str(x)


def render_transitions_md() -> str:
    c = AebConfig()
    L = ["| rule | from | to | condition (rows checked top to bottom, first match wins) |", "|---|---|---|---|"]
    for tr in TRANSITIONS:
        L.append(f"| {tr.rid} | {', '.join(s.name for s in tr.sources)} | {tr.target.name} | "
                 f"{GUARDS[tr.guard][1]} |")
    L.append("")
    L.append(ARMED_TEXT + ".")
    L.append("")
    L.append("| parameter (project choice) | value |")
    L.append("|---|---:|")
    for a, av in c.__dict__.items():
        L.append(f"| `{a}` | {av} |")
    return "\n".join(L)


def render_requirements_md() -> str:
    L = ["| id | requirement (project-defined pass criterion) |", "|---|---|"]
    L += [f"| {k} | {v} |" for k, v in REQUIREMENTS.items()]
    return chr(10).join(L)


def render_kpi_md(s: dict) -> str:
    L = ["| scenario | v0 km/h | TTC@FCW bus / true (s) | TTC@brake bus / true (s) | TTC@full bus (s) | "
         "FCW→brake (ms) | contact | v_rel impact bus / true (km/h) | fault→FAULT bus / true (ms) | states on bus |",
         "|---|---:|---:|---:|---:|---:|---|---:|---:|---|"]
    for k in s["kpi"]:
        L.append(f"| {k['sid']} | {_v(k['v0_kph'], 0)} | {_v(k['ttc_fcw_bus'])} / {_v(k['ttc_fcw_true'])} | "
                 f"{_v(k['ttc_brake_bus'])} / {_v(k['ttc_brake_true'])} | {_v(k['ttc_full_bus'])} | "
                 f"{_v(k['fcw_to_brake_ms'], 0)} | {'n/a' if k['contact_bus'] is None else _v(k['contact_bus'])} | "
                 f"{_v(k['v_rel_impact_bus_kph'], 1)} / {_v(k['v_rel_impact_true_kph'], 1)} | "
                 f"{_v(k['fault_reaction_bus_ms'], 0)} / {_v(k['fault_reaction_true_ms'], 0)} | {k['states']} |")
    t, b = s["ttc_fcw_bus_minus_true"], s["ttc_brake_bus_minus_true"]
    v, r = s["v_impact_bus_minus_true_kph"], s["fault_reaction_bus_minus_true_ms"]
    mm = s["contact_bus_vs_true_mismatch"]
    L.append("")
    L.append(f"Bus minus truth: TTC at FCW {_v(t['min'])} to {_v(t['max'])} s (n={t['n']}); "
             f"TTC at brake request {_v(b['min'])} to {_v(b['max'])} s (n={b['n']}); "
             f"impact speed {_v(v['min'])} to {_v(v['max'])} km/h (n={v['n']}); "
             f"fault reaction {', '.join(_v(x, 1) for x in r) or '-'} ms; "
             f"contact verdict differs in {len(mm)} scenario(s){' (' + ', '.join(mm) + ')' if mm else ''}; "
             f"not observable from the bus in {', '.join(s['contact_unobservable_on_bus']) or 'none'} "
             f"(LEAD_INFO stopped; the simulator reports contact there).")
    return "\n".join(L)


def render_testcases_md(s: dict, full: bool = False) -> str:
    L = [f"{s['n_cases']} test cases generated by running the matrix: {s['n_pass']} PASS, {s['n_fail']} FAIL.", ""]
    L.append("| family | cases | pass | contact | FCW issued | brake requested |")
    L.append("|---|---:|---:|---:|---:|---:|")
    for fam, f in s["families"].items():
        L.append(f"| {fam} | {f['n']} | {f['pass']} | {f['contact']} | {f['fcw']} | {f['brake']} |")
    if s["failed_cases"]:
        L.append("")
        L.append("Failed: " + "; ".join(f"{c['id']} {c['scenario']} ({c['failed']})" for c in s["failed_cases"]) + ".")
    ab = s["ab_contacts"]
    L.append("")
    L.append(f"Brake-release A/B over the {ab['n']} CCR cases: contacts with the default rule "
             f"(release only when no longer closing, or at standstill) {ab['default']}; with the "
             f"TTC-hysteresis release variant {ab['ttc_hyst_release']}"
             + (f" (extra contacts: {', '.join(ab['ttc_hyst_release_only'])})" if ab["ttc_hyst_release_only"] else "")
             + ".")
    if full:
        L.append("")
        L.append("| id | scenario | requirements | preconditions | stimulus | expected states | actual | result |")
        L.append("|---|---|---|---|---|---|---|---|")
        for r in s["testcases"]:
            res = r["result"] + (f" ({r['failed']})" if r["failed"] else "")
            L.append(f"| {r['id']} | {r['scenario']} | {r['requirements']} | {r['preconditions']} | "
                     f"{r['stimulus']} | {r['expected_states']} | {r['actual']} | {res} |")
    return "\n".join(L)
