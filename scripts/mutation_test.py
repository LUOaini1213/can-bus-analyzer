"""Hand-written mutation testing for the analyzer.

Each mutant is a single textual replacement in a source file.  The project is
copied to a temporary directory, the mutant applied there (the working tree
is never modified), and the full pytest suite run.  A mutant is KILLED when
the suite fails.  Writes results/mutation.md + results/mutation.csv.

    python scripts/mutation_test.py
"""
from __future__ import annotations

import csv
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
A = "canbus_analyzer/analyzer.py"
D = "canbus_analyzer/dbcutil.py"
F = "canbus_analyzer/aeb_function.py"
K = "canbus_analyzer/function_kpi.py"
X = "canbus_analyzer/function_matrix.py"

# (id, file, original, mutated, description, equivalent_reason or None)
MUTANTS = [
    ("M01", A, "if d > DROP_FACTOR * period:", "if d >= DROP_FACTOR * period:",
     "drop threshold off-by-one (> -> >=)", None),
    ("M02", A, "n_miss = max(1, round(d / period) - 1)", "n_miss = max(1, round(d / period))",
     "missing-frame count off by one", None),
    ("M03", A, "elif abs(d - period) > tol:", "elif abs(d - period) >= tol:",
     "jitter tolerance boundary (> -> >=)", None),
    ("M04", A, "elif abs(d - period) > tol:", "elif False:",
     "timing (jitter/delay) detector disabled", None),
    ("M05", A, "if d > DROP_FACTOR * period:", "if False:",
     "gap / missing-frame detector disabled", None),
    ("M06", A, "pos = (len(xs) - 1) * q / 100.0", "pos = len(xs) * q / 100.0",
     "percentile index off by one", None),
    ("M07", A, "if v < lo or v > hi:", "if v < lo or v >= hi:",
     "range check: upper bound made exclusive", None),
    ("M08", A, "if v < lo or v > hi:", "if v > hi:",
     "range check ignores lower bound", None),
    ("M09", A, "return len(data) == load_db().get_message_by_name(name).length",
     "return len(data) <= load_db().get_message_by_name(name).length",
     "DLC check only catches too-long frames", None),
    ("M10", D, "    for b in data[:-1]:", "    for b in data:",
     "checksum includes its own byte", None),
    ("M11", D, "    s = len(data)\n", "    s = 0\n",
     "checksum forgets the length term", None),
    ("M12", A, "    if not has_checksum(name):\n        return True\n    return data[-1]",
     "    return True\n    return data[-1]", "checksum detector disabled", None),
    ("M13", A, "step = (c1 - c0) % modulo", "step = c1 - c0",
     "rolling counter ignores wrap-around", None),
    ("M14", A, "if step > 1 and round((t1 - t0) / period) == step:", "if step > 1:",
     "counter accepts any jump (no time-gap check)", None),
    ("M15", A, "if step == 1:\n            continue", "if step <= 1:\n            continue",
     "counter accepts a repeated value", None),
    ("M16", A, "if t1 - t0 >= STUCK_MIN_S:", "if t1 - t0 > STUCK_MIN_S:",
     "stuck duration boundary (>= -> >)", None),
    ("M17", A, "if ws and max(ws) - min(ws) > STUCK_WHEEL_DELTA_KPH:", "if ws:",
     "stuck detector drops the wheel-speed cross-check", None),
    ("M18", A, "    findings.extend(stuck_findings(speed, wheels))\n", "",
     "stuck detector disabled", None),
    ("M19", A, "return (self.request_s - self.trigger_s) * 1000.0",
     "return (self.trigger_s - self.request_s) * 1000.0",
     "latency endpoints swapped", None),
    ("M20", A, "if t >= trigger and act == 1 and force < -1e-5), None)",
     "if t > trigger and act == 1 and force < -1e-5), None)",
     "request at the trigger timestamp ignored (>= -> >)", None),
    ("M21", A, "if t >= trigger and act == 1 and force < -1e-5), None)",
     "if t >= trigger and act == 1), None)",
     "AEB request ignores FORCE < 0 condition", None),
    ("M22", A, "if v is not None and v < TTC_TRIGGER_S:", "if v is not None and v <= TTC_TRIGGER_S:",
     "TTC trigger boundary (< -> <=)", None),
    ("M23", A, "pressure = next((t for t, p in brake if t >= trigger and p >= PRESSURE_X), None)",
     "pressure = next((t for t, p in brake if t >= trigger and p > PRESSURE_X), None)",
     "pressure threshold boundary (>= -> >)", None),
    ("M24", A, "    if closing <= MIN_CLOSING_MPS:\n        return None\n    return dist / closing",
     "    if closing <= 0:\n        return None\n    return dist / closing",
     "TTC ignores minimum closing speed", None),
    ("M25", A, "        if not dlc_ok(name, f.data):", "        if False:",
     "DLC detector disabled", None),
    ("M26", A, "lo = sum(47 + 8 * len(f.data) for f in frames)", "lo = sum(44 + 8 * len(f.data) for f in frames)",
     "bus-load bit count wrong", None),
    ("M28", A, "n_miss = max(1, round(d / period) - 1)", "n_miss = round(d / period) - 1",
     "drop the max(1, .) guard in missing count",
     "EQUIVALENT: this branch only runs for d > 1.5*T, so round(d/T) >= 2 and "
     "round(d/T) - 1 >= 1 already; max(1, .) never changes the value."),
    ("M29", A, "        if not bad:\n            valid[name].append((f.t, decoded))",
     "        valid[name].append((f.t, decoded))",
     "out-of-range frames still used for latency / stuck", None),
    ("M30", A, "        bad = range_violations(name, decoded)\n", "        bad = []\n",
     "range detector disabled", None),
    ("M31", A, "        findings.extend(counter_findings(name, seq, CYCLE_TIME_S[name], modulo))\n", "",
     "rolling-counter detector disabled", None),
    ("M32", A, "    trigger = None\n    for t, dist, rel in lead:", "    trigger = None\n    for t, dist, rel in lead[1:]:",
     "latency trigger skips the first radar frame", None),
    # ---- AEB function: state machine, supervision, KPI extraction, test cases ----
    ("N01", F, "and i.ttc < c.ttc_fb_s,", "and i.ttc <= c.ttc_fb_s,",
     "full-brake TTC threshold comparison (< -> <=)", None),
    ("N02", F, "and not i.brake_pressed and i.ttc < c.ttc_fcw_s,",
     "and not i.brake_pressed and i.ttc <= c.ttc_fcw_s,",
     "warning TTC threshold comparison (< -> <=)", None),
    ("N03", F, "return i.ttc is None or i.ttc >= threshold + c.ttc_hyst_s",
     "return i.ttc is None or i.ttc >= threshold", "de-escalation without hysteresis", None),
    ("N04", F, '"fcw_threat_gone": (lambda c, i, d: d >= c.fcw_min_dwell_s and _gone(',
     '"fcw_threat_gone": (lambda c, i, d: _gone(', "FCW exit without minimum dwell", None),
    ("N05", F, "and i.t - i.ok_since >= c.fault_recover_s,", ",",
     "FAULT not latched (heals on the first valid task)", None),
    ("N06", F, '    Transition("R04", _ON, S.FAULT, "input_fault"),\n', "",
     "input fault ignored once the function is on", None),
    ("N07", F, "            and i.gas_pct < c.gas_override_pct)", "            )",
     "accelerator override does not inhibit activation", None),
    ("N08", F, "and c.v_min_kph <= i.v_ego_kph <= c.v_max_kph", "and i.v_ego_kph <= c.v_max_kph",
     "speed window ignores the lower bound", None),
    ("N09", F, "_armed(c, i) and not i.brake_pressed and i.ttc", "_armed(c, i) and i.ttc",
     "brake pedal does not suppress the warning", None),
    ("N10", F, "if t - ref > spec.timeout_cycles * spec.period_s:",
     "if t - ref >= spec.timeout_cycles * spec.period_s:", "timeout boundary (> -> >=)", None),
    ("N11", F, "if st.consecutive_bad >= spec.e2e_max_consecutive:",
     "if st.consecutive_bad > spec.e2e_max_consecutive:", "E2E error debounce off by one", None),
    ("N12", F, "        st.consecutive_bad = 0\n        st.last_valid = t", "        st.last_valid = t",
     "valid frame does not reset the E2E error count", None),
    ("N13", F, "            st.bad_total += 1\n            return None",
     "            st.bad_total += 1\n            st.last_valid = t\n            return None",
     "invalid frames refresh the timeout", None),
    ("N14", F, "            if not lo <= float(dec[sig]) <= hi:", "            if False:",
     "input range plausibility disabled", None),
    ("N15", F, '"brake_threat_gone": (lambda c, i, d: d >= c.brake_min_dwell_s and (',
     '"brake_threat_gone": (lambda c, i, d: (', "brake release without minimum dwell", None),
    ("N16", F, "        elif not ok:\n            self.ok_since = None", "        elif False:\n            self.ok_since = None",
     "healing timer not reset when inputs fail", None),
    ("N17", F, 'Transition("R11", (S.STANDBY, S.FCW, S.PARTIAL_BRAKE)', 'Transition("R11", (S.STANDBY, S.FCW)',
     "no escalation from partial to full braking", None),
    ("N18", K, "prev = [x for x in log.lead if x[0] <= t]", "prev = [x for x in log.lead if x[0] >= t]",
     "KPI TTC taken from the radar frame after the event", None),
    ("N19", K, "brk_edges = _rising([(t, a == 1 and f < -1e-5) for t, _, a, f in log.pre])",
     "brk_edges = _rising([(t, a == 1) for t, _, a, f in log.pre])",
     "KPI brake request ignores FORCE < 0", None),
    ("N20", K, "if len(data) != msg.length or (has_cs and data[-1] != toyota_checksum(m.arbitration_id, data)):",
     "if len(data) != msg.length:", "KPI uses frames with a bad checksum", None),
    ("N21", K, "                cands.append(round(a + T, 6))", "                cands.append(round(a, 6))",
     "fault onset = last frame instead of first missed deadline", None),
    ("N22", K, "degr = [t for t, _, ind in log.hud if ind == 2]", "degr = [t for t, _, ind in log.hud if ind != 0]",
     "degradation KPI also counts PCS_INDICATOR = off", None),
    ("N23", K, "            sp = [v for ts, v in log.speed if ts <= t_c]",
     "            sp = [v for ts, v in log.speed if ts <= tp]",
     "impact speed from the SPEED frame before the last radar frame", None),
    ("N24", X, "    if any(s in exp.forbidden for s in actual):\n        return False", "    if False:\n        return False",
     "state-sequence check ignores forbidden states", None),
    ("N25", X, "return k.ttc_fcw_bus is not None and k.ttc_fcw_bus >= 2.0",
     "return k.ttc_fcw_bus is not None and k.ttc_fcw_bus > 2.0", "REQ-01 boundary (>= -> >)", None),
    ("N26", X, 'if spec.family == "CCRb" or _closing0_kph(spec) <= 50.0:',
     'if spec.family == "CCRb" or _closing0_kph(spec) < 50.0:', "REQ-03 avoidance boundary (<= -> <)", None),
    ("N27", X, '"result": "PASS" if all(verdict.values()) else "FAIL"',
     '"result": "PASS" if any(verdict.values()) else "FAIL"', "test case passes if any criterion passes", None),
    ("N28", K, "if degr[0] <= t < end and a == 1 and f < -1e-5)",
     "if degr[0] <= t < end and a == 1 and f < -1e-5 and False)",
     "brake requests while FAULT is shown are never counted", None),
]



def run_suite(cwd: Path) -> bool:
    r = subprocess.run([sys.executable, "-m", "pytest", "-x", "-q", "-p", "no:cacheprovider",
                        "--ignore=tests/test_readme_numbers.py", "tests"],
                       cwd=cwd, capture_output=True, text=True, timeout=600)
    return r.returncode == 0


def main() -> int:
    rows = []
    with tempfile.TemporaryDirectory(prefix="canbus-mut-") as tmp:
        base = Path(tmp) / "proj"
        shutil.copytree(ROOT, base, ignore=shutil.ignore_patterns(
            ".venv", ".git", "results", "__pycache__", "*.asc", "*.blf"))
        if not run_suite(base):
            print("baseline test suite fails; aborting")
            return 2
        for mid, rel, old, new, desc, equiv in MUTANTS:
            path = base / rel
            src = path.read_text(encoding="utf-8")
            if src.count(old) != 1:
                raise SystemExit(f"{mid}: pattern found {src.count(old)} times in {rel}")
            path.write_text(src.replace(old, new), encoding="utf-8")
            try:
                passed = run_suite(base)
            finally:
                path.write_text(src, encoding="utf-8")
            status = "survived" if passed else "killed"
            rows.append((mid, rel, desc, status, equiv or ""))
            print(f"{mid} {status:8s} {desc}")

    killed = sum(r[3] == "killed" for r in rows)
    equiv = [r for r in rows if r[4]]
    nonequiv = [r for r in rows if not r[4]]
    nonequiv_killed = sum(r[3] == "killed" for r in nonequiv)
    out = ROOT / "results"
    out.mkdir(exist_ok=True)
    with (out / "mutation.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh, lineterminator="\n")
        w.writerow(["id", "file", "mutation", "status", "equivalent_reason"])
        w.writerows(rows)
    L = ["# Mutation testing\n",
         f"Mutants: {len(rows)}; killed: {killed}; declared equivalent: {len(equiv)}; "
         f"non-equivalent killed: {nonequiv_killed}/{len(nonequiv)}.\n",
         "| id | file | mutation | status |", "|---|---|---|---|"]
    L += [f"| {r[0]} | {r[1]} | {r[2]} | {r[3]} |" for r in rows]
    if equiv:
        L.append("\nEquivalent mutants:\n")
        L += [f"- {r[0]} ({r[3]}): {r[4]}" for r in equiv]
    (out / "mutation.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print(f"non-equivalent killed {nonequiv_killed}/{len(nonequiv)}; total killed {killed}/{len(rows)}")
    return 0 if nonequiv_killed == len(nonequiv) else 1


if __name__ == "__main__":
    raise SystemExit(main())
