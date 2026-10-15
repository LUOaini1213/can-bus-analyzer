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
