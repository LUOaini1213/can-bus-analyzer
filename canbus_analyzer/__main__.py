"""CLI.

  python -m canbus_analyzer run   --seed 0 --faults all       # one run -> results/run_seed0_all/
  python -m canbus_analyzer run   --seed 0 --faults drop,dlc
  python -m canbus_analyzer sweep --seeds 20                  # -> results/sweep/
  python -m canbus_analyzer function                          # AEB function scenario matrix -> results/function/
  python -m canbus_analyzer all                               # clean + faulted run + sweep + function matrix
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .config import RESULTS_DIR
from .faults import FAULT_KINDS, FaultConfig
from .function_matrix import write_function_results
from .logio import roundtrip
from .pipeline import run_once
from .report import write_run
from .sweep import write_sweep


def cmd_run(seed: int, faults: str, out: Path | None) -> int:
    cfg = FaultConfig.preset(faults)
    out = out or RESULTS_DIR / f"run_seed{seed}_{faults.replace(',', '+')}"
    res = run_once(seed, cfg)
    rts = roundtrip(res.received, out)
    summary = write_run(res, rts, out)
    lat = summary["latency_ms"]
    print(f"[run] seed={seed} faults={faults} -> {out}")
    print(f"      frames sent/received {summary['frames_sent']}/{summary['frames_received']}, "
          f"findings {summary['finding_counts']}")
    print(f"      latency trigger->request {lat['bus_request']} ms, trigger->pressure {lat['bus_pressure']} ms")
    print("      round trip: " + ", ".join(f"{r['format']} ok={r['ok']}" for r in summary["roundtrip"]))
    return 0 if all(r["ok"] for r in summary["roundtrip"]) else 1


def cmd_function(out: Path) -> int:
    s = write_function_results(out)
    print(f"[function] {s['n_cases']} test cases -> {out}: {s['n_pass']} PASS, {s['n_fail']} FAIL")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="canbus_analyzer", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="one scenario run with optional faults")
    r.add_argument("--seed", type=int, default=0)
    r.add_argument("--faults", default="none",
                   help=f"'none', 'all' or a comma list of {','.join(FAULT_KINDS)}")
    r.add_argument("--out", type=Path, default=None)
    s = sub.add_parser("sweep", help="detection / latency sweep over seeds")
    s.add_argument("--seeds", type=int, default=20)
    s.add_argument("--out", type=Path, default=RESULTS_DIR / "sweep")
    f = sub.add_parser("function", help="AEB function scenario matrix, KPIs and test-case catalogue")
    f.add_argument("--out", type=Path, default=RESULTS_DIR / "function")
    sub.add_parser("all", help="clean run, all-faults run, sweep and function matrix (regenerates results/)")
    args = ap.parse_args(argv)

    if args.cmd == "run":
        return cmd_run(args.seed, args.faults, args.out)
    if args.cmd == "sweep":
        summary = write_sweep(args.out, args.seeds)
        print(f"[sweep] {args.seeds} seeds -> {args.out}  (collisions in {summary['collisions']} scenarios)")
        return 0
    if args.cmd == "function":
        return cmd_function(args.out)
    rc = cmd_run(0, "none", RESULTS_DIR / "run_seed0_none")
    rc |= cmd_run(0, "all", RESULTS_DIR / "run_seed0_all")
    summary = write_sweep(RESULTS_DIR / "sweep", 20)
    print(f"[sweep] 20 seeds -> {RESULTS_DIR / 'sweep'}  (collisions in {summary['collisions']} scenarios)")
    cmd_function(RESULTS_DIR / "function")
    return rc


if __name__ == "__main__":
    sys.exit(main())
