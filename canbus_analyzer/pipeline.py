"""One end-to-end run: simulate -> inject -> replay on virtual bus -> analyze."""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

import can

from .analyzer import Analysis, analyze, from_messages
from .bus import replay
from .faults import FaultConfig, FaultResult, inject
from .scenario import Frame, ScenarioParams, Truth, simulate
from .scoring import Score, score


@dataclass
class RunResult:
    seed: int
    faults: FaultConfig
    params: ScenarioParams
    truth: Truth
    injected: FaultResult
    received: list[can.Message]
    analysis: Analysis
    scores: dict[str, Score]

    @property
    def sent(self) -> int:
        return len(self.injected.frames)

    def truth_latency_ms(self) -> tuple[float | None, float | None]:
        t = self.truth
        if t.ttc_cross_s is None:
            return None, None
        req = None if t.request_s is None else (t.request_s - t.ttc_cross_s) * 1000.0
        prs = None if t.pressure_x_s is None else (t.pressure_x_s - t.ttc_cross_s) * 1000.0
        return req, prs


@lru_cache(maxsize=64)
def _scenario(seed: int) -> tuple[tuple[Frame, ...], Truth, ScenarioParams]:
    params = ScenarioParams.from_seed(seed)
    frames, truth = simulate(params)
    return tuple(frames), truth, params


def run_once(seed: int, faults: FaultConfig) -> RunResult:
    frames, truth, params = _scenario(seed)
    inj = inject(list(frames), faults, seed)
    rx = replay(inj.frames)
    if len(rx) != len(inj.frames):
        raise RuntimeError(f"virtual bus lost frames: sent {len(inj.frames)}, got {len(rx)}")
    analysis = analyze(from_messages(rx))
    return RunResult(seed, faults, params, truth, inj, rx, analysis, score(inj.events, analysis.findings))
