"""Score analyzer findings against injected ground-truth events."""
from __future__ import annotations

from dataclasses import dataclass

from .analyzer import Finding
from .faults import Injected

# fault kind -> finding kind expected to catch it
EXPECTED = {
    "drop": "missing", "jitter": "timing", "delay": "timing", "range": "range",
    "dlc": "dlc", "counter": "counter", "checksum": "checksum", "stuck": "stuck",
}
EPS = 5e-7   # frame times live on a 1 us grid


def matches(ev: Injected, fd: Finding) -> bool:
    if fd.kind != EXPECTED[ev.kind] or fd.name != ev.name:
        return False
    if ev.kind == "drop":           # nominal time of the lost frame lies inside the gap
        return fd.t_start < ev.t < fd.t
    if ev.kind == "delay":          # the delayed frame is an endpoint of a flagged interval
        return abs(fd.t - ev.t) < EPS or abs(fd.t_start - ev.t) < EPS
    if ev.kind == "stuck":          # intervals overlap
        return fd.t_start <= ev.t_end and ev.t <= fd.t
    return abs(fd.t - ev.t) < EPS   # single-frame faults


@dataclass(frozen=True)
class Score:
    kind: str
    injected: int
    detected: int

    @property
    def recall(self) -> float | None:
        return self.detected / self.injected if self.injected else None


def score(events: list[Injected], findings: list[Finding]) -> dict[str, Score]:
    out: dict[str, Score] = {}
    for kind in EXPECTED:
        evs = [e for e in events if e.kind == kind]
        if kind == "jitter":
            continue        # jitter perturbs every frame: no per-event ground truth
        det = sum(1 for e in evs if any(matches(e, f) for f in findings))
        out[kind] = Score(kind, len(evs), det)
    return out
