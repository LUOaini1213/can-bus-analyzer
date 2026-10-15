"""Switchable fault injection on an ideal frame schedule.

Every injected fault is recorded as an `Injected` event (ground truth) so the
analyzer's findings can be scored.  Value faults are applied first, then
drops (never on a frame that already carries a value fault), then timing
faults (explicit delays, then Gaussian jitter on every frame).
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field, replace

from .config import CHECKSUM_MESSAGES, COUNTER_SIGNALS, CYCLE_TIME_S
from .dbcutil import encode, load_db
from .scenario import Frame

FAULT_KINDS = ("drop", "jitter", "delay", "range", "dlc", "counter", "checksum", "stuck")

# signal, out-of-range physical value (still representable in the raw field)
OUT_OF_RANGE = {
    "SPEED": ("SPEED", 300.0),               # DBC [0|250] km/h, raw max 655.35
    "LEAD_INFO": ("LEAD_LONG_DIST", 350.0),  # DBC [0|300] m,   raw max 409.55
    "ACC_CONTROL": ("ACCEL_CMD", 25.0),      # DBC [-20|20],    raw max 32.767
}


@dataclass(frozen=True)
class FaultConfig:
    drop_prob: float = 0.0        # per-frame drop probability (all messages)
    jitter_ms: float = 0.0        # sigma of Gaussian timestamp noise (all frames)
    delay_n: int = 0              # frames delayed by delay_frac x cycle
    delay_frac: float = 0.4
    range_n: int = 0              # frames with an out-of-range physical value
    dlc_n: int = 0                # frames one byte short (or long if < 8 bytes)
    counter_n: int = 0            # PRE_COLLISION rolling counter repeated / skipped
    checksum_n: int = 0           # frames with a corrupted CHECKSUM byte
    stuck_s: float = 0.0          # SPEED.SPEED frozen for this long (random start)

    @staticmethod
    def preset(kinds: str) -> "FaultConfig":
        """Moderate default intensities; kinds = 'none' | 'all' | 'drop,dlc,...'."""
        default = dict(drop_prob=0.01, jitter_ms=1.0, delay_n=3, range_n=3, dlc_n=3,
                       counter_n=3, checksum_n=3, stuck_s=1.0)
        key = {"drop": "drop_prob", "jitter": "jitter_ms", "delay": "delay_n", "range": "range_n",
               "dlc": "dlc_n", "counter": "counter_n", "checksum": "checksum_n", "stuck": "stuck_s"}
        if kinds == "none":
            return FaultConfig()
        chosen = FAULT_KINDS if kinds == "all" else tuple(k.strip() for k in kinds.split(","))
        bad = [k for k in chosen if k not in key]
        if bad:
            raise ValueError(f"unknown fault kind(s): {bad}; choose from {FAULT_KINDS}")
        return FaultConfig(**{key[k]: default[key[k]] for k in chosen})


@dataclass(frozen=True)
class Injected:
    kind: str
    name: str
    t: float            # actual (post-fault) frame time; for drops the nominal time
    t_end: float = 0.0  # for 'stuck': end of the frozen window


@dataclass
class FaultResult:
    frames: list[Frame]
    events: list[Injected] = field(default_factory=list)


def _pick(rng: random.Random, candidates: list[int], n: int, taken: set[int],
          spacing: int = 0) -> list[int]:
    """Pick up to n indices not in `taken` and not within `spacing` of each other."""
    pool = [i for i in candidates if i not in taken]
    rng.shuffle(pool)
    out: list[int] = []
    for i in pool:
        if len(out) == n:
            break
        if spacing and any(abs(i - j) <= spacing for j in out):
            continue
        out.append(i)
    taken.update(out)
    return sorted(out)


def inject(frames: list[Frame], cfg: FaultConfig, seed: int) -> FaultResult:
    rng = random.Random(f"faults-{seed}-{cfg}")
    db = load_db()
    fr = list(frames)
    # (kind, name, frame index, end frame index or -1); times are filled in at
    # the end so they reflect the final (post-jitter) frame times.
    pending: list[tuple[str, str, int, int]] = []
    events: list[Injected] = []
    taken: set[int] = set()
    by_name: dict[str, list[int]] = {}
    for i, f in enumerate(fr):
        by_name.setdefault(f.name, []).append(i)

    # --- value faults ---------------------------------------------------------
    if cfg.range_n:
        cand = [i for m in OUT_OF_RANGE for i in by_name[m]]
        for i in _pick(rng, cand, cfg.range_n, taken):
            f = fr[i]
            sig, val = OUT_OF_RANGE[f.name]
            vals = db.get_message_by_name(f.name).decode(f.data, decode_choices=False)
            vals[sig] = val
            fr[i] = replace(f, data=encode(f.name, vals))
            pending.append(("range", f.name, i, -1))

    if cfg.counter_n:
        cname = next(iter(COUNTER_SIGNALS))
        sig, modulo = COUNTER_SIGNALS[cname]
        idx = by_name[cname]
        # not the first/last frame, and keep faults >= 3 frames apart so each one
        # is a separate event
        pos = _pick(rng, list(range(1, len(idx) - 1)), cfg.counter_n,
                    set(), spacing=2)
        for p in pos:
            i = idx[p]
            if i in taken:
                continue
            taken.add(i)
            f = fr[i]
            vals = db.get_message_by_name(f.name).decode(f.data, decode_choices=False)
            step = rng.choice((-1, +1))      # -1: repeated value, +1: skipped value
            vals[sig] = (int(vals[sig]) + step) % modulo
            fr[i] = replace(f, data=encode(f.name, vals))
            pending.append(("counter", f.name, i, -1))

    if cfg.checksum_n:
        cand = [i for m in CHECKSUM_MESSAGES for i in by_name[m]]
        for i in _pick(rng, cand, cfg.checksum_n, taken):
            f = fr[i]
            buf = bytearray(f.data)
            buf[-1] ^= rng.randrange(1, 256)
            fr[i] = replace(f, data=bytes(buf))
            pending.append(("checksum", f.name, i, -1))

    if cfg.dlc_n:
        for i in _pick(rng, list(range(len(fr))), cfg.dlc_n, taken):
            f = fr[i]
            # classic CAN carries at most 8 data bytes, so 8-byte frames can only
            # be shortened; shorter frames are shortened or lengthened by one byte
            if len(f.data) >= 8 or rng.random() < 0.5:
                data = f.data[:-1]
            else:
                data = f.data + b"\x00"
            fr[i] = replace(f, data=data)
            pending.append(("dlc", f.name, i, -1))

    if cfg.stuck_s > 0:
        idx = by_name["SPEED"]
        t_last = fr[idx[-1]].t
        start = rng.uniform(0.5, max(0.5, t_last - cfg.stuck_s))
        window = [i for i in idx if start <= fr[i].t < start + cfg.stuck_s and i not in taken]
        if window:
            frozen = db.get_message_by_name("SPEED").decode(fr[window[0]].data, decode_choices=False)
            for i in window:
                taken.add(i)
                fr[i] = replace(fr[i], data=encode("SPEED", frozen))
            pending.append(("stuck", "SPEED", window[0], window[-1]))

    # --- drops ------------------------------------------------------------------
    keep = [True] * len(fr)
    if cfg.drop_prob > 0:
        for i, f in enumerate(fr):
            if i not in taken and rng.random() < cfg.drop_prob:
                keep[i] = False
                events.append(Injected("drop", f.name, f.t))

    # --- timing -----------------------------------------------------------------
    if cfg.delay_n:
        cand = [i for i in range(len(fr)) if keep[i] and i not in taken]
        for i in _pick(rng, cand, cfg.delay_n, taken):
            f = fr[i]
            fr[i] = replace(f, t=round(f.t + cfg.delay_frac * CYCLE_TIME_S[f.name], 6))
            pending.append(("delay", f.name, i, -1))

    if cfg.jitter_ms > 0:
        sigma = cfg.jitter_ms / 1000.0
        # timestamps are kept on a 1 us grid (the resolution of an ASC log)
        fr = [replace(f, t=max(0.0, round(f.t + rng.gauss(0.0, sigma), 6))) for f in fr]

    for kind, name, i, j in pending:
        events.append(Injected(kind, name, fr[i].t, fr[j].t if j >= 0 else 0.0))

    out = [f for f, k in zip(fr, keep) if k]
    out.sort(key=lambda f: (f.t, f.arbitration_id))
    return FaultResult(out, events)

