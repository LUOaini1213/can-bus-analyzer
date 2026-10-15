"""Synthetic AEB scenario -> ideal CAN frame schedule + ground truth.

Ego cruises; the lead vehicle brakes hard; a simple DSU model computes TTC
from its own radar samples (the same LEAD_INFO values it puts on the bus) and
raises the AEB request in PRE_COLLISION; a first-order brake actuator with a
dead time builds up BRAKE_PRESSURE which decelerates ego.

The physics runs on an integer-millisecond clock (dt = 1 ms).  Every message
is emitted at phase + j * cycle, with a seed-dependent phase on the ms grid.
This is a toy plant chosen to exercise the analyzer, not a vehicle model.
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field

from .config import CYCLE_TIME_S, PRESSURE_X, TTC_TRIGGER_S, MIN_CLOSING_MPS
from .dbcutil import encode, load_db


@dataclass(frozen=True)
class Frame:
    t: float            # seconds since scenario start (logical bus time)
    name: str
    arbitration_id: int
    data: bytes


@dataclass(frozen=True)
class ScenarioParams:
    seed: int
    duration_s: float = 9.0
    v0_mps: float = 20.0          # 72 km/h, ego and lead
    gap0_m: float = 40.0
    lead_brake_t_s: float = 3.0
    lead_decel_mps2: float = 8.0
    dsu_proc_delay_ms: int = 20   # DSU uses a radar sample at least this old
    brake_dead_ms: int = 80       # actuator dead time after the request
    brake_tau_s: float = 0.12     # first-order pressure build-up
    p_max: float = 400.0          # raw BRAKE_PRESSURE at full AEB
    ego_decel_max: float = 9.0    # m/s^2 at p_max
    aeb_force_n: float = -6000.0  # PRE_COLLISION.FORCE while active

    @staticmethod
    def from_seed(seed: int) -> "ScenarioParams":
        rng = random.Random(f"scenario-{seed}")
        return ScenarioParams(
            seed=seed,
            gap0_m=round(rng.uniform(30.0, 45.0), 2),
            lead_brake_t_s=round(rng.uniform(2.5, 3.5), 3),
            lead_decel_mps2=round(rng.uniform(6.0, 9.0), 2),
        )


@dataclass
class Truth:
    """Ground truth from the simulation (not visible on the bus)."""
    ttc_cross_s: float | None = None      # continuous TTC < threshold, 1 ms resolution
    request_s: float | None = None        # first PRE_COLLISION frame with AEB active
    pressure_x_s: float | None = None     # plant pressure first >= PRESSURE_X
    collision: bool = False
    min_gap_m: float = float("inf")
    ego_speed_kph: list[float] = field(default_factory=list)  # per ms

    def ego_speed_range_kph(self, t0: float, t1: float) -> float:
        a = max(0, int(round(t0 * 1000)))
        b = min(len(self.ego_speed_kph) - 1, int(round(t1 * 1000)))
        if b < a:
            return 0.0
        seg = self.ego_speed_kph[a:b + 1]
        return max(seg) - min(seg)


def _phases(seed: int) -> dict[str, int]:
    rng = random.Random(f"phase-{seed}")
    return {m: rng.randrange(int(round(T * 1000))) for m, T in CYCLE_TIME_S.items()}


def _ttc(dist: float, rel_speed: float) -> float | None:
    closing = -rel_speed
    if closing <= MIN_CLOSING_MPS:
        return None
    return dist / closing


def simulate(params: ScenarioParams) -> tuple[list[Frame], Truth]:
    db = load_db()
    ids = {m: db.get_message_by_name(m).frame_id for m in CYCLE_TIME_S}
    period_ms = {m: int(round(T * 1000)) for m, T in CYCLE_TIME_S.items()}
    phase = _phases(params.seed)
    n_ms = int(round(params.duration_s * 1000))
    dt = 0.001

    x_e, v_e = 0.0, params.v0_mps
    x_l, v_l = params.gap0_m, params.v0_mps
    p = 0.0
    truth = Truth()
    radar: list[tuple[int, float | None]] = []   # (ms, ttc from bus-quantized values)
    aeb_ms: int | None = None
    pc_count = 0
    frames: list[Frame] = []

    for k in range(n_ms + 1):
        if k > 0:
            t = k * dt
            a_l = -params.lead_decel_mps2 if t >= params.lead_brake_t_s and v_l > 0 else 0.0
            v_l = max(0.0, v_l + a_l * dt)
            x_l += v_l * dt
            cmd = params.p_max if aeb_ms is not None and k >= aeb_ms + params.brake_dead_ms else 0.0
            p += (cmd - p) * dt / params.brake_tau_s
            a_e = -params.ego_decel_max * p / params.p_max
            v_e = max(0.0, v_e + a_e * dt)
            x_e += v_e * dt
            if x_l - x_e <= 0.0:      # contact: ego is pushed to lead speed
                truth.collision = True
                x_e = x_l
                v_e = min(v_e, v_l)
        gap = x_l - x_e
        truth.min_gap_m = min(truth.min_gap_m, gap)
        truth.ego_speed_kph.append(v_e * 3.6)
        ttc_true = _ttc(gap, v_l - v_e)
        if truth.ttc_cross_s is None and ttc_true is not None and ttc_true < TTC_TRIGGER_S:
            truth.ttc_cross_s = k / 1000
        if truth.pressure_x_s is None and p >= PRESSURE_X:
            truth.pressure_x_s = k / 1000

        for m in CYCLE_TIME_S:
            if k < phase[m] or (k - phase[m]) % period_ms[m]:
                continue
            if m == "SPEED":
                vals = {"SPEED": round(v_e * 3.6, 2)}
            elif m == "WHEEL_SPEEDS":
                w = round(v_e * 3.6, 2)
                vals = {f"WHEEL_SPEED_{c}": w for c in ("FL", "FR", "RL", "RR")}
            elif m == "BRAKE_MODULE":
                vals = {"BRAKE_PRESSURE": int(round(p))}
            elif m == "LEAD_INFO":
                d_raw = min(int(round(gap / 0.05)), 6000)          # 300 m DBC max
                r_raw = max(-2048, min(2047, int(round((v_l - v_e) / 0.025))))
                dist, rel = d_raw * 0.05, r_raw * 0.025
                vals = {"LEAD_LONG_DIST": dist, "LEAD_REL_SPEED": rel}
                radar.append((k, _ttc(dist, rel)))
            elif m == "ACC_CONTROL":
                vals = {"ACCEL_CMD": 0.0, "PERMIT_BRAKING": 1, "ACC_TYPE": 1}
            else:  # PRE_COLLISION
                if aeb_ms is None:
                    usable = [ttc for (ms, ttc) in radar if ms <= k - params.dsu_proc_delay_ms]
                    if usable and usable[-1] is not None and usable[-1] < TTC_TRIGGER_S:
                        aeb_ms = k
                        truth.request_s = k / 1000
                active = aeb_ms is not None
                vals = {
                    "_COUNTER": pc_count % 256,
                    "FORCE": params.aeb_force_n if active else 0.0,
                    "STATE": 3 if active else 0,
                    "PRECOLLISION_ACTIVE": 1 if active else 0,
                }
                pc_count += 1
            frames.append(Frame(k / 1000, m, ids[m], encode(m, vals)))

    frames.sort(key=lambda f: (f.t, f.arbitration_id))
    return frames, truth
