"""Function KPIs extracted from the recorded bus log (decoded frames only),
and the simulator ground truth they are compared with.

Bus side (what a tester with only the log would see):
  FCW onset        first PCS_HUD frame with FCW = 1
  brake request    first PRE_COLLISION frame with PRECOLLISION_ACTIVE = 1 and FORCE < 0
  full brake       first PRE_COLLISION frame with STATE = FULL_BRAKE (project encoding)
  TTC at an event  LEAD_LONG_DIST / -LEAD_REL_SPEED of the latest valid LEAD_INFO
                   frame at or before the event frame
  contact          first valid LEAD_INFO frame with LEAD_LONG_DIST == 0 (raw 0, i.e. < 2.5 cm)
  impact speed     SPEED at the contact time extrapolated from the previous LEAD_INFO frame
  degradation      first PCS_HUD frame with PCS_INDICATOR = 2 (faulted)
  fault onset      first monitored-input frame that fails DLC/checksum, or the first
                   missed deadline (previous frame + one cycle) of a gap > 1.5 cycles
  state sequence   PRE_COLLISION.STATE values, consecutive duplicates removed
Frames that fail DLC or checksum are not used for any KPI.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field

import can

from .aeb_function import INPUT_CYCLES_S, AebConfig, S
from .analyzer import ttc as ttc_from_radar
from .closed_loop import ClosedLoopResult
from .config import DROP_FACTOR, EPOCH
from .dbcutil import load_db, toyota_checksum, unpack_byte_signal

DECODED = ("SPEED", "LEAD_INFO", "PRE_COLLISION", "PCS_HUD", "BRAKE_MODULE", "GAS_PEDAL")


@dataclass
class BusLog:
    lead: list[tuple[float, float, float]] = field(default_factory=list)  # t, dist, rel
    speed: list[tuple[float, float]] = field(default_factory=list)        # t, km/h
    pre: list[tuple[float, int, int, float]] = field(default_factory=list)  # t, state, active, force
    hud: list[tuple[float, int, int]] = field(default_factory=list)       # t, fcw, indicator
    brake_pressed: list[tuple[float, int]] = field(default_factory=list)
    gas: list[tuple[float, float]] = field(default_factory=list)
    times: dict[str, list[float]] = field(default_factory=dict)           # every frame, per input
    invalid: list[tuple[float, str]] = field(default_factory=list)


def decode_log(msgs: list[can.Message]) -> BusLog:
    db = load_db()
    by_id = {db.get_message_by_name(n).frame_id: n for n in DECODED}
    out = BusLog(times={n: [] for n in INPUT_CYCLES_S})
    for m in sorted(msgs, key=lambda x: x.timestamp):
        name = by_id.get(m.arbitration_id)
        if name is None:
            continue
        t = round(m.timestamp - EPOCH, 6)
        data = bytes(m.data)
        if name in out.times:
            out.times[name].append(t)
        msg = db.get_message_by_name(name)
        has_cs = any(s.name == "CHECKSUM" for s in msg.signals)
        if len(data) != msg.length or (has_cs and data[-1] != toyota_checksum(m.arbitration_id, data)):
            out.invalid.append((t, name))
            continue
        if name == "PCS_HUD":
            out.hud.append((t, unpack_byte_signal(name, "FCW", data), unpack_byte_signal(name, "PCS_INDICATOR", data)))
            continue
        d = msg.decode(data, decode_choices=False)
        if name == "LEAD_INFO":
            out.lead.append((t, float(d["LEAD_LONG_DIST"]), float(d["LEAD_REL_SPEED"])))
        elif name == "SPEED":
            out.speed.append((t, float(d["SPEED"])))
        elif name == "PRE_COLLISION":
            out.pre.append((t, int(d["STATE"]), int(d["PRECOLLISION_ACTIVE"]), float(d["FORCE"])))
        elif name == "BRAKE_MODULE":
            out.brake_pressed.append((t, int(d["BRAKE_PRESSED"])))
        elif name == "GAS_PEDAL":
            out.gas.append((t, float(d["GAS_PEDAL"])))
    return out


def _bus_ttc(log: BusLog, t: float) -> float | None:
    prev = [x for x in log.lead if x[0] <= t]
    if not prev or prev[-1][1] >= 300.0:
        return None
    return ttc_from_radar(prev[-1][1], prev[-1][2])


def _rising(seq: list[tuple[float, bool]]) -> list[float]:
    out, last = [], False
    for t, v in seq:
        if v and not last:
            out.append(t)
        last = v
    return out


def state_sequence(log: BusLog) -> list[str]:
    seq: list[str] = []
    for _, st, _, _ in log.pre:
        name = S(st).name if st in S._value2member_map_ else f"?{st}"
        if not seq or seq[-1] != name:
            seq.append(name)
    return seq


def fault_onset(log: BusLog, after: float = 0.0) -> float | None:
    cands = [t for t, n in log.invalid if n in INPUT_CYCLES_S and t >= after]
    for name, ts in log.times.items():
        T = INPUT_CYCLES_S[name]
        for a, b in zip(ts, ts[1:]):
            if b - a > DROP_FACTOR * T and a + T >= after:
                cands.append(round(a + T, 6))
                break
        if ts and log.pre and log.pre[-1][0] - ts[-1] > DROP_FACTOR * T and ts[-1] + T >= after:
            cands.append(round(ts[-1] + T, 6))      # message stopped for good
    return min(cands) if cands else None


@dataclass
class Kpi:
    sid: str
    v0_kph: float | None = None
    t_fcw: float | None = None
    ttc_fcw_bus: float | None = None
    ttc_fcw_true: float | None = None
    t_brake: float | None = None
    ttc_brake_bus: float | None = None
    ttc_brake_true: float | None = None
    t_full: float | None = None
    ttc_full_bus: float | None = None
    fcw_to_brake_ms: float | None = None
    n_fcw: int = 0
    n_brake: int = 0
    contact_bus: bool | None = False   # None: not observable (radar frames stopped)
    contact_true: bool = False
    v_impact_bus_kph: float | None = None
    v_impact_true_kph: float | None = None
    v_rel_impact_bus_kph: float | None = None
    v_rel_impact_true_kph: float | None = None
    speed_reduction_bus_kph: float | None = None
    t_fault_onset_bus: float | None = None
    t_fault_onset_true: float | None = None
    t_degraded_bus: float | None = None
    t_fault_entry_ecu: float | None = None
    fault_reaction_bus_ms: float | None = None
    fault_reaction_true_ms: float | None = None
    t_recovered_bus: float | None = None
    brake_while_faulted: int = 0
    heal_after_valid_s: float | None = None
    t_contact_bus: float | None = None
    t_gas_bus: float | None = None
    t_brake_after_gas: float | None = None
    t_brake_pedal_bus: float | None = None
    fcw_cancel_ms: float | None = None
    off_indicated: bool = False
    states: list[str] = field(default_factory=list)
    invalid_frames: int = 0

    def row(self) -> dict:
        d = asdict(self)
        d["states"] = " > ".join(self.states)
        return d


def _r(x, nd=3):
    return None if x is None else round(x, nd)


def extract(res: ClosedLoopResult, cfg: AebConfig = AebConfig()) -> Kpi:
    log = decode_log(res.log)
    tr = res.truth
    k = Kpi(sid=res.spec.sid, states=state_sequence(log), invalid_frames=len(log.invalid))
    k.v0_kph = log.speed[0][1] if log.speed else None

    fcw_edges = _rising([(t, bool(f)) for t, f, _ in log.hud])
    brk_edges = _rising([(t, a == 1 and f < -1e-5) for t, _, a, f in log.pre])
    k.n_fcw, k.n_brake = len(fcw_edges), len(brk_edges)
    if fcw_edges:
        k.t_fcw = fcw_edges[0]
        k.ttc_fcw_bus = _r(_bus_ttc(log, k.t_fcw))
        k.ttc_fcw_true = _r(tr.ttc_at(k.t_fcw))
    if brk_edges:
        k.t_brake = brk_edges[0]
        k.ttc_brake_bus = _r(_bus_ttc(log, k.t_brake))
        k.ttc_brake_true = _r(tr.ttc_at(k.t_brake))
    full = next((t for t, st, _, _ in log.pre if st == S.FULL_BRAKE), None)
    if full is not None:
        k.t_full = full
        k.ttc_full_bus = _r(_bus_ttc(log, full))
    if k.t_fcw is not None and k.t_brake is not None:
        k.fcw_to_brake_ms = _r((k.t_brake - k.t_fcw) * 1000.0, 1)

    # contact from the bus
    for i, (t, dist, rel) in enumerate(log.lead):
        if dist == 0.0 and i > 0:
            tp, dp, rp = log.lead[i - 1]
            closing = -rp
            t_c = tp + dp / closing if closing > 0 else t
            sp = [v for ts, v in log.speed if ts <= t_c]
            sp_prev = [v for ts, v in log.speed if ts <= tp]
            if sp and sp_prev:
                k.contact_bus = True
                k.t_contact_bus = _r(t_c, 6)
                k.v_impact_bus_kph = _r(sp[-1], 2)
                v_target = sp_prev[-1] + rp * 3.6
                k.v_rel_impact_bus_kph = _r(sp[-1] - v_target, 2)
                if k.v0_kph is not None:
                    k.speed_reduction_bus_kph = _r(k.v0_kph - sp[-1], 2)
            break
    end = max((t for t, *_ in log.pre), default=0.0)
    if not k.contact_bus and (not log.lead or end - log.lead[-1][0] > DROP_FACTOR * INPUT_CYCLES_S["LEAD_INFO"]):
        k.contact_bus = None
    k.contact_true = tr.contact_t is not None
    k.v_impact_true_kph = _r(tr.v_impact_kph, 2)
    k.v_rel_impact_true_kph = _r(tr.v_rel_impact_kph, 2)

    # degradation
    startup = 0.5
    k.t_fault_onset_bus = fault_onset(log, after=startup)
    k.t_fault_onset_true = tr.fault_frames[0][2] if tr.fault_frames else None
    degr = [t for t, _, ind in log.hud if ind == 2]
    if degr:
        k.t_degraded_bus = degr[0]
        later_ok = [t for t, _, ind in log.hud if ind == 0 and t > degr[0]]
        k.t_recovered_bus = later_ok[0] if later_ok else None
        end = k.t_recovered_bus if k.t_recovered_bus is not None else float("inf")
        k.brake_while_faulted = sum(1 for t, _, a, f in log.pre
                                    if degr[0] <= t < end and a == 1 and f < -1e-5)
    if k.t_fault_onset_bus is not None and k.t_degraded_bus is not None:
        k.fault_reaction_bus_ms = _r((k.t_degraded_bus - k.t_fault_onset_bus) * 1000.0, 1)
    if k.t_recovered_bus is not None:
        bad = [t for t, n in log.invalid if n in INPUT_CYCLES_S and t < k.t_recovered_bus]
        if bad:
            k.heal_after_valid_s = _r(k.t_recovered_bus - bad[-1])

    # driver override
    k.t_gas_bus = next((t for t, g in log.gas if g >= cfg.gas_override_pct), None)
    if k.t_gas_bus is not None:
        k.t_brake_after_gas = next((t for t, _, a, f in log.pre
                                    if t > k.t_gas_bus + 0.150 and a == 1 and f < -1e-5), None)
    k.t_brake_pedal_bus = next((t for t, b in log.brake_pressed if b), None)
    if k.t_brake_pedal_bus is not None:
        off = next((t for t, f, _ in log.hud if t >= k.t_brake_pedal_bus and not f), None)
        if off is not None:
            k.fcw_cancel_ms = _r((off - k.t_brake_pedal_bus) * 1000.0, 1)
    k.off_indicated = bool(log.hud) and all(ind == 1 for _, _, ind in log.hud)

    entry = next((t for t, _, to, _ in res.transitions if to == S.FAULT), None)
    k.t_fault_entry_ecu = entry
    if entry is not None and k.t_fault_onset_true is not None:
        k.fault_reaction_true_ms = _r((entry - k.t_fault_onset_true) * 1000.0, 1)
    return k
