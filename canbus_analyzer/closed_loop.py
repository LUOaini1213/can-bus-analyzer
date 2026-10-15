"""Closed-loop function test on the python-can virtual bus.

Three nodes share one in-process `virtual` channel:

  plant  -- sends the sensor/driver messages (WHEEL_SPEEDS, SPEED, BRAKE_MODULE,
            GAS_PEDAL, LEAD_INFO) and plays the brake actuator: it receives
            PRE_COLLISION from the bus and turns FORCE into deceleration;
  aeb    -- the function under test (`AebEcu`): decodes the received input
            frames, supervises them, runs the state machine every 10 ms task
            and sends PRE_COLLISION + PCS_HUD;
  logger -- records every frame; KPIs are later extracted from this log only.

The physics runs on a 1 ms logical clock and every frame carries its logical
timestamp (`preserve_timestamps=True`), so a run is deterministic and faster
than real time.  The plant is a toy point-mass model (no tyre, no pitch, no
road friction limit other than a max deceleration), the target is a point on
the same line (100 % overlap only), radar values are exact up to DBC
quantisation (no noise, no field of view).  This is NOT a vehicle model.
"""
from __future__ import annotations

import itertools
import random
from dataclasses import dataclass, field

import can

from .aeb_function import INPUT_CYCLES_S, OUTPUT_CYCLES_S, AebConfig, AebEcu, S
from .config import EPOCH, MIN_CLOSING_MPS
from .dbcutil import encode, load_db, pack_byte_signals, toyota_checksum, unpack_byte_signal

PLANT_CYCLES_S = {"WHEEL_SPEEDS": 0.010, **INPUT_CYCLES_S}
BRAKE_DEAD_S = 0.080          # actuator dead time after a request is received (toy)
BRAKE_TAU_S = 0.120           # first-order deceleration build-up (toy)
EGO_DECEL_MAX = 9.0           # m/s^2, plant saturation
P_MAX_RAW = 400.0             # BRAKE_PRESSURE raw value at EGO_DECEL_MAX (toy scale)
PREROLL_S = 2.0               # steady driving before T0
_channels = itertools.count()


@dataclass(frozen=True)
class FaultSpec:
    kind: str                         # "stop" (message no longer sent) | "checksum" (corrupt CHECKSUM byte)
    message: str
    t_start: float | None = None      # absolute scenario time, or ...
    after_brake_s: float | None = None  # ... this long after the plant first receives a brake request
    n_frames: int | None = None       # frames affected (None = until the end)


@dataclass(frozen=True)
class Spec:
    sid: str
    family: str                       # CCRs | CCRm | CCRb | NoThreat | Override | Fault | Off
    title: str
    v_ego_kph: float
    v_target_kph: float
    gap0_m: float
    threat: bool                      # a collision would occur without intervention
    source: str                       # where the parameters come from
    target_accel_mps2: float = 0.0    # signed; < 0 = target brakes (until standstill)
    target_accel_t_s: float = 0.0
    cutout_t_s: float | None = None   # target leaves the lane: radar reports no target
    gas_pct: float = 0.0              # accelerator pedal from gas_t_s on (ego speed unchanged: toy)
    gas_t_s: float = 0.0
    driver_brake_after_fcw_s: float | None = None   # driver brakes this long after the FCW frame
    driver_decel_mps2: float = 4.0
    main_switch: bool = True
    faults: tuple[FaultSpec, ...] = ()
    duration_s: float = 14.0
    t0_s: float = PREROLL_S           # T0 of the scenario (for the catalogue)


@dataclass
class Truth:
    t: list[float] = field(default_factory=list)
    gap: list[float] = field(default_factory=list)
    v_ego: list[float] = field(default_factory=list)      # m/s
    v_target: list[float] = field(default_factory=list)   # m/s
    target_present: list[bool] = field(default_factory=list)
    contact_t: float | None = None
    v_impact_kph: float | None = None
    v_rel_impact_kph: float | None = None
    fault_frames: list[tuple[str, str, float]] = field(default_factory=list)  # (kind, msg, nominal t)
    first_request_rx_t: float | None = None

    def ttc_at(self, t: float) -> float | None:
        k = max(0, min(len(self.t) - 1, int(round(t * 1000))))
        if not self.target_present[k]:
            return None
        closing = self.v_ego[k] - self.v_target[k]
        if closing <= MIN_CLOSING_MPS or self.gap[k] <= 0:
            return None
        return self.gap[k] / closing


@dataclass
class ClosedLoopResult:
    spec: Spec
    log: list[can.Message]
    truth: Truth
    transitions: list[tuple[float, S, S, str]]
    fault_reasons: list[tuple[float, list[str]]]


def _enc(name: str, vals: dict) -> bytes:
    if name == "PCS_HUD":       # cantools cannot encode this message, see dbcutil.pack_byte_signals
        return pack_byte_signals(name, {k: int(v) for k, v in vals.items()})
    data = encode(name, vals)
    msg = load_db().get_message_by_name(name)
    if any(s.name == "CHECKSUM" for s in msg.signals):   # e.g. GAS_PEDAL (not in CHECKSUM_MESSAGES)
        buf = bytearray(data)
        buf[-1] = toyota_checksum(msg.frame_id, buf)
        data = bytes(buf)
    return data


def _drain(bus: can.BusABC) -> list[can.Message]:
    out = []
    while (m := bus.recv(timeout=0)) is not None:
        out.append(m)
    return out


def run(spec: Spec, cfg: AebConfig = AebConfig()) -> ClosedLoopResult:
    db = load_db()
    rng = random.Random(f"phase-{spec.sid}")
    periods = {**PLANT_CYCLES_S, **OUTPUT_CYCLES_S}
    pms = {m: int(round(T * 1000)) for m, T in periods.items()}
    phase = {m: rng.randrange(p) for m, p in pms.items()}
    ecu_phase = rng.randrange(int(round(cfg.task_s * 1000)))
    ids = {m: db.get_message_by_name(m).frame_id for m in periods}

    ch = f"aeb-closed-loop-{next(_channels)}"
    plant = can.Bus(interface="virtual", channel=ch, preserve_timestamps=True)
    aeb = can.Bus(interface="virtual", channel=ch, preserve_timestamps=True)
    logger = can.Bus(interface="virtual", channel=ch)
    ecu = AebEcu(cfg, main_switch=spec.main_switch)
    truth = Truth()

    x_e, v_e = 0.0, spec.v_ego_kph / 3.6
    x_t, v_t = spec.gap0_m, spec.v_target_kph / 3.6
    decel_now = 0.0               # actuated AEB deceleration (first order)
    pending: list[tuple[float, float]] = []   # (apply time, requested decel)
    aeb_cmd = 0.0
    driver_brake_from: float | None = None
    fcw_seen: float | None = None
    fault_left = {i: f.n_frames for i, f in enumerate(spec.faults)}
    fault_start: dict[int, float | None] = {i: f.t_start for i, f in enumerate(spec.faults)}
    counters = {m: 0 for m in OUTPUT_CYCLES_S}
    end_at: float | None = None
    n_ms = int(round(spec.duration_s * 1000))
    dt = 0.001
    log: list[can.Message] = []

    def send(bus: can.BusABC, name: str, t: float, data: bytes) -> None:
        bus.send(can.Message(timestamp=EPOCH + t, arbitration_id=ids[name], is_extended_id=False,
                             data=data, dlc=len(data)))

    try:
        for k in range(n_ms + 1):
            t = k / 1000
            # ---------------- physics ------------------------------------
            if k > 0:
                if t >= spec.target_accel_t_s and spec.target_accel_mps2:
                    v_t = max(0.0, v_t + spec.target_accel_mps2 * dt)
                x_t += v_t * dt
                while pending and pending[0][0] <= t:
                    aeb_cmd = pending.pop(0)[1]
                decel_now += (aeb_cmd - decel_now) * dt / BRAKE_TAU_S
                drv = spec.driver_decel_mps2 if driver_brake_from is not None and t >= driver_brake_from else 0.0
                a_e = -min(EGO_DECEL_MAX, max(decel_now, drv))
                v_e = max(0.0, v_e + a_e * dt)
                x_e += v_e * dt
            present = spec.cutout_t_s is None or t < spec.cutout_t_s
            gap = x_t - x_e
            if present and gap <= 0.0:
                if truth.contact_t is None:
                    truth.contact_t = t
                    truth.v_impact_kph = v_e * 3.6
                    truth.v_rel_impact_kph = (v_e - v_t) * 3.6
                    end_at = t + 0.3
                x_e, v_e, gap = x_t, min(v_e, v_t), 0.0
            truth.t.append(t)
            truth.gap.append(gap)
            truth.v_ego.append(v_e)
            truth.v_target.append(v_t)
            truth.target_present.append(present)
            if end_at is None and k > 0 and v_e <= 0.0:
                end_at = t + 1.0

            # ---------------- plant transmits -----------------------------
            braking_total = max(decel_now, spec.driver_decel_mps2
                                if driver_brake_from is not None and t >= driver_brake_from else 0.0)
            for m in PLANT_CYCLES_S:
                if k < phase[m] or (k - phase[m]) % pms[m]:
                    continue
                if m == "SPEED":
                    vals = {"SPEED": round(v_e * 3.6, 2)}
                elif m == "WHEEL_SPEEDS":
                    vals = {f"WHEEL_SPEED_{c}": round(v_e * 3.6, 2) for c in ("FL", "FR", "RL", "RR")}
                elif m == "BRAKE_MODULE":
                    vals = {"BRAKE_PRESSURE": int(round(min(braking_total, EGO_DECEL_MAX) / EGO_DECEL_MAX * P_MAX_RAW)),
                            "BRAKE_PRESSED": int(driver_brake_from is not None and t >= driver_brake_from)}
                elif m == "GAS_PEDAL":
                    vals = {"GAS_PEDAL": spec.gas_pct if t >= spec.gas_t_s else 0.0}
                else:  # LEAD_INFO
                    if present:
                        d_raw = min(int(round(gap / 0.05)), 6000)
                        r_raw = max(-2048, min(2047, int(round((v_t - v_e) / 0.025))))
                        vals = {"LEAD_LONG_DIST": d_raw * 0.05, "LEAD_REL_SPEED": r_raw * 0.025}
                    else:
                        vals = {"LEAD_LONG_DIST": 300.0, "LEAD_REL_SPEED": 0.0}
                data = _enc(m, vals)
                dropped = False
                for i, f in enumerate(spec.faults):
                    st = fault_start[i]
                    if f.message != m or st is None or t < st or fault_left[i] == 0:
                        continue
                    if fault_left[i] is not None:
                        fault_left[i] -= 1
                    truth.fault_frames.append((f.kind, m, t))
                    if f.kind == "stop":
                        dropped = True
                    else:
                        buf = bytearray(data)
                        buf[-1] ^= 0x5A
                        data = bytes(buf)
                if not dropped:
                    send(plant, m, t, data)

            # ---------------- AEB ECU -------------------------------------
            for msg in _drain(aeb):
                ecu.on_frame(round(msg.timestamp - EPOCH, 6), msg.arbitration_id, bytes(msg.data))
            if k % int(round(cfg.task_s * 1000)) == ecu_phase:
                ecu.step(t)
            for m in OUTPUT_CYCLES_S:
                if k >= phase[m] and (k - phase[m]) % pms[m] == 0:
                    send(aeb, m, t, _enc(m, ecu.output_values(m, counters[m])))
                    counters[m] += 1

            # ---------------- plant receives (brake actuator, driver) ------
            for msg in _drain(plant):
                if msg.arbitration_id == ids["PRE_COLLISION"]:
                    d = bytes(msg.data)
                    if d[-1] != toyota_checksum(msg.arbitration_id, d):
                        continue
                    dec = db.get_message_by_name("PRE_COLLISION").decode(d, decode_choices=False)
                    req = -float(dec["FORCE"]) / cfg.vehicle_mass_kg if int(dec["PRECOLLISION_ACTIVE"]) else 0.0
                    req = max(0.0, req)
                    if req > 0 and truth.first_request_rx_t is None:
                        truth.first_request_rx_t = t
                        for i, f in enumerate(spec.faults):
                            if f.after_brake_s is not None:
                                fault_start[i] = t + f.after_brake_s
                    if not pending or pending[-1][1] != req:
                        pending.append((t + BRAKE_DEAD_S, req))
                elif msg.arbitration_id == ids["PCS_HUD"]:
                    if unpack_byte_signal("PCS_HUD", "FCW", bytes(msg.data)) and fcw_seen is None:
                        fcw_seen = t
                        if spec.driver_brake_after_fcw_s is not None:
                            driver_brake_from = t + spec.driver_brake_after_fcw_s
            if end_at is not None and t >= end_at:
                break
        log = _drain(logger)
    finally:
        plant.shutdown()
        aeb.shutdown()
        logger.shutdown()
    return ClosedLoopResult(spec, log, truth, list(ecu.sm.log), list(ecu.fault_reasons))
