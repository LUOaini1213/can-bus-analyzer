"""AEB state machine: every row of the transition table, guard boundaries, and
input supervision (timeout / E2E)."""
from dataclasses import replace

import pytest

from canbus_analyzer.aeb_function import (TRANSITIONS, AebConfig, Inputs, MonitorSpec, S,
                                          StateMachine, Supervisor)
from canbus_analyzer.dbcutil import encode, load_db

C = AebConfig()
T = 10.0
BASE = Inputs(t=T, ttc=None, v_ego_kph=50.0, gas_pct=0.0, brake_pressed=False,
              inputs_ok=True, input_fault=False, ok_since=0.0, main_switch=True)


def step(state: S, entered: float = 0.0, cfg: AebConfig = C, **kw) -> tuple[S, str | None]:
    sm = StateMachine(cfg, state, entered)
    sm.step(replace(BASE, **kw))
    return sm.state, (sm.log[-1][3] if sm.log else None)


# (rule, inputs that make it fire, state the machine must stay in / go to otherwise)
FIRE = {
    "R01": dict(main_switch=False),
    "R02": dict(inputs_ok=False, input_fault=True, ok_since=None),
    "R03": dict(),
    "R04": dict(inputs_ok=False, input_fault=True, ok_since=None),
    "R05": dict(ok_since=T - C.fault_recover_s),
    "R06": dict(gas_pct=C.gas_override_pct, ttc=0.5),
    "R07": dict(brake_pressed=True, ttc=2.0),
    "R08": dict(v_ego_kph=0.4, ttc=0.5),
    "R09": dict(ttc=None),
    "R10": dict(ttc=C.ttc_fcw_s + C.ttc_hyst_s),
    "R11": dict(ttc=0.89),
    "R12": dict(ttc=1.59),
    "R13": dict(ttc=2.59),
}


def test_every_edge_of_the_table_fires():
    seen = set()
    for tr in TRANSITIONS:
        for src in tr.sources:
            new, rid = step(src, **FIRE[tr.rid])
            assert (new, rid) == (tr.target, tr.rid), (tr.rid, src)
            seen.add((tr.rid, src))
    assert len(seen) == sum(len(tr.sources) for tr in TRANSITIONS) == 27
    assert len(TRANSITIONS) == 13


def test_table_has_no_self_loops_and_ids_are_unique():
    assert len({t.rid for t in TRANSITIONS}) == len(TRANSITIONS)
    assert all(t.target not in t.sources for t in TRANSITIONS)


@pytest.mark.parametrize("ttc,expected", [(2.59, S.FCW), (2.6, S.STANDBY), (1.59, S.PARTIAL_BRAKE),
                                          (1.6, S.FCW), (0.89, S.FULL_BRAKE), (0.9, S.PARTIAL_BRAKE)])
def test_activation_thresholds_are_strict(ttc, expected):
    assert step(S.STANDBY, ttc=ttc)[0] == expected


@pytest.mark.parametrize("v,expected", [(7.99, S.STANDBY), (8.0, S.FCW), (130.0, S.FCW), (130.01, S.STANDBY)])
def test_speed_window(v, expected):
    assert step(S.STANDBY, ttc=2.0, v_ego_kph=v)[0] == expected


def test_no_activation_without_speed_signal():
    assert step(S.STANDBY, ttc=0.5, v_ego_kph=None)[0] == S.STANDBY


def test_gas_override_boundary_and_inhibit():
    assert step(S.STANDBY, ttc=0.5, gas_pct=C.gas_override_pct)[0] == S.STANDBY
    assert step(S.STANDBY, ttc=0.5, gas_pct=C.gas_override_pct - 0.1)[0] == S.FULL_BRAKE
    assert step(S.PARTIAL_BRAKE, ttc=1.2, gas_pct=C.gas_override_pct - 0.1)[0] == S.PARTIAL_BRAKE


def test_brake_pedal_suppresses_warning_but_not_braking():
    assert step(S.STANDBY, ttc=2.0, brake_pressed=True)[0] == S.STANDBY
    assert step(S.STANDBY, ttc=1.5, brake_pressed=True)[0] == S.PARTIAL_BRAKE
    assert step(S.PARTIAL_BRAKE, ttc=1.2, brake_pressed=True)[0] == S.PARTIAL_BRAKE


def test_fcw_hysteresis_and_dwell():
    thr = C.ttc_fcw_s + C.ttc_hyst_s
    assert step(S.FCW, ttc=thr - 0.01)[0] == S.FCW                    # inside hysteresis band
    assert step(S.FCW, ttc=thr)[0] == S.STANDBY
    assert step(S.FCW, entered=T - C.fcw_min_dwell_s + 0.01, ttc=None)[0] == S.FCW   # dwell not over
    assert step(S.FCW, entered=T - C.fcw_min_dwell_s, ttc=None)[0] == S.STANDBY


def test_brake_release_rules():
    # default: braking continues while still closing, even with a large TTC
    assert step(S.PARTIAL_BRAKE, ttc=5.0)[0] == S.PARTIAL_BRAKE
    assert step(S.FULL_BRAKE, ttc=5.0)[0] == S.FULL_BRAKE
    assert step(S.PARTIAL_BRAKE, entered=T - C.brake_min_dwell_s + 0.01, ttc=None)[0] == S.PARTIAL_BRAKE
    assert step(S.FULL_BRAKE, entered=T - C.brake_min_dwell_s, ttc=None)[0] == S.STANDBY
    assert step(S.FULL_BRAKE, ttc=1.5)[0] == S.FULL_BRAKE              # no de-escalation to partial
    # variant: release on TTC >= ttc_pb + hysteresis
    alt = replace(C, brake_release_ttc_hyst=True)
    thr = C.ttc_pb_s + C.ttc_hyst_s
    assert step(S.PARTIAL_BRAKE, cfg=alt, ttc=thr)[0] == S.STANDBY
    assert step(S.PARTIAL_BRAKE, cfg=alt, ttc=thr - 0.01)[0] == S.PARTIAL_BRAKE


def test_standstill_boundary():
    assert step(S.FULL_BRAKE, entered=T, ttc=0.5, v_ego_kph=C.v_stop_kph)[0] == S.FULL_BRAKE
    assert step(S.FULL_BRAKE, entered=T, ttc=0.5, v_ego_kph=C.v_stop_kph - 0.01)[0] == S.STANDBY


def test_fault_is_latched_until_inputs_valid_long_enough():
    assert step(S.FAULT, ok_since=T - C.fault_recover_s + 0.01)[0] == S.FAULT
    assert step(S.FAULT, ok_since=None)[0] == S.FAULT
    assert step(S.FAULT, inputs_ok=False, input_fault=True, ok_since=None, ttc=0.5)[0] == S.FAULT
    assert step(S.FAULT, ok_since=T - C.fault_recover_s)[0] == S.STANDBY


def test_fault_has_priority_over_activation_and_inhibits_braking():
    for s in (S.STANDBY, S.FCW, S.PARTIAL_BRAKE, S.FULL_BRAKE):
        assert step(s, ttc=0.5, inputs_ok=False, input_fault=True, ok_since=None)[0] == S.FAULT
    sm = StateMachine(C, S.FAULT, 0.0)
    assert sm.decel_request() == 0.0


def test_off_and_startup():
    assert step(S.OFF, inputs_ok=False, ok_since=None)[0] == S.OFF      # inputs not yet received
    assert step(S.OFF, main_switch=False, inputs_ok=False, input_fault=True)[0] == S.OFF
    assert step(S.FAULT, main_switch=False)[0] == S.OFF


def test_decel_request_per_state():
    for s, want in ((S.PARTIAL_BRAKE, C.partial_decel_mps2), (S.FULL_BRAKE, C.full_decel_mps2),
                    (S.FCW, 0.0), (S.STANDBY, 0.0)):
        assert StateMachine(C, s).decel_request() == want


# -------------------------------------------------------------- supervisor --
def _lead(dist=40.0, rel=-5.0):
    return load_db().get_message_by_name("LEAD_INFO").frame_id, encode(
        "LEAD_INFO", {"LEAD_LONG_DIST": dist, "LEAD_REL_SPEED": rel})


def test_timeout_after_n_cycles():
    sup = Supervisor([MonitorSpec("LEAD_INFO", 0.05, timeout_cycles=3)])
    fid, data = _lead()
    assert sup.on_frame("LEAD_INFO", 1.0, fid, data) is not None
    assert sup.faults(1.15) == []                 # exactly 3 cycles: still fine
    assert sup.faults(1.1501) == ["LEAD_INFO:timeout"]


def test_never_received_times_out_from_t0():
    sup = Supervisor([MonitorSpec("LEAD_INFO", 0.05, timeout_cycles=3)], t0=0.0)
    assert sup.faults(0.15) == [] and not sup.all_received()
    assert sup.faults(0.151) == ["LEAD_INFO:timeout"]


def test_e2e_checksum_dlc_and_range():
    sup = Supervisor([MonitorSpec("LEAD_INFO", 0.05, e2e_max_consecutive=2)])
    fid, data = _lead()
    bad = data[:-1] + bytes([data[-1] ^ 0xFF])
    assert sup.on_frame("LEAD_INFO", 0.00, fid, bad) is None
    assert sup.faults(0.01) == []                 # one bad frame is tolerated
    assert sup.on_frame("LEAD_INFO", 0.05, fid, bad) is None
    assert "LEAD_INFO:e2e" in sup.faults(0.06)
    assert sup.on_frame("LEAD_INFO", 0.10, fid, data) is not None   # valid frame clears the counter
    assert sup.faults(0.11) == []
    assert sup.on_frame("LEAD_INFO", 0.15, fid, data[:-1]) is None  # DLC
    fid, far = _lead(dist=350.0)                                      # outside DBC [0|300]
    assert sup.on_frame("LEAD_INFO", 0.20, fid, far) is None
    assert sup.state["LEAD_INFO"].bad_total == 4


def test_timeout_boundary_exact():
    # binary-exact numbers so the strict '>' is tested, not float rounding
    sup = Supervisor([MonitorSpec("LEAD_INFO", 0.25, timeout_cycles=2)])
    fid, data = _lead()
    sup.on_frame("LEAD_INFO", 1.0, fid, data)
    assert sup.faults(1.5) == []
    assert sup.faults(1.5 + 2 ** -10) == ["LEAD_INFO:timeout"]


def test_invalid_frames_do_not_refresh_the_timeout():
    sup = Supervisor([MonitorSpec("LEAD_INFO", 0.25, timeout_cycles=2, e2e_max_consecutive=99)])
    fid, data = _lead()
    sup.on_frame("LEAD_INFO", 1.0, fid, data)
    bad = data[:-1] + bytes([data[-1] ^ 0x01])
    for k in range(1, 5):
        sup.on_frame("LEAD_INFO", 1.0 + 0.25 * k, fid, bad)
    assert sup.faults(2.0) == ["LEAD_INFO:timeout"]
