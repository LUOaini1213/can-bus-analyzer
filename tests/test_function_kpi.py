"""Closed loop on the virtual bus, KPI extraction from the log, scenario matrix
and the generated test-case catalogue."""
from dataclasses import replace

import can
import pytest

from canbus_analyzer.aeb_function import S
from canbus_analyzer.analyzer import ttc
from canbus_analyzer.closed_loop import ClosedLoopResult, Truth, run
from canbus_analyzer.config import EPOCH
from canbus_analyzer.dbcutil import encode, load_db, pack_byte_signals, unpack_byte_signal
from canbus_analyzer.function_kpi import Kpi, decode_log, extract, fault_onset
from canbus_analyzer.function_matrix import (Expectation, build_matrix, check, expectation,
                                             sequence_ok, build_testcases)

MATRIX = {s.sid: s for s in build_matrix()}


@pytest.fixture(scope="module")
def runs():
    out = {}
    for sid in ("CCRs-40", "CCRs-70", "NT-02", "FLT-01", "FLT-03", "OVR-02", "OFF-01"):
        r = run(MATRIX[sid])
        out[sid] = (r, extract(r))
    return out


# ------------------------------------------------------------- matrix -------
def test_matrix_parameters_read_from_protocol():
    ccrs = sorted(s.v_ego_kph for s in MATRIX.values() if s.family == "CCRs")
    assert ccrs == list(range(10, 85, 5))
    ccrm = sorted(s.v_ego_kph for s in MATRIX.values() if s.family == "CCRm")
    assert ccrm == list(range(30, 85, 5))
    ccrb = {(s.v_ego_kph, s.v_target_kph, s.gap0_m, s.target_accel_mps2)
            for s in MATRIX.values() if s.family == "CCRb"}
    assert ccrb == {(50.0, 50.0, hw, a) for hw in (12.0, 40.0) for a in (-2.0, -6.0)}
    for s in MATRIX.values():
        if s.family in ("CCRs", "CCRm"):       # T0 (TTC = 4 s) after the 2 s pre-roll
            assert s.gap0_m / ((s.v_ego_kph - s.v_target_kph) / 3.6) == pytest.approx(6.0, abs=1e-3)


def test_pcs_hud_byte_signals_match_dbc_positions():
    data = pack_byte_signals("PCS_HUD", {"FCW": 1, "PCS_INDICATOR": 2})
    assert data == bytes([0x90, 0, 0, 0, 0, 0, 0, 0])
    assert unpack_byte_signal("PCS_HUD", "FCW", data) == 1
    assert unpack_byte_signal("PCS_HUD", "PCS_INDICATOR", data) == 2


# ---------------------------------------------------------- closed loop -----
def test_closed_loop_threat_case(runs):
    r, k = runs["CCRs-40"]
    log = decode_log(r.log)
    assert sequence_ok(k.states, expectation(MATRIX["CCRs-40"]))
    # KPI uses the first FCW frame and the latest LEAD_INFO frame at or before it
    t_fcw = next(t for t, f, _ in log.hud if f)
    assert k.t_fcw == t_fcw
    lead = [x for x in log.lead if x[0] <= t_fcw][-1]
    assert k.ttc_fcw_bus == pytest.approx(ttc(lead[1], lead[2]), abs=1e-3)
    t_brk = next(t for t, _, a, f in log.pre if a == 1 and f < 0)
    assert k.fcw_to_brake_ms == pytest.approx((t_brk - t_fcw) * 1000, abs=0.1)
    assert 2.0 <= k.ttc_fcw_bus < 2.6 and 1.0 < k.ttc_brake_bus < 1.6
    assert k.contact_bus is False and k.contact_true is False
    assert k.n_fcw == 1 and k.n_brake == 1 and k.invalid_frames == 0
    # ECU-internal transitions agree with what the bus shows
    assert [to.name for _, _, to, _ in r.transitions][:4] == ["STANDBY", "FCW", "PARTIAL_BRAKE", "FULL_BRAKE"]


def test_closed_loop_collision_impact_speed(runs):
    _, k = runs["CCRs-70"]
    assert k.contact_bus and k.contact_true
    assert k.v_rel_impact_bus_kph == pytest.approx(k.v_rel_impact_true_kph, abs=1.0)
    assert 0 < k.v_rel_impact_true_kph < 70


def test_no_threat_cut_out(runs):
    _, k = runs["NT-02"]
    assert k.n_fcw == 0 and k.n_brake == 0 and check("REQ-04", MATRIX["NT-02"], k)


def test_timeout_drives_fault_and_inhibits_braking(runs):
    r, k = runs["FLT-01"]
    assert S.FAULT.name in k.states and k.n_brake == 0
    assert k.t_fault_onset_bus == pytest.approx(k.t_fault_onset_true, abs=1e-6)
    # 3 x 50 ms timeout, measured from the first missed frame; + ECU task + PCS_HUD cycle
    assert 100.0 < k.fault_reaction_bus_ms <= 145.0
    assert 100.0 < k.fault_reaction_true_ms <= 115.0
    assert k.contact_bus is None and k.contact_true      # radar gone: contact invisible on the bus
    assert any("LEAD_INFO:timeout" in reasons for _, reasons in r.fault_reasons)


def test_checksum_errors_fault_then_heal(runs):
    _, k = runs["FLT-03"]
    assert k.fault_reaction_bus_ms is not None and k.fault_reaction_bus_ms < 60.0
    assert k.heal_after_valid_s >= 1.0 and k.t_recovered_bus is not None
    assert k.brake_while_faulted == 0 and not k.contact_bus
    assert sequence_ok(k.states, expectation(MATRIX["FLT-03"]))


def test_gas_override_aborts_braking(runs):
    _, k = runs["OVR-02"]
    assert k.t_brake is not None and k.t_brake_after_gas is None
    assert k.states[-1] == S.STANDBY.name


def test_main_switch_off(runs):
    _, k = runs["OFF-01"]
    assert k.states == ["OFF"] and k.off_indicated and check("REQ-09", MATRIX["OFF-01"], k)


# --------------------------------------------------- KPI on a crafted log ---
def _msg(name, t, data):
    return can.Message(timestamp=EPOCH + t, arbitration_id=load_db().get_message_by_name(name).frame_id,
                       is_extended_id=False, data=data, dlc=len(data))


def _crafted():
    L = [
        _msg("SPEED", 0.00, encode("SPEED", {"SPEED": 36.0})),
        _msg("LEAD_INFO", 0.00, encode("LEAD_INFO", {"LEAD_LONG_DIST": 30.0, "LEAD_REL_SPEED": -10.0})),
        _msg("LEAD_INFO", 0.05, encode("LEAD_INFO", {"LEAD_LONG_DIST": 25.0, "LEAD_REL_SPEED": -10.0})),
        _msg("PCS_HUD", 0.07, pack_byte_signals("PCS_HUD", {"FCW": 1})),
        _msg("LEAD_INFO", 0.10, encode("LEAD_INFO", {"LEAD_LONG_DIST": 20.0, "LEAD_REL_SPEED": -10.0})),
        _msg("PRE_COLLISION", 0.11, encode("PRE_COLLISION", {"STATE": 3, "PRECOLLISION_ACTIVE": 1, "FORCE": 0.0})),
        _msg("PRE_COLLISION", 0.12, encode("PRE_COLLISION", {"STATE": 3, "PRECOLLISION_ACTIVE": 1, "FORCE": -6000.0})),
        _msg("PRE_COLLISION", 0.14, encode("PRE_COLLISION", {"STATE": 4, "PRECOLLISION_ACTIVE": 1, "FORCE": -13500.0})),
    ]
    bad = bytearray(encode("LEAD_INFO", {"LEAD_LONG_DIST": 5.0, "LEAD_REL_SPEED": -10.0}))
    bad[-1] ^= 0xFF
    L.append(_msg("LEAD_INFO", 0.06, bytes(bad)))       # corrupted: must be ignored
    truth = Truth(t=[i / 1000 for i in range(200)], gap=[30.0] * 200, v_ego=[10.0] * 200,
                  v_target=[0.0] * 200, target_present=[True] * 200)
    return ClosedLoopResult(MATRIX["CCRs-40"], L, truth, [], [])


def test_kpi_picks_the_right_frames():
    k = extract(_crafted())
    assert k.t_fcw == pytest.approx(0.07)
    assert k.ttc_fcw_bus == pytest.approx(2.5)        # LEAD at 0.05, not 0.00, 0.10 or the bad 0.06
    assert k.ttc_fcw_true == pytest.approx(3.0)
    assert k.t_brake == pytest.approx(0.12)           # FORCE = 0 at 0.11 is not a brake request
    assert k.ttc_brake_bus == pytest.approx(2.0)
    assert k.t_full == pytest.approx(0.14) and k.ttc_full_bus == pytest.approx(2.0)
    assert k.fcw_to_brake_ms == pytest.approx(50.0)
    assert k.invalid_frames == 1
    assert k.states == ["PARTIAL_BRAKE", "FULL_BRAKE"]


def test_fault_onset_from_gap_and_from_invalid_frame():
    sp = [_msg("SPEED", 0.5 + 0.02 * i, encode("SPEED", {"SPEED": 50.0})) for i in range(10)]
    pre = [_msg("PRE_COLLISION", 0.5 + 0.03 * i, encode("PRE_COLLISION", {"STATE": 1})) for i in range(30)]
    log = decode_log(sp + pre)
    assert fault_onset(log, after=0.5) == pytest.approx(0.5 + 0.02 * 9 + 0.02)
    bad = bytearray(encode("SPEED", {"SPEED": 50.0}))
    bad[-1] ^= 1
    log = decode_log(sp[:5] + [_msg("SPEED", 0.6, bytes(bad))] + sp[5:] + pre)
    assert fault_onset(log, after=0.5) == pytest.approx(0.6)


# ------------------------------------------------------ requirements / TC ---
def test_requirement_boundaries():
    s = MATRIX["CCRs-60"]
    assert check("REQ-01", s, Kpi("x", ttc_fcw_bus=2.0)) and not check("REQ-01", s, Kpi("x", ttc_fcw_bus=1.99))
    hit = dict(contact_bus=True, t_contact_bus=5.0, t_fcw=4.0, t_brake=4.5)
    assert check("REQ-02", s, Kpi("x", **hit))
    assert not check("REQ-02", s, Kpi("x", **{**hit, "t_brake": 3.9}))      # brake before warning
    assert not check("REQ-02", s, Kpi("x", **{**hit, "t_brake": 5.0}))      # brake not before contact
    assert not check("REQ-03", s, Kpi("x", contact_bus=True))                # 60 km/h closing <= 50? no ->
    s50 = MATRIX["CCRs-50"]
    assert not check("REQ-03", s50, Kpi("x", contact_bus=True, v_rel_impact_bus_kph=1.0))
    assert check("REQ-03", s50, Kpi("x", contact_bus=False))
    assert check("REQ-03", s, Kpi("x", contact_bus=True, v_rel_impact_bus_kph=40.0))
    assert not check("REQ-03", s, Kpi("x", contact_bus=True, v_rel_impact_bus_kph=40.01))
    assert not check("REQ-03", s, Kpi("x", contact_bus=None))
    assert check("REQ-07", s, Kpi("x", fault_reaction_bus_ms=200.0))
    assert not check("REQ-07", s, Kpi("x", fault_reaction_bus_ms=200.1))
    assert not check("REQ-07", s, Kpi("x", fault_reaction_bus_ms=50.0, brake_while_faulted=1))
    assert check("REQ-08", s, Kpi("x", heal_after_valid_s=1.0))
    assert not check("REQ-08", s, Kpi("x", heal_after_valid_s=0.99))
    assert check("REQ-06", s, Kpi("x", fcw_cancel_ms=200.0)) and not check("REQ-06", s, Kpi("x"))


def test_sequence_matching():
    exp = Expectation((("STANDBY",), ("FCW",), ("PARTIAL_BRAKE", "FULL_BRAKE")), ("FAULT",), ())
    assert sequence_ok(["OFF", "STANDBY", "FCW", "FULL_BRAKE", "STANDBY"], exp)
    assert not sequence_ok(["OFF", "STANDBY", "FULL_BRAKE", "FCW"], exp)          # wrong order
    assert not sequence_ok(["STANDBY", "FCW", "PARTIAL_BRAKE", "FAULT"], exp)     # forbidden state
    assert not sequence_ok(["STANDBY", "FCW"], exp)


def test_testcase_generator_marks_pass_and_fail():
    s = MATRIX["CCRs-40"]
    good = Kpi("CCRs-40", ttc_fcw_bus=2.5, t_fcw=4.0, t_brake=5.0, contact_bus=False,
               states=["OFF", "STANDBY", "FCW", "PARTIAL_BRAKE", "STANDBY"])
    bad = replace(good, ttc_fcw_bus=1.9, contact_bus=True, t_contact_bus=6.0, v_rel_impact_bus_kph=10.0)
    rows = build_testcases([(s, good, {r: check(r, s, good) for r in expectation(s).reqs}),
                          (s, bad, {r: check(r, s, bad) for r in expectation(s).reqs})])
    assert [r["id"] for r in rows] == ["TC-001", "TC-002"]
    assert rows[0]["result"] == "PASS" and rows[0]["failed"] == ""
    assert rows[1]["result"] == "FAIL" and rows[1]["failed"] == "REQ-01, REQ-03"
    assert rows[0]["expected_states"].startswith("STANDBY > FCW > PARTIAL_BRAKE|FULL_BRAKE")
    assert "no contact" in rows[0]["actual"] and "v_rel 10.0" in rows[1]["actual"]


def test_brake_requests_while_faulted_are_counted():
    res = _crafted()
    extra = [_msg("PCS_HUD", 0.15, pack_byte_signals("PCS_HUD", {"PCS_INDICATOR": 2})),
             _msg("PRE_COLLISION", 0.17, encode("PRE_COLLISION", {"STATE": 5, "PRECOLLISION_ACTIVE": 1,
                                                                   "FORCE": -6000.0})),
             _msg("PCS_HUD", 0.20, pack_byte_signals("PCS_HUD", {"PCS_INDICATOR": 0}))]
    k = extract(ClosedLoopResult(res.spec, res.log + extra, res.truth, [], []))
    assert k.t_degraded_bus == pytest.approx(0.15) and k.t_recovered_bus == pytest.approx(0.20)
    assert k.brake_while_faulted == 1


def test_fault_onset_for_a_gap_in_the_middle():
    ts = [0.5 + 0.02 * i for i in range(20) if not 5 <= i <= 8]      # frames 5..8 missing
    sp = [_msg("SPEED", t, encode("SPEED", {"SPEED": 50.0})) for t in ts]
    pre = [_msg("PRE_COLLISION", 0.5 + 0.03 * i, encode("PRE_COLLISION", {"STATE": 1})) for i in range(14)]
    assert fault_onset(decode_log(sp + pre), after=0.5) == pytest.approx(0.5 + 0.02 * 5)
