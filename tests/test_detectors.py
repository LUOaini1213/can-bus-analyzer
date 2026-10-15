import pytest

from canbus_analyzer.analyzer import (checksum_ok, counter_findings, dlc_ok, range_violations,
                                      stuck_findings)
from canbus_analyzer.dbcutil import encode, load_db


def _dec(name, data):
    return load_db().get_message_by_name(name).decode(data, decode_choices=False)


# ---- DLC ---------------------------------------------------------------------
def test_dlc_check():
    good = encode("PRE_COLLISION", {})
    assert len(good) == 7 and dlc_ok("PRE_COLLISION", good)
    assert not dlc_ok("PRE_COLLISION", good[:-1])
    assert not dlc_ok("PRE_COLLISION", good + b"\x00")


# ---- checksum ----------------------------------------------------------------
def test_checksum_check_detects_any_single_byte_change():
    data = encode("LEAD_INFO", {"LEAD_LONG_DIST": 30.0, "LEAD_REL_SPEED": -5.0})
    assert checksum_ok("LEAD_INFO", 0x2E6, data)
    for i in range(len(data)):
        bad = bytearray(data)
        bad[i] ^= 0x01
        assert not checksum_ok("LEAD_INFO", 0x2E6, bytes(bad)), i
    # message without CHECKSUM signal is always "ok"
    assert checksum_ok("BRAKE_MODULE", 0x226, bytes(8))


# ---- range -------------------------------------------------------------------
def test_range_upper_bound_inclusive():
    assert range_violations("SPEED", _dec("SPEED", encode("SPEED", {"SPEED": 250.0}))) == []
    v = range_violations("SPEED", _dec("SPEED", encode("SPEED", {"SPEED": 250.01})))
    assert [x[0] for x in v] == ["SPEED"]


def test_range_lower_bound_inclusive():
    assert range_violations("ACC_CONTROL", {"ACCEL_CMD": -20.0}) == []
    assert [x[0] for x in range_violations("ACC_CONTROL", {"ACCEL_CMD": -20.001})] == ["ACCEL_CMD"]
    assert [x[0] for x in range_violations("ACC_CONTROL", {"ACCEL_CMD": 20.001})] == ["ACCEL_CMD"]


def test_range_override_for_force():
    assert range_violations("PRE_COLLISION", {"FORCE": -6000.0}) == []      # DBC [0|255] would flag this
    assert range_violations("PRE_COLLISION", {"FORCE": 2.0}) != []


# ---- rolling counter ---------------------------------------------------------
T = 0.03


def _seq(counters, times=None):
    times = times or [i * T for i in range(len(counters))]
    return list(zip(times, counters))


def test_counter_wrap_is_not_an_error():
    assert counter_findings("PRE_COLLISION", _seq([253, 254, 255, 0, 1, 2]), T, 256) == []


def test_counter_repeat_and_skip_detected():
    f = counter_findings("PRE_COLLISION", _seq([10, 11, 11, 12]), T, 256)
    assert [x.t for x in f] == [pytest.approx(2 * T)]
    f = counter_findings("PRE_COLLISION", _seq([10, 11, 13, 14]), T, 256)   # skip without time gap
    assert [x.t for x in f] == [pytest.approx(2 * T)]


def test_counter_jump_explained_by_time_gap_is_accepted():
    # frame with counter 12 lost: jump of 2 across a 2-cycle gap
    assert counter_findings("PRE_COLLISION", _seq([10, 11, 13], [0.0, T, 3 * T]), T, 256) == []
    # jump of 3 across a 2-cycle gap is NOT explained
    assert len(counter_findings("PRE_COLLISION", _seq([10, 11, 14], [0.0, T, 3 * T]), T, 256)) == 1
    # across the wrap: 255 -> 1 over a 2-cycle gap
    assert counter_findings("PRE_COLLISION", _seq([254, 255, 1], [0.0, T, 3 * T]), T, 256) == []


# ---- stuck signal ------------------------------------------------------------
def _series(values, dt=0.02):
    return [(i * dt, v) for i, v in enumerate(values)]


def test_stuck_detected_when_wheels_move():
    speed = _series([72.0] * 15)                         # 0.28 s constant
    wheels = _series([72.0 - 0.5 * i for i in range(15)])  # decelerating 7 km/h
    f = stuck_findings(speed, wheels)
    assert len(f) == 1 and f[0].t_start == 0.0 and f[0].t == pytest.approx(0.28)


def test_constant_cruise_is_not_stuck():
    speed = _series([72.0] * 100)
    wheels = _series([72.0] * 100)
    assert stuck_findings(speed, wheels) == []


def test_stuck_duration_boundary():
    wheels = _series([72.0 - i for i in range(20)])
    # times 0.0, 0.1, 0.2 -> run length exactly STUCK_MIN_S (0.2 s) is flagged
    speed = [(0.0, 50.0), (0.1, 50.0), (0.2, 50.0), (0.3, 49.0)]
    assert len(stuck_findings(speed, wheels)) == 1
    speed = [(0.0, 50.0), (0.1, 50.0), (0.19, 50.0), (0.3, 49.0)]
    assert stuck_findings(speed, wheels) == []


def test_stuck_wheel_threshold_boundary():
    speed = _series([60.0] * 12)
    assert stuck_findings(speed, _series([60.0] * 11 + [59.0])) == []        # exactly 1.0 km/h: not stuck
    assert len(stuck_findings(speed, _series([60.0] * 11 + [58.9]))) == 1
