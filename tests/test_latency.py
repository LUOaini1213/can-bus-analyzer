import pytest

from canbus_analyzer.analyzer import Latency, measure_latency, ttc
from canbus_analyzer.faults import FaultConfig
from canbus_analyzer.pipeline import run_once


def test_ttc():
    assert ttc(18.0, -10.0) == pytest.approx(1.8)
    assert ttc(10.0, 0.0) is None
    assert ttc(10.0, 2.0) is None          # opening
    assert ttc(10.0, -0.1) is None         # closing <= 0.1 m/s: undefined
    assert ttc(10.0, -0.2) == pytest.approx(50.0)


LEAD = [(0.00, 40.0, -5.0),     # TTC 8
        (0.05, 18.0, -10.0),    # TTC exactly 1.8 -> not below threshold
        (0.10, 17.0, -10.0),    # TTC 1.7 -> trigger
        (0.15, 16.0, -10.0)]


def test_trigger_is_first_frame_strictly_below_threshold():
    lat = measure_latency(LEAD, [], [])
    assert lat.trigger_s == pytest.approx(0.10)
    assert lat.request_s is None and lat.to_request_ms is None


def test_latency_endpoints_and_sign():
    pc = [(0.07, 1, -6000.0),     # before trigger: ignored
          (0.12, 1, 0.0),         # active but FORCE == 0: not a brake request
          (0.13, 1, -6000.0)]
    brake = [(0.11, 300.0), (0.30, 199.0), (0.35, 200.0)]
    lat = measure_latency(LEAD, pc, brake)
    assert lat.request_s == pytest.approx(0.13)
    assert lat.to_request_ms == pytest.approx(30.0)
    # 0.11 is after trigger and >= X; boundary: 200 counts, 199 does not
    assert lat.pressure_s == pytest.approx(0.11)
    lat = measure_latency(LEAD, pc, brake[1:])
    assert lat.to_pressure_ms == pytest.approx(250.0)


def test_request_at_same_timestamp_as_trigger_counts():
    lat = measure_latency(LEAD, [(0.10, 1, -6000.0)], [])
    assert lat.to_request_ms == pytest.approx(0.0)


def test_latency_properties_handle_missing():
    assert Latency(None, 1.0, 1.0).to_request_ms is None
    assert Latency(1.0, None, None).to_pressure_ms is None


def test_clean_run_bus_request_equals_ground_truth_request():
    r = run_once(0, FaultConfig())
    lat = r.analysis.latency
    assert lat.request_s == pytest.approx(r.truth.request_s, abs=1e-6)
    # bus-visible trigger (radar frame) can only lag the physical crossing,
    # by less than one LEAD_INFO cycle (50 ms)
    lag = lat.trigger_s - r.truth.ttc_cross_s
    assert 0.0 <= lag < 0.050
    truth_req, _ = r.truth_latency_ms()
    assert lat.to_request_ms - truth_req == pytest.approx(-lag * 1000, abs=1e-3)
    assert 0 < lat.to_request_ms < lat.to_pressure_ms


def test_out_of_range_frames_are_not_used_for_latency():
    from canbus_analyzer.analyzer import RxFrame, analyze
    from canbus_analyzer.dbcutil import encode, load_db
    db = load_db()
    lid = db.get_message_by_name("LEAD_INFO").frame_id
    pid = db.get_message_by_name("PRE_COLLISION").frame_id
    frames = [
        RxFrame(0.00, lid, encode("LEAD_INFO", {"LEAD_LONG_DIST": 17.0, "LEAD_REL_SPEED": -10.0})),
        # FORCE -20000 N is outside the [-10000, 0] plausibility range
        RxFrame(0.03, pid, encode("PRE_COLLISION", {"_COUNTER": 1, "PRECOLLISION_ACTIVE": 1, "FORCE": -20000.0})),
        RxFrame(0.06, pid, encode("PRE_COLLISION", {"_COUNTER": 2, "PRECOLLISION_ACTIVE": 1, "FORCE": -6000.0})),
    ]
    a = analyze(frames)
    assert a.counts.get("range") == 1
    assert a.latency.request_s == pytest.approx(0.06)
