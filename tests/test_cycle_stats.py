import math

import pytest

from canbus_analyzer.analyzer import bus_load, cycle_findings, percentile, RxFrame


def test_percentile_matches_linear_interpolation():
    xs = list(range(1, 11))
    assert percentile(xs, 95) == pytest.approx(9.55)     # numpy.percentile default
    assert percentile(xs, 50) == pytest.approx(5.5)
    assert percentile(xs, 100) == 10
    assert percentile(xs, 0) == 1
    assert percentile([7.0], 95) == 7.0
    assert math.isnan(percentile([], 95))


def test_perfect_periodic_stream_has_no_findings():
    ts = [i * 10.0 for i in range(20)]
    st, f = cycle_findings("X", ts, 10.0)
    assert f == []
    assert st.n == 20 and st.missing == 0 and st.timing_violations == 0
    assert st.mean_ms == pytest.approx(10000.0) and st.max_ms == pytest.approx(10000.0)


def test_gap_counts_missing_frames():
    # 0,10,20 | 50 (two frames missing at 30, 40) | 60 | 80 (one missing at 70)
    ts = [0.0, 10.0, 20.0, 50.0, 60.0, 80.0]
    st, f = cycle_findings("X", ts, 10.0)
    assert st.missing == 3
    gaps = [x for x in f if x.kind == "missing"]
    assert [(g.t_start, g.t) for g in gaps] == [(20.0, 50.0), (60.0, 80.0)]
    assert st.max_ms == pytest.approx(30000.0)


def test_drop_threshold_boundary_is_strict():
    # delta exactly 1.5 T is NOT a drop (it is a timing violation)
    st, f = cycle_findings("X", [0.0, 10.0, 25.0], 10.0)
    assert st.missing == 0 and st.timing_violations == 1
    st, f = cycle_findings("X", [0.0, 10.0, 25.5], 10.0)
    assert st.missing == 1 and st.timing_violations == 0


def test_jitter_tolerance_boundary_is_strict():
    # tolerance = 20 % of T = 2.0: |12 - 10| = 2 is OK, 12.5 is a violation
    st, _ = cycle_findings("X", [0.0, 12.0, 20.0], 10.0)
    assert st.timing_violations == 0
    st, f = cycle_findings("X", [0.0, 12.5, 20.0], 10.0)
    assert st.timing_violations == 2          # 12.5 and 7.5 both off by 2.5
    assert [x.kind for x in f] == ["timing", "timing"]


def test_p95_and_mean_of_irregular_stream():
    ts = [0.0, 10.0, 21.0, 30.0, 40.0]       # deltas 10, 11, 9, 10 (seconds -> ms x1000)
    st, _ = cycle_findings("X", ts, 10.0)
    assert st.mean_ms == pytest.approx(10000.0)
    assert st.p95_ms == pytest.approx(percentile([10000.0, 11000.0, 9000.0, 10000.0], 95))
    assert st.max_ms == pytest.approx(11000.0)


def test_bus_load_formula():
    frames = [RxFrame(t, 1, bytes(8)) for t in (0.0, 1.0)]
    lo, hi = bus_load(frames, 1.0, bitrate=1000)
    assert lo == pytest.approx(100 * 2 * (47 + 64) / 1000)
    assert hi == pytest.approx(100 * 2 * (47 + 64 + (34 + 64 - 1) // 4) / 1000)
