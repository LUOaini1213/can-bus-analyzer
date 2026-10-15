"""End-to-end: scenario -> fault -> virtual bus -> analyzer, one fault at a time."""
import pytest

from canbus_analyzer.bus import replay
from canbus_analyzer.faults import FaultConfig
from canbus_analyzer.pipeline import _scenario, run_once

SEEDS = (0, 1, 2)


@pytest.mark.parametrize("seed", SEEDS)
def test_clean_run_has_no_findings(seed):
    r = run_once(seed, FaultConfig())
    assert r.analysis.findings == []
    assert r.analysis.unknown_ids == {}
    assert r.analysis.latency.to_request_ms is not None


def test_virtual_bus_delivers_every_frame_in_order():
    frames = list(_scenario(0)[0])
    rx = replay(frames)
    assert len(rx) == len(frames)
    assert [(m.arbitration_id, bytes(m.data)) for m in rx] == [(f.arbitration_id, f.data) for f in frames]


@pytest.mark.parametrize("cfg,kind", [
    (FaultConfig(drop_prob=0.02), "drop"),
    (FaultConfig(delay_n=5), "delay"),
    (FaultConfig(range_n=5), "range"),
    (FaultConfig(dlc_n=5), "dlc"),
    (FaultConfig(counter_n=5), "counter"),
    (FaultConfig(checksum_n=5), "checksum"),
])
def test_each_fault_is_detected_and_localised(cfg, kind):
    for seed in SEEDS:
        r = run_once(seed, cfg)
        sc = r.scores[kind]
        assert sc.injected > 0
        # drops of the very first/last frame of a message are invisible to a gap detector
        if kind == "drop":
            assert sc.detected >= sc.injected - 2
        else:
            assert sc.detected == sc.injected, (seed, kind, sc)


def test_drop_counts_match_missing_frames():
    r = run_once(1, FaultConfig(drop_prob=0.01))
    drops = [e for e in r.injected.events if e.kind == "drop"]
    assert sum(s.missing for s in r.analysis.stats) == r.scores["drop"].detected <= len(drops)
    assert r.analysis.counts.get("counter", 0) == 0   # counter jumps explained by gaps


def test_stuck_detected_when_window_covers_braking():
    # with a 9 s window starting in [0.5, ...] the frozen SPEED always overlaps braking
    r = run_once(0, FaultConfig(stuck_s=8.0))
    assert r.scores["stuck"].detected == 1
    assert {f.kind for f in r.analysis.findings} == {"stuck"}


def test_jitter_produces_timing_findings_only_above_tolerance():
    small = run_once(0, FaultConfig(jitter_ms=0.1))
    assert small.analysis.counts.get("timing", 0) == 0
    big = run_once(0, FaultConfig(jitter_ms=2.0))
    assert big.analysis.counts.get("timing", 0) > 0


def test_runs_are_deterministic():
    a = run_once(3, FaultConfig.preset("all"))
    b = run_once(3, FaultConfig.preset("all"))
    assert a.analysis.findings == b.analysis.findings
    assert a.injected.events == b.injected.events


def test_preset_rejects_unknown_kind():
    with pytest.raises(ValueError):
        FaultConfig.preset("drop,bogus")
