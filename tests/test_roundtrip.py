import can

from canbus_analyzer.faults import FaultConfig
from canbus_analyzer.logio import compare, for_logging, read_log, roundtrip, write_log
from canbus_analyzer.pipeline import run_once


def test_asc_and_blf_roundtrip_frame_by_frame(tmp_path):
    r = run_once(0, FaultConfig.preset("all"))      # includes short/long DLC frames
    results = roundtrip(r.received, tmp_path)
    assert {x.fmt for x in results} == {"asc", "blf"}
    for x in results:
        assert x.ok, x
        assert x.written == x.read == len(r.received)


def test_readback_values_explicitly(tmp_path):
    msgs = for_logging(run_once(1, FaultConfig(dlc_n=3)).received)
    for suffix in ("asc", "blf"):
        p = tmp_path / f"t.{suffix}"
        write_log(msgs, p)
        back = read_log(p)
        assert [m.arbitration_id for m in back] == [m.arbitration_id for m in msgs]
        assert [bytes(m.data) for m in back] == [bytes(m.data) for m in msgs]
        t0, b0 = msgs[0].timestamp, back[0].timestamp
        assert max(abs((a.timestamp - t0) - (b.timestamp - b0)) for a, b in zip(msgs, back)) <= 1e-6


def _msg(t, data=b"\x01\x02", aid=0x100):
    return can.Message(timestamp=t, arbitration_id=aid, is_extended_id=False, data=data)


def test_compare_detects_mismatches():
    base = [_msg(0.0), _msg(0.01)]
    assert compare(base, [_msg(5.0), _msg(5.01)], "asc", "x").ok       # offset-free comparison
    assert not compare(base, [_msg(0.0), _msg(0.01, b"\x01\x03")], "asc", "x").ok
    assert not compare(base, [_msg(0.0), _msg(0.01, aid=0x101)], "asc", "x").ok
    assert not compare(base, [_msg(0.0), _msg(0.0105)], "blf", "x").ok
    assert not compare(base, [_msg(0.0)], "blf", "x").ok
    assert not compare(base, [_msg(0.0), _msg(0.01, b"\x01")], "blf", "x").ok
