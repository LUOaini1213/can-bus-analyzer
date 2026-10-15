import subprocess
import sys
from pathlib import Path

import pytest

from canbus_analyzer.config import CYCLE_TIME_S
from canbus_analyzer.dbcutil import encode, load_db, signal_limits, toyota_checksum, with_checksum

ROOT = Path(__file__).resolve().parents[1]


def test_generated_dbc_is_reproducible_from_upstream():
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "build_dbc.py"), "--check"],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_dbc_has_no_cycle_time_attributes_for_used_messages():
    # documents why config.CYCLE_TIME_S exists
    db = load_db()
    assert all(db.get_message_by_name(m).cycle_time is None for m in CYCLE_TIME_S)


def test_toyota_checksum_hand_computed_vector():
    # SPEED (0xB4), 72.00 km/h -> raw 7200 = 0x1C20 in bytes 5..6.
    # checksum = len(8) + 0xB4 + 0x1C + 0x20 = 8 + 180 + 28 + 32 = 248 = 0xF8
    data = bytes([0, 0, 0, 0, 0, 0x1C, 0x20, 0])
    assert toyota_checksum(0xB4, data) == 0xF8
    # multi-byte address: 0x283 contributes 0x83 + 0x02
    assert toyota_checksum(0x283, bytes(7)) == (7 + 0x83 + 0x02) & 0xFF


def test_checksum_ignores_its_own_byte_and_wraps():
    a = toyota_checksum(0x2E6, bytes([0xFF] * 7 + [0x00]))
    b = toyota_checksum(0x2E6, bytes([0xFF] * 7 + [0x55]))
    assert a == b == (8 + 0xE6 + 0x02 + 7 * 0xFF) & 0xFF


def test_encode_sets_valid_checksum_and_decodes_back():
    data = encode("SPEED", {"SPEED": 72.0})
    assert data == bytes([0, 0, 0, 0, 0, 0x1C, 0x20, 0xF8])
    dec = load_db().get_message_by_name("SPEED").decode(data)
    assert dec["SPEED"] == pytest.approx(72.0)
    assert dec["CHECKSUM"] == 0xF8


def test_wheel_speed_offset_and_signed_signals_decode():
    db = load_db()
    ws = db.get_message_by_name("WHEEL_SPEEDS").decode(encode("WHEEL_SPEEDS", {"WHEEL_SPEED_FL": 0.0}))
    assert ws["WHEEL_SPEED_FL"] == pytest.approx(0.0, abs=1e-9)   # offset -67.67
    lead = db.get_message_by_name("LEAD_INFO").decode(
        encode("LEAD_INFO", {"LEAD_LONG_DIST": 25.5, "LEAD_REL_SPEED": -3.275}))
    assert lead["LEAD_LONG_DIST"] == pytest.approx(25.5)
    assert lead["LEAD_REL_SPEED"] == pytest.approx(-3.275)
    pc = db.get_message_by_name("PRE_COLLISION").decode(
        encode("PRE_COLLISION", {"FORCE": -6000.0, "PRECOLLISION_ACTIVE": 1, "STATE": 3, "_COUNTER": 255}),
        decode_choices=False)
    assert pc["FORCE"] == -6000.0 and pc["PRECOLLISION_ACTIVE"] == 1 and pc["STATE"] == 3
    assert pc["_COUNTER"] == 255


def test_with_checksum_leaves_messages_without_checksum_untouched():
    raw = bytes(range(8))
    assert with_checksum("BRAKE_MODULE", raw) == raw


def test_range_limits_dbc_and_overrides():
    assert signal_limits("SPEED", "SPEED") == (0.0, 250.0)
    assert signal_limits("PRE_COLLISION", "FORCE") == (-10000.0, 0.0)   # override, DBC says [0|255]
    assert signal_limits("WHEEL_SPEEDS", "WHEEL_SPEED_RR") == (0.0, 250.0)
