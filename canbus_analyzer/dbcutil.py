"""DBC loading, Toyota checksum, encode helpers."""
from __future__ import annotations

from functools import lru_cache

import cantools

from .config import CHECKSUM_MESSAGES, DBC_PATH, RANGE_CHECKED, RANGE_OVERRIDES


@lru_cache(maxsize=1)
def load_db() -> cantools.database.can.Database:
    # strict=False: opendbc DBCs contain overlapping placeholder signals in
    # messages we do not use; cantools' strict mode rejects the whole file.
    return cantools.database.load_file(str(DBC_PATH), strict=False)


def toyota_checksum(address: int, data: bytes) -> int:
    """Same algorithm as opendbc/car/toyota/toyotacan.py::toyota_checksum.

    Sum of frame length, every byte of the 11-bit address and every data
    byte except the last (which holds the checksum), modulo 256.
    """
    s = len(data)
    addr = address
    while addr:
        s += addr & 0xFF
        addr >>= 8
    for b in data[:-1]:
        s += b
    return s & 0xFF


def has_checksum(name: str) -> bool:
    return name in CHECKSUM_MESSAGES


def with_checksum(name: str, data: bytes) -> bytes:
    """Return data with the CHECKSUM byte (last byte) recomputed."""
    if not has_checksum(name):
        return bytes(data)
    msg = load_db().get_message_by_name(name)
    buf = bytearray(data)
    buf[-1] = toyota_checksum(msg.frame_id, buf)
    return bytes(buf)


@lru_cache(maxsize=None)
def _template(name: str) -> dict[str, float]:
    return {s.name: 0 for s in load_db().get_message_by_name(name).signals}


def encode(name: str, values: dict[str, float]) -> bytes:
    """Encode physical values (missing signals = 0) and fix the checksum.

    strict=False so that fault injection can write out-of-range values that
    are still representable in the raw bit field.
    """
    full = dict(_template(name))
    full.update(values)
    data = load_db().get_message_by_name(name).encode(full, scaling=True, strict=False)
    return with_checksum(name, data)


def signal_limits(name: str, signal: str) -> tuple[float, float]:
    if (name, signal) in RANGE_OVERRIDES:
        return RANGE_OVERRIDES[(name, signal)]
    sig = load_db().get_message_by_name(name).get_signal_by_name(signal)
    if sig.minimum is None or sig.maximum is None:
        raise KeyError(f"{name}.{signal} has no usable DBC range; add an override")
    return float(sig.minimum), float(sig.maximum)


def all_limits() -> dict[tuple[str, str], tuple[float, float]]:
    return {(m, s): signal_limits(m, s) for m, sigs in RANGE_CHECKED.items() for s in sigs}
