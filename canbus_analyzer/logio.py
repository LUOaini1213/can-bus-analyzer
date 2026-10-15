"""Export recorded traffic to Vector ASC / BLF with python-can and read it back.

The files are written and parsed only by python-can's ASCWriter/ASCReader and
BLFWriter/BLFReader.  They have not been opened in CANalyzer/CANoe.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import can

# ASC stores timestamps with 6 decimals (1 us).  BLF stores them as integer
# nanoseconds or 10 us units depending on the object header; python-can's
# BLFWriter uses nanoseconds.  Tolerance is applied to times *relative to the
# first frame* so the absolute-epoch float rounding does not matter.
TOLERANCE_S = {"asc": 1.0e-6, "blf": 1.0e-6}


@dataclass(frozen=True)
class RoundTrip:
    fmt: str
    path: str
    written: int
    read: int
    id_mismatch: int
    data_mismatch: int
    dlc_mismatch: int
    max_dt_us: float
    tolerance_us: float

    @property
    def ok(self) -> bool:
        return (self.written == self.read and self.id_mismatch == 0 and self.data_mismatch == 0
                and self.dlc_mismatch == 0 and self.max_dt_us <= self.tolerance_us)


def write_log(msgs: list[can.Message], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with can.Logger(str(path)) as writer:    # picks ASCWriter / BLFWriter by suffix
        for m in msgs:
            writer.on_message_received(m)


def read_log(path: Path) -> list[can.Message]:
    return list(can.LogReader(str(path)))


def compare(orig: list[can.Message], back: list[can.Message], fmt: str, path: str) -> RoundTrip:
    t0o = orig[0].timestamp if orig else 0.0
    t0b = back[0].timestamp if back else 0.0
    id_mm = sum(1 for a, b in zip(orig, back)
                if a.arbitration_id != b.arbitration_id or a.is_extended_id != b.is_extended_id)
    data_mm = sum(1 for a, b in zip(orig, back) if bytes(a.data) != bytes(b.data))
    dlc_mm = sum(1 for a, b in zip(orig, back) if a.dlc != b.dlc)
    max_dt = max((abs((a.timestamp - t0o) - (b.timestamp - t0b)) for a, b in zip(orig, back)),
                 default=0.0)
    return RoundTrip(fmt, path, len(orig), len(back), id_mm, data_mm, dlc_mm,
                     max_dt * 1e6, TOLERANCE_S[fmt] * 1e6)


def for_logging(msgs: list[can.Message]) -> list[can.Message]:
    """Copies on channel 0 (written as CAN channel 1 in ASC)."""
    return [can.Message(timestamp=m.timestamp, arbitration_id=m.arbitration_id,
                        is_extended_id=m.is_extended_id, data=bytes(m.data), dlc=m.dlc,
                        channel=0, check=True) for m in msgs]


def roundtrip(msgs: list[can.Message], out_dir: Path, stem: str = "traffic") -> list[RoundTrip]:
    msgs = for_logging(msgs)
    results = []
    for fmt in ("asc", "blf"):
        path = out_dir / f"{stem}.{fmt}"
        write_log(msgs, path)
        back = read_log(path)
        results.append(compare(msgs, back, fmt, path.name))
    return results
