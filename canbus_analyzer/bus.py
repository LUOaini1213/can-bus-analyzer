"""Replay frames over python-can's in-process `virtual` interface.

A sender thread transmits every frame on one VirtualBus instance; a receiver
on the same channel records what actually arrives.  `preserve_timestamps=True`
keeps the scenario's logical timestamps (EPOCH + t), so the replay is
deterministic and runs faster than real time.  The virtual interface is a set
of in-memory queues: it does NOT model arbitration, bit timing, bus load or
error frames -- all timing faults come from the scenario/fault injector.
"""
from __future__ import annotations

import itertools
import threading

import can

from .config import EPOCH
from .scenario import Frame

_channel_ids = itertools.count()


def to_message(f: Frame) -> can.Message:
    return can.Message(timestamp=EPOCH + f.t, arbitration_id=f.arbitration_id,
                       is_extended_id=False, data=f.data, dlc=len(f.data), check=True)


def replay(frames: list[Frame], idle_timeout_s: float = 2.0) -> list[can.Message]:
    channel = f"canbus-analyzer-{next(_channel_ids)}"
    rx = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
    tx = can.Bus(interface="virtual", channel=channel, preserve_timestamps=True)
    received: list[can.Message] = []
    try:
        def send_all() -> None:
            for f in frames:
                tx.send(to_message(f))

        sender = threading.Thread(target=send_all, name="can-sender", daemon=True)
        sender.start()
        while len(received) < len(frames):
            msg = rx.recv(timeout=idle_timeout_s)
            if msg is None:      # nothing for idle_timeout_s: give up
                break
            received.append(msg)
        sender.join(timeout=idle_timeout_s)
    finally:
        tx.shutdown()
        rx.shutdown()
    return received
