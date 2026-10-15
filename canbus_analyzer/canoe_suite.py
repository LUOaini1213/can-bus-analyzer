"""Portable AEB bench fixtures and independent ASC/BLF contract checks.

The hand-written expected states below are never inferred from the implementation.
The generated reference log is Python evidence, not evidence of CANoe execution.
"""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

import can

from .aeb_function import AebEcu
from .closed_loop import _enc
from .config import EPOCH, ROOT
from .dbcutil import load_db, toyota_checksum

CONTROL_ID = 0x7E0
MAGIC = bytes((65, 69, 66, 1))
INPUTS = {"SPEED": 20, "LEAD_INFO": 50, "GAS_PEDAL": 20, "BRAKE_MODULE": 25}
PRE_ID, HUD_ID = 0x283, 0x411
SLOT_MS = 3000
DURATION_MS = 2300
DEFAULT_OUT = ROOT / "artifacts" / "canoe_aeb"


@dataclass(frozen=True)
class Case:
    key: str
    description: str
    changes: tuple[tuple[int, dict], ...]
    checks: tuple[tuple[int, int], ...]
    corrupt: str | None = None
    controls: tuple[tuple[int, int], ...] = ()


CASES = (
    Case("activation_ladder", "Warning, partial, full, then standstill release",
         ((200, {"distance": 20.0}), (500, {"distance": 12.0}),
          (800, {"distance": 5.0}), (1200, {"speed": 0.0})),
         ((150, 1), (350, 2), (650, 3), (950, 4), (1350, 1))),
    Case("strict_ttc_thresholds", "TTC equal to a threshold does not activate that tier",
         ((200, {"distance": 26.0}), (500, {"distance": 25.9}),
          (800, {"distance": 16.0}), (1100, {"distance": 15.9}),
          (1400, {"distance": 9.0}), (1700, {"distance": 8.9})),
         ((350, 1), (650, 2), (950, 2), (1250, 3), (1550, 3), (1850, 4))),
    Case("gas_override", "40 percent accelerator inhibits braking; 39.5 does not",
         ((200, {"distance": 20.0}), (500, {"distance": 5.0}),
          (800, {"gas": 40.0}), (1100, {"gas": 39.5})),
         ((350, 2), (650, 4), (950, 1), (1250, 4))),
    Case("checksum_and_recovery", "One bad SPEED checksum is discarded; two latch FAULT",
         ((200, {"distance": 20.0}),),
         ((600, 2), (900, 5), (1650, 5), (2100, 2)), "checksum"),
    Case("radar_timeout_and_recovery", "Missing LEAD_INFO expires and recovery stays latched",
         ((200, {"distance": 20.0}),),
         ((450, 2), (800, 5), (1800, 5), (2100, 2)), "timeout"),
    Case("dlc_and_recovery", "One short BRAKE_MODULE frame is discarded; two latch FAULT",
         ((200, {"distance": 20.0}),),
         ((600, 2), (900, 5), (1650, 5), (2100, 2)), "dlc"),
    Case("range_and_unknown_id", "Out-of-range SPEED is invalid; unrelated ID is ignored",
         ((200, {"distance": 20.0}),),
         ((600, 2), (900, 5), (1650, 5), (2100, 2)), "range"),
    Case("driver_brake_and_switch", "Brake cancels FCW; switch off has priority",
         ((200, {"distance": 20.0}), (600, {"brake": 1}), (900, {"brake": 0})),
         ((350, 2), (750, 1), (1050, 2), (1450, 0), (2100, 2)),
         controls=((1305, 0), (1905, 1))),
)


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _control(case_id: int, enabled: int, opcode: int = 0) -> bytes:
    return MAGIC + bytes((case_id, enabled, opcode, 0))


def _expected(state: int) -> dict:
    return {"state": state, "active": int(state in (3, 4)),
            "force": -6000 if state == 3 else (-13500 if state == 4 else 0),
            "fcw": int(state in (2, 3, 4)),
            "indicator": 2 if state == 5 else (1 if state == 0 else 0)}


def build_contract() -> dict:
    db = load_db()
    cases, frames, checks = [], [], []
    for i, case in enumerate(CASES, 1):
        base = (i - 1) * SLOT_MS
        cases.append({"id": i, "key": case.key, "description": case.description,
                      "base_ms": base, "duration_ms": DURATION_MS})
        frames.append({"ms": base + 5, "id": CONTROL_ID, "data": _control(i, 1).hex()})
        for t, enabled in case.controls:
            frames.append({"ms": base + t, "id": CONTROL_ID,
                           "data": _control(i, enabled, 1).hex()})
        for name, period in INPUTS.items():
            for nominal in range(15, DURATION_MS + 1, period):
                # Keep arrivals away from the 10-ms ECU tick. BRAKE's 25-ms
                # schedule consequently has alternating 24/26-ms intervals.
                t = nominal + int(nominal % 10 == 0)
                state = {"speed": 50.0, "distance": 300.0, "relative": -10.0,
                         "gas": 0.0, "brake": 0}
                for when, values in case.changes:
                    if t >= when:
                        state.update(values)
                if case.corrupt == "timeout" and name == "LEAD_INFO" and 500 <= t < 900:
                    continue
                if name == "SPEED":
                    values = {"SPEED": 250.01 if case.corrupt == "range" and t in (515, 715, 735)
                              else state["speed"]}
                elif name == "LEAD_INFO":
                    values = {"LEAD_LONG_DIST": state["distance"], "LEAD_REL_SPEED": state["relative"]}
                elif name == "GAS_PEDAL":
                    values = {"GAS_PEDAL": state["gas"]}
                else:
                    values = {"BRAKE_PRESSED": state["brake"]}
                data = _enc(name, values)
                if case.corrupt == "checksum" and name == "SPEED" and t in (515, 715, 735):
                    data = data[:-1] + bytes((data[-1] ^ 0x5A,))
                if case.corrupt == "dlc" and name == "BRAKE_MODULE" and t in (515, 715, 741):
                    data = data[:-1]
                frames.append({"ms": base + t, "id": db.get_message_by_name(name).frame_id,
                               "data": data.hex()})
        if case.corrupt == "range":
            frames.append({"ms": base + 405, "id": 0x555, "data": "0102030405060708"})
        for t, state in case.checks:
            checks.append({"ms": base + t, "case": i, **_expected(state)})
    frames.sort(key=lambda x: x["ms"])
    checks.sort(key=lambda x: x["ms"])
    return {"schema_version": 1, "channel_zero_based": 0,
            "timing_tolerance_ms": 5, "observation_window_ms": 65,
            "end_ms": (len(CASES) - 1) * SLOT_MS + DURATION_MS + 100,
            "cases": cases, "frames": frames, "checks": checks}


def _decode_pre(data: bytes) -> dict:
    raw = int.from_bytes(data[2:4], "big", signed=True)
    return {"state": (data[4] >> 2) & 7, "active": (data[5] >> 1) & 1, "force": raw * 2}


def _decode_hud(data: bytes) -> dict:
    return {"fcw": (data[0] >> 4) & 1, "indicator": (data[0] >> 6) & 3}


def reference_log(contract: dict) -> list[can.Message]:
    """Run the original Python ECU on the exported stimuli, with hand expectations separate."""
    frames_by_ms: dict[int, list[dict]] = {}
    for frame in contract["frames"]:
        frames_by_ms.setdefault(frame["ms"], []).append(frame)
    ecu = AebEcu()
    counter, log = 0, []
    for ms in range(contract["end_ms"] + 1):
        t = ms / 1000
        for f in frames_by_ms.get(ms, ()):
            data = bytes.fromhex(f["data"])
            log.append(can.Message(timestamp=EPOCH + t, arbitration_id=f["id"], data=data,
                                   is_extended_id=False, channel=0))
            if f["id"] == CONTROL_ID:
                if data[6] == 0:
                    ecu = AebEcu(main_switch=bool(data[5]))
                    ecu.sup.t0, ecu.sm.entered = t, t
                    counter = 0
                else:
                    ecu.main_switch = bool(data[5])
            else:
                ecu.on_frame(t, f["id"], data)
        if ms % 10 == 0:
            ecu.step(t)
        if ms % 30 == 0:
            for name, frame_id in (("PRE_COLLISION", PRE_ID), ("PCS_HUD", HUD_ID)):
                data = _enc(name, ecu.output_values(name, counter))
                log.append(can.Message(timestamp=EPOCH + t, arbitration_id=frame_id, data=data,
                                       is_extended_id=False, channel=0))
            counter += 1
    return log


def _capl_data(contract: dict) -> str:
    frames, checks = contract["frames"], contract["checks"]
    def array(type_: str, name: str, values: list) -> str:
        rows = [",".join(map(str, values[i:i + 16])) for i in range(0, len(values), 16)]
        return f"  {type_} {name}[{len(values)}] = {{\n    " + ",\n    ".join(rows) + "\n  };"
    lines = ["/* GENERATED by python -m canbus_analyzer canoe-build; do not edit. */", "variables {",
             f"  long benchFrameCount = {len(frames)};", f"  long benchCheckCount = {len(checks)};",
             f"  long benchEndMs = {contract['end_ms']};",
             array("long", "benchFrameMs", [f["ms"] for f in frames]),
             array("dword", "benchFrameId", [f["id"] for f in frames]),
             array("long", "benchFrameDlc", [len(bytes.fromhex(f["data"])) for f in frames]),
             f"  byte benchFrameData[{len(frames)}][8] = {{"]
    for f in frames:
        row = bytes.fromhex(f["data"]).ljust(8, b"\x00")
        lines.append("    {" + ",".join(str(b) for b in row) + "},")
    lines.append("  };")
    for suffix, key in (("Ms", "ms"), ("Case", "case"), ("State", "state"), ("Active", "active"),
                        ("Force", "force"), ("Fcw", "fcw"), ("Indicator", "indicator")):
        lines.append(array("long", "benchCheck" + suffix, [c[key] for c in checks]))
    lines.extend(("}", ""))
    return "\n".join(lines)


def _write_asc(path: Path, messages: list[can.Message]) -> None:
    with can.ASCWriter(str(path), channel=1) as writer:
        for msg in messages:
            writer.on_message_received(msg)


def export_bundle(out: Path = DEFAULT_OUT) -> dict:
    import shutil
    out = Path(out).resolve()
    if out == ROOT.resolve() or ROOT.resolve().is_relative_to(out):
        raise ValueError("Use a dedicated output directory, not the repository or its ancestor")
    contract = build_contract()
    (out / "canoe" / "generated").mkdir(parents=True, exist_ok=True)
    (out / "dbc").mkdir(exist_ok=True)
    contract_path = out / "contract.json"
    contract_path.write_text(json.dumps(contract, indent=2) + "\n", encoding="utf-8")
    (out / "canoe" / "generated" / "regression_data.cin").write_text(_capl_data(contract), encoding="ascii")
    for name in ("aeb_core.cin", "aeb_ecu.can", "regression_driver.can"):
        shutil.copyfile(ROOT / "canoe" / name, out / "canoe" / name)
    shutil.copyfile(ROOT / "dbc" / "toyota_nodsu_pt_generated.dbc", out / "dbc" / "toyota_nodsu_pt_generated.dbc")
    for name in ("LICENSE", "THIRD_PARTY_NOTICES", "dbc/SOURCE.md"):
        shutil.copyfile(ROOT / name, out / name)
    messages = reference_log(contract)
    _write_asc(out / "python_reference.asc", messages)
    with can.BLFWriter(str(out / "python_reference.blf"), channel=1) as writer:
        for message in messages:
            writer.on_message_received(message)
    generated_names = ["contract.json", "canoe/generated/regression_data.cin",
                       "canoe/aeb_core.cin", "canoe/aeb_ecu.can", "canoe/regression_driver.can",
                       "dbc/toyota_nodsu_pt_generated.dbc", "LICENSE", "THIRD_PARTY_NOTICES", "dbc/SOURCE.md",
                       "python_reference.asc", "python_reference.blf"]
    if (ROOT / "canoe" / "README.md").is_file():
        shutil.copyfile(ROOT / "canoe" / "README.md", out / "README.md")
        generated_names.append("README.md")
    manifest = {"schema_version": 1, "cases": len(contract["cases"]),
                "checks": len(contract["checks"]), "stimulus_frames": len(contract["frames"]),
                "reference_origin": "Python AebEcu on synthetic open-loop bench inputs",
                "canoe_compiled": False, "canoe_measured": False,
                "source_sha256": {str(p.relative_to(ROOT)).replace("\\", "/"): _sha(p) for p in (
                    Path(__file__), ROOT / "canbus_analyzer" / "aeb_function.py",
                    ROOT / "canbus_analyzer" / "dbcutil.py", ROOT / "canbus_analyzer" / "config.py",
                    ROOT / "canbus_analyzer" / "closed_loop.py", ROOT / "canoe" / "aeb_core.cin",
                    ROOT / "canoe" / "aeb_ecu.can", ROOT / "canoe" / "regression_driver.can")},
                "files_sha256": {name: _sha(out / name) for name in generated_names}}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    for suffix in ("asc", "blf"):
        report = verify_capture(out, out / ("python_reference." + suffix), reference=True)
        if not report["passed"]:
            raise ValueError("Python reference violates hand-written bench contract: " + json.dumps(report["errors"]))
    return {"bundle": str(out.resolve()), **{k: manifest[k] for k in ("cases", "checks", "stimulus_frames")},
            "python_reference_passed": True, "reference_formats": ["asc", "blf"],
            "canoe_compiled": False, "canoe_measured": False}


def verify_capture(bundle: Path, capture: Path, *, reference: bool = False) -> dict:
    """Strict single-channel trace contract check; never authenticates the producer."""
    bundle, capture = Path(bundle), Path(capture)
    manifest = json.loads((bundle / "manifest.json").read_text(encoding="utf-8"))
    errors, assertions = [], []
    for name, expected_sha in manifest["files_sha256"].items():
        target = (bundle / name).resolve()
        if not target.is_relative_to(bundle.resolve()):
            raise ValueError("Manifest file escapes bundle")
        if not target.is_file() or _sha(target) != expected_sha:
            errors.append(f"bundle checksum mismatch: {name}")
    contract = json.loads((bundle / "contract.json").read_text(encoding="utf-8"))
    if capture.suffix.lower() not in (".asc", ".blf"):
        raise ValueError("Capture must be ASC or BLF")
    with can.LogReader(str(capture)) as reader:
        messages = list(reader)
    if any(m.is_extended_id or m.is_fd or m.is_error_frame or m.is_remote_frame or m.channel != 0
           or m.dlc != len(m.data) for m in messages):
        errors.append("capture contains a non-Classic-CAN1 data frame or inconsistent DLC")
    if any(a.timestamp > b.timestamp for a, b in zip(messages, messages[1:])):
        errors.append("capture timestamps are not monotonic")
    if any(not math.isfinite(m.timestamp) for m in messages):
        errors.append("capture contains a non-finite timestamp")
    controls = [m for m in messages if m.arbitration_id == CONTROL_ID]
    resets = [m for m in controls if len(m.data) == 8 and bytes(m.data[:4]) == MAGIC and m.data[6] == 0]
    if [m.data[4] for m in resets] != [c["id"] for c in contract["cases"]]:
        errors.append("missing, duplicated, or reordered case reset markers")
    if not resets:
        return {"passed": False, "errors": errors, "assertions": [], "capture_sha256": _sha(capture),
                "origin": "python_reference" if reference else "provided_log_origin_unverified",
                "canoe_execution_confirmed": False}
    origin = resets[0].timestamp - 0.005
    reset_map = {m.data[4]: m.timestamp for m in resets}
    stimuli_ids = {f["id"] for f in contract["frames"]}
    actual_stimuli = [m for m in messages if m.arbitration_id in stimuli_ids]
    if len(actual_stimuli) != len(contract["frames"]):
        errors.append(f"stimulus count mismatch: {len(actual_stimuli)} vs {len(contract['frames'])}")
    # Different IDs scheduled together may arbitrate in a different order.
    # Preserve exact payload order within each ID, and independently check
    # every frame's time. Do not demand Python's cross-ID insertion order.
    for frame_id in sorted(stimuli_ids):
        actual_id = [m for m in actual_stimuli if m.arbitration_id == frame_id]
        expected_id = [f for f in contract["frames"] if f["id"] == frame_id]
        if len(actual_id) != len(expected_id):
            errors.append(f"stimulus count mismatch for ID {frame_id:X}")
        for index, (m, f) in enumerate(zip(actual_id, expected_id)):
            if bytes(m.data) != bytes.fromhex(f["data"]):
                errors.append(f"stimulus payload/order mismatch at ID {frame_id:X} frame {index}")
                break
            if abs((m.timestamp - origin) * 1000 - f["ms"]) > contract["timing_tolerance_ms"] + 0.01:
                errors.append(f"stimulus timestamp mismatch at ID {frame_id:X} frame {index}")
                break
    for m in messages:
        if m.arbitration_id == PRE_ID and (m.dlc != 7 or len(m.data) != 7 or
                                           m.data[-1] != toyota_checksum(PRE_ID, bytes(m.data))):
            errors.append("invalid PRE_COLLISION DLC/checksum")
            break
        if m.arbitration_id == HUD_ID and (m.dlc != 8 or len(m.data) != 8):
            errors.append("invalid PCS_HUD DLC")
            break
    for frame_id, name in ((PRE_ID, "PRE_COLLISION"), (HUD_ID, "PCS_HUD")):
        output = [m for m in messages if m.arbitration_id == frame_id]
        # CANoe starts its first output timer at 30 ms; the Python reference
        # additionally emits at 0 ms. Both must cover the start and end, on
        # each output stream. An unrelated late frame is not completion proof.
        coverage_slack_ms = 30 + contract["timing_tolerance_ms"] + 0.01
        if not output or (output[0].timestamp - origin) * 1000 > coverage_slack_ms:
            errors.append(f"{name} output start coverage missing")
        if not output or (output[-1].timestamp - origin) * 1000 < contract["end_ms"] - coverage_slack_ms:
            errors.append(f"{name} output end coverage missing")
        for previous, current in zip(output, output[1:]):
            delta_ms = (current.timestamp - previous.timestamp) * 1000
            if abs(delta_ms - 30) > contract["timing_tolerance_ms"] + 0.01:
                errors.append(f"{name} output period mismatch")
                break
            reset_between = any(previous.timestamp < reset.timestamp <= current.timestamp for reset in resets)
            if (frame_id == PRE_ID and not reset_between and len(previous.data) == 7 and len(current.data) == 7
                    and (current.data[0] - previous.data[0]) % 256 != 1):
                errors.append("PRE_COLLISION output counter discontinuity")
                break
    for c in contract["checks"]:
        case_id = c["case"]
        if case_id not in reset_map:
            assertions.append({"case": case_id, "ms": c["ms"], "passed": False, "reason": "missing case marker"})
            continue
        base = reset_map[case_id] - 0.005
        at = base + (c["ms"] - (case_id - 1) * SLOT_MS) / 1000
        previous_controls = [m.timestamp for m in controls if m.timestamp <= at and len(m.data) == 8
                             and m.data[4] == case_id]
        after = max([base, *previous_controls])
        window = [m for m in messages if max(after, at - contract["observation_window_ms"] / 1000) < m.timestamp <= at]
        pre = [m for m in window if m.arbitration_id == PRE_ID and len(m.data) == 7]
        hud = [m for m in window if m.arbitration_id == HUD_ID and len(m.data) == 8]
        ok = bool(pre and hud)
        for group, decoder in ((pre, _decode_pre), (hud, _decode_hud)):
            for m in group:
                ok = ok and all(value == c[key] for key, value in decoder(bytes(m.data)).items())
        assertions.append({"case": case_id, "ms": c["ms"], "passed": ok,
                           "expected": {k: c[k] for k in ("state", "active", "force", "fcw", "indicator")},
                           "pre_samples": len(pre), "hud_samples": len(hud)})
    return {"schema_version": 1, "passed": not errors and all(a["passed"] for a in assertions),
            "origin": "python_reference" if reference else "provided_log_origin_unverified",
            "canoe_execution_confirmed": False, "capture_sha256": _sha(capture),
            "checks_passed": sum(a["passed"] for a in assertions), "checks_total": len(contract["checks"]),
            "errors": errors, "assertions": assertions}
