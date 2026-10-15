"""Independent negative acceptance tests for the portable CANoe bench contract.

These tests deliberately alter recorded traffic rather than reimplementing the
AEB algorithm.  A passing Python or supplied capture never proves CANoe ran.
"""
from __future__ import annotations

import copy
import json
import shutil
from pathlib import Path

import can
import pytest

from canbus_analyzer.canoe_suite import (
    CONTROL_ID,
    HUD_ID,
    MAGIC,
    PRE_ID,
    export_bundle,
    verify_capture,
)
from canbus_analyzer.dbcutil import toyota_checksum


@pytest.fixture(scope="module")
def bench(tmp_path_factory):
    bundle = tmp_path_factory.mktemp("canoe-bench") / "bundle"
    result = export_bundle(bundle)
    with can.ASCReader(str(bundle / "python_reference.asc")) as reader:
        messages = list(reader)
    contract = json.loads((bundle / "contract.json").read_text(encoding="utf-8"))
    return bundle, result, messages, contract


def write_capture(path: Path, messages: list[can.Message]) -> Path:
    # ASCWriter normalizes its origin to the first frame.  Tests retain a
    # zero-time output frame unless removal itself is the intended mutation.
    with can.ASCWriter(str(path), channel=1) as writer:
        for message in messages:
            writer.on_message_received(message)
    return path


def reset_marker(message: can.Message) -> bool:
    return (message.arbitration_id == CONTROL_ID and len(message.data) == 8
            and bytes(message.data[:4]) == MAGIC and message.data[6] == 0)


def assert_rejected(bench, tmp_path, messages, *, error=None):
    bundle = bench[0]
    capture = write_capture(tmp_path / "mutated.asc", messages)
    report = verify_capture(bundle, capture)
    assert report["passed"] is False, report
    assert report["canoe_execution_confirmed"] is False
    assert report["errors"] or any(not row["passed"] for row in report["assertions"])
    if error is not None:
        assert any(error in value for value in report["errors"]), report
    return report


@pytest.mark.parametrize("extension", ["asc", "blf"])
@pytest.mark.parametrize("reference", [False, True])
def test_reference_passes_without_claiming_canoe_execution(bench, reference, extension):
    bundle, exported, _, contract = bench
    report = verify_capture(bundle, bundle / f"python_reference.{extension}", reference=reference)
    assert exported["python_reference_passed"] is True
    assert exported["canoe_compiled"] is False
    assert exported["canoe_measured"] is False
    assert report["passed"] is True, report
    assert report["checks_passed"] == report["checks_total"] == len(contract["checks"])
    assert report["checks_total"] > 0
    assert report["canoe_execution_confirmed"] is False
    assert report["origin"] == ("python_reference" if reference
                                 else "provided_log_origin_unverified")


def test_nonzero_epoch_asc_round_trip_preserves_contract(bench, tmp_path):
    messages = copy.deepcopy(bench[2])
    for message in messages:
        message.timestamp += 1_700_000_000.0
    path = write_capture(tmp_path / "offset.asc", messages)
    report = verify_capture(bench[0], path)
    assert report["passed"] is True, report
    assert report["canoe_execution_confirmed"] is False


def test_simultaneous_different_ids_may_arrive_in_different_order(bench, tmp_path):
    messages = copy.deepcopy(bench[2])
    original_order = [(message.timestamp, message.arbitration_id) for message in messages]
    messages.sort(key=lambda message: (message.timestamp, -message.arbitration_id))
    assert [(message.timestamp, message.arbitration_id) for message in messages] != original_order
    stimulus_ids = {frame["id"] for frame in bench[3]["frames"]}
    before = [(message.timestamp, message.arbitration_id) for message in bench[2]
              if message.arbitration_id in stimulus_ids]
    after = [(message.timestamp, message.arbitration_id) for message in messages
             if message.arbitration_id in stimulus_ids]
    assert before != after, "The positive case must actually reorder stimuli, not just outputs."
    capture = write_capture(tmp_path / "arbitration-reordered.asc", messages)
    report = verify_capture(bench[0], capture)
    assert report["passed"] is True, report
    assert report["canoe_execution_confirmed"] is False


@pytest.mark.parametrize("which", ["all", "one"])
def test_missing_case_markers_fail(bench, tmp_path, which):
    messages = copy.deepcopy(bench[2])
    removed = [message for message in messages
               if reset_marker(message) and (which == "all" or message.data[4] == 3)]
    assert removed
    messages = [message for message in messages if message not in removed]
    assert_rejected(bench, tmp_path, messages, error="case reset markers")


def test_duplicate_case_marker_fails(bench, tmp_path):
    messages = copy.deepcopy(bench[2])
    index = next(i for i, message in enumerate(messages) if reset_marker(message))
    messages.insert(index + 1, copy.deepcopy(messages[index]))
    assert_rejected(bench, tmp_path, messages, error="case reset markers")


@pytest.mark.parametrize("frame_id", [PRE_ID, HUD_ID])
def test_missing_output_stream_fails(bench, tmp_path, frame_id):
    messages = [copy.deepcopy(message) for message in bench[2]
                if message.arbitration_id != frame_id]
    report = assert_rejected(bench, tmp_path, messages)
    assert any(not row["passed"] for row in report["assertions"])


def test_missing_outputs_for_one_observation_window_fail(bench, tmp_path):
    # First case's 350-ms FCW checkpoint must have evidence in (285,350] ms.
    messages = [copy.deepcopy(message) for message in bench[2]
                if not (message.arbitration_id in (PRE_ID, HUD_ID)
                        and 0.285 < message.timestamp <= 0.350)]
    assert len(messages) < len(bench[2])
    report = assert_rejected(bench, tmp_path, messages)
    row = next(row for row in report["assertions"] if row["case"] == 1 and row["ms"] == 350)
    assert row["passed"] is False
    assert row["pre_samples"] == row["hud_samples"] == 0


def test_wrong_state_with_valid_checksum_is_rejected(bench, tmp_path):
    messages = copy.deepcopy(bench[2])
    changed = 0
    for message in messages:
        if message.arbitration_id == PRE_ID and 0.285 < message.timestamp <= 0.350:
            # Advertise disabled state while retaining the otherwise valid frame.
            message.data[4] &= ~(7 << 2)
            message.data[-1] = toyota_checksum(PRE_ID, bytes(message.data))
            assert message.data[-1] == toyota_checksum(PRE_ID, bytes(message.data))
            changed += 1
    assert changed
    report = assert_rejected(bench, tmp_path, messages)
    assert not any("checksum" in error for error in report["errors"])
    assert any(not row["passed"] for row in report["assertions"])


def test_transient_wrong_state_cannot_be_hidden_by_later_correct_output(bench, tmp_path):
    messages = copy.deepcopy(bench[2])
    correct_samples = [message for message in messages
                       if message.arbitration_id == PRE_ID and 0.285 < message.timestamp <= 0.350]
    assert len(correct_samples) >= 2
    assert all(((message.data[4] >> 2) & 7) == 2 for message in correct_samples)
    wrong = copy.deepcopy(correct_samples[0])
    wrong.timestamp = (correct_samples[0].timestamp + correct_samples[-1].timestamp) / 2
    wrong.data[4] &= ~(7 << 2)
    wrong.data[-1] = toyota_checksum(PRE_ID, bytes(wrong.data))
    assert wrong.timestamp < correct_samples[-1].timestamp
    messages.append(wrong)
    messages.sort(key=lambda message: message.timestamp)
    report = assert_rejected(bench, tmp_path, messages)
    assert not any("checksum" in error for error in report["errors"]), report
    checkpoint = next(row for row in report["assertions"]
                      if row["case"] == 1 and row["ms"] == 350)
    assert checkpoint["passed"] is False
    assert checkpoint["pre_samples"] == len(correct_samples) + 1


def test_bad_output_checksum_is_rejected(bench, tmp_path):
    messages = copy.deepcopy(bench[2])
    message = next(message for message in messages if message.arbitration_id == PRE_ID)
    message.data[-1] ^= 0x01
    assert_rejected(bench, tmp_path, messages, error="PRE_COLLISION DLC/checksum")


def test_counter_discontinuity_with_valid_checksum_is_rejected(bench, tmp_path):
    messages = copy.deepcopy(bench[2])
    message = next(message for message in messages
                   if message.arbitration_id == PRE_ID and message.timestamp > 0.1)
    message.data[0] = (message.data[0] + 2) % 256
    message.data[-1] = toyota_checksum(PRE_ID, bytes(message.data))
    report = assert_rejected(bench, tmp_path, messages, error="output counter discontinuity")
    assert not any("checksum" in error for error in report["errors"]), report
    assert all(row["passed"] for row in report["assertions"])


@pytest.mark.parametrize("frame_id,name", [(PRE_ID, "PRE_COLLISION"), (HUD_ID, "PCS_HUD")])
def test_missing_output_outside_checkpoints_breaks_period(bench, tmp_path, frame_id, name):
    messages = copy.deepcopy(bench[2])
    index = next(i for i, message in enumerate(messages)
                 if message.arbitration_id == frame_id and abs(message.timestamp - 0.210) < 0.000001)
    removed = messages.pop(index)
    observation_ms = bench[3]["observation_window_ms"]
    assert not any(row["ms"] - observation_ms < removed.timestamp * 1000 <= row["ms"]
                   for row in bench[3]["checks"])
    report = assert_rejected(bench, tmp_path, messages, error=f"{name} output period mismatch")
    assert all(row["passed"] for row in report["assertions"])


def test_continuous_counter_wraparound_is_accepted(bench, tmp_path):
    messages = copy.deepcopy(bench[2])
    wrapped = False
    previous = None
    for message in messages:
        if message.arbitration_id != PRE_ID:
            continue
        message.data[0] = (message.data[0] + 200) % 256
        message.data[-1] = toyota_checksum(PRE_ID, bytes(message.data))
        if previous is not None and previous.data[0] == 255 and message.data[0] == 0:
            wrapped = True
        previous = message
    assert wrapped
    capture = write_capture(tmp_path / "counter-wraparound.asc", messages)
    report = verify_capture(bench[0], capture)
    assert report["passed"] is True, report
    assert report["canoe_execution_confirmed"] is False


@pytest.mark.parametrize("attribute,value", [
    ("channel", 1),
    ("is_extended_id", True),
    ("is_remote_frame", True),
    ("is_fd", True),
])
def test_nonclassic_or_wrong_channel_traffic_fails(bench, tmp_path, attribute, value):
    messages = copy.deepcopy(bench[2])
    message = next(message for message in messages if message.arbitration_id == PRE_ID)
    setattr(message, attribute, value)
    assert_rejected(bench, tmp_path, messages, error="non-Classic-CAN1")


@pytest.mark.parametrize("mutation", ["remove", "change", "duplicate", "retime"])
def test_stimulus_must_match_exact_exported_contract(bench, tmp_path, mutation):
    messages = copy.deepcopy(bench[2])
    stimulus_ids = {frame["id"] for frame in bench[3]["frames"]} - {CONTROL_ID}
    index = next(i for i, message in enumerate(messages)
                 if message.arbitration_id in stimulus_ids and message.timestamp > 0.1)
    if mutation == "remove":
        messages.pop(index)
    elif mutation == "change":
        messages[index].data[0] ^= 0x01
    elif mutation == "duplicate":
        messages.insert(index + 1, copy.deepcopy(messages[index]))
    else:
        messages[index].timestamp += 0.006  # More than the allowed five milliseconds.
        messages.sort(key=lambda message: message.timestamp)
    assert_rejected(bench, tmp_path, messages, error="stimulus")


def test_same_id_payload_order_must_not_change(bench, tmp_path):
    messages = copy.deepcopy(bench[2])
    stimulus_ids = {frame["id"] for frame in bench[3]["frames"]} - {CONTROL_ID}
    selected = None
    for frame_id in sorted(stimulus_ids):
        stream = [message for message in messages if message.arbitration_id == frame_id]
        selected = next(((before, after) for before, after in zip(stream, stream[1:])
                         if bytes(before.data) != bytes(after.data)), None)
        if selected is not None:
            break
    assert selected is not None
    before, after = selected
    assert before.timestamp < after.timestamp
    # Swap whole payloads, keeping both valid same-ID checksums, timestamps,
    # total counts, and the multiset of data. Only temporal payload order changes.
    before.data, after.data = after.data, before.data
    report = assert_rejected(bench, tmp_path, messages, error="stimulus payload/order mismatch")
    assert all(row["passed"] for row in report["assertions"])


def test_capture_truncated_after_all_checkpoints_still_fails(bench, tmp_path):
    cutoff = (bench[3]["end_ms"] - 66) / 1000
    assert cutoff * 1000 > max(row["ms"] for row in bench[3]["checks"])
    messages = [copy.deepcopy(message) for message in bench[2]
                if message.timestamp <= cutoff]
    report = assert_rejected(bench, tmp_path, messages, error="output end coverage missing")
    assert all(row["passed"] for row in report["assertions"])


def test_missing_initial_output_periods_fail_despite_passing_checkpoints(bench, tmp_path):
    # The first observation window starts after these three output periods.
    messages = [copy.deepcopy(message) for message in bench[2]
                if not (message.arbitration_id in (PRE_ID, HUD_ID)
                        and message.timestamp < 0.090)]
    assert len(messages) == len(bench[2]) - 6
    report = assert_rejected(bench, tmp_path, messages)
    for name in ("PRE_COLLISION", "PCS_HUD"):
        assert f"{name} output start coverage missing" in report["errors"]
    assert len(report["assertions"]) == 36
    assert all(row["passed"] for row in report["assertions"])


@pytest.mark.parametrize("missing_ids", [(PRE_ID,), (HUD_ID,), (PRE_ID, HUD_ID)])
def test_unrelated_late_frame_cannot_replace_output_end_coverage(bench, tmp_path, missing_ids):
    cutoff = max(row["ms"] for row in bench[3]["checks"]) / 1000
    assert cutoff == 23.100
    messages = [copy.deepcopy(message) for message in bench[2]
                if not (message.arbitration_id in missing_ids and message.timestamp > cutoff)]
    assert len(messages) < len(bench[2])
    # Keeping the last timestamp cannot prove that either output kept running.
    messages.append(can.Message(timestamp=bench[3]["end_ms"] / 1000,
                                arbitration_id=0x600, data=b"12345678",
                                is_extended_id=False, channel=0))
    messages.sort(key=lambda message: message.timestamp)
    report = assert_rejected(bench, tmp_path, messages)
    for frame_id, name in ((PRE_ID, "PRE_COLLISION"), (HUD_ID, "PCS_HUD")):
        error = f"{name} output end coverage missing"
        assert (error in report["errors"]) == (frame_id in missing_ids)
    assert len(report["assertions"]) == 36
    assert all(row["passed"] for row in report["assertions"])


def test_first_output_at_30ms_is_accepted_for_capl_start_timer(bench, tmp_path):
    # CAPL starts its output timer at 30 ms; the Python reference also emits at 0.
    messages = [copy.deepcopy(message) for message in bench[2]
                if not (message.arbitration_id in (PRE_ID, HUD_ID)
                        and message.timestamp == 0)]
    assert len(messages) == len(bench[2]) - 2
    capture = write_capture(tmp_path / "first-output-at-30ms.asc", messages)
    report = verify_capture(bench[0], capture)
    assert report["passed"] is True, report
    assert report["checks_passed"] == report["checks_total"] == 36
    assert report["canoe_execution_confirmed"] is False


@pytest.mark.parametrize("relative_path", [
    "canoe/aeb_core.cin", "canoe/generated/regression_data.cin", "contract.json",
    "dbc/toyota_nodsu_pt_generated.dbc",
])
def test_bundle_file_tampering_invalidates_acceptance(bench, tmp_path, relative_path):
    bundle = tmp_path / "tampered-bundle"
    shutil.copytree(bench[0], bundle)
    path = bundle / relative_path
    # Whitespace preserves JSON/DBC semantics while invalidating the recorded hash.
    path.write_bytes(path.read_bytes() + b"\n ")
    report = verify_capture(bundle, bundle / "python_reference.asc")
    assert report["passed"] is False
    assert f"bundle checksum mismatch: {relative_path}" in report["errors"]
    assert report["canoe_execution_confirmed"] is False


def test_missing_manifest_listed_file_fails(bench, tmp_path):
    bundle = tmp_path / "missing-file-bundle"
    shutil.copytree(bench[0], bundle)
    (bundle / "canoe" / "aeb_ecu.can").unlink()
    report = verify_capture(bundle, bundle / "python_reference.asc")
    assert report["passed"] is False
    assert "bundle checksum mismatch: canoe/aeb_ecu.can" in report["errors"]


@pytest.mark.parametrize("absolute", [False, True])
def test_manifest_must_not_escape_bundle(bench, tmp_path, absolute):
    bundle = tmp_path / "unsafe-manifest-bundle"
    shutil.copytree(bench[0], bundle)
    manifest_path = bundle / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    outside = tmp_path / "outside.txt"
    outside.write_text("This file is outside the bundle.", encoding="utf-8")
    unsafe_name = str(outside.resolve()) if absolute else "../outside.txt"
    manifest["files_sha256"][unsafe_name] = "0" * 64
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="escapes bundle"):
        verify_capture(bundle, bundle / "python_reference.asc")


def test_unsupported_capture_extension_is_rejected(bench, tmp_path):
    capture = tmp_path / "not-a-supported-capture.txt"
    capture.write_text("", encoding="utf-8")
    with pytest.raises(ValueError, match="ASC or BLF"):
        verify_capture(bench[0], capture)


def test_verify_cli_writes_reviewable_report_and_success_exit(bench, tmp_path, capsys):
    from canbus_analyzer.__main__ import main

    bundle = bench[0]
    report_path = tmp_path / "reports" / "reference.json"
    exit_code = main(["canoe-verify", str(bundle / "python_reference.blf"),
                      "--bundle", str(bundle), "--report", str(report_path), "--reference"])
    output = capsys.readouterr()
    report = json.loads(output.out)
    assert exit_code == 0
    assert output.err == ""
    assert json.loads(report_path.read_text(encoding="utf-8")) == report
    assert report["passed"] is True
    assert report["canoe_execution_confirmed"] is False


def test_verify_cli_failure_is_nonzero_and_preserves_failed_report(bench, tmp_path, capsys):
    from canbus_analyzer.__main__ import main

    messages = [copy.deepcopy(message) for message in bench[2]
                if message.arbitration_id != PRE_ID]
    capture = write_capture(tmp_path / "missing-output.asc", messages)
    report_path = tmp_path / "failed.json"
    exit_code = main(["canoe-verify", str(capture), "--bundle", str(bench[0]),
                      "--report", str(report_path)])
    output = capsys.readouterr()
    report = json.loads(output.out)
    assert exit_code == 1
    assert report["passed"] is False
    assert report["canoe_execution_confirmed"] is False
    assert json.loads(report_path.read_text(encoding="utf-8")) == report


def test_verify_cli_input_error_is_distinct_from_contract_failure(bench, tmp_path, capsys):
    from canbus_analyzer.__main__ import main

    capture = tmp_path / "unsupported.txt"
    capture.write_text("", encoding="utf-8")
    exit_code = main(["canoe-verify", str(capture), "--bundle", str(bench[0])])
    output = capsys.readouterr()
    assert exit_code == 2
    assert output.out == ""
    assert "ASC or BLF" in output.err
