"""Compiler availability must not hide a failed build or core comparison."""
from __future__ import annotations

import json
import subprocess
import sys

import pytest

from scripts import check_capl_core as checker


def probe(monkeypatch, name, calls, *, error=None):
    def invoke():
        calls.append(name)
        if error is not None:
            raise error
        return checker.CompilerBackend(name, (name,), name + " test version")
    monkeypatch.setattr(checker, {"msvc": "msvc_backend", "g++": "gxx_backend",
                                  "wsl": "wsl_backend"}[name], invoke)


def test_windows_auto_prefers_native_msvc_without_starting_wsl(monkeypatch):
    calls = []
    probe(monkeypatch, "msvc", calls)
    probe(monkeypatch, "wsl", calls, error=AssertionError("WSL must not start"))
    backend, failures = checker.select_compiler("auto", windows=True)
    assert backend.name == "msvc"
    assert calls == ["msvc"]
    assert failures == []


def test_windows_auto_falls_back_once_and_keeps_discovery_reason(monkeypatch):
    calls = []
    probe(monkeypatch, "msvc", calls, error=FileNotFoundError("no Visual C++ tools"))
    probe(monkeypatch, "wsl", calls)
    backend, failures = checker.select_compiler("auto", windows=True)
    assert backend.name == "wsl"
    assert calls == ["msvc", "wsl"]
    assert failures == [{"backend": "msvc", "error": "no Visual C++ tools"}]


def test_unavailable_compilers_return_bounded_discovery_failures(monkeypatch):
    calls = []
    probe(monkeypatch, "msvc", calls, error=FileNotFoundError("MSVC unavailable"))
    probe(monkeypatch, "wsl", calls,
          error=subprocess.TimeoutExpired(["wsl", "--exec", "g++"], 30))
    with pytest.raises(checker.CompilerSelectionError) as error:
        checker.select_compiler("auto", windows=True)
    assert calls == ["msvc", "wsl"]
    assert [item["backend"] for item in error.value.attempts] == ["msvc", "wsl"]
    assert "timed out" in error.value.attempts[-1]["error"]


def test_explicit_backend_does_not_silently_fall_back(monkeypatch):
    calls = []
    probe(monkeypatch, "msvc", calls, error=FileNotFoundError("missing cl"))
    probe(monkeypatch, "wsl", calls)
    with pytest.raises(checker.CompilerSelectionError, match="missing cl"):
        checker.select_compiler("msvc", windows=True)
    assert calls == ["msvc"]


def test_linux_auto_keeps_native_gxx(monkeypatch):
    calls = []
    for name in ("msvc", "wsl", "g++"):
        probe(monkeypatch, name, calls)
    backend, failures = checker.select_compiler("auto", windows=False)
    assert backend.name == "g++"
    assert calls == ["g++"]
    assert not failures


@pytest.mark.parametrize("name", ["msvc", "wsl"])
def test_windows_only_backend_is_a_clear_error_on_linux(name):
    with pytest.raises(checker.CompilerSelectionError, match="requires Windows"):
        checker.select_compiler(name, windows=False)


def test_empty_version_is_a_controlled_probe_failure(monkeypatch):
    monkeypatch.setattr(checker.shutil, "which", lambda _: "/fake/g++")
    monkeypatch.setattr(checker, "run_tool", lambda *_a, **_k:
                        subprocess.CompletedProcess(["g++"], 0, stdout=" \n", stderr=""))
    with pytest.raises(checker.CompilerSelectionError, match="version probe returned empty"):
        checker.select_compiler("g++", windows=False)


@pytest.mark.parametrize("failure", ["compile", "compare"])
def test_failed_selected_backend_is_reported_without_fallback(monkeypatch, tmp_path, capsys, failure):
    calls = []
    probe(monkeypatch, "g++", calls)
    probe(monkeypatch, "wsl", calls, error=AssertionError("must not fall back"))
    def run_program(*_args):
        if failure == "compile":
            raise subprocess.CalledProcessError(2, ["g++", "runner.cpp"],
                                                stderr="actual compiler diagnostic")
        return [], ""
    monkeypatch.setattr(checker, "run_core_program", run_program)
    report_path = tmp_path / "failure.json"
    monkeypatch.setattr(sys, "argv", ["check_capl_core.py", "--compiler", "g++",
                                      "--out", str(report_path)])
    assert checker.main() == 1
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert json.loads(capsys.readouterr().out) == report
    assert calls == ["g++"]
    assert report["compiler_backend"] == "g++"
    assert report["compiler_command"] == ["g++"]
    assert report["portable_core_checked"] is False
    assert report["canoe_compiled"] is report["canoe_executed"] is False
    if failure == "compile":
        assert report["tool_stderr"] == "actual compiler diagnostic"
    else:
        assert "step count differs" in report["error"]


def test_missing_explicit_backend_cli_preserves_failure_report(monkeypatch, tmp_path, capsys):
    calls = []
    probe(monkeypatch, "g++", calls, error=FileNotFoundError("compiler removed from PATH"))
    report_path = tmp_path / "unavailable.json"
    monkeypatch.setattr(sys, "argv", ["check_capl_core.py", "--compiler", "g++",
                                      "--out", str(report_path)])
    assert checker.main() == 1
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert json.loads(capsys.readouterr().out) == report
    assert calls == ["g++"]
    assert report["compiler_requested"] == "g++"
    assert report["compiler_probe_failures"] == [
        {"backend": "g++", "error": "compiler removed from PATH"}]
    assert report["portable_core_checked"] is False
