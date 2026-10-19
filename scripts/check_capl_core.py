"""Check the literal portable .cin functions with C++ and the Python AebEcu.

This does NOT compile CAPL, execute CANoe, check CAPL handlers, or validate a
Vector configuration. The only source adaptation unwraps `variables { ... }`;
CAPL byte/dword/long are mapped to 8/32/32-bit C++ integer types. Windows
auto-selects MSVC (an isolated vcvars64 environment), then WSL g++; Linux
uses native g++. No project results are regenerated. Temporary source/binary
are removed automatically.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from canbus_analyzer.aeb_function import AebEcu, S, TRANSITIONS  # noqa: E402
from canbus_analyzer.dbcutil import encode, load_db, pack_byte_signals, toyota_checksum  # noqa: E402

CORE = ROOT / "canoe" / "aeb_core.cin"
INPUT_NAMES = ("SPEED", "LEAD_INFO", "GAS_PEDAL", "BRAKE_MODULE")


@dataclass(frozen=True)
class CompilerBackend:
    name: str
    command: tuple[str, ...]
    version: str
    environment: dict[str, str] | None = None


class CompilerSelectionError(ValueError):
    def __init__(self, attempts: list[dict]):
        self.attempts = attempts
        super().__init__("No usable compiler: " + "; ".join(
            f"{item['backend']}: {item['error']}" for item in attempts))


class CoreCheckError(ValueError):
    def __init__(self, backend: CompilerBackend, probe_failures: list[dict], cause: Exception):
        self.backend = backend
        self.probe_failures = probe_failures
        self.cause = cause
        super().__init__(str(cause))


def backend_evidence(backend: CompilerBackend, probe_failures: list[dict]) -> dict:
    return {"compiler": backend.version, "compiler_backend": backend.name,
            "compiler_command": list(backend.command), "compiler_probe_failures": probe_failures,
            "uses_wsl": backend.name == "wsl"}


def version_line(stdout: str, name: str) -> str:
    lines = stdout.strip().splitlines()
    if not lines:
        raise ValueError(f"{name} version probe returned empty output")
    return lines[0]


def run_tool(command, *, timeout=30, environment=None, **kwargs):
    """All native argument arrays bypass a shell; decode diagnostics safely."""
    return subprocess.run(command, check=True, text=True, encoding="utf-8",
                          errors="replace", capture_output=True, timeout=timeout,
                          env=environment, **kwargs)


def msvc_backend() -> CompilerBackend:
    vswhere = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / (
        "Microsoft Visual Studio/Installer/vswhere.exe")
    if not vswhere.is_file():
        raise FileNotFoundError(f"vswhere not found: {vswhere}")
    install = run_tool([str(vswhere), "-latest", "-products", "*", "-requires",
                        "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-utf8",
                        "-property", "installationPath"]).stdout.strip()
    if not install:
        raise FileNotFoundError("vswhere found no Visual C++ x64 tools installation")
    vcvars = Path(install) / "VC/Auxiliary/Build/vcvars64.bat"
    if not vcvars.is_file():
        raise FileNotFoundError(f"vcvars64.bat not found: {vcvars}")
    cmd = Path(os.environ.get("SystemRoot", r"C:\Windows")) / "System32/cmd.exe"
    # cmd must interpret the batch file. The command is fixed: the discovered
    # path is passed as an environment value, inside quotes, not interpolated
    # into shell syntax. /d disables AutoRun; /u makes SET output UTF-16LE.
    environment = dict(os.environ, AEB_VCVARS64=str(vcvars), VSLANG="1033")
    command = subprocess.list2cmdline([str(cmd)]) + (
        ' /d /u /s /c "call "%AEB_VCVARS64%" >nul && set"')
    loaded = subprocess.run(command, env=environment, check=True, capture_output=True,
                            timeout=30)
    child_environment = {}
    for line in loaded.stdout.decode("utf-16-le", errors="replace").splitlines():
        if "=" in line and not line.startswith("="):
            key, value = line.split("=", 1)
            child_environment[key] = value
    tools = next((v for k, v in child_environment.items()
                  if k.upper() == "VCTOOLSINSTALLDIR"), None)
    if not tools:
        raise ValueError("vcvars64 did not provide VCToolsInstallDir")
    compiler = Path(tools) / "bin/Hostx64/x64/cl.exe"
    if not compiler.is_file():
        raise FileNotFoundError(f"MSVC x64 compiler not found: {compiler}")
    # cl without input prints its version and normally exits 2.
    probe = subprocess.run([str(compiler)], env=child_environment, capture_output=True, timeout=30)
    # Installed language packs may ignore VSLANG; numeric version bytes are
    # ASCII in both localized ANSI and English banners, avoiding mojibake.
    version = re.search(rb"\b\d+\.\d+\.\d+\b", probe.stderr + probe.stdout)
    if probe.returncode not in (0, 2) or version is None:
        raise ValueError("MSVC version probe did not return a compiler version")
    return CompilerBackend("msvc", (str(compiler),),
                           "Microsoft C/C++ " + version.group().decode("ascii") + " (x64)",
                           child_environment)


def gxx_backend() -> CompilerBackend:
    compiler = shutil.which("g++")
    if not compiler:
        raise FileNotFoundError("g++ not found on PATH")
    version = version_line(run_tool([compiler, "--version"]).stdout, "g++")
    return CompilerBackend("g++", (compiler,), version)


def wsl_backend() -> CompilerBackend:
    launcher = shutil.which("wsl")
    if not launcher:
        raise FileNotFoundError("wsl not found on PATH")
    command = (launcher, "--exec", "/usr/bin/g++")
    version = version_line(run_tool([*command, "--version"]).stdout, "WSL g++")
    return CompilerBackend("wsl", command, version)


def select_compiler(choice: str, *, windows: bool | None = None):
    """Fallback only while discovering a usable backend, never after a build."""
    if windows is None:
        windows = os.name == "nt"
    if choice not in ("auto", "g++", "wsl", "msvc"):
        raise ValueError(f"Unknown compiler backend: {choice}")
    candidates = (["msvc", "wsl"] if windows else ["g++"]) if choice == "auto" else [choice]
    attempts = []
    probes = {"msvc": msvc_backend, "g++": gxx_backend, "wsl": wsl_backend}
    for name in candidates:
        try:
            if name in ("msvc", "wsl") and not windows:
                raise ValueError(f"{name} backend requires Windows")
            return probes[name](), attempts
        except (ValueError, OSError, subprocess.SubprocessError) as exc:
            attempts.append({"backend": name, "error": str(exc)})
    raise CompilerSelectionError(attempts)


def unwrap_variables(source: str) -> str:
    """Remove only the CAPL global-declaration wrapper, preserve functions."""
    import re

    matches = list(re.finditer(r"(?m)^variables\s*\{", source))
    if len(matches) != 1:
        raise ValueError("expected exactly one variables block")
    match = matches[0]
    opening = source.index("{", match.start())
    depth = 1
    end = opening + 1
    while depth and end < len(source):
        if source[end] == "{":
            depth += 1
        elif source[end] == "}":
            depth -= 1
        end += 1
    if depth:
        raise ValueError("unclosed variables block")
    return source[:match.start()] + source[opening + 1:end - 1] + source[end:]


RUNNER = r'''
int main() {
  std::string op;
  std::cout << std::setprecision(17);
  while (std::cin >> op) {
    if (op == "R") { double t; std::cin >> t; aebReset(t); }
    else if (op == "M") { int v; std::cin >> v; aebMainSwitch = v; }
    else if (op == "X") { // Verification-only state/time fixture, no production API.
      int state, known; double entered, ok;
      std::cin >> state >> entered >> known >> ok;
      aebState=state; aebEntered=entered; aebOkSinceKnown=known; aebOkSince=ok;
    }
    else if (op == "F") {
      double t; uint32_t id; int n, v; byte data[64] = {};
      std::cin >> t >> id >> n;
      if (n < 0 || n > 64) return 2;
      for (int j=0; j<n; j++) { std::cin >> v; data[j]=static_cast<byte>(v); }
      aebOnFrame(t, id, n, data);
    }
    else if (op == "S") {
      double t; int counter; byte pre[8]={}, hud[8]={};
      std::cin >> t >> counter; aebStep(t);
      int plen=aebBuildPre(pre,counter), hlen=aebBuildHud(hud);
      std::cout << "{\"state\":" << aebState << ",\"rule\":" << aebLastRule
        << ",\"speed\":" << aebSpeed << ",\"distance\":" << aebDistance
        << ",\"relative\":" << aebRelative << ",\"gas\":" << aebGas
        << ",\"brake\":" << aebBrakePressed << ",\"decel\":" << aebDecel;
      std::cout << ",\"pre\":[";
      for(int j=0;j<plen;j++){if(j)std::cout<<',';std::cout<<int(pre[j]);}
      std::cout << "],\"hud\":[";
      for(int j=0;j<hlen;j++){if(j)std::cout<<',';std::cout<<int(hud[j]);}
      std::cout << "],\"received\":[";
      for(int j=0;j<4;j++){if(j)std::cout<<',';std::cout<<aebReceived[j];}
      std::cout << "],\"bad\":[";
      for(int j=0;j<4;j++){if(j)std::cout<<',';std::cout<<aebBad[j];}
      std::cout << "],\"last_valid\":[";
      for(int j=0;j<4;j++){if(j)std::cout<<',';std::cout<<aebLastValid[j];}
      std::cout << "]}\n";
    }
    else return 3;
  }
  return 0;
}
'''


def frame_bytes(name: str, values: dict) -> tuple[int, bytes]:
    msg = load_db().get_message_by_name(name)
    data = encode(name, values)
    if any(s.name == "CHECKSUM" for s in msg.signals):
        data = data[:-1] + bytes([toyota_checksum(msg.frame_id, data)])
    return msg.frame_id, data


class Vectors:
    def __init__(self) -> None:
        self.commands: list[str] = []
        self.expected: list[dict] = []
        self.labels: list[str] = []
        self.hand_checks = 0
        self.edges: set[tuple[int, int, int]] = set()
        self.ecu = AebEcu()
        self.t0 = 0.0

    def reset(self, t: float = 0.0) -> None:
        self.commands.append(f"R {t:.17g}")
        self.ecu = AebEcu()
        # Reset at an absolute bench timestamp: translate origin, no rule changes.
        self.ecu.sup.t0 = t
        self.ecu.sm.entered = t
        self.t0 = t

    def switch(self, value: int) -> None:
        self.commands.append(f"M {value}")
        self.ecu.main_switch = bool(value)

    def seed(self, state: int, entered: float = 0.0, ok_since: float | None = 0.0) -> None:
        self.commands.append(f"X {state} {entered:.17g} {int(ok_since is not None)} {(ok_since or 0):.17g}")
        self.ecu.sm.state = S(state)
        self.ecu.sm.entered = entered
        self.ecu.ok_since = ok_since

    def raw(self, t: float, fid: int, data: bytes) -> None:
        self.commands.append(f"F {t:.17g} {fid} {len(data)} " + " ".join(str(b) for b in data))
        self.ecu.on_frame(t, fid, data)

    def frame(self, t: float, name: str, values: dict) -> None:
        fid, data = frame_bytes(name, values)
        self.raw(t, fid, data)

    def healthy(self, t: float, *, speed: float = 50.0, distance: float = 300.0,
                relative: float = 0.0, gas: float = 0.0, brake: int = 0) -> None:
        for name, values in (
            ("SPEED", {"SPEED": speed}),
            ("LEAD_INFO", {"LEAD_LONG_DIST": distance, "LEAD_REL_SPEED": relative}),
            ("GAS_PEDAL", {"GAS_PEDAL": gas}),
            ("BRAKE_MODULE", {"BRAKE_PRESSED": brake}),
        ):
            self.frame(t, name, values)

    def step(self, t: float, label: str, want: tuple[int, int] | None = None,
             *, counter: int | None = None) -> dict:
        counter = len(self.expected) % 1024 if counter is None else counter
        self.commands.append(f"S {t:.17g} {counter}")
        before = len(self.ecu.sm.log)
        old = int(self.ecu.sm.state)
        state = int(self.ecu.step(t))
        rule = int(self.ecu.sm.log[-1][3][1:]) if len(self.ecu.sm.log) > before else 0
        if rule:
            self.edges.add((rule, old, state))
        if want is not None:
            if (state, rule) != want:
                raise AssertionError(f"hand vector {label}: Python gives {(state, rule)}, expected {want}")
            self.hand_checks += 1
        pre = encode("PRE_COLLISION", self.ecu.output_values("PRE_COLLISION", counter))
        hud = pack_byte_signals("PCS_HUD", self.ecu.output_values("PCS_HUD", counter))
        latest = self.ecu.latest
        row = {
            "state": state, "rule": rule,
            "speed": latest.get("SPEED", {}).get("SPEED", 0.0),
            "distance": latest.get("LEAD_INFO", {}).get("LEAD_LONG_DIST", 300.0),
            "relative": latest.get("LEAD_INFO", {}).get("LEAD_REL_SPEED", 0.0),
            "gas": latest.get("GAS_PEDAL", {}).get("GAS_PEDAL", 0.0),
            "brake": int(latest.get("BRAKE_MODULE", {}).get("BRAKE_PRESSED", 0)),
            "decel": self.ecu.sm.decel_request(), "pre": list(pre), "hud": list(hud),
            "received": [int(self.ecu.sup.state[n].last_valid is not None) for n in INPUT_NAMES],
            "bad": [self.ecu.sup.state[n].consecutive_bad for n in INPUT_NAMES],
            "last_valid": [self.ecu.sup.state[n].last_valid if self.ecu.sup.state[n].last_valid is not None
                           else self.t0 for n in INPUT_NAMES],
        }
        self.expected.append(row)
        self.labels.append(label)
        return row


def build_vectors() -> Vectors:
    v = Vectors()
    # Hand-authored source/target list, checked against the original table below.
    edges = [(1, s, 0) for s in (1, 2, 3, 4, 5)] + [(2, 0, 5), (3, 0, 1)]
    edges += [(4, s, 5) for s in (1, 2, 3, 4)] + [(5, 5, 1)]
    edges += [(6, s, 1) for s in (2, 3, 4)] + [(7, 2, 1)]
    edges += [(8, s, 1) for s in (3, 4)] + [(9, s, 1) for s in (3, 4)]
    edges += [(10, 2, 1)] + [(11, s, 4) for s in (1, 2, 3)]
    edges += [(12, s, 3) for s in (1, 2)] + [(13, 1, 2)]
    for rule, source, target in edges:
        args: dict = {}
        if rule == 6: args.update(distance=5, relative=-10, gas=40)
        if rule == 7: args.update(distance=20, relative=-10, brake=1)
        if rule == 8: args.update(distance=5, relative=-10, speed=0.4)
        if rule == 10: args.update(distance=29, relative=-10)
        if rule == 11: args.update(distance=8.9, relative=-10)
        if rule == 12: args.update(distance=15.9, relative=-10)
        if rule == 13: args.update(distance=25.9, relative=-10)
        v.reset()
        v.healthy(10.0, **args)
        v.seed(source)
        if rule == 1: v.switch(0)
        if rule in (2, 4):
            fid, data = frame_bytes("SPEED", {"SPEED": 50})
            bad = data[:-1] + bytes([data[-1] ^ 1])
            v.raw(10, fid, bad)
            v.raw(10, fid, bad)
        v.step(10, f"edge_R{rule:02d}_{source}_{target}", (target, rule))
    expected_edges = {(int(tr.rid[1:]), int(s), int(tr.target)) for tr in TRANSITIONS for s in tr.sources}
    if set(edges) != expected_edges or len(edges) != 27:
        raise AssertionError("hand edge catalogue differs from original 13-rule/27-edge table")

    # Strict TTC boundaries, inclusive speed window, pedal and release behavior.
    cases = [
        ("ttc_fcw_equal", 1, {"distance": 26, "relative": -10}, 0, (1, 0)),
        ("ttc_fcw_below", 1, {"distance": 25.95, "relative": -10}, 0, (2, 13)),
        ("ttc_partial_equal", 1, {"distance": 16, "relative": -10}, 0, (2, 13)),
        ("ttc_partial_below", 1, {"distance": 15.95, "relative": -10}, 0, (3, 12)),
        ("ttc_full_equal", 1, {"distance": 9, "relative": -10}, 0, (3, 12)),
        ("ttc_full_below", 1, {"distance": 8.95, "relative": -10}, 0, (4, 11)),
        ("speed_low_below", 1, {"speed": 7.99, "distance": 20, "relative": -10}, 0, (1, 0)),
        ("speed_low_equal", 1, {"speed": 8, "distance": 20, "relative": -10}, 0, (2, 13)),
        ("speed_high_equal", 1, {"speed": 130, "distance": 20, "relative": -10}, 0, (2, 13)),
        ("speed_high_above", 1, {"speed": 130.01, "distance": 20, "relative": -10}, 0, (1, 0)),
        ("gas_equal", 1, {"gas": 40, "distance": 5, "relative": -10}, 0, (1, 0)),
        ("gas_below", 1, {"gas": 39.5, "distance": 5, "relative": -10}, 0, (4, 11)),
        ("driver_suppresses_fcw", 1, {"brake": 1, "distance": 20, "relative": -10}, 0, (1, 0)),
        ("driver_does_not_suppress_braking", 1, {"brake": 1, "distance": 15, "relative": -10}, 0, (3, 12)),
        ("full_no_partial_downgrade", 4, {"distance": 15, "relative": -10}, 0, (4, 0)),
        ("braking_no_ttc_hysteresis_release", 3, {"distance": 50, "relative": -10}, 0, (3, 0)),
        ("closing_equal_minimum", 1, {"distance": 0.05, "relative": -0.1}, 0, (1, 0)),
        ("closing_above_minimum", 1, {"distance": 0.05, "relative": -0.125}, 0, (4, 11)),
        ("no_target_sentinel", 1, {"distance": 300, "relative": -40}, 0, (1, 0)),
        ("zero_distance_defined_ttc", 1, {"distance": 0, "relative": -10}, 0, (4, 11)),
        ("standstill_equal", 4, {"speed": 0.5, "distance": 5, "relative": -10}, 10, (4, 0)),
        ("standstill_below", 4, {"speed": 0.49, "distance": 5, "relative": -10}, 10, (1, 8)),
        ("brake_dwell_below", 4, {}, 9.501, (4, 0)),
        ("brake_dwell_equal", 4, {}, 9.5, (1, 9)),
        ("fcw_dwell_below", 2, {}, 9.201, (2, 0)),
        ("fcw_dwell_over", 2, {}, 9.199, (1, 10)),
        ("fcw_hysteresis_below", 2, {"distance": 28.95, "relative": -10}, 0, (2, 0)),
        ("fcw_hysteresis_equal", 2, {"distance": 29, "relative": -10}, 0, (1, 10)),
    ]
    for label, state, args, entered, want in cases:
        v.reset()
        v.healthy(10, **args)
        v.seed(state, entered)
        v.step(10, label, want)

    # Timeout boundary at the literal floating expression used by the oracle.
    for name, period in zip(INPUT_NAMES, (0.020, 0.050, 0.020, 0.025)):
        deadline = 3 * period
        for delta, want in ((0.0, (1, 0)), (1e-6, (5, 4))):
            v.reset()
            v.healthy(0)
            v.step(0, "timeout_start", (1, 3))
            t = deadline + delta
            for other in INPUT_NAMES:
                if other != name:
                    vals = {"SPEED": {"SPEED": 50}, "LEAD_INFO": {"LEAD_LONG_DIST": 300, "LEAD_REL_SPEED": 0},
                            "GAS_PEDAL": {"GAS_PEDAL": 0}, "BRAKE_MODULE": {"BRAKE_PRESSED": 0}}[other]
                    v.frame(t, other, vals)
            v.step(t, f"timeout_{name}_{delta}", want)

    # Invalid frames cannot refresh a timeout; one bad is tolerated, two trip.
    for kind in ("checksum", "dlc_short", "dlc_long", "range_speed", "range_distance"):
        v.reset()
        v.healthy(0)
        v.step(0, f"bad_{kind}_startup", (1, 3))
        name = "LEAD_INFO" if kind == "range_distance" else "SPEED"
        vals = {"LEAD_LONG_DIST": 350, "LEAD_REL_SPEED": 0} if name == "LEAD_INFO" else {"SPEED": 260 if kind == "range_speed" else 50}
        fid, data = frame_bytes(name, vals)
        if kind == "checksum": data = data[:-1] + bytes([data[-1] ^ 1])
        if kind == "dlc_short": data = data[:-1]
        if kind == "dlc_long": data += b"\0"
        v.raw(0.01, fid, data)
        v.step(0.01, f"bad_{kind}_one", (1, 0))
        v.raw(0.02, fid, data)
        v.step(0.02, f"bad_{kind}_two", (5, 4))
        for k in range(3, 104):
            t = k / 100
            v.healthy(t)
            v.step(t, f"bad_{kind}_heal_{k}")
        v.step(1.03, f"bad_{kind}_recovered", (1, 0))

    # Long deterministic frame stream: stale values, errors, pedals and main switch.
    v.reset(20.0)
    for k in range(2001):
        t = 20 + k / 1000
        if k % 20 == 0:
            v.frame(t, "SPEED", {"SPEED": max(0, 60 - k * .02)})
            v.frame(t, "GAS_PEDAL", {"GAS_PEDAL": 45 if 1100 <= k < 1250 else 0})
        if k % 25 == 0: v.frame(t, "BRAKE_MODULE", {"BRAKE_PRESSED": int(500 <= k < 700)})
        if k % 50 == 0 and not 800 <= k < 1000:
            fid, data = frame_bytes("LEAD_INFO", {"LEAD_LONG_DIST": max(0, 45 - k * .025), "LEAD_REL_SPEED": -15})
            if 1400 <= k < 1500: data = data[:-1] + bytes([data[-1] ^ 1])
            v.raw(t, fid, data)
        if k == 1700: v.switch(0)
        if k == 1800: v.switch(1)
        if k % 10 == 0: v.step(t, f"stream_{k}")

    # Exact output bytes at all states and counter wrap, independently DBC encoded.
    for state in range(6):
        for counter in (0, 1, 255, 256, 511):
            v.reset()
            v.healthy(10, distance=15 if state in (3, 4) else 300, relative=-10 if state in (3, 4) else 0)
            v.seed(state, 10, 10)
            if state == 0: v.switch(0)
            v.step(10, f"output_state_{state}_counter_{counter}", counter=counter)
    if not expected_edges.issubset(v.edges):
        raise AssertionError(f"unvisited edges: {expected_edges - v.edges}")
    return v


def compare(actual: list[dict], vectors: Vectors) -> None:
    if len(actual) != len(vectors.expected):
        raise AssertionError(f"step count differs: C++ {len(actual)} vs Python {len(vectors.expected)}")
    for label, got, want in zip(vectors.labels, actual, vectors.expected):
        for key, expected in want.items():
            value = got[key]
            if isinstance(expected, float):
                ok = math.isclose(value, expected, rel_tol=0, abs_tol=1e-9)
            elif key == "last_valid":
                ok = len(value) == len(expected) and all(math.isclose(a, b, rel_tol=0, abs_tol=1e-9)
                                                       for a, b in zip(value, expected))
            else:
                ok = value == expected
            if not ok:
                raise AssertionError(f"{label} {key}: C++ {value!r} vs Python {expected!r}")


def run_core_program(backend: CompilerBackend, source_cpp: str, commands: str):
    use_wsl = backend.name == "wsl"
    with tempfile.TemporaryDirectory(prefix="aeb_portable_core_") as temp:
        cpp = Path(temp) / "runner.cpp"
        binary = Path(temp) / ("runner.exe" if os.name == "nt" and not use_wsl else "runner")
        cpp.write_text(source_cpp, encoding="utf-8")
        def native_path(path: Path) -> str:
            if not use_wsl: return str(path)
            return run_tool([backend.command[0], "--exec", "wslpath", "-a", str(path)]).stdout.strip()
        cpp_path, binary_path = native_path(cpp), native_path(binary)
        if backend.name == "msvc":
            # /W3 reports normal CAPL byte/narrowing diagnostics; no algorithm
            # casts or warning suppressions are added to the literal source.
            flags = ["/nologo", "/std:c++17", "/O2", "/W3", "/EHsc", "/utf-8",
                     cpp_path, "/Fe" + binary_path, "/Fo" + str(Path(temp) / "runner.obj")]
        else:
            flags = ["-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", cpp_path, "-o", binary_path]
        compiled = run_tool([*backend.command, *flags], timeout=60,
                            environment=backend.environment, cwd=temp)
        run_cmd = [backend.command[0], "--exec", binary_path] if use_wsl else [binary_path]
        out = run_tool(run_cmd, input=commands, timeout=60, environment=backend.environment)
        actual = [json.loads(line) for line in out.stdout.splitlines() if line.strip()]
    return actual, compiled.stdout + compiled.stderr


def run_check(compiler_choice: str = "auto") -> dict:
    source = CORE.read_text(encoding="utf-8")
    portable = unwrap_variables(source)
    vectors = build_vectors()
    prefix = "#include <cstdint>\n#include <iostream>\n#include <iomanip>\n#include <string>\n"
    prefix += "using byte=uint8_t; using dword=uint32_t;\n#define long int32_t\n"
    source_cpp = prefix + portable + "\n#undef long\n" + RUNNER
    commands = "\n".join(vectors.commands) + "\n"
    backend, probe_failures = select_compiler(compiler_choice)
    try:
        actual, diagnostics = run_core_program(backend, source_cpp, commands)
        compare(actual, vectors)
    except (AssertionError, ValueError, OSError, subprocess.SubprocessError) as exc:
        raise CoreCheckError(backend, probe_failures, exc) from exc
    return {
        "label": "portable_core_checked", "portable_core_checked": True,
        "canoe_compiled": False, "canoe_executed": False,
        "scope": "Literal .cin functions via C++ against unmodified Python AebEcu; CAPL handlers and CANoe are unverified.",
        "adaptation": "Unwrap variables block; byte=uint8_t, dword=uint32_t, long=int32_t; function source unchanged.",
        **backend_evidence(backend, probe_failures),
        "compiler_requested": compiler_choice, "compiler_diagnostics": diagnostics,
        "core_sha256": hashlib.sha256(source.encode()).hexdigest(),
        "oracle_sha256": hashlib.sha256((ROOT / "canbus_analyzer" / "aeb_function.py").read_bytes()).hexdigest(),
        "runner_sha256": hashlib.sha256(source_cpp.encode()).hexdigest(),
        "transcript_sha256": hashlib.sha256(commands.encode()).hexdigest(),
        "compared_steps": len(actual), "hand_expected_steps": vectors.hand_checks,
        "transition_rules": len({r for r, _, _ in vectors.edges}), "transition_edges": len(vectors.edges),
        "checks": ["13 rules / 27 source-state edges", "handwritten TTC, speed, pedal, dwell and hysteresis boundaries",
                   "per-input strict timeout, DLC, checksum, range, valid-frame recovery",
                   "decoded values, supervisor history, state, rule, deceleration and all PRE/HUD output bytes",
                   "signed Motorola fields, counter wrap, main switch without reset, absolute reset origin",
                   "deterministic asynchronous input stream"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, help="optional JSON evidence file")
    parser.add_argument("--compiler", choices=("auto", "g++", "wsl", "msvc"), default="auto",
                        help="Windows auto: MSVC, then WSL; other platforms: native g++")
    args = parser.parse_args()
    try:
        report = run_check(args.compiler)
        code = 0
    except (AssertionError, ValueError, OSError, subprocess.SubprocessError) as exc:
        report = {"label": "portable_core_checked", "portable_core_checked": False,
                  "canoe_compiled": False, "canoe_executed": False,
                  "compiler_requested": args.compiler, "error": str(exc)}
        if isinstance(exc, CompilerSelectionError):
            report["compiler_probe_failures"] = exc.attempts
        if isinstance(exc, CoreCheckError):
            report.update(backend_evidence(exc.backend, exc.probe_failures))
            exc = exc.cause
        if isinstance(exc, subprocess.CalledProcessError):
            report["tool_stderr"] = (exc.stderr.decode("utf-16-le", errors="replace")
                                     if isinstance(exc.stderr, bytes) else exc.stderr)
        code = 1
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    print(text, end="")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
