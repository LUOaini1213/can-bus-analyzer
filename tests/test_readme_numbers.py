"""README result blocks must equal what the scripts generated in results/."""
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
NEEDED = ["results/run_seed0_none/summary.json", "results/run_seed0_all/summary.json",
          "results/sweep/summary.json", "results/mutation.csv", "results/function/summary.json"]


def test_readme_numbers_match_results():
    missing = [p for p in NEEDED if not (ROOT / p).exists()]
    if missing:
        pytest.fail(f"results not generated yet: {missing} (run `python -m canbus_analyzer all` "
                    f"and `python scripts/mutation_test.py`)")
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "readme_check.py")],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_readme_check_detects_a_changed_number(tmp_path, monkeypatch):
    sys.path.insert(0, str(ROOT / "scripts"))
    import readme_check
    text = (ROOT / "README.md").read_text(encoding="utf-8")
    rendered = readme_check.render(text)
    # flip one digit inside the sweep block and make sure render() restores it
    start = rendered.index("<!-- BEGIN:sweep -->")
    i = next(k for k in range(start + 25, len(rendered)) if rendered[k].isdigit())
    tampered = rendered[:i] + str((int(rendered[i]) + 1) % 10) + rendered[i + 1:]
    assert tampered != rendered and readme_check.render(tampered) == rendered
