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


def test_readme_check_detects_a_changed_headline_number():
    sys.path.insert(0, str(ROOT / "scripts"))
    import readme_check
    rendered = readme_check.render((ROOT / "README.md").read_text(encoding="utf-8"))
    start = rendered.index("<!-- BEGIN:headline -->")
    i = rendered.index("PASS", start) - 2          # last digit of the pass count
    assert rendered[i].isdigit()
    tampered = rendered[:i] + str((int(rendered[i]) + 1) % 10) + rendered[i + 1:]
    assert readme_check.render(tampered) == rendered


@pytest.mark.parametrize("idx", range(15))
def test_readme_check_detects_a_changed_prose_number(idx):
    """Every result number quoted in hand-written prose is pinned: change one digit
    of any listed fragment and the check reports it."""
    sys.path.insert(0, str(ROOT / "scripts"))
    import readme_check
    text = (ROOT / "README.md").read_text(encoding="utf-8")
    facts = readme_check.prose_facts()
    assert len(facts) == 15 and readme_check.missing_prose(text) == []
    frag = facts[idx]
    j = next((k for k, ch in enumerate(frag) if ch.isdigit()), None)
    if j is None:          # fragment without digits (e.g. 'four'): change a letter instead
        j = len(frag) - 1
        bad = frag[:j] + "X" + frag[j + 1:]
    else:
        bad = frag[:j] + str((int(frag[j]) + 1) % 10) + frag[j + 1:]
    assert readme_check.missing_prose(text.replace(frag, bad)) == [frag]
