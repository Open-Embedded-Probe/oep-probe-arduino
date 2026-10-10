"""Run existing portable C++ and profile checks inside the uv/pytest workspace."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def test_portable_cpp(record_property):
    record_property("oep_spec_commit", (ROOT / "tests/vectors/SPEC_COMMIT").read_text().strip())
    record_property("oep_scope", "portable regression against vendored SPEC; no physical conformance")
    proc = subprocess.run(['sh', 'tests/host/run.sh'], cwd=ROOT, capture_output=True, text=True, timeout=600)
    assert proc.returncode == 0, (proc.stdout + proc.stderr)[-12000:]


def test_release_profiles():
    proc = subprocess.run([sys.executable, 'tests/compile/test_profiles.py'], cwd=ROOT,
                          capture_output=True, text=True, timeout=60)
    assert proc.returncode == 0, proc.stdout + proc.stderr
