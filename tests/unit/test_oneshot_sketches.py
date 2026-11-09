"""A one-shot test's sketch (tests/oneshot/<name>) builds the released firmware by including it; what cannot be
included - the sketch folder's build_opt.h - must be the firmware's own, byte for byte (or absent with it)."""
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "examples/Firmware/OepProbe"
ONESHOTS = sorted(p for p in (ROOT / "tests/oneshot").iterdir() if p.is_dir() and any(p.glob("*.ino")))


@pytest.mark.parametrize("sketch", ONESHOTS, ids=lambda p: p.name)
def test_a_oneshot_sketch_builds_the_firmware_as_released(sketch):
    ino = next(sketch.glob("*.ino")).read_text()
    if "examples/Firmware/OepProbe/OepProbe.ino" not in ino:
        pytest.skip("not the released firmware")
    opt, own = sketch / "build_opt.h", FIRMWARE / "build_opt.h"
    assert opt.exists() == own.exists()
    if own.exists():
        assert opt.read_bytes() == own.read_bytes()
