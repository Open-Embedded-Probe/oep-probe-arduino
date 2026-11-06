"""A one-shot test's sketch (tests/oneshot/<name>) builds the released firmware by including it; what cannot be
included - the sketch folder's build_opt.h - must be the firmware's own, byte for byte."""
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
    opt = sketch / "build_opt.h"
    assert opt.exists() and opt.read_bytes() == (FIRMWARE / "build_opt.h").read_bytes()
