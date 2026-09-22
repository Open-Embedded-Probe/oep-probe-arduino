"""Worklist P0 reliability gate on the new stack: 20 full verifies, 20 alternating differential
programs, and 5 host-silence interruptions mid-program (watchdog must reset the target and the
next connection must recover the image). Destructive to the last 8 KiB of the X035 flash."""

import os
import statistics
import time

from oep_client.v0 import codec
from oep_client.v0.flash_image import Target, program_image, verify_image


def _stats(values):
    values = sorted(values)
    return f"mean {statistics.mean(values):.3f} median {statistics.median(values):.3f} p95 {values[int(len(values) * 0.95) - 1]:.3f} max {values[-1]:.3f} s"


def test_reliability_gate(probe):
    target = Target(probe)
    target.preflight()
    geometry = target.flash.geometry()
    base_image = bytearray(target.memory.read_range(geometry.base, geometry.size))
    tail = geometry.size - 8192
    image_a = bytes(base_image[:tail]) + os.urandom(8192)
    image_b = bytes(base_image[:tail]) + os.urandom(8192)
    first = program_image(target, image_a)
    assert first.verified

    # 1. full verify x 20
    verify_times = []
    for _ in range(20):
        outcome = verify_image(target, image_a)
        assert outcome.verified
        verify_times.append(outcome.timings["verify"])
    print(f"\nHIL gate verify x20: {_stats(verify_times)}")

    # 2. differential program x 20 alternating A/B (32 pages each time)
    program_times, verify2 = [], []
    for i in range(20):
        image = image_b if i % 2 == 0 else image_a
        outcome = program_image(target, image)
        assert outcome.verified and outcome.pages_changed == 32, outcome.as_dict()
        program_times.append(outcome.timings["program"])
        verify2.append(outcome.timings["verify"])
    print(f"HIL gate program x20 (32 pages): {_stats(program_times)}; verify after program: {_stats(verify2)}")

    # 3. host goes silent mid-program x 5: send a few pages, stop, wait for the probe watchdog
    for i in range(5):
        image = image_a if i % 2 == 0 else image_b
        target.preflight()
        pages = [(geometry.base + tail + k * geometry.page, image[tail + k * geometry.page:tail + (k + 1) * geometry.page])
                 for k in range(32)]
        failed = target.flash.program_pages(pages[:6])   # partial: 6 of 32 pages
        assert not failed
        time.sleep(2.5)                                   # > 1.5 s idle -> abandonAll() -> target reset, lease released
        status = target.control.status()
        assert not (status.flags & 4), f"watchdog did not release the halted target (flags={status.flags})"
        outcome = program_image(target, image)            # recovery: remaining pages, then full verify
        assert outcome.verified, outcome.as_dict()
        print(f"HIL gate interrupt {i + 1}/5: recovered pages={outcome.pages_changed} verified={outcome.verified}")

    final = target.control.status()
    assert not (final.flags & 2)
