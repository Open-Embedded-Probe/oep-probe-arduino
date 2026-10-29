# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Open Embedded Probe
"""The host side of LibraryTest.cpp: capture SDA / SCL while a case runs on the probe, then place every line of its
log in the capture by the probe's clock.

    OEP_LIBTEST_PORT=<port> OEP_LIBTEST_PINS=32,33 pytest extras/LibraryTest/test_library_test.py

(pip install oep-client-python pytest; the pins as LibraryTest.cpp chose them for the chip: P4 32,33, classic ESP32
25,26, RP2040 / RP2350 26,27.)
"""
import os
import struct

import pytest

from oep_client import capture, core, link
from oep_client.probe_log import ProbeLog, sample_of

PORT = os.environ.get("OEP_LIBTEST_PORT")
SDA, SCL = (int(p) for p in os.environ.get("OEP_LIBTEST_PINS", "32,33").split(","))
NAME = "io.github.example.library-test"
RUN = 0x10

pytestmark = pytest.mark.skipif(not PORT, reason="OEP_LIBTEST_PORT names the probe")


@pytest.fixture
def probe():
    hst = link.open_host(PORT)
    hst.open(5000)
    yield hst
    hst.end()


def falls(bits):
    return [i for i in range(1, len(bits)) if bits[i - 1] and not bits[i]]


def test_write_to_an_absent_address_lines_up_with_the_wire(probe):
    lc, log = capture.LogicCapture(probe), ProbeLog(probe, NAME)
    core.plan_apply(probe, [(lc.fn, 0, SDA), (lc.fn, 1, SCL)])
    try:
        log.clear()
        # the START (SDA falls while SCL is high) is the trigger; 200 samples kept before it
        cfg = lc.configure(rate=1_000_000, samples=8192, trigger=(capture.EDGE, 0, 1), pretrigger=200)
        lc.start()
        result = struct.unpack("<i", probe.call(log.fn, RUN, bytes([1])).payload)[0]
        (seg,) = lc.wait(timeout=5.0)
        data = lc.read_segment(seg)
        sda, scl = lc.channel(data, 0, seg.samples), lc.channel(data, 1, seg.samples)
        lines = log.log()
        assert not lines.lost and not lines.gap, lines.lost
        texts = [line.text for line in lines.lines]
        assert texts[0] == "## write 2 bytes to 0x50" and texts[-1] == "##" and f"endTransmission={result}" in texts
        assert result != 0                                   # nobody at 0x50 (2 NACK; 4 on a bus with no pull-ups)
        # where the lines fall: the heading before the START, the closing one after the last SCL edge
        tolerance = seg.start_uncertainty_ns * float(cfg.rate) / 1e9 + 1
        at = {line.text: sample_of(line, seg.start_ns, cfg.rate) for line in lines.lines}
        start = seg.trigger_index
        last_scl = max(falls(scl) + [i for i in range(1, len(scl)) if scl[i] and not scl[i - 1]])
        assert at["## write 2 bytes to 0x50"] <= start + tolerance, (at, start)
        assert at["##"] >= last_scl - tolerance, (at, last_scl)
        print({k: round(v, 1) for k, v in at.items()}, "START", start, "last SCL edge", last_scl,
              "+-", round(tolerance, 1), "samples")
    finally:
        core.plan_release(probe, [lc.fn])
