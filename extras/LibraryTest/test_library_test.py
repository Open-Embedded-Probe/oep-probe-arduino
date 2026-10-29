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
import time

import pytest

from oep_client import capture, core, link
from oep_client.probe_log import ProbeLog, sample_of

PORT = os.environ.get("OEP_LIBTEST_PORT")
SDA, SCL = (int(p) for p in os.environ.get("OEP_LIBTEST_PINS", "32,33").split(","))
NAME = "io.github.example.library-test"
RUN, STATUS = 0x10, 0x20
IDLE, RUNNING, DONE = 0, 1, 2

pytestmark = pytest.mark.skipif(not PORT, reason="OEP_LIBTEST_PORT names the probe")


@pytest.fixture
def probe():
    hst = link.open_host(PORT)
    hst.open(5000)
    yield hst
    hst.end()
    hst.link.close()                                         # the port free for the next test


def run_case(hst, log, case, timeout_s=10.0):
    """Start a case and wait for its end by status (the probe answers all along). -> (result, the states seen)."""
    hst.call(log.fn, RUN, bytes([case]))
    seen, deadline = [], time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state, which, result = struct.unpack("<BBi", hst.call(log.fn, STATUS, locked=False).payload[:6])
        seen.append(state)
        if state == DONE:
            assert which == case
            return result, seen
        time.sleep(0.005)
    raise TimeoutError(f"case {case} still running after {timeout_s} s")


def captured(lc, log, hst, case, rate, samples):
    """START (SDA falling) as the trigger, the case run, the segment and the log read."""
    log.clear()
    cfg = lc.configure(rate=rate, samples=samples, trigger=(capture.EDGE, 0, 1), pretrigger=200)
    lc.start()
    result, seen = run_case(hst, log, case)
    (seg,) = lc.wait(timeout=5.0)
    data = lc.read_segment(seg)
    lines = log.log()
    assert not lines.lost and not lines.gap, lines.lost
    at = [(line.text, sample_of(line, seg.start_ns, cfg.rate)) for line in lines.lines]
    tolerance = seg.start_uncertainty_ns * float(cfg.rate) / 1e9 + 1
    return cfg, seg, data, result, seen, at, tolerance


def rises(bits):
    return [i for i in range(1, len(bits)) if bits[i] and not bits[i - 1]]


def test_write_to_an_absent_address_lines_up_with_the_wire(probe):
    lc, log = capture.LogicCapture(probe), ProbeLog(probe, NAME)
    core.plan_apply(probe, [(lc.fn, 0, SDA), (lc.fn, 1, SCL)])
    try:
        cfg, seg, data, result, _, at, tolerance = captured(lc, log, probe, 1, 1_000_000, 8192)
        scl = lc.channel(data, 1, seg.samples)
        texts = [t for t, _ in at]
        assert texts[0] == "## write 2 bytes to 0x50" and texts[-1] == "##" and f"endTransmission={result}" in texts
        assert result != 0                                   # nobody at 0x50 (2 NACK; 4 on a bus with no pull-ups)
        where = dict(at)
        last_scl = max(rises(scl))
        assert where["## write 2 bytes to 0x50"] <= seg.trigger_index + tolerance, (at, seg.trigger_index)
        assert where["##"] >= last_scl - tolerance, (at, last_scl)
        print(at, "START", seg.trigger_index, "last SCL rise", last_scl, "+-", round(tolerance, 1), "samples")
    finally:
        core.plan_release(probe, [lc.fn])


def test_a_long_sequence_runs_while_the_probe_answers(probe):
    """The display-like case (about 100 ms of commands, waits and rows): status answers 'running' while it runs, and
    every line falls in the capture in its order, each phase's first transfer after its marker."""
    lc, log = capture.LogicCapture(probe), ProbeLog(probe, NAME)
    core.plan_apply(probe, [(lc.fn, 0, SDA), (lc.fn, 1, SCL)])
    try:
        cfg, seg, data, result, seen, at, tolerance = captured(lc, log, probe, 4, 1_000_000, 200_000)
        assert RUNNING in seen, f"status never saw the case running: {seen}"
        samples = [s for _, s in at]
        assert samples == sorted(samples), at
        sda = lc.channel(data, 0, seg.samples)
        falls = [i for i in range(1, len(sda)) if sda[i - 1] and not sda[i]]   # SDA falling: STARTs and data bits
        for text in ("## init", "## draw"):
            s = dict(at)[text]
            assert [i for i in falls if i >= s - tolerance], f"no transfer after {text} at {s}"
        print(len(at), "lines;", [(t, round(s)) for t, s in at if t.startswith("##")], "+-", round(tolerance, 1))
    finally:
        core.plan_release(probe, [lc.fn])
