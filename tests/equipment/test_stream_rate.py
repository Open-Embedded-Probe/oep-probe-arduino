"""Explicit equipment only: an ESP32-P4 / P4X probe streams its logic capture at the rate the guide states (docs/guide
capture.ja.md, "流し続け"): 40 MB/s - 2 lines at 160 MHz, 4 at 80, 8 at 40, 16 at 20 - for OEP_STREAM_SECONDS with
nothing lost. The bytes are counted and dropped as they come (a 20 s run is 800 MB), so the host's memory and CPU stay
out of the measurement; a host busier than twice its cores (load average) cannot judge it and the case is skipped as busy.

  OEP_STREAM_ADDRESS   the probe (oep://<unit_id>, a serial port, ...)
  OEP_STREAM_UNIT_ID   the unit id it must answer with
  OEP_STREAM_CHANNELS  free channels by name, comma separated (GPIO12,GPIO13,...): the first n make an n-line case
  OEP_STREAM_SECONDS   optional, default 20
  OEP_STREAM_MAX_LOAD  optional: the load average over which the case is skipped as busy (default: twice the cores -
                       on a 4-core host every case kept 40 MB/s at 6-9, 8 lines lost segments at 12-15)
"""
import os
import struct
import time

import pytest
from oep_client import capture, core, link, registry as reg

STREAM_BYTES_PER_S = 40_000_000          # the guide's streaming rate on HS USB (capture.ja.md)
SLICE_S = 0.5                            # bytes are counted and dropped after each slice
SUBSCRIBE_MIN_BYTES, SUBSCRIBE_MAX_DELAY_MS = 16384, 20   # batched pushes, as oep_client.acquire subscribes a stream
P4_MODELS = ("esp32p4", "esp32p4x")


def _env():
    keys = ("OEP_STREAM_ADDRESS", "OEP_STREAM_UNIT_ID", "OEP_STREAM_CHANNELS")
    got = {k: os.environ.get(k, "") for k in keys}
    if not any(got.values()):
        pytest.skip("no OEP_STREAM_* equipment configured")
    missing = [k for k, v in got.items() if not v]
    assert not missing, f"partly configured: {', '.join(missing)} missing"
    return got["OEP_STREAM_ADDRESS"], got["OEP_STREAM_UNIT_ID"], got["OEP_STREAM_CHANNELS"].split(","), \
        float(os.environ.get("OEP_STREAM_SECONDS", "20"))


def _channel_numbers(hst, names):
    """The probe's channel numbers by name (core §7.5 channel_name: channel u16, name)."""
    tag = reg.CORE.tlv["describe"]["channel_name"]
    by_name = {v[2:].decode(): struct.unpack_from("<H", v)[0] for t, v in core.describe(hst, 0) if t == tag}
    unknown = [n for n in names if n not in by_name]
    assert not unknown, f"not channels of this probe: {unknown}"
    return [by_name[n] for n in names]


@pytest.mark.equipment
@pytest.mark.parametrize("lines", [2, 4, 8, 16])
def test_stream_at_the_guides_rate(lines, record_property):
    address, unit, names, seconds = _env()
    if len(names) < lines:
        pytest.skip(f"{len(names)} channels given, {lines} needed")
    load = os.getloadavg()[0]
    most = float(os.environ.get("OEP_STREAM_MAX_LOAD", 2 * (os.cpu_count() or 1)))
    record_property("load_average", load)
    if load > most:
        pytest.skip(f"host busy: load average {load:.1f} over {most:g} - the rate needs an idle host")
    rate = STREAM_BYTES_PER_S * 8 // lines
    hst = link.open_host(address)
    try:
        core.take(hst, 10000, owner="oep-probe-arduino stream rate")
        assert (core.unit_id(hst) or "").lower() == unit.lower(), f"unit id {core.unit_id(hst)}, expected {unit}"
        model = next((v.decode() for t, v in core.describe(hst, 0) if t == reg.CORE.tlv["describe"]["model"]), "")
        if model not in P4_MODELS:
            pytest.skip(f"model {model}: the guide's 40 MB/s is the ESP32-P4 / P4X's")
        cap = capture.LogicCapture(hst)
        channels = _channel_numbers(hst, names[:lines])
        core.plan_apply(hst, [(cap.fn, role, ch) for role, ch in enumerate(channels)])
        try:
            cfg = cap.configure(rate=rate, mode=capture.STREAMING)
            record_property("rate", rate)
            record_property("actual_rate", str(cfg.rate))
            cap.subscribe(SUBSCRIBE_MIN_BYTES, SUBSCRIBE_MAX_DELAY_MS)
            cap.start()
            got, total, gaps, skipped = capture.Received(), 0, 0, 0
            t0 = time.monotonic()
            try:
                while time.monotonic() - t0 < seconds:
                    cap.stream(hst.link, seconds=SLICE_S, into=got)
                    total += len(got.data)
                    gaps += len(got.gaps)
                    skipped += sum(n for _, n in got.gaps)
                    if got.start is not None:          # the next push follows on from here: keep the position, drop the bytes
                        got.start += len(got.data) + sum(n for _, n in got.gaps)
                    got.data, got.gaps = bytearray(), []
            finally:
                cap.stop()
            got = cap.finish(hst.link, got)
            total += len(got.data)
            gaps += len(got.gaps)
            skipped += sum(n for _, n in got.gaps)
            took = time.monotonic() - t0
            st = cap.status()
        finally:
            core.plan_release(hst, [cap.fn])
        record_property("bytes", total)
        record_property("seconds", round(took, 2))
        record_property("bytes_per_s", round(total / took))
        record_property("gaps", gaps)
        record_property("seq_lost", got.seq_lost)
        assert st.state != capture.STATE["error"], f"the capture stopped with error {st.error}"
        assert not gaps and not got.seq_lost, \
            f"{gaps} gaps ({skipped} bytes the probe dropped), {got.seq_lost} frames lost in {total} bytes"
        assert total >= STREAM_BYTES_PER_S * seconds * 0.95, f"{total} bytes in {took:.1f} s"
    finally:
        try:
            hst.end()
        finally:
            hst.link.close()
