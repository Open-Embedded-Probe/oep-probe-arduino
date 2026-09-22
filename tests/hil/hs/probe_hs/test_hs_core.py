"""OEP v0 over the P4 HS vendor bulk transport: confirm/list/identity, round trip, bulk read throughput."""

import statistics
import time

from oep_client.v0 import codec
from oep_client.v0.services import FixtureCapture, ProbeIdentity


def test_hs_confirm_identity_roundtrip_throughput(probe):
    limits = probe.confirm()
    functions = probe.list_functions()
    identity = ProbeIdentity(probe, probe.find(*codec.DEF_PROBE_IDENTITY[:2]).function).get()
    print(f"\nHIL HS confirm: max_frame={limits.max_frame} window={limits.window_bytes} inflight={limits.max_inflight} functions={len(functions)} "
          f"profile=0x{identity.profile_id:08x} firmware=0x{identity.firmware_revision:08x}")
    assert (limits.max_frame, limits.window_bytes, limits.max_inflight) == (1024, 16384, 16)
    assert identity.profile_id == 0x50344853  # 'P4HS'

    samples = []
    for _ in range(200):
        t0 = time.perf_counter(); probe.ping(); samples.append(time.perf_counter() - t0)
    samples.sort()
    print(f"HIL HS ping x200: median {samples[100]*1e6:.0f} us min {samples[0]*1e6:.0f} p95 {samples[189]*1e6:.0f} us")

    # pipelined pings: 200 requests admitted within the window, coalesced into bulk writes
    t0 = time.perf_counter()
    responses = probe.pipeline([(codec.DEF_CORE_FUNCTION, codec.CORE_OP_PING, b"") for _ in range(200)])
    dt = time.perf_counter() - t0
    assert len(responses) == 200 and all(r.succeeded for r in responses)
    print(f"HIL HS pipelined ping x200: {dt*1e6/200:.0f} us/request")

    # bulk data path: a capture of 261k samples (one byte each) read back through the pipeline
    capture = FixtureCapture(probe, probe.find(*codec.DEF_FIXTURE_CAPTURE[:2]).function)
    lease, _ = probe.plan_apply(capture.assignments(4))   # any allowed pin; nothing connected on the USB bench
    try:
        cfg = capture.configure(20_000_000, 8 * 65_000 // 2)
        capture.arm(); st = capture.wait(2.0)
        assert st.flags & FixtureCapture.COMPLETE
        t0 = time.perf_counter(); data = capture.read_all(st.samples); dt = time.perf_counter() - t0
        print(f"HIL HS capture read: {len(data)} bytes in {dt:.3f} s = {len(data)/dt/1e6:.2f} MB/s (HWCDC path: 261632 B in 0.34 s = 0.77 MB/s)")
        assert len(data) == st.samples
    finally:
        probe.plan_release(lease)
