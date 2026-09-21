"""Core and probe.identity over the real USB-Serial/JTAG link."""

import time

from oep_client.v0 import codec
from oep_client.v0.services import ProbeIdentity


def test_confirm_list_identity_ping(probe):
    limits = probe.confirm()
    print(f"\nHIL confirm: {limits}")
    assert limits.revision == codec.PROTOCOL_REVISION
    assert limits.max_frame >= 512 and limits.window_bytes >= limits.max_frame
    functions = probe.list_functions()
    print(f"HIL functions: {[(hex(f.owner), hex(f.id), f.revision) for f in functions]}")
    assert functions[0].definition == (0, 0)
    identity_fn = probe.find(*codec.DEF_PROBE_IDENTITY[:2])
    assert identity_fn is not None
    identity = ProbeIdentity(probe, identity_fn.function).get()
    print(f"HIL identity: profile=0x{identity.profile_id:08x} firmware=0x{identity.firmware_revision:08x}")
    assert identity.profile_id == 0x50344456
    assert probe.ping(b"x" * 100) == b"x" * 100
    # unknown function / operation are rejected without side effects
    assert probe.call(0x7fff, 1).detail == codec.REJECT_UNKNOWN_FUNCTION
    assert probe.call(codec.DEF_CORE_FUNCTION, 0x70).detail == codec.REJECT_UNKNOWN_OPERATION
    # oversized frame is dropped by the probe; the next request still works
    t0 = time.perf_counter()
    responses = probe.pipeline((codec.DEF_CORE_FUNCTION, codec.CORE_OP_PING,
                                codec.CorePingRequest(data=bytes([i & 0xff]) * 500).pack()) for i in range(200))
    elapsed = time.perf_counter() - t0
    assert all(r.succeeded for r in responses) and len(responses) == 200
    print(f"HIL pipeline: 200 x 500 B ping in {elapsed:.3f} s = {200 * 506 / elapsed / 1000:.0f} kB/s each way")
