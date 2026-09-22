"""fixture.capture against the peer P4 controller on GPIO32/33: a write to an address nobody
answers (NACK trace, the worklist-B shape) and a write the P4 I2C target answers, observed on
the same channels inside one plan."""

import re
import time

from oep_client.v0 import codec, tlv
from oep_client.v0.decode import decode_i2c
from oep_client.v0.services import FixtureCapture, P4I2cTarget

SDA, SCL = 32, 33
RATE, SAMPLES = 1_000_000, 20_000  # 20 ms window at 1 MHz


def _peer(peer, command, pattern, timeout=5):
    peer.write(command)
    return peer.expect(re.compile(pattern), timeout=timeout)


def _capture_write(probe, capture, peer, hz, payload):
    capture.arm()
    m = _peer(peer, f"WRITE {hz} {payload.hex()}", rb"WRITE result=0x([0-9a-f]+) bytes=(\d+)")
    st = capture.wait(2.0)
    assert st.flags & FixtureCapture.COMPLETE, f"capture did not complete: flags=0x{st.flags:02x}"
    t0 = time.perf_counter()
    data = capture.read_all(st.samples)
    dt = time.perf_counter() - t0
    trace = decode_i2c(data, scl_bit=0, sda_bit=1)
    period = sorted(trace.scl_periods)[len(trace.scl_periods) // 2] if trace.scl_periods else 0
    print(f"HIL capture @{hz} Hz: controller=0x{m.group(1).decode()} samples={len(data)} read {dt:.3f} s "
          f"scl_period_median={period} samples trace: {trace.summary()}")
    return int(m.group(1), 16), trace, period


def test_capture_i2c_nack_and_ack(probe, peers):
    peer = peers["p4b"]
    peer.write("?")
    peer.expect_exact("# HIL peer_p4b", timeout=10)
    probe.confirm(); probe.list_functions()
    capture = FixtureCapture(probe, probe.find(*codec.DEF_FIXTURE_CAPTURE[:2]).function)
    target = P4I2cTarget(probe, probe.find(*codec.DEF_P4_I2C_TARGET[:2]).function)
    payload = bytes((0x10 + i) & 0xFF for i in range(4))

    # 1. nobody at 0x42: START, address byte NACKed, STOP
    lease, _ = probe.plan_apply(capture.assignments(SCL, SDA))
    try:
        cfg = capture.configure(RATE, SAMPLES)
        print(f"\nHIL capture configure: rate={cfg.actual_sample_rate_hz} samples={cfg.samples} lines={cfg.lines}")
        result, trace, period = _capture_write(probe, capture, peer, 100_000, payload)
        assert result != 0, "controller reported success with no target on the bus"
        assert [e.kind for e in trace.events][:2] == ["start", "byte"], trace.summary()
        first = trace.bytes()[0]
        assert first == (0x42 << 1, False), f"expected address 0x84 NACK, got {first}"
        assert trace.events[-1].kind == "stop", trace.summary()
        assert 8 <= period <= 12, f"SCL period {period} samples at 1 MHz for 100 kHz"
    finally:
        probe.plan_release(lease)

    # 2. one plan: I2C target + capture on the same SDA/SCL, target ACKs and receives
    lease, _ = probe.plan_apply(target.assignments(sda=SDA, scl=SCL) + capture.assignments(SCL, SDA))
    try:
        target.configure(0x42, P4I2cTarget.MODE_FIXED_RX)
        target.arm_rx(len(payload))
        capture.configure(RATE, SAMPLES)
        result, trace, period = _capture_write(probe, capture, peer, 100_000, payload)
        time.sleep(0.05)
        pending, got = target.read_rx()
        print(f"HIL target received {got.hex()} (match={got == payload})")
        assert result == 0
        assert trace.bytes() == [(0x42 << 1, True)] + [(b, True) for b in payload], trace.summary()
        assert got == payload
        # 3. the declared maximum sample rate must be real: at max_hz a 100 kHz SCL period is max_hz / 100 kHz samples
        max_hz = int.from_bytes({t: v for t, v in tlv.decode(probe.describe(capture.function))}[codec.TLV_CORE_MAX_CLOCK_HZ], "little")
        cfg = capture.configure(max_hz, 8 * 65000)
        result, trace, period = _capture_write(probe, capture, peer, 100_000, payload)
        expected = max_hz // 100_000
        print(f"HIL capture at declared max {max_hz} Hz: configured samples={cfg.samples} SCL period {period} samples (expected {expected})")
        assert result == 0 and abs(period - expected) <= max(1, expected // 20), f"declared {max_hz} Hz not confirmed: period {period}"
    finally:
        probe.plan_release(lease)
