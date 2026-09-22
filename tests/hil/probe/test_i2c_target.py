"""P4 vendor I2C target (fixed-rx / framed-rx / preloaded-tx) against the peer P4 controller on GPIO32/33."""

import re
import time

from oep_client.v0 import codec
from oep_client.v0.services import P4I2cTarget
from oep_client.v0 import tlv

SDA, SCL = 32, 33


def _peer(peer, command, pattern, timeout=5):
    peer.write(command)
    return peer.expect(re.compile(pattern), timeout=timeout)


def test_p4_i2c_target_modes(probe, peers):
    peer = peers["p4b"]
    peer.write("?")
    peer.expect_exact("# HIL peer_p4b", timeout=10)
    probe.confirm(); probe.list_functions()
    fn = probe.find(*codec.DEF_P4_I2C_TARGET[:2])
    assert fn is not None, "vendor tool p4.i2c-target not offered"
    target = P4I2cTarget(probe, fn.function)
    items = tlv.decode(probe.describe(target.function))
    limits = {t: v for t, v in items if t in (codec.TLV_CORE_MAX_LENGTH, codec.TLV_CORE_MAX_CLOCK_HZ)}
    print(f"\nHIL p4.i2c-target describe: max_length={int.from_bytes(limits[codec.TLV_CORE_MAX_LENGTH], 'little')} "
          f"max_clock_hz={int.from_bytes(limits[codec.TLV_CORE_MAX_CLOCK_HZ], 'little')}")
    max_hz = int.from_bytes(limits[codec.TLV_CORE_MAX_CLOCK_HZ], "little")
    lease, _ = probe.plan_apply(target.assignments(sda=SDA, scl=SCL))
    try:
        # fixed-rx: 4 bytes at 100 kHz, then 32 bytes
        target.configure(0x42, P4I2cTarget.MODE_FIXED_RX)
        for length, hz in ((4, 100000), (32, 400000)):
            target.arm_rx(length)
            payload = bytes((0x10 + i) & 0xFF for i in range(length))
            m = _peer(peer, f"WRITE {hz} {payload.hex()}", rb"WRITE result=0x([0-9a-f]+) bytes=(\d+)")
            assert m.group(1) == b"0", f"controller reported 0x{m.group(1).decode()}"
            time.sleep(0.05)
            pending, data = target.read_rx()
            print(f"HIL fixed-rx {length} B @ {hz} Hz: got {len(data)} B match={data == payload} pending={pending}")
            assert data == payload
        # framed-rx: header + payload transactions, 100 kHz and the declared maximum clock
        target.configure(0x42, P4I2cTarget.MODE_FRAMED_RX)
        for length, hz in ((16, 100000), (128, max_hz)):
            payload = bytes((0x11 + i) & 0xFF for i in range(length))
            m = _peer(peer, f"FRAME {hz} {payload.hex()}", rb"FRAME header=0x([0-9a-f]+) payload=0x([0-9a-f]+) bytes=(\d+)")
            assert m.group(1) == b"0" and m.group(2) == b"0"
            time.sleep(0.05)
            pending, data = target.read_rx()
            print(f"HIL framed-rx {length} B @ {hz} Hz: got {len(data)} B match={data == payload}")
            assert data == payload
        # preloaded-tx: two 16-byte slots read back in order at 400 kHz
        target.configure(0x42, P4I2cTarget.MODE_PRELOADED_TX)
        slots = [bytes((0x80 + k * 16 + i) & 0xFF for i in range(16)) for k in range(2)]
        for s in slots:
            target.preload_tx(s)
        for s in slots:
            m = _peer(peer, "READ 400000 16", rb"READ result=0x([0-9a-f]+) data=([0-9a-f]+)")
            assert m.group(1) == b"0"
            got = bytes.fromhex(m.group(2).decode())
            print(f"HIL preloaded-tx slot: match={got == s}")
            assert got == s
        st = target.status()
        print(f"HIL status flags=0x{st.flags:02x} rx_frames={st.rx_frames} tx_slots={st.tx_slots} errors={st.errors}")
        assert st.errors == 0 and st.rx_frames == 4
    finally:
        probe.plan_release(lease)
