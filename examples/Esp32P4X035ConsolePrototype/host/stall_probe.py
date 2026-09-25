# /// script
# dependencies = ["pyserial>=3.5"]
# ///
"""Deep look at the dmseq console that comes back late after a refused automatic attach (probe-cdc-and-persistence
§7.5.1). Brings the fixture into the stall (bind attach on open with a wrong chip_id, then the right one), then joins
the connection as a host and looks through DMI: DATA0 sampled densely, DMCONTROL / CFGR, the reaction to CFGR written
again and to an invalid answer written to DATA0.

  stall_probe.py <console tty> [tries]
"""
import pathlib, struct, sys, time
HERE = pathlib.Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[4] / "oep-client-python" / "src"))
import serial  # noqa: E402
from oep_client.v1 import core, link, message as m, riscv  # noqa: E402

PORT = "/run/board-identify/by-id/esp32-series-30eda0e31108"
A = link.open_host(PORT)
A.open(30000)
fc, fb = core.find(A, "oep.probe.config"), core.find(A, "oep.test.console-bind")
wire = riscv.Wire(A, "oep.wire.rvswd")
tlv = lambda t, v: bytes([t, len(v)]) + v
st = lambda: struct.unpack("<BBBBIIB", A.call(fb, 0x01, locked=False).payload[:13])
ds = lambda: struct.unpack("<IIII", A.call(fb, 0x02, locked=False).payload[:16])


def force_detach():
    if st()[2]:
        A.call(wire.fn, 0x03, bytes([1]) + m.tlv(1, b"", critical=True))


def into_stall(tty):
    force_detach()
    A.call(fc, 2, tlv(4, bytes([0, 2, 1, 0]) + struct.pack("<HB", 1, 2)) + tlv(5, struct.pack("<HI", 1, 0x12345678)))
    p = serial.Serial(tty, 115200, timeout=0.05); time.sleep(0.6); p.close()
    A.call(fc, 2, tlv(5, struct.pack("<HI", 1, 0x035e0601)))
    p = serial.Serial(tty, 115200, timeout=0.05)
    got, t0 = b"", time.monotonic()
    while time.monotonic() - t0 < 1.5:
        got += p.read(4096)
    return p, got


def uniq(vals):
    out = []
    for v in vals:
        if not out or out[-1][0] != v:
            out.append([v, 1])
        else:
            out[-1][1] += 1
    return " ".join(f"{v:08x}x{n}" for v, n in out)


tty = sys.argv[1]
tries = int(sys.argv[2]) if len(sys.argv) > 2 else 5
for attempt in range(tries):
    p, got = into_stall(tty)
    if b"uptime" not in got:
        break
    print(f"attempt {attempt}: no stall (console came at once), again")
    p.close()
else:
    sys.exit("could not reproduce the stall")
print("in the stall:", st(), "probe dmseq stats", ds())
conn, _ = wire.attach(halt=False)
dm = riscv.RiscvDm(A, conn)
R = dm.step_read
_, v = dm.dmi([R(0x04)] * 60)
print("A  DATA0 x60 back to back:", uniq(v))
_, (ctl, sts, cfg, shd, d1) = dm.dmi([R(0x10), R(0x11), R(0x7d), R(0x7e), R(0x05)])
print(f"B  DMCONTROL {ctl:08x} DMSTATUS {sts:08x} CFGR {cfg:08x} SHDWCFGR {shd:08x} DATA1 {d1:08x}")
time.sleep(0.3)
_, v = dm.dmi([R(0x04)] * 20)
print("   DATA0 after 0.3 s:", uniq(v))
W = dm.step_write
_, v = dm.dmi([W(0x7e, 0x5aa50400), W(0x7d, 0x5aa50400), W(0x7e, 0x5aa50400), W(0x7d, 0x5aa50400)] + [R(0x04)] * 40)
print("C  after CFGR written again, DATA0:", uniq(v))
time.sleep(0.1)
_, v = dm.dmi([R(0x04)] * 20)
print("   100 ms later:", uniq(v))
_, v = dm.dmi([W(0x04, 0x00000001)] + [R(0x04)] * 40)
print("D  after an invalid answer 0x00000001, DATA0:", uniq(v))
time.sleep(0.1)
_, v = dm.dmi([R(0x04)] * 20)
print("   100 ms later:", uniq(v))
got = b""
t0 = time.monotonic()
while time.monotonic() - t0 < 2:
    got += p.read(4096)
print("port meanwhile:", got[-60:])
p.close()
force_detach()
A.call(fc, 2, tlv(4, b"") + tlv(5, b""))
A.end(); A.link.close()
